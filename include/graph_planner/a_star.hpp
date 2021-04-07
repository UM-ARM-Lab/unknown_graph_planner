#ifndef LOGGING_A_STAR_HPP
#define LOGGING_A_STAR_HPP

#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/timing.hpp>
#include <boost/heap/fibonacci_heap.hpp>

/********
 *   This is a copy from arc_utilities/dijkstras.hpp, with added logging
 *
 *
 */

namespace arc_dijkstras {
template <typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
class AstarLogging {
 protected:
  AstarLogging() {}

 private:
  /*
   *   Implementation of LazyAStar
   *   This function is templated to allow const and non-const versions of
   *   Edge validity checking.
   *
   *   Template Parameters
   *   GraphType: either "const Graph<NodeValueType,...>" or "Graph<NodeValueType,...>"
   *   GraphEdgeType: either "const GraphEdge" or "GraphEdge"
   *   EdgeValidityFunctionType: either
   *                           "std::function<bool(Graph<NodeValueType, Allocator>&,
   *                                          GraphEdge&)>"
   *                       or
   *                            "std::function<bool(const Graph<NodeValueType, Allocator>&,
   *                                                const GraphEdge&)>"
   *
   *
   *
   *    Based on:
   *    https://papers.nips.cc/paper/2382-ara-anytime-a-with-provable-bounds-on-sub-optimality.pdf
   */
  template <typename GraphType, typename GraphEdgeType, typename EdgeValidityFunctionType>
  static arc_helpers::AstarResult PerformLazyAstar_impl(
      GraphType& graph, const int64_t start_index, const std::vector<int64_t> goal_indices,
      const EdgeValidityFunctionType& edge_validity_check_fn,
      const std::function<double(const Graph<NodeValueType, Allocator>&, const GraphEdge&)>& distance_fn,
      const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
      const bool limit_pqueue_duplicates) {
    using namespace arc_helpers;
    // Enforced sanity checks
    if ((start_index < 0) || (start_index >= (int64_t)graph.getNodes().size())) {
      throw std::invalid_argument("Start index out of range");
    }
    for(const auto& goal_index: goal_indices) {
      if ((goal_index < 0) || (goal_index >= (int64_t)graph.getNodes().size())) {
        throw std::invalid_argument("Goal index out of range");
      }
      if (start_index == goal_index) {
        throw std::invalid_argument("Start and goal indices must be different");
      }
    }
    // Make helper heuristic function
    const auto heuristic_function = [&](const int64_t node_index) {
      double min_heuristic = std::numeric_limits<double>::infinity();
      for(const auto goal_index: goal_indices){
        double h = heuristic_fn(graph.getNode(node_index).getValue(), graph.getNode(goal_index).getValue());
        if(h < min_heuristic){
          min_heuristic = h;
        }
      }
      return min_heuristic;
    };

    // Profiling information
    int num_node_expansions = 0;
    int num_edge_expansions = 0;
    int num_useful_expansions = 0;

    // Setup
    std::priority_queue<AstarPQueueElement, std::vector<AstarPQueueElement>, CompareAstarPQueueElementFn> open;

    // Key is the node index in the provided graph
    // Value is a pair<backpointer, cost-to-come>
    // backpointer is the parent index in the provided graph
    std::unordered_map<int64_t, std::pair<int64_t, double>> g_value;

    // Initialize
    open.push(AstarPQueueElement(start_index, -1, 0.0, heuristic_function(start_index)));

    // Search
    while (open.size() > 0) {
      // Get the top of the priority queue
      const arc_helpers::AstarPQueueElement n = open.top();
      open.pop();

      num_node_expansions++;

      for(const auto goal_index: goal_indices) {
        if (n.id() == goal_index) {
          // Solution found
          g_value[n.id()] = std::make_pair(n.backpointer(), n.costToCome());
          PROFILE_RECORD_DOUBLE("astar_num_node_expansions", num_node_expansions);
          PROFILE_RECORD_DOUBLE("astar_num_edge_expansions", num_edge_expansions);
          PROFILE_RECORD_DOUBLE("astar_num_lower_cost_edge_expansions", num_useful_expansions);

          return ExtractAstarResult(g_value, start_index, goal_index);
        }
      }

      PROFILE_START("astar_add_neighbors");
      // Explore and add the children
      for (GraphEdgeType& current_out_edge : graph.getNode(n.id()).getOutEdges()) {
        num_edge_expansions++;

        PROFILE_START("astar_edge_validity_check");
        if (!edge_validity_check_fn(graph, current_out_edge)) {
          PROFILE_RECORD("astar_edge_validity_check");
          continue;
        }
        PROFILE_RECORD("astar_edge_validity_check");

        // Get the next potential child node
        const int64_t child_id = current_out_edge.getToIndex();

        if (g_value.count(child_id) == 0) {
          g_value[child_id] = std::make_pair(n.id(), std::numeric_limits<double>::infinity());
        }

        PROFILE_START("astar_cost_to_come");
        // Compute the cost-to-come for the new child
        const double child_cost_to_come = n.costToCome() + distance_fn(graph, current_out_edge);
        PROFILE_RECORD("astar_cost_to_come");

        if (g_value[child_id].second <= child_cost_to_come) {
          PROFILE_START("astar_expansion_in_closed_list");
          PROFILE_RECORD("astar_expansion_in_closed_list");
          continue;
        }

        num_useful_expansions++;

        PROFILE_START("astar_heuristic");
        g_value[child_id] = std::make_pair(n.id(), child_cost_to_come);
        const double child_value = child_cost_to_come + heuristic_function(child_id);
        PROFILE_RECORD("astar_heuristic");
        PROFILE_START("astar_push_to_queue");
        open.push(AstarPQueueElement(child_id, n.id(), child_cost_to_come, child_value));
        PROFILE_RECORD("astar_push_to_queue");
      }
      PROFILE_RECORD("astar_add_neighbors");
    }
    throw std::logic_error("No solution found");
  }

 public:
  // Version that takes const graph and const edge_validity_check
  static arc_helpers::AstarResult PerformLazyAstar(
      const Graph<NodeValueType, Allocator>& graph, const int64_t start_index, const std::vector<int64_t>& goal_indices,
      const std::function<bool(const Graph<NodeValueType, Allocator>&, const GraphEdge&)>& edge_validity_check_fn,
      const std::function<double(const Graph<NodeValueType, Allocator>&, const GraphEdge&)>& distance_fn,
      const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
      const bool limit_pqueue_duplicates) {
    return PerformLazyAstar_impl<const Graph<NodeValueType, Allocator>, const GraphEdge,
                                 std::function<bool(const Graph<NodeValueType, Allocator>&, const GraphEdge&)>>(
        graph, start_index, goal_indices, edge_validity_check_fn, distance_fn, heuristic_fn, limit_pqueue_duplicates);
  }

  // Version that takes non-const graph and non-const edge_validity_check
  // i.e. this can change the graph when doing edge_validity checks,
  //  the intent is that edge validity check can change the edge validitys from
  //  "Unknown" to "Invalid" or "Valid"
  static arc_helpers::AstarResult PerformLazyAstar(
      Graph<NodeValueType, Allocator>& graph, const int64_t start_index, const std::vector<int64_t>& goal_indices,
      const std::function<bool(Graph<NodeValueType, Allocator>&, GraphEdge&)>& edge_validity_check_fn,
      const std::function<double(const Graph<NodeValueType, Allocator>&, const GraphEdge&)>& distance_fn,
      const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
      const bool limit_pqueue_duplicates) {
    return PerformLazyAstar_impl<Graph<NodeValueType, Allocator>, GraphEdge,
                                 std::function<bool(Graph<NodeValueType, Allocator>&, GraphEdge&)>>(
        graph, start_index, goal_indices, edge_validity_check_fn, distance_fn, heuristic_fn, limit_pqueue_duplicates);
  }

  static arc_helpers::AstarResult PerformAstar(
      const Graph<NodeValueType, Allocator>& graph, const int64_t start_index, const std::vector<int64_t> goal_indices,
      const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
      const bool limit_pqueue_duplicates) {
    const auto edge_validity_check_function = [&](const Graph<NodeValueType, Allocator>& search_graph,
                                                  const GraphEdge& edge) {
      UNUSED(search_graph);
      if (edge.getValidity() == EDGE_VALIDITY::INVALID) {
        return false;
      }

      return edge.getWeight() < std::numeric_limits<double>::infinity();
    };
    const auto distance_function = [&](const Graph<NodeValueType, Allocator>& search_graph, const GraphEdge& edge) {
      UNUSED(search_graph);
      return edge.getWeight();
    };
    return PerformLazyAstar(graph, start_index, goal_indices, edge_validity_check_function, distance_function,
                            heuristic_fn, limit_pqueue_duplicates);
  }
};

/***************************
 **     RA*
 **************************/

/*Repeated version of standard A*
 */

// template<typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
// class RAstar
// {
// protected:
//     const Graph<NodeValueType, Allocator>& graph;
//     const int64_t start_index;
//     const int64_t goal_index;
//     const std::function<bool(const Graph<NodeValueType, Allocator>&,
//                              const GraphEdge&)>& edge_validity_check_fn;
//     const std::function<double(const Graph<NodeValueType, Allocator>&,
//                                const GraphEdge&)>& distance_fn;
//     const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn;

//     std::priority_queue<arc_helpers::AstarPQueueElement,
//                         std::vector<arc_helpers::AstarPQueueElement>,
//                         arc_helpers::CompareAstarPQueueElementFn> open;
// public:
//     RAstar(const Graph<NodeValueType, Allocator>& graph,
//            const int64_t start_index,
//            const int64_t goal_index,
//            const std::function<bool(const Graph<NodeValueType, Allocator>&,
//                                     const GraphEdge&)>& edge_validity_check_fn,
//            const std::function<double(const Graph<NodeValueType, Allocator>&,
//                                       const GraphEdge&)>& distance_fn,
//            const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn) :
//         graph(graph), start_index(start_index), goal_index(goal_index),
//         edge_validity_check_fn(edge_validity_check_fn),
//         distance_fn(distance_fn),
//         heuristic_fn(heuristic_fn)
//     {
//         // Enforced sanity checks
//         if ((start_index < 0) || (start_index >= (int64_t)graph.getNodes().size()))
//         {
//             throw std::invalid_argument("Start index out of range");
//         }
//         if ((goal_index < 0) || (goal_index >= (int64_t)graph.getNodes().size()))
//         {
//             throw std::invalid_argument("Goal index out of range");
//         }
//         if (start_index == goal_index)
//         {
//             throw std::invalid_argument("Start and goal indices must be different");
//         }

//         rhs_map[start_index] = 0;
//         // g_map[start_index] = 0;

//         queue.insert({calculateKey(start_index), LPAstarPQueueElement(start_index)});

// };

}  // namespace arc_dijkstras
#endif
