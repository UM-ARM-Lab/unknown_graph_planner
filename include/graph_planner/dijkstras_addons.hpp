#ifndef DIJKSTRAS_ADDONS_HPP
#define DIJKSTRAS_ADDONS_HPP

#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/timing.hpp>
#include <utility>

#include "a_star.hpp"
#include "lpa_pqueue.hpp"

typedef arc_dijkstras::Graph<std::vector<double>> GraphD;

namespace arc_dijkstras {
typedef std::pair<int64_t, int64_t> HashableEdge;
typedef std::map<HashableEdge, double> EvaluatedEdges;

enum FORWARD_LAZY_CHECK_RESULT { EDGE_INVALID, EDGE_VALID, PATH_VALID };

static inline HashableEdge getHashable(const GraphEdge& edge) {
  return std::make_pair(edge.getFromIndex(), edge.getToIndex());
}

static inline HashableEdge getSortedHashable(const GraphEdge& edge) {
  if (edge.getFromIndex() < edge.getToIndex()) {
    return std::make_pair(edge.getFromIndex(), edge.getToIndex());
  }
  return std::make_pair(edge.getToIndex(), edge.getFromIndex());
}

/*
 *  Returns true iff all edges have same validity in g1 and g2
 *  Assumes topology is the same
 */
static bool haveSameEdgeValidity(const GraphD& g1, const GraphD& g2) {
  for (size_t n_id = 0; n_id < g1.getNodes().size(); n_id++) {
    const auto& n1 = g1.getNode(n_id);
    const auto& n2 = g2.getNode(n_id);
    for (size_t e_id = 0; e_id < n1.getOutEdges().size(); e_id++) {
      if (n1.getOutEdges()[e_id].getValidity() != n2.getOutEdges()[e_id].getValidity()) {
        return false;
      }
    }
  }
  return true;
}

/******************************
 **   Lazy SP
 *****************************/

template <typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
class LazySP {
 public:
  /**
   *  This performs a shortest path search using A* specific for the LazySP algortithm.
   *   The weights of the graph are assumed to be initialized to heuristic weights
   *   evalutedEdges is the true edge cost for any evaluated edges
   */
  static arc_helpers::AstarResult PerformAstarForLazySP(
      const Graph<NodeValueType, Allocator>& graph, int64_t start_index, int64_t goal_index,
      const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
      const bool limit_pqueue_duplicates, const EvaluatedEdges& evaluatedEdges) {
    const auto edge_validity_check_function = [&](const Graph<NodeValueType, Allocator>& search_graph,
                                                  const GraphEdge& edge) {
      UNUSED(search_graph);
      if (edge.isInvalid()) {
        return false;
      }

      return edge.getWeight() < std::numeric_limits<double>::infinity();
    };

    const auto distance_function = [&](const Graph<NodeValueType, Allocator>& search_graph, const GraphEdge& edge) {
      UNUSED(search_graph);
      if (evaluatedEdges.count(getHashable(edge))) {
        // std::cout << "Found already evaluted edge!\n";
        return evaluatedEdges.at(getHashable(edge));
      }
      // double planning_cost = 0;
      // if(edge.getValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
      // {
      // PROFILE_START("astar_adding planning cost");
      // planning_cost += 0.06;
      // planning_cost += 1;
      // PROFILE_RECORD("astar_adding planning cost");
      // }
      // else
      // {
      //     PROFILE_START("astar_not_adding_planning_cost");
      //     PROFILE_RECORD("astar_not_adding_planning_cost");
      // }

      // std::cout << "Using heuristic weight\n";
      // return edge.getWeight() + planning_cost;
      return edge.getWeight();
    };

    return AstarLogging<NodeValueType>::PerformLazyAstar(
        // return SimpleGraphAstar<NodeValueType>::PerformLazyAstar(
        graph, start_index, goal_index, edge_validity_check_function, distance_function, heuristic_fn,
        limit_pqueue_duplicates);
  }

  static std::vector<int> ForwardSelector(std::vector<int64_t> path, Graph<NodeValueType, Allocator>& g,
                                          const EvaluatedEdges& evaluatedEdges) {
    int i = 0;
    while (i < (int)path.size() - 1) {
      GraphEdge& e = g.getNode(path[i]).getEdgeTo(path[i + 1]);
      if (e.getValidity() == EDGE_VALIDITY::INVALID) {
        std::cout << "Forward selector encountered edge alredy known to be invalid: ( " << e.getFromIndex() << ", "
                  << e.getToIndex() << ")\n";
        assert(false);
      }

      // if(evaluatedEdges.count(getSortedHashable(e)) > 0 &&
      //    evaluatedEdges.at(getSortedHashable(e)) == std::numeric_limits<double>::infinity())
      // {
      //     std::cout << "Forward selector encountered edge with inf cost: ( " << e.getFromIndex() << ", " <<
      //     e.getToIndex() << ")\n"; assert(false);
      // }

      if (evaluatedEdges.count(getHashable(e)) == 0) {
        return std::vector<int>{i};
      }
      i++;
    }
    return std::vector<int>();
  }

  static std::vector<int> BisectionSelector(std::vector<int64_t> path, Graph<NodeValueType, Allocator>& g,
                                            const EvaluatedEdges& evaluatedEdges) {
    auto sgn = [](int val) {
      if (val > 0) return 1.0;
      return -1.0;
    };

    // std::vector<int> indices(path.size()-1);
    // for(int i=0; i<path.size()-1; i++)
    // {
    //     indices[i] = i;
    // }

    int i = ((int)path.size()) / 2 - 1;
    int next = 1.0;
    std::cout << "Path size: " << path.size() << "\n";
    while (i >= 0 && i < (int)path.size() - 1) {
      std::cout << "Checking edge " << i << "\n";
      GraphEdge& e = g.getNode(path[i]).getEdgeTo(path[i + 1]);
      if (evaluatedEdges.count(getHashable(e)) == 0) {
        return std::vector<int>{i};
      }

      i += next;
      next = -(next + sgn(next));
    }
    return std::vector<int>();
  }

  // static bool checkPath(const std::vector<int64_t> &path,
  //                       Graph<NodeValueType, Allocator>& g,
  //                       EvaluatedEdges &evaluated_edges,
  //                       const std::function<double(Graph<NodeValueType, Allocator>&,
  //                                                  GraphEdge&)>& eval_edge_fn)
  // {
  //     return checkPath(path, g, evaluated_edges, eval_edge_fn, &ForwardSelector);
  // }

  static bool checkPath(
      const std::vector<int64_t>& path, Graph<NodeValueType, Allocator>& g, EvaluatedEdges& evaluated_edges,
      const std::function<double(Graph<NodeValueType, Allocator>&, GraphEdge&)>& eval_edge_fn,
      const std::function<std::vector<int>(const std::vector<int64_t>&, Graph<NodeValueType, Allocator>&,
                                           EvaluatedEdges&)>& selector) {
    bool path_could_be_optimal = true;

    while (path_could_be_optimal) {
      auto path_indicies_to_check = selector(path, g, evaluated_edges);
      if (path_indicies_to_check.size() == 0) {
        return true;
      }

      for (auto i : path_indicies_to_check) {
        GraphEdge& e = g.getNode(path[i]).getEdgeTo(path[i + 1]);
        double evaluated_cost = eval_edge_fn(g, e);
        evaluated_edges[getHashable(e)] = evaluated_cost;
        // if(e.getValidity() == EDGE_VALIDITY::UNKNOWN)
        // {
        //     assert(false && "Edge was evaluated, should not be unknown");
        // }

        if (e.getValidity() == EDGE_VALIDITY::INVALID) {
          evaluated_edges[getHashable(e)] = std::numeric_limits<double>::infinity();
        }

        if (e.getValidity() == EDGE_VALIDITY::INVALID || e.getWeight() < evaluated_cost) {
          path_could_be_optimal = false;
        }
      }
    }
    return false;
  }

  // static arc_helpers::AstarResult
  // PerformLazySP(Graph<NodeValueType, Allocator>& g,
  //               int64_t start_index,
  //               int64_t goal_index,
  //               const std::function<double(const NodeValueType&,
  //                                          const NodeValueType&)>& heuristic_fn,
  //               const std::function<double(Graph<NodeValueType, Allocator>&,
  //                                          GraphEdge&)>& eval_edge_fn,
  //               bool limit_pqueue_duplicates)
  // {
  //     return PerformLazySP(g, start_index, goal_index, heuristic_fn, eval_edge_fn,
  //                          &ForwardSelector);
  // }

  static arc_helpers::AstarResult PerformLazySP(
      Graph<NodeValueType, Allocator>& g, int64_t start_index, int64_t goal_index,
      const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
      const std::function<double(Graph<NodeValueType, Allocator>&, GraphEdge&)>& eval_edge_fn,
      const std::function<std::vector<int>(const std::vector<int64_t>&, Graph<NodeValueType, Allocator>&,
                                           EvaluatedEdges&)>& selector = &ForwardSelector) {
    EvaluatedEdges evaluated_edges;

    int num_astar_iters = 0;

    while (true) {
      PROFILE_START("lazy_sp a_star");
      auto prelim_result = PerformAstarForLazySP(g, start_index, goal_index, heuristic_fn, false, evaluated_edges);
      PROFILE_RECORD("lazy_sp a_star");
      num_astar_iters++;

      auto path = prelim_result.first;

      if (checkPath(path, g, evaluated_edges, eval_edge_fn, selector)) {
        PROFILE_RECORD_DOUBLE("lazysp astar iters", num_astar_iters);
        return prelim_result;
      }
    }
  }

  /***
   *   Bidirectional LazySP
   *   This uses unidirectional A*, searching from start to goal, or goal to start
   *   Edge are reused across searches
   *
   *   param use_time_selection: If True, do not alternate direction, instead choose
                                 the search direction that has been tried for the least time
   */
  static arc_helpers::AstarResult PerformBiLazySP(
      Graph<NodeValueType, Allocator>& g, int64_t start_index, int64_t goal_index,
      const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
      const std::function<double(Graph<NodeValueType, Allocator>&, GraphEdge&)>& eval_edge_fn,
      const std::function<std::vector<int>(const std::vector<int64_t>&, Graph<NodeValueType, Allocator>&,
                                           EvaluatedEdges&)>& selector = &ForwardSelector,
      double time_limit_seconds = std::numeric_limits<double>::max(), bool use_time_selection = true) {
    EvaluatedEdges evaluated_edges;

    int num_astar_iters = 0;
    bool reversed = false;

    double tot_time_forward = 0;
    double tot_time_reverse = 0;
    double tot_time = 0;
    PROFILE_START("total_lazy_sp");

    while (PROFILE_RECORD("total_lazy_sp") < time_limit_seconds) {
      PROFILE_START("lazy_sp a_star");
      PROFILE_START("lazy_sp a_star forward");
      PROFILE_START("lazy_sp a_star reverse");

      if (use_time_selection) {
        reversed = tot_time_forward > tot_time_reverse;
      }

      auto prelim_result =
          PerformAstarForLazySP(g, (!reversed ? start_index : goal_index), (!reversed ? goal_index : start_index),
                                heuristic_fn, true, evaluated_edges);

      PROFILE_RECORD("lazy_sp a_star");

      if (reversed) {
        tot_time_reverse += PROFILE_RECORD("lazy_sp a_star reverse");
      } else {
        tot_time_forward += PROFILE_RECORD("lazy_sp a_star forward");
      }

      num_astar_iters++;

      auto path = prelim_result.first;

      if (checkPath(path, g, evaluated_edges, eval_edge_fn, selector)) {
        PROFILE_RECORD_DOUBLE("lazysp astar iters", num_astar_iters);
        if (reversed) {
          std::reverse(prelim_result.first.begin(), prelim_result.first.end());
        }
        return prelim_result;
      }
      reversed = !reversed;
    }
    std::cout << "LazySP did not find a path in the time limit of " << time_limit_seconds << "s\n";
    return std::make_pair(std::vector<int64_t>(), std::numeric_limits<double>::infinity());
  }
};
}  // namespace arc_dijkstras
#endif
