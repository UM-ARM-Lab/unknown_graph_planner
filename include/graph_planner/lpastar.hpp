#ifndef LPASTAR_HPP
#define LPASTAR_HPP

// NOTE: THIS IS UNFINISHED. DO NOT USE.

namespace arc_dijkstras {
/*****************************
 **    LPA*
 ****************************/

/*
 *  See paper: http://idm-lab.org/bib/abstracts/papers/icaps05.pdf
 *  A generalized framework for lifelong planning A* search
 *
 *    Note - this is unfinished. Do not use without review
 */

typedef std::pair<double, double> LPAstarKey;
// typedef std::array<double, 3> LPAstarKey;

// struct LPAstarPQueueElement
// {
//     int64_t node_id;

//     LPAstarPQueueElement(int64_t node_id) :
//         node_id(node_id){}
// };
typedef int64_t LPAstarPQueueElement;

class CompareLPAstarKey {
 public:
  bool operator()(const LPAstarKey& lhs, const LPAstarKey& rhs) const {
    if (lhs.first == rhs.first) {
      return lhs.second <= rhs.second;
    }

    return lhs.first < rhs.first;
  }
};

template <typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
class LPAstar {
 protected:
  const Graph<NodeValueType, Allocator>& graph;
  const int64_t start_index;
  const int64_t goal_index;
  const std::function<bool(const Graph<NodeValueType, Allocator>&, const GraphEdge&)>& edge_validity_check_fn;
  const std::function<double(const Graph<NodeValueType, Allocator>&, const GraphEdge&)>& distance_fn;
  const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn;
  const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_consistent_fn;

  // std::priority_queue<LPAstarPQueueElement,
  //                     std::vector<LPAstarPQueueElement>,
  //                     CompareLPAstarPQueueElementFn> queue;
  pqueue<LPAstarKey, LPAstarPQueueElement, CompareLPAstarKey> queue;

  std::unordered_map<int64_t, double> g_map;
  std::unordered_map<int64_t, double> rhs_map;

 protected:
  double g(int64_t node_id) {
    if (g_map.count(node_id) == 0) {
      return std::numeric_limits<double>::infinity();
    }
    return g_map[node_id];
  }

  double rhs(int64_t node_id) {
    if (rhs_map.count(node_id) == 0) {
      return std::numeric_limits<double>::infinity();
    }
    return rhs_map[node_id];
  }

  LPAstarKey calculateKey(int64_t node_id) {
    int64_t s = node_id;
    double h = heuristic_fn(graph.getNode(node_id).getValue(), graph.getNode(goal_index).getValue());
    double h_cons = heuristic_consistent_fn(graph.getNode(node_id).getValue(), graph.getNode(goal_index).getValue());

    if (g(node_id) < rhs(node_id)) {
      return std::make_pair(g(s) + h_cons, g(s));
    }
    return std::make_pair(rhs(s) + h, rhs(s));

    // double k1 = std::min(g(node_id),rhs(node_id)) + heuristic_fn(graph.getNode(node_id).getValue(),
    //                                                              graph.getNode(goal_index).getValue());
    // double k2 = std::min(g(node_id), rhs(node_id));
    // return std::make_pair(k1, k2);
  }

  double cost(const GraphEdge& e) {
    if (!edge_validity_check_fn(graph, e)) {
      return std::numeric_limits<double>::infinity();
    }
    return distance_fn(graph, e);
  }

  void updateVertex(int64_t u) {
    // std::cout << "Updating vertex " << u << "\n";
    if (u != start_index) {
      double min_pred_cost = std::numeric_limits<double>::infinity();
      for (const GraphEdge& e : graph.getNode(u).getInEdges()) {
        int64_t s_prime = e.getFromIndex();
        double val = g(s_prime) + cost(e);
        if (val < min_pred_cost) {
          min_pred_cost = val;
        }
      }
      rhs_map[u] = min_pred_cost;
    }

    if (queue.contains(LPAstarPQueueElement(u)) && g(u) != rhs(u)) {
      queue.update({calculateKey(u), LPAstarPQueueElement(u)});
    } else if (queue.contains(LPAstarPQueueElement(u)) && g(u) == rhs(u)) {
      queue.remove(u);
    } else if (!queue.contains(LPAstarPQueueElement(u)) && g(u) != rhs(u)) {
      queue.insert({calculateKey(u), LPAstarPQueueElement(u)});
    }
    // std::cout << "Vertex updated\n";
  }

  arc_helpers::AstarResult findPath() {
    // std::cout << "Returning path\n";
    arc_helpers::AstarResult result;

    result.second = g(goal_index);
    if (result.second >= std::numeric_limits<double>::max()) {
      return result;
    }

    // std::cout << "Path cost " << result.second << "\n";

    std::vector<int64_t> path{goal_index};
    int64_t cur_index = goal_index;
    while (cur_index != start_index) {
      const auto& node = graph.getNode(cur_index);
      double min_val = std::numeric_limits<double>::infinity();
      for (const auto& e : node.getInEdges()) {
        double cc = g(e.getFromIndex()) + distance_fn(graph, e);
        // std::cout << "  cost to " << e.getFromIndex() << " is " << cc << "\n";
        if (cc < min_val) {
          min_val = cc;
          cur_index = e.getFromIndex();
        }
      }
      if (min_val >= std::numeric_limits<double>::max()) {
        throw std::logic_error("Best path has infinite cost");
      }
      // std::cout << "Node " << cur_index << " on path with cost " << min_val << "\n";
      path.push_back(cur_index);
    }
    std::reverse(path.begin(), path.end());
    result.first = path;
    // std::cout << "Found\n";
    return result;
  }

 public:
  LPAstar(const Graph<NodeValueType, Allocator>& graph, const int64_t start_index, const int64_t goal_index,
          const std::function<bool(const Graph<NodeValueType, Allocator>&, const GraphEdge&)>& edge_validity_check_fn,
          const std::function<double(const Graph<NodeValueType, Allocator>&, const GraphEdge&)>& distance_fn,
          const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn)
      : LPAstar(graph, start_index, goal_index, edge_validity_check_fn, distance_fn, heuristic_fn, heuristic_fn) {}

  LPAstar(const Graph<NodeValueType, Allocator>& graph, const int64_t start_index, const int64_t goal_index,
          const std::function<bool(const Graph<NodeValueType, Allocator>&, const GraphEdge&)>& edge_validity_check_fn,
          const std::function<double(const Graph<NodeValueType, Allocator>&, const GraphEdge&)>& distance_fn,
          const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
          const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_consistent_fn)
      :

        graph(graph),
        start_index(start_index),
        goal_index(goal_index),
        edge_validity_check_fn(edge_validity_check_fn),
        distance_fn(distance_fn),
        heuristic_fn(heuristic_fn),
        heuristic_consistent_fn(heuristic_consistent_fn) {
    // Enforced sanity checks
    if ((start_index < 0) || (start_index >= (int64_t)graph.getNodes().size())) {
      throw std::invalid_argument("Start index out of range");
    }
    if ((goal_index < 0) || (goal_index >= (int64_t)graph.getNodes().size())) {
      throw std::invalid_argument("Goal index out of range");
    }
    if (start_index == goal_index) {
      throw std::invalid_argument("Start and goal indices must be different");
    }

    rhs_map[start_index] = 0;
    // g_map[start_index] = 0;

    queue.insert({calculateKey(start_index), LPAstarPQueueElement(start_index)});
  }

  arc_helpers::AstarResult computeShortestPath() {
    // std::cout << "Starting compute shortest path\n";
    while (!queue.isEmpty() &&
           (CompareLPAstarKey()(queue.top().first, calculateKey(goal_index)) || rhs(goal_index) != g(goal_index))) {
      // std::cout << "Computing shortest path\n";
      int64_t u = queue.top().second;
      queue.pop();
      if (g(u) > rhs(u)) {
        g_map[u] = rhs(u);
        for (const GraphEdge& e : graph.getNode(u).getOutEdges()) {
          updateVertex(e.getToIndex());
        }
      } else {
        g_map[u] = std::numeric_limits<double>::infinity();
        for (const GraphEdge& e : graph.getNode(u).getOutEdges()) {
          updateVertex(e.getToIndex());
        }
        updateVertex(u);
      }
      // std::cout << "Iteration complete\n";
    }
    // std::cout << "Path found...\n";
    return findPath();
  }

  void updateEdgeCost(const GraphEdge& e) { updateVertex(e.getToIndex()); }
};
}  // namespace arc_dijkstras
#endif
