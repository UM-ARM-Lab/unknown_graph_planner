#ifndef CTP_HPP
#define CTP_HPP

#include <random>

#include "2d_obstacles.hpp"
#include "dijkstras_addons.hpp"
#include "halton_graph.hpp"

namespace CTP {
typedef int64_t Action;
typedef int64_t Location;
typedef std::vector<int64_t> Path;

class Agent {
 public:
  Location current_node;
  Location goal_node;
  Agent(int start, int goal) : current_node(start), goal_node(goal) {}
};

/*************************
 *    N-lookahead traveler's problem
 **************************/
template <typename BeliefGraph>
class NltpProblem {
 public:
  BeliefGraph belief_graph;
  GraphD true_graph;
  Agent agent;
  bool inprogress = true;
  int look_ahead;

 public:
  /*
   *  the belief graph and true graph must have the same topology
   */
  NltpProblem(BeliefGraph g, GraphD t, Agent a, int look_ahead)
      : belief_graph(g), true_graph(t), agent(a), look_ahead(look_ahead) {
    updateBeliefGraph();
  }

  bool solved() { return agent.current_node == agent.goal_node; }

  /*
   *   Updates the belief graph to match the true graph
   *    for all edges within distance d of node location l
   *
   *   Node: This function is very inefficient for large d
   */
  void lookAhead(Location l, int d) {
    if (d == 0) {
      return;
    }

    auto &n_b = belief_graph.getNode(l);
    const auto &n_t = true_graph.getNode(l);

    auto &e_b = n_b.getOutEdges();
    const auto &e_t = n_t.getOutEdges();

    for (size_t i = 0; i < e_b.size(); i++) {
      auto &e = e_b[i];
      e.setValidity(e_t[i].getValidity());
      // set reverse edge as well
      belief_graph.getReverseEdge(e).setValidity(e_t[i].getValidity());
      lookAhead(e.getToIndex(), d - 1);
    }
  }

  virtual void updateBeliefGraph() { lookAhead(agent.current_node, look_ahead); }

  /*
   *  Moves the agent to the new node. Returns the cost
   */
  virtual double move(Action new_node) {
    auto &e = true_graph.getNode(agent.current_node).getEdgeTo(new_node);
    bool valid = e.getValidity() == arc_dijkstras::EDGE_VALIDITY::VALID;
    if (valid) {
      agent.current_node = new_node;
      updateBeliefGraph();
      inprogress = agent.current_node != agent.goal_node;
      return e.getWeight();
    }

    if (look_ahead > 0) {
      throw std::invalid_argument("Invalid move: no valid edge from current node to new node");
    }

    // It is possible with 0 lookahead that we attempt an invalid edge
    auto &be = belief_graph.getNode(agent.current_node).getEdgeTo(new_node);
    be.setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
    return e.getWeight();  // This is not accurate for all problems.
  }

  std::vector<Action> getActions() {
    std::vector<Action> actions;
    for (const auto &e : belief_graph.getNode(agent.current_node).getOutEdges()) {
      if (e.getValidity() != arc_dijkstras::EDGE_VALIDITY::INVALID) {
        actions.push_back(e.getToIndex());
      }
    }
    return actions;
  }

  bool isEquiv(const NltpProblem<BeliefGraph> &other) {
    if (agent.current_node != other.agent.current_node) {
      return false;
    }
    if (look_ahead != other.look_ahead) {
      return false;
    }

    return arc_dijkstras::haveSameEdgeValidity(belief_graph, other.belief_graph);
  }
};

template <typename BeliefGraph>
class CtpProblem : public NltpProblem<BeliefGraph> {
 public:
  CtpProblem(BeliefGraph g, GraphD t, Agent a) : NltpProblem<BeliefGraph>(g, t, a, 1){};
};

}  // namespace CTP

#endif
