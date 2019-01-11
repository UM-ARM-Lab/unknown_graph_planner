#ifndef CTP_HPP
#define CTP_HPP

#include "halton_graph.hpp"
#include "dijkstras_addons.hpp"
#include <random>
#include "2d_obstacles.hpp"

namespace CTP{
    typedef int64_t Action;

    class Agent
    {
    public:
        int current_node;
        int goal_node;
        Agent(int start, int goal):
            current_node(start), goal_node(goal)
        {}
    };


    template <typename BeliefGraph>
    class CtpProblem
    {
    public:
        BeliefGraph belief_graph;
        GraphD true_graph;
        Agent agent;
        bool inprogress = true;

    public:
        /*
         *  the belief graph and true graph must have the same topology
         */
        CtpProblem(BeliefGraph g, GraphD t, Agent a) :
            belief_graph(g), true_graph(t), agent(a)
        {
            updateBeliefGraph();
        }

        bool solved()
        {
            return agent.current_node == agent.goal_node;
        }


        void updateBeliefGraph()
        {
            auto &n_b = belief_graph.GetNodeMutable(agent.current_node);
            const auto &n_t = true_graph.GetNodeImmutable(agent.current_node);

            auto &e_b = n_b.GetOutEdgesMutable();
            const auto &e_t = n_t.GetOutEdgesImmutable();

            for(size_t i = 0; i<e_b.size(); i++)
            {
                auto &e = e_b[i];
                e.SetValidity(e_t[i].GetValidity());
                //set reverse edge as well
                belief_graph.GetReverseEdgeMutable(e).SetValidity(e_t[i].GetValidity());
            }
        }


        /*
         *  Moves the agent to the new node. Returns the cost
         */
        double move(Action new_node)
        {
            auto &e = true_graph.GetNodeMutable(agent.current_node).GetEdgeMutable(new_node);
            if(e.GetValidity() != arc_dijkstras::EDGE_VALIDITY::VALID)
            {
                throw std::invalid_argument("No valid edge from current node to new node");
            }
            agent.current_node = new_node;
            updateBeliefGraph();
            inprogress = agent.current_node != agent.goal_node;
            return e.GetWeight();
        }

        std::vector<Action> getActions()
        {
            std::vector<Action> actions;
            for(const auto& e:belief_graph.GetNodeImmutable(agent.current_node).GetOutEdgesImmutable())
            {
                if(e.GetValidity() != arc_dijkstras::EDGE_VALIDITY::INVALID)
                {
                    actions.push_back(e.GetToIndex());
                }
            }
            return actions;
        }

        bool isEquiv(const CtpProblem<BeliefGraph> &other)
        {
            if(agent.current_node != other.agent.current_node)
            {
                return false;
            }
            return arc_dijkstras::haveSameEdgeValidity(belief_graph, other.belief_graph);
        }
        
    };
}

#endif