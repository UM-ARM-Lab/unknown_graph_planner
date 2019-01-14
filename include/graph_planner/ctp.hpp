#ifndef CTP_HPP
#define CTP_HPP

#include "halton_graph.hpp"
#include "dijkstras_addons.hpp"
#include <random>
#include "2d_obstacles.hpp"

namespace CTP{
    typedef int64_t Action;
    typedef int64_t Location;

    class Agent
    {
    public:
        Location current_node;
        Location goal_node;
        Agent(int start, int goal):
            current_node(start), goal_node(goal)
        {}
    };


    /*************************
    *    N-lookahead traveler's problem
    **************************/
    template <typename BeliefGraph>
    class NltpProblem
    {
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
        NltpProblem(BeliefGraph g, GraphD t, Agent a, int look_ahead) :
            belief_graph(g), true_graph(t), agent(a), look_ahead(look_ahead)
        {
            std::cout << "Constructing Nltp\n";
            updateBeliefGraph();
        }

        bool solved()
        {
            return agent.current_node == agent.goal_node;
        }

        /*
         *   Updates the belief graph to match the true graph
         *    for all edges within distance d of node location l
         *
         *   Node: This function is very inefficient for large d
         */
        void lookAhead(Location l, int d)
        {
            if(d == 0)
            {
                return;
            }
            
            auto &n_b = belief_graph.GetNodeMutable(l);
            const auto &n_t = true_graph.GetNodeImmutable(l);

            auto &e_b = n_b.GetOutEdgesMutable();
            const auto &e_t = n_t.GetOutEdgesImmutable();

            for(size_t i = 0; i<e_b.size(); i++)
            {
                auto &e = e_b[i];
                e.SetValidity(e_t[i].GetValidity());
                //set reverse edge as well
                belief_graph.GetReverseEdgeMutable(e).SetValidity(e_t[i].GetValidity());
                lookAhead(e.GetToIndex(), d-1);
            }

        }


        virtual void updateBeliefGraph()
        {
            lookAhead(agent.current_node, look_ahead);
        }


        /*
         *  Moves the agent to the new node. Returns the cost
         */
        virtual double move(Action new_node)
        {
            auto &e = true_graph.GetNodeMutable(agent.current_node).GetEdgeMutable(new_node);
            bool valid = e.GetValidity() == arc_dijkstras::EDGE_VALIDITY::VALID;
            if(valid)
            {
                agent.current_node = new_node;
                updateBeliefGraph();
                inprogress = agent.current_node != agent.goal_node;
                return e.GetWeight();
            }
            
            if(look_ahead > 0)
            {
                throw std::invalid_argument("Invalid move: no valid edge from current node to new node");
            }

            //It is possible with 0 lookahead that we attempt an invalid edge            
            auto &be = belief_graph.GetNodeMutable(agent.current_node).GetEdgeMutable(new_node);
            be.SetValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
            return e.GetWeight(); // This is not accurate for all problems. 
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

        bool isEquiv(const NltpProblem<BeliefGraph> &other)
        {
            if(agent.current_node != other.agent.current_node)
            {
                return false;
            }
            if(look_ahead != other.look_ahead)
            {
                return false;
            }
            
            return arc_dijkstras::haveSameEdgeValidity(belief_graph, other.belief_graph);
        }
        
    };

    
    template <typename BeliefGraph>
    class CtpProblem: public NltpProblem<BeliefGraph>
    {
    public:
        CtpProblem(BeliefGraph g, GraphD t, Agent a) :
            NltpProblem<BeliefGraph>(g, t, a, 1)
        {};
    };

}

#endif
