#ifndef CTP_HPP
#define CTP_HPP

#include "halton_graph.hpp"
#include "dijkstras_addons.hpp"
#include <arc_utilities/serialization.hpp>
#include <random>
#include "2d_obstacles.hpp"

namespace CTP{


    class BctpGraph : public virtual HaltonGraph
    {
    public:
        BctpGraph(): HaltonGraph(0,0) {}; //This constructor call is ignored
        virtual GraphD sampleInstance() =0;
    };
    

    /*
     * Graph for a canadian traveler's problem
     */
    class CtpGraph : public BctpGraph
    {
    public:
        std::map<arc_dijkstras::HashableEdge, double> edge_probabilities;
        std::mt19937 rng;
        
    public:
        CtpGraph(int num_vert, double max_edge_dist, int dim=2, double edge_probabilities=0.9):
            HaltonGraph(num_vert, max_edge_dist, dim)
        {
            setEdgeProbabilities(edge_probabilities);
        }

        void setEdgeProbabilities(double universal_probability)
        {
            for(const auto n:GetNodesImmutable())
            {
                for(const auto e:n.GetOutEdgesImmutable())
                {
                    edge_probabilities[arc_dijkstras::getHashable(e)] = universal_probability;
                }
            }
        }


        GraphD sampleInstance()
        {
            GraphD instance(*this);
            std::uniform_real_distribution<> dist(0.0, 1.0);
            for(auto &n:instance.GetNodesMutable())
            {
                for(auto &e:n.GetOutEdgesMutable())
                {
                    if(e.GetValidity() != arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
                    {
                        continue;
                    }
                    
                    double p = edge_probabilities[arc_dijkstras::getHashable(e)];
                    arc_dijkstras::EDGE_VALIDITY validity = arc_dijkstras::EDGE_VALIDITY::INVALID;
                    if(dist(rng) < p)
                    {
                        validity = arc_dijkstras::EDGE_VALIDITY::VALID;
                    }

                    e.SetValidity(validity);
                    //set reverse edge as well, since arc_dijkstras is a directed graph,
                    //but validity applies symmetrically in CTP
                    instance.GetReverseEdgeMutable(e).SetValidity(validity);
                }
            }
            return instance;
        }
    };


    class BctpGrid : public BctpGraph
    {
    public:
        std::mt19937 rng;
        std::vector<HaltonGraph> templates;
        Obstacles2D::Rect storm;
        
        BctpGrid(int rows) : HaltonGraph(0,0), storm(0,0,0,0)
        {
            int cols = rows;
            r_disc = 1.0/((double)rows - 1.00001) * 1.4143;
            initGrid(rows, cols);
            generateTemplates(100);
        }

        void initGrid(int rows, int cols)
        {
            for(int i=0; i<rows; i++)
            {
                for(int j=0; j<cols; j++)
                {
                    std::vector<double> q{(double)i/((double)rows - 1.0), (double)j/((double)cols - 1.0)};
                    addVertexAndEdges(q);
                }
            }
        }

        void generateTemplates(int num_templates)
        {
            for(int i=0; i<num_templates; i++)
            {
                HaltonGraph g(*this);
                Obstacles2D::Rect rect = sampleRect();
                
                
                templates.push_back(g);
            }
        }

        Obstacles2D::Rect sampleRect()
        {
            std::uniform_real_distribution<> dist(0.0, 1.0);
            double w = dist(rng)*0.4 + 0.1;
            double h = dist(rng)*0.4 + 0.1;
            
            double x1 = dist(rng)*(1-w);
            double y1 = dist(rng)*(1-h);
            
            return Obstacles2D::Rect(x1, y1, x1 + w, y1 + h);
        }

        GraphD sampleInstance()
        {
            GraphD instance(*this);
            storm = sampleRect();

            std::uniform_real_distribution<> dist(0.0, 1.0);
            
            for(auto &n:instance.GetNodesMutable())
            {
                for(auto &e:n.GetOutEdgesMutable())
                {
                    if(e.GetValidity() != arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
                    {
                        continue;
                    }

                    double p_free = 0.4;
                    auto n2 = GetNodeImmutable(e.GetToIndex());
                    if(storm.isValid(n.GetValueImmutable()) && storm.isValid(n2.GetValueImmutable()))
                    {
                        p_free = 0.9;
                    }
                    
                    arc_dijkstras::EDGE_VALIDITY validity = arc_dijkstras::EDGE_VALIDITY::INVALID;
                    if(dist(rng) < p_free)
                    {
                        validity = arc_dijkstras::EDGE_VALIDITY::VALID;
                    }

                    e.SetValidity(validity);
                    //set reverse edge as well, since arc_dijkstras is a directed graph,
                    //but validity applies symmetrically in CTP
                    instance.GetReverseEdgeMutable(e).SetValidity(validity);
                }
            }
            
            return instance;
        }
    };

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
        double move(int new_node)
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

        void sampleInstance()
        {
            true_graph = belief_graph.sampleInstance();
        }

        std::vector<int> getActions()
        {
            std::vector<int> actions;
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
