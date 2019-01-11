#ifndef CTP_WORLDS_HPP
#define CTP_WORLDS_HPP

#include "halton_graph.hpp"


namespace CTP
{
    class BctpGraph : public virtual HaltonGraph
    {
    public:
        BctpGraph(): HaltonGraph(0,0) {}; //This constructor call is ignored
        virtual GraphD sampleInstance(std::mt19937 &rng) =0;
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


        GraphD sampleInstance(std::mt19937 &rng) override
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
        // std::vector<HaltonGraph> templates;
        Obstacles2D::Rect storm;

        Obstacles2D::Rect getObstacle()
        {
            return storm;
        }
        
        BctpGrid(int rows) : HaltonGraph(0,0), storm(0,0,0,0)
        {
            int cols = rows;
            r_disc = 1.0/((double)rows - 1.00001) * 1.4143;
            initGrid(rows, cols);
            // generateTemplates(100, std::mt19937());
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

        // void generateTemplates(int num_templates, std::mt19937 &rng)
        // {
        //     for(int i=0; i<num_templates; i++)
        //     {
        //         HaltonGraph g(*this);
        //         Obstacles2D::Rect rect = sampleRect(rng);
        //         templates.push_back(g);
        //     }
        // }

        Obstacles2D::Rect sampleRect(std::mt19937 &rng)
        {
            std::uniform_real_distribution<> dist(0.0, 1.0);
            double w = dist(rng)*0.4 + 0.1;
            double h = dist(rng)*0.4 + 0.1;
            
            double x1 = dist(rng)*(1-w);
            double y1 = dist(rng)*(1-h);
            
            // return Obstacles2D::Rect(x1, y1, x1 + w, y1 + h);
            double r = dist(rng);
            if(r < 1.0/3.0)
            {
                return Obstacles2D::Rect(0.5, 0.2, 0.6, 1.01);
            }
            if(r < 2.0/3.0)
            {
                return Obstacles2D::Rect(0.6, 0.2, 0.7, 1.01);
            }
            return Obstacles2D::Rect(0.7, 0.2, 0.8, 1.01);
        }

        GraphD sampleInstance(std::mt19937 &rng) override
        {
            GraphD instance(*this);
            storm = sampleRect(rng);

            std::uniform_real_distribution<> dist(0.0, 1.0);
            
            for(auto &n:instance.GetNodesMutable())
            {
                for(auto &e:n.GetOutEdgesMutable())
                {
                    if(e.GetValidity() != arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
                    {
                        continue;
                    }

                    double p_free = 0.0;
                    auto n2 = GetNodeImmutable(e.GetToIndex());
                    if(storm.isValid(n.GetValueImmutable()) && storm.isValid(n2.GetValueImmutable()))
                    {
                        p_free = 1.0;
                    }
                    
                    arc_dijkstras::EDGE_VALIDITY validity = arc_dijkstras::EDGE_VALIDITY::INVALID;
                    if(dist(rng) < p_free)
                    {
                        validity = arc_dijkstras::EDGE_VALIDITY::VALID;
                    }


                    //
                    //  TEMPORARILY MAKE ALL EDGES VALID
                    //
                    // validity = arc_dijkstras::EDGE_VALIDITY::VALID;

                    
                    e.SetValidity(validity);
                    //set reverse edge as well, since arc_dijkstras is a directed graph,
                    //but validity applies symmetrically in CTP
                    instance.GetReverseEdgeMutable(e).SetValidity(validity);
                }
            }
            
            return instance;
        }
    };

}


#endif
