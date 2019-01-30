#ifndef CTP_WORLDS_HPP
#define CTP_WORLDS_HPP

#include "halton_graph.hpp"


namespace CTP
{
    class BctpGraph : public HaltonGraph
    {
    public:
        BctpGraph(): HaltonGraph(0,0) {};
        BctpGraph(int num_vert, double max_edge_dist, int dim=2):
            HaltonGraph(num_vert, max_edge_dist, dim) {};
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
        CtpGraph(){};
        
        CtpGraph(int num_vert, double max_edge_dist, int dim=2, double edge_probabilities=0.9)
        {
            setEdgeProbabilities(edge_probabilities);
        }

        void setEdgeProbabilities(double universal_probability)
        {
            for(const auto n:getNodes())
            {
                for(const auto e:n.getOutEdges())
                {
                    edge_probabilities[arc_dijkstras::getHashable(e)] = universal_probability;
                }
            }
        }


        GraphD sampleInstance(std::mt19937 &rng) override
        {
            GraphD instance(*this);
            std::uniform_real_distribution<> dist(0.0, 1.0);
            for(auto &n:instance.getNodes())
            {
                for(auto &e:n.getOutEdges())
                {
                    if(e.getValidity() != arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
                    {
                        continue;
                    }
                    
                    double p = edge_probabilities[arc_dijkstras::getHashable(e)];
                    arc_dijkstras::EDGE_VALIDITY validity = arc_dijkstras::EDGE_VALIDITY::INVALID;
                    if(dist(rng) < p)
                    {
                        validity = arc_dijkstras::EDGE_VALIDITY::VALID;
                    }

                    e.setValidity(validity);
                    //set reverse edge as well, since arc_dijkstras is a directed graph,
                    //but validity applies symmetrically in CTP
                    instance.getReverseEdge(e).setValidity(validity);
                }
            }
            return instance;
        }
    };

    /***************************
     **  Tricky CTP
     **************************/
    class CtpPitfall : public CtpGraph
    {
    public:
        CtpPitfall()
        {
            using namespace arc_dijkstras;
            auto v0 = addNode(std::vector<double>{0.5, 0.0});
            auto vf = addNode(std::vector<double>{0.5, 1.0});
            auto v7 = addNode(std::vector<double>{1.0, 0.5});

            auto v1 = addNode(std::vector<double>{0.5, 0.2});

            auto v2 = addNode(std::vector<double>{0.6, 0.6});
            auto v3 = addNode(std::vector<double>{0.52, 0.6});
            auto v4 = addNode(std::vector<double>{0.4, 0.6});
            auto v5 = addNode(std::vector<double>{0.0, 0.5});
            auto v6 = addNode(std::vector<double>{0.1, 0.9});

            addProbabililisticEdge(v0, v7, 50);       //e07
            addProbabililisticEdge(v7, vf, 50);       //e01
            addProbabililisticEdge(v0, v1, 10);       //e01
            addProbabililisticEdge(v1, v2, 60, 0.99); //12
            addProbabililisticEdge(v1, v3, 60, 0.99); //13
            addProbabililisticEdge(v1, v4, 60, 0.99); //14
            addProbabililisticEdge(v2, vf, 0, 0.5);   //2f
            addProbabililisticEdge(v3, vf, 0, 0.5);   //3f
            addProbabililisticEdge(v4, vf, 0, 0.5);   //4f
            addProbabililisticEdge(v0, v5, 20);       //05
            addProbabililisticEdge(v5, v6, 40, 0.99); //56
            addProbabililisticEdge(v6, vf, 0, 0.01);  //6f
            addProbabililisticEdge(v5, vf, 70, 0.99); //5f
        }

        void addProbabililisticEdge(int64_t n0, int64_t n1, double weight, double p=1.0)
        {
            auto es = addEdgesBetweenNodes(n0, n1, weight);
            updateEdgeProbabilities(es, p);
        }

        void updateEdgeProbabilities(std::pair<const arc_dijkstras::GraphEdge,
                                     const arc_dijkstras::GraphEdge> es,
                                     double p)
        {
            using namespace arc_dijkstras;
            edge_probabilities[getHashable(es.first)] = p;
            edge_probabilities[getHashable(es.second)] = p;
        }


        std::pair<std::vector<std::string>, std::vector<std::vector<double>>>
        getEdgeMsgs()
        {
            using namespace arc_dijkstras;
            std::vector<std::string> texts;
            std::vector<std::vector<double>> locs;
            for(auto& n:getNodes())
            {
                for(auto& e:n.getOutEdges())
                {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2);
                    double p = edge_probabilities[getHashable(e)];
                    if(p < 1.0)
                        ss << "p " << p << " : " ;
                    ss << e.getWeight();
                    texts.push_back(ss.str());
                    std::vector<double> loc(2);
                    auto &p1 = n.getValue();
                    auto &p2 = getNode(e.getToIndex()).getValue();
                    loc[0] = 0.5* p1[0] + 0.5 * p2[0] + 0.01;
                    loc[1] = 0.5* p1[1] + 0.5 * p2[1];
                    locs.push_back(loc);
                }
            }
            return std::make_pair(texts, locs);
        }
    };


    

    /****************************
     **    BCTP Grid Worlds
     ***************************/

    class BctpGrid : public BctpGraph
    {
    public:
        // std::vector<HaltonGraph> templates;
        Obstacles2D::Rect storm;

        Obstacles2D::Rect getObstacle()
        {
            return storm;
        }
        
        BctpGrid(int rows) : storm(0,0,0,0)
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
            
            for(auto &n:instance.getNodes())
            {
                for(auto &e:n.getOutEdges())
                {
                    if(e.getValidity() != arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
                    {
                        continue;
                    }

                    double p_free = 0.0;
                    auto n2 = getNode(e.getToIndex());
                    if(storm.isValid(n.getValue()) && storm.isValid(n2.getValue()))
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

                    
                    e.setValidity(validity);
                    //set reverse edge as well, since arc_dijkstras is a directed graph,
                    //but validity applies symmetrically in CTP
                    instance.getReverseEdge(e).setValidity(validity);
                }
            }
            
            return instance;
        }
    };

}


#endif
