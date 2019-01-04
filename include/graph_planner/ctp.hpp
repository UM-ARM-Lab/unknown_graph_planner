#ifndef CTP_HPP
#define CTP_HPP

#include "halton_graph.hpp"
#include "dijkstras_addons.hpp"


namespace CTP{

    /*
     * Graph for a canadian traveler's problem
     */
    class CtpGraph : public HaltonGraph
    {
    public:
        std::map<arc_dijkstras::HashableEdge, double> edge_probabilities;
        
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

    };
}

#endif
