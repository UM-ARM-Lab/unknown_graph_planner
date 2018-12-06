#ifndef CSPACE_HALTON_GRAPH_HPP
#define CSPACE_HALTON_GRAPH_HPP

#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include "halton.hpp"
#include <vector>
#include <cmath>



double distanceHeuristic(std::vector<double> q1, std::vector<double> q2)
{
    return EigenHelpers::Distance(q1, q2);
}


#define EDGE_DISTANCE 0.1

class CSpaceHaltonGraph : public arc_dijkstras::Graph<std::vector<double>>
{
public:
    double r_disc;

    int64_t addVertexAndEdges(std::vector<double> q)
    {
        int64_t new_node_ind = AddNode(q);
        for(int64_t node_ind = 0; node_ind < nodes_.size()-1; node_ind++)
        {
            double d = EigenHelpers::Distance(nodes_[node_ind].GetValueImmutable(), q);
            if(d < r_disc)
            {
                AddEdgesBetweenNodes(node_ind, new_node_ind, d);
            }
        }
        return new_node_ind;
    }
    
    CSpaceHaltonGraph(int num_vert)
    {
        r_disc = EDGE_DISTANCE;
        std::vector<int> bases{2,3};
        std::vector<int> offsets{100, 120};
        auto qs = haltonPoints(bases, num_vert, offsets);
        for(auto q: qs)
        {
            addVertexAndEdges(q);
        }
    }

};

#endif
