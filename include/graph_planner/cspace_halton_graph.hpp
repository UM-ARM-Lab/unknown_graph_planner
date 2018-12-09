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
    
    CSpaceHaltonGraph(int num_vert, double max_edge_dist)
    {
        r_disc = max_edge_dist;
        auto qs = halton::haltonPoints(num_vert, 2);
        for(auto q: qs)
        {
            addVertexAndEdges(q);
        }
    }

};

#endif
