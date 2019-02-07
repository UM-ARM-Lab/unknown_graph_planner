#include "increasing_density_halton.hpp"


int numNodesAtDepth(int depth, int dim)
{
    int vert_per_edge = std::pow(2, depth) + 1;
    int num_vert = std::pow(vert_per_edge, dim);
    return num_vert;
}


void IDHaltonGraph::generateGraph(int max_depth)
{
    int dim=2;
    
    auto qs = halton::haltonPoints(numNodesAtDepth(max_depth, dim)-2, dim);
    qs.insert(qs.begin(), std::vector<double>{1, 1});
    qs.insert(qs.begin(), std::vector<double>{0, 0});


    for(int depth=0; depth <= max_depth; depth++)
    {
        for(int i=0; i<numNodesAtDepth(depth, dim); i++)
        {

            addVertexAndEdges(depth, qs[i]);
        }
    }
    
}


double IDHaltonGraph::distanceHeuristic(const std::vector<double> &raw1,
                                   const std::vector<double> &raw2) const
{

    DepthNode d1(raw1);
    DepthNode d2(raw2);
    // std::cout << "Calling dist heuristic with depth " << d1.depth << "\n";
     // std::pow(2, d1.depth);
    return EigenHelpers::Distance(d1.q, d2.q)*std::pow(1.5, d1.depth);
}



int64_t IDHaltonGraph::addVertexAndEdges(int depth, std::vector<double> q)
{
    DepthNode new_node = DepthNode(depth, q);
    int64_t new_node_ind = addNode(new_node.toRaw());
    int64_t above_ind = getNodeAt(depth - 1, q);
    if(above_ind >= 0)
    {
        addEdgesBetweenNodes(new_node_ind, above_ind,
                             verticalEdgeCost(new_node, DepthNode(getNode(above_ind).getValue())));
    }

    double edge_radius = 1.4 / std::pow(2, depth);
    auto inds_within_radius = getVerticesWithinRadius(new_node.toRaw(), edge_radius);

    for(const auto &near_ind:inds_within_radius)
    {
        if(new_node_ind == near_ind)
        {
            continue;
        }
        if(new_node.depth != DepthNode(getNode(near_ind).getValue()).depth)
        {
            continue;
        }
        addEdgesBetweenNodes(new_node_ind, (int64_t)near_ind, edgeCost(new_node, getNodeValue(near_ind)));
    }
    return new_node_ind;
}


