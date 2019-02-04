#include "increasing_density_grid.hpp"


IncreasingDensityGrid::IncreasingDensityGrid() : RDiscGraph(1.0)
{
}

void IncreasingDensityGrid::generateGraph(int max_depth)
{
    for(int i=0; i<= max_depth; i++)
    {
        addDenseGrid(i);
    }
}


DepthNode IncreasingDensityGrid::getNodeValue(int64_t ind) const
{
    return DepthNode(getNode(ind).getValue());
}

void IncreasingDensityGrid::addDenseGrid(int depth)
{
    // std::cout << "Adding grid at depth " << depth << "\n";
    for(double x = 0.0; x <= 1.0; x += 1.0/std::pow(2,depth))
    {
        for(double y = 0.0; y <= 1.0; y += 1.0/std::pow(2,depth))
        {
            addVertexAndEdges(depth, std::vector<double>{x, y});
        }
    }
}

int64_t IncreasingDensityGrid::getNodeAt(int depth, const std::vector<double> &q) const
{
    int64_t nearest = getNearest(DepthNode(depth, q).toRaw());

    DepthNode n = getNodeValue(nearest);
    if(n.depth != depth || EigenHelpers::Distance(n.q, q) > eps)
    {
        return -1;
    }
    return nearest;
}

bool IncreasingDensityGrid::isInGraph(int depth, const std::vector<double> &q) const
{
    return getNodeAt(depth, q) >= 0;
}

int64_t IncreasingDensityGrid::addVertexAndEdges(int depth, std::vector<double> q)
{
    DepthNode new_node = DepthNode(depth, q);
    int64_t new_node_ind = addNode(new_node.toRaw());
    int64_t above_ind = getNodeAt(depth - 1, q);
    if(above_ind >= 0)
    {
        addEdgesBetweenNodes(new_node_ind, above_ind, 0);
    }

    auto inds_within_radius = getVerticesWithinRadius(new_node.toRaw(), 1.0/std::pow(2, depth) + eps);

    for(const auto &ind:inds_within_radius)
    {
        if(new_node_ind == ind)
        {
            continue;
        }
        addEdgesBetweenNodes(new_node_ind, (int64_t)ind, edgeCost(new_node, getNodeValue(ind)));
    }
    return new_node_ind;
}




/***************************
 *    DoublingIDG
 **************************/
double DoublingIDG::edgeCost(const DepthNode &n1, const DepthNode &n2) const
{
    double d = EigenHelpers::Distance(n1.q, n2.q);
    return d*pow(2, n1.depth);
}

