#include "increasing_density_grid.hpp"


SelectiveDensificationGraph::SelectiveDensificationGraph() : RDiscGraph(1.0)
{
}



DepthNode SelectiveDensificationGraph::getNodeValue(int64_t ind) const
{
    return DepthNode(getNode(ind).getValue());
}


int64_t SelectiveDensificationGraph::getNodeAt(int depth, const std::vector<double> &q) const
{
    int64_t nearest = getNearest(DepthNode(depth, q).toRaw());

    DepthNode n = getNodeValue(nearest);
    if(n.depth != depth || EigenHelpers::Distance(n.q, q) > eps)
    {
        return -1;
    }
    return nearest;
}

double SelectiveDensificationGraph::verticalEdgeCost(const DepthNode &n1, const DepthNode &n2) const
{
    return 0;
}

bool SelectiveDensificationGraph::isInGraph(int depth, const std::vector<double> &q) const
{
    return getNodeAt(depth, q) >= 0;
}

int64_t SelectiveDensificationGraph::addVertexAndEdges(int depth, std::vector<double> q)
{
    DepthNode new_node = DepthNode(depth, q);
    int64_t new_node_ind = addNode(new_node.toRaw());
    int64_t above_ind = getNodeAt(depth - 1, q);
    if(above_ind >= 0)
    {
        addEdgesBetweenNodes(new_node_ind, above_ind,
                             verticalEdgeCost(new_node, DepthNode(getNode(above_ind).getValue())));
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




/**************************
 * Increasing Density Grid
 *************************/
void SelectiveDensificationGrid::generateGraph(int max_depth)
{
    for(int i=0; i<= max_depth; i++)
    {
        addDenseGrid(i);
    }
}


void SelectiveDensificationGrid::addDenseGrid(int depth)
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




/***************************
 *    DoublingSDG
 **************************/
double DoublingSDG::edgeCost(const DepthNode &n1, const DepthNode &n2) const
{
    double d = EigenHelpers::Distance(n1.q, n2.q);
    return d*pow(2, n1.depth);
}

double DoublingSDG::distanceHeuristic(const std::vector<double> &raw1,
                                      const std::vector<double> &raw2) const
{

    DepthNode d1(raw1);
    DepthNode d2(raw2);
    // std::cout << "Calling dist heuristic with depth " << d1.depth << "\n";
    return EigenHelpers::Distance(d1.q, d2.q) * std::pow(2, d1.depth);
}



/***************************
 *      Conic
 **************************/
double ConicSDG::edgeCost(const DepthNode &n1, const DepthNode &n2) const
{
    double d = EigenHelpers::Distance(n1.q, n2.q);
    return d;
}

double ConicSDG::distanceHeuristic(const std::vector<double> &raw1,
                                   const std::vector<double> &raw2) const
{

    DepthNode d1(raw1);
    DepthNode d2(raw2);
    // std::cout << "Calling dist heuristic with depth " << d1.depth << "\n";
     // std::pow(2, d1.depth);
    return EigenHelpers::Distance(d1.q, d2.q)*std::pow(1.5, d1.depth);
    // return std::abs(d1.q[0] - d2.q[0]) + std::abs(d1.q[1] - d2.q[1]) + eps*(double)d1.depth;
    // return EigenHelpers::Distance(d1.q, d2.q);
    // return EigenHelpers::Distance(d1.q, d2.q) * (d1.depth+1) * 0.5;
    // return EigenHelpers::Distance(d1.q, d2.q) * d1.depth;
    // return EigenHelpers::Distance(d1.q, d2.q) +
    //     std::abs(1.0 / std::pow(2, d1.depth) - 1.0 / std::pow(2, d1.depth));;
}

                                                         

double ConicSDG::verticalEdgeCost(const DepthNode &n1, const DepthNode &n2) const
{
    return 0;
    // return std::abs(1.0 / std::pow(2, n1.depth) - 1.0 / std::pow(2, n1.depth));
}
