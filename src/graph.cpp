#include "graph.hpp"
#include "halton.hpp"
#include <cmath>

#define EDGE_DISTANCE 0.1


double distance(const Node &n1, const Node &n2)
{
    double sum = 0;
    for(int i=0; i<n1.q.size(); i++)
    {
        sum += (n1.q[i] - n2.q[i]) * (n1.q[i] - n2.q[i]);
    }
    return std::sqrt(sum);
}

Graph::Graph(int num_vert)
{
    std::vector<int> bases{2,3};
    std::vector<int> offsets{100, 120};
    V = toNodes(haltonPoints(bases, num_vert, offsets));
    addEdges(EDGE_DISTANCE);
}

void Graph::addEdges(double max_dist)
{
    for(int i=0; i<V.size(); i++)
    {
        for(int j=i+1; j<V.size(); j++)
        {
            double d = distance(V[i], V[j]);

            if(d > max_dist)
            {
                continue;
            }

            Edge e;
            e.weight = d;
            e.v1_ind = i;
            e.v2_ind = j;
            E.push_back(e);

            V[i].edge_inds.push_back(E.size()-1);
            V[j].edge_inds.push_back(E.size()-1);
        }
    }
}

std::vector<Node> Graph::toNodes(std::vector<std::vector<double>> points)
{
    std::vector<Node> nodes;
    nodes.reserve(points.size());
    for(auto p:points)
    {
        nodes.push_back(Node(p));
    }
    return nodes;
}





