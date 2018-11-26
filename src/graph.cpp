#include "graph.hpp"
#include "halton.hpp"


#define EDGE_DISTANCE 0.1



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


Edge& Graph::getEdge(int v1_ind, int v2_ind)
{
    Node v1 = V[v1_ind];
    for(int edge_ind: v1.edge_inds)
    {
        Edge& e = E[edge_ind];
        if(e.v1_ind == v2_ind || e.v2_ind == v2_ind)
        {
            return e;
        }
    }
    assert(false);
}


