#ifndef GRAPH_HPP
#define GRAPH_HPP


#include <vector>
#include <cmath>


enum EDGE_VALIDITY
{
    VALID, INVALID, UNKNOWN
};

class Edge;

class Node
{
public:
    std::vector<double> q;
    std::vector<int> edge_inds;
    Node(std::vector<double> q_):
        q(q_){};
};


class Edge
{
public:
    double weight;
    int v1_ind;
    int v2_ind;
    EDGE_VALIDITY validity = EDGE_VALIDITY::UNKNOWN;
};

class Graph
{
public:
    std::vector<Node> V;
    std::vector<Edge> E;
    Graph(int num_vert);
    std::vector<Node> toNodes(std::vector<std::vector<double>> points);
    void addEdges(double max_dist);

    Edge& getEdge(int v1_ind, int v2_ind);
};

double distance(const Node &n1, const Node &n2)
{
    double sum = 0;
    for(int i=0; i<n1.q.size(); i++)
    {
        sum += (n1.q[i] - n2.q[i]) * (n1.q[i] - n2.q[i]);
    }
    return std::sqrt(sum);
}




#endif
