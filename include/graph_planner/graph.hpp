#ifndef GRAPH_HPP
#define GRAPH_HPP


#include <vector>

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
};


#endif
