#ifndef INCREASING_DENSITY_SERACH_HPP
#define INCREASING_DENSITY_SERACH_HPP
#include "halton_graph.hpp"


class IncrementalDensityNode
{
public:
    int depth;
    std::vector<double> q;
    IncrementalDensityNode(int depth, std::vector<double> q):
        depth(depth), q(q) {}

};



class IncreasingDensityGrid: public arc_dijkstras::Graph<IncrementalDensityNode>
{
public:
    const double eps = 0.0000001;
    IncreasingDensityGrid(int max_depth)
    {
        for(int i=0; i<= max_depth; i++)
        {
            addDenseGrid(i);
        }
    }

    
    void addDenseGrid(int depth)
    {
        for(double x = 0.0; x <= 1.0; x += 1.0/std::pow(2,depth))
        {
            for(double y = 0.0; y <= 1.0; y += 1.0/std::pow(2,depth))
            {
                std::vector<double> q{x, y};
                if(!isInGraph(q))
                {
                    addVertexAndEdges(depth, q);
                }
            }
        }
    }


    int64_t getNode(const std::vector<double> &q) const
    {
        for(int64_t node_ind = 0; node_ind < nodes_.size(); node_ind++)
        {
            double d = EigenHelpers::Distance(nodes_[node_ind].getValue().q, q);
            if(d < eps)
            {
                return node_ind;
            }
        }
        return -1;
    }

    bool isInGraph(const std::vector<double> &q) const
    {
        return getNode(q) >= 0;
    }
    
    int64_t addVertexAndEdges(int depth, std::vector<double> q)
    {
        int64_t new_node_ind = addNode(IncrementalDensityNode(depth, q));
        for(int64_t node_ind = 0; node_ind < nodes_.size()-1; node_ind++)
        {
            double d = EigenHelpers::Distance(nodes_[node_ind].getValue().q, q);
            double r_disc = 1.0/std::pow(2, depth);
            if(d < r_disc + eps)
            {
                addEdgesBetweenNodes(node_ind, new_node_ind, d*pow(2, depth));
            }
        }
        return new_node_ind;
    }



    /* 
     *  Copys the graph to a GraphD graph (removing information about depth)
     */
    GraphD toGraphD()
    {
        GraphD g;
        for(int64_t node_id=0; node_id < nodes_.size(); node_id++)
        {
            g.addNode(nodes_[node_id].getValue().q);
        }
        
        for(int64_t node_id=0; node_id < nodes_.size(); node_id++)
        {
            for(const auto e:nodes_[node_id].getOutEdges())
            {
                auto &e_other = g.addEdgeBetweenNodes(e.getFromIndex(), e.getToIndex(), e.getWeight());
                e_other.setValidity(e.getValidity());
            }
        }
        return g;
    }
};






#endif
