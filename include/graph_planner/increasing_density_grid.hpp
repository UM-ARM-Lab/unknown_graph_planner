#ifndef INCREASING_DENSITY_GRID_HPP
#define INCREASING_DENSITY_GRID_HPP
#include "halton_graph.hpp"
#include <arc_utilities/timing.hpp>


class DepthNode
{
public:
    int depth;
    std::vector<double> q;
    DepthNode(int depth, std::vector<double> q):
        depth(depth), q(q) {}

    DepthNode(const std::vector<double> &raw):
        depth(raw[0]), 
        q(raw.begin()+1, raw.end())
    {}

    std::vector<double> toRaw() const
    {
        std::vector<double> raw{(double)depth};
        raw.insert(raw.end(), q.begin(), q.end());
        return raw;
    }
};



class IncreasingDensityGrid: public RDiscGraph
{
protected:
    void generateGraph(int max_depth);
    
public:
    const double eps = 0.0000001;
    IncreasingDensityGrid();

    virtual double edgeCost(const DepthNode &n1, const DepthNode &n2) const = 0;

    DepthNode getNodeValue(int64_t ind) const;
    
    void addDenseGrid(int depth);

    int64_t getNodeAt(int depth, const std::vector<double> &q) const;

    bool isInGraph(int depth, const std::vector<double> &q) const;
    
    int64_t addVertexAndEdges(int depth, std::vector<double> q);
};


class DoublingIDG: public IncreasingDensityGrid
{
public:
    DoublingIDG(int max_depth){
        generateGraph(max_depth);
    };

    virtual double edgeCost(const DepthNode &n1, const DepthNode &n2) const override;
};





#endif
