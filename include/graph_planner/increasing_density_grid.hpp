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



class SelectiveDensificationGraph: public RDiscGraph
{
protected:
    virtual void generateGraph(int max_depth) = 0;
    
public:
    const double eps = 0.0000001;
    SelectiveDensificationGraph();

    virtual double edgeCost(const DepthNode &n1, const DepthNode &n2) const = 0;

    virtual double verticalEdgeCost(const DepthNode &n1, const DepthNode &n2) const;
    
    virtual double distanceHeuristic(const std::vector<double> &raw1,
                                     const std::vector<double> &raw2) const = 0;


    DepthNode getNodeValue(int64_t ind) const;
    


    int64_t getNodeAt(int depth, const std::vector<double> &q) const;

    bool isInGraph(int depth, const std::vector<double> &q) const;
    
    int64_t addVertexAndEdges(int depth, std::vector<double> q);
};



class SelectiveDensificationGrid: public SelectiveDensificationGraph
{
protected:
    void addDenseGrid(int depth);
    virtual void generateGraph(int max_depth) override;
};


class DoublingSDG: public SelectiveDensificationGrid
{
public:
    DoublingSDG(int max_depth){
        generateGraph(max_depth);
    };

    virtual double edgeCost(const DepthNode &n1, const DepthNode &n2) const override;

    virtual double distanceHeuristic(const std::vector<double> &raw1,
                                     const std::vector<double> &raw2) const override;

};


class ConicSDG: public SelectiveDensificationGrid
{
public:
    ConicSDG(int max_depth){
        generateGraph(max_depth);
    };

    virtual double edgeCost(const DepthNode &n1, const DepthNode &n2) const override;

    virtual double verticalEdgeCost(const DepthNode &n1, const DepthNode &n2) const override;

    virtual double distanceHeuristic(const std::vector<double> &raw1,
                                     const std::vector<double> &raw2) const override;


};





#endif
