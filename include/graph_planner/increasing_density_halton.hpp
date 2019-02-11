#ifndef INCREASING_DENSITY_HALTON_HPP
#define INCREASING_DENSITY_HALTON_HPP

#include "increasing_density_grid.hpp"


class IDHaltonGraph: public IncreasingDensityGraph
{
protected:
    virtual void generateGraph(int max_depth) override;
public:
    IDHaltonGraph(int max_depth)
    {
        generateGraph(max_depth);
    }

    virtual double edgeCost(const DepthNode &n1, const DepthNode &n2) const override
    {
        return EigenHelpers::Distance(n1.q, n2.q);
    }
    
    virtual double distanceHeuristic(const std::vector<double> &raw1,
                                     const std::vector<double> &raw2) const override;

    int64_t addVertexAndEdges(int depth, std::vector<double> q);
    
};


class IterativeDeepeningHaltonGraph : public IncreasingDensityGraph
{
protected:
    virtual void generateGraph(int max_depth) override;
public:
    IterativeDeepeningHaltonGraph(int max_depth)
    {
        generateGraph(max_depth);
    }

    virtual double edgeCost(const DepthNode &n1, const DepthNode &n2) const override
    {
        return EigenHelpers::Distance(n1.q, n2.q);
    }
    
    virtual double distanceHeuristic(const std::vector<double> &raw1,
                                     const std::vector<double> &raw2) const override;

    int64_t addVertexAndEdges(int depth, std::vector<double> q);
    
    virtual double verticalEdgeCost(const DepthNode &n1, const DepthNode &n2) const override;

};


#endif
