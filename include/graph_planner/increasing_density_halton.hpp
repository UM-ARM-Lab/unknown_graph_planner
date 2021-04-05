#ifndef INCREASING_DENSITY_HALTON_HPP
#define INCREASING_DENSITY_HALTON_HPP

#include "increasing_density_grid.hpp"

class SDHaltonGraph : public SelectiveDensificationGraph {
 protected:
  virtual void generateGraph(int max_depth) override;
  int deepest_layer;
  bool ignore_last_vertical;

 public:
  SDHaltonGraph(int max_depth, bool ignore_last_vertical = false)
      : deepest_layer(max_depth), ignore_last_vertical(ignore_last_vertical) {
    generateGraph(max_depth);
  }

  virtual double edgeCost(const DepthNode &n1, const DepthNode &n2) const override {
    return EigenHelpers::Distance(n1.q, n2.q);
  }

  virtual double distanceHeuristic(const std::vector<double> &raw1, const std::vector<double> &raw2) const override;

  int64_t addVertexAndEdges(int depth, std::vector<double> q);
};

class IterativeDeepeningHaltonGraph : public SelectiveDensificationGraph {
 protected:
  virtual void generateGraph(int max_depth) override;

 public:
  IterativeDeepeningHaltonGraph(int max_depth) { generateGraph(max_depth); }

  virtual double edgeCost(const DepthNode &n1, const DepthNode &n2) const override {
    return EigenHelpers::Distance(n1.q, n2.q);
  }

  virtual double distanceHeuristic(const std::vector<double> &raw1, const std::vector<double> &raw2) const override;

  int64_t addVertexAndEdges(int depth, std::vector<double> q);

  virtual double verticalEdgeCost(const DepthNode &n1, const DepthNode &n2) const override;
};

#endif
