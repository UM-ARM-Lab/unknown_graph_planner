#ifndef INCREASING_DENSITY_PLANNING_HPP
#define INCREASING_DENSITY_PLANNING_HPP

#include "increasing_density_grid.hpp"

#include "2d_obstacles.hpp"
#include <arc_utilities/pretty_print.hpp>


inline void evaluateEdge(IncreasingDensityGrid &g,
                          arc_dijkstras::GraphEdge &e, const Obstacles2D::Obstacles &obs)
{
    using namespace arc_dijkstras;
    if(e.getValidity() != EDGE_VALIDITY::UNKNOWN)
    {
        return;
    }
    
    std::vector<double> q1 = g.getNode(e.getFromIndex()).getValue().q;
    std::vector<double> q2 = g.getNode(e.getToIndex()).getValue().q;

    e.setValidity(obs.isValid(q1, q2) ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID);
}

inline void evaluateAllEdges(IncreasingDensityGrid &g, 
                             const Obstacles2D::Obstacles &obs)
{
    for(auto &n:g.getNodes())
    {
        for(auto &e:n.getOutEdges())
        {
            evaluateEdge(g, e, obs);
        }
    }
}

inline arc_helpers::AstarResult Plan(IncreasingDensityGrid &g,
                                     const Obstacles2D::Obstacles &obs,
                                     const std::vector<double> &start,
                                     const std::vector<double> &goal)
{
    using namespace arc_dijkstras;
    const auto eval_fun = [&obs](Graph<IncrementalDensityNode> &g, GraphEdge &e)
    {
        if(e.getValidity() == EDGE_VALIDITY::UNKNOWN)
        {
            std::vector<double> q1 = g.getNode(e.getFromIndex()).getValue().q;
            std::vector<double> q2 = g.getNode(e.getToIndex()).getValue().q;
            bool valid = obs.isValid(q1, q2);
            e.setValidity(valid ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID);
            // std::cout << "Edge between " << e.getFromIndex() << ", " << e.getToIndex();
            // std::cout << " evaluated to " << (valid ? "Valid" : "Invalid") << "\n";
            if(!valid)
            {
                return std::numeric_limits<double>::max();
            }
        }

        //We should never be evaluating edges we already know are invalid
        assert(e.getValidity() == EDGE_VALIDITY::VALID); 
        
        return e.getWeight();
    };

    const auto dist_heur = [](const IncrementalDensityNode &n1,
                              const IncrementalDensityNode &n2)
        {
            return distanceHeuristic(n1.q, n2.q);
        };

    if(!g.isInGraph(0, start))
    {
        std::cout << "Start node <" << PrettyPrint::PrettyPrint(start) << "> is not in graph\n";
    }
    if(!g.isInGraph(0, goal))
    {
        std::cout << "Goal node <" << PrettyPrint::PrettyPrint(goal) << "> is not in graph\n";
    }

    int64_t from_node = g.getNodeAt(0, start);
    int64_t goal_node = g.getNodeAt(0, goal);

    std::cout << "Planning from start: <" << PrettyPrint::PrettyPrint(start) <<
        "> (node " << from_node << ") to <" <<
        PrettyPrint::PrettyPrint(goal) << "> (node " << goal_node << ")\n";
    
    return LazySP<IncrementalDensityNode>::PerformLazySP(
        g, from_node, goal_node, dist_heur, eval_fun, true);
}


inline arc_helpers::AstarResult AstarPlan(IncreasingDensityGrid &g,
                                          const Obstacles2D::Obstacles &obs,
                                          const std::vector<double> &start,
                                          const std::vector<double> &goal)
{
    using namespace arc_dijkstras;
    const auto dist_heur = [](const IncrementalDensityNode &n1,
                              const IncrementalDensityNode &n2)
        {
            return distanceHeuristic(n1.q, n2.q);;
        };

    int64_t from_node = g.getNodeAt(0, start);
    int64_t goal_node = g.getNodeAt(0, goal);

    return SimpleGraphAstar<IncrementalDensityNode>::PerformAstar(g, from_node, goal_node, dist_heur, true);
}


#endif
