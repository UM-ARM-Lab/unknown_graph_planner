#ifndef INCREASING_DENSITY_PLANNING_HPP
#define INCREASING_DENSITY_PLANNING_HPP

#include "increasing_density_grid.hpp"
#include "a_star.hpp"

#include "2d_obstacles.hpp"
#include <arc_utilities/pretty_print.hpp>



namespace increasing_density_planning
{
    inline bool checkEdge(GraphD &g,
                          arc_dijkstras::GraphEdge &e, const Obstacles2D::Obstacles &obs)
    {
        using namespace arc_dijkstras;
        if(e.getValidity() == EDGE_VALIDITY::VALID)
        {
            return true;;
        }
        if(e.getValidity() == EDGE_VALIDITY::INVALID)
        {
            return false;
        }
    
        std::vector<double> q1 = DepthNode(g.getNode(e.getFromIndex()).getValue()).q;
        std::vector<double> q2 = DepthNode(g.getNode(e.getToIndex()).getValue()).q;

        bool validity = obs.isValid(q1, q2);
        e.setValidity(validity ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID);
        return validity;
    }


    inline double evaluateEdge(GraphD &g,
                               arc_dijkstras::GraphEdge &e, const Obstacles2D::Obstacles &obs)
    {
        if(!checkEdge(g, e, obs))
        {
            return std::numeric_limits<double>::infinity();
        }
        return e.getWeight();
    }

    
    inline void checkAllEdges(IncreasingDensityGraph &g, 
                              const Obstacles2D::Obstacles &obs)
    {
        for(auto &n:g.getNodes())
        {
            for(auto &e:n.getOutEdges())
            {
                checkEdge(g, e, obs);
            }
        }
    }


    inline double depthDoublingDistance(const std::vector<double> &n1,
                                        const std::vector<double> &n2)
    {
        DepthNode d1(n1);
        DepthNode d2(n2);
        return EigenHelpers::Distance(d1.q, d2.q) * std::pow(2, d1.depth);
    }
    
    
    // inline arc_helpers::AstarResult Plan(IncreasingDensityGraph &g,
    //                                      const Obstacles2D::Obstacles &obs,
    //                                      const std::vector<double> &start,
    //                                      const std::vector<double> &goal)
    // {
    //     using namespace arc_dijkstras;
    //     const auto eval_fun = [&obs](GraphD &g, GraphEdge &e)
    //         {
    //             return evaluateEdge(g, e, obs);
    //         };

    //     if(!g.isInGraph(0, start))
    //     {
    //         std::cout << "Start node <" << PrettyPrint::PrettyPrint(start) << "> is not in graph\n";
    //     }
    //     if(!g.isInGraph(0, goal))
    //     {
    //         std::cout << "Goal node <" << PrettyPrint::PrettyPrint(goal) << "> is not in graph\n";
    //     }

    //     int64_t from_node = g.getNodeAt(0, start);
    //     int64_t goal_node = g.getNodeAt(0, goal);

    //     std::cout << "Planning from start: <" << PrettyPrint::PrettyPrint(start) <<
    //         "> (node " << from_node << ") to <" <<
    //         PrettyPrint::PrettyPrint(goal) << "> (node " << goal_node << ")\n";
    
    //     return LazySP<std::vector<double>>::PerformLazySP(
    //         g, from_node, goal_node, &depthDoublingDistance, eval_fun, true);
    // }


    inline arc_helpers::AstarResult AstarPlan(IncreasingDensityGraph &g,
                                              const Obstacles2D::Obstacles &obs,
                                              const std::vector<double> &start,
                                              const std::vector<double> &goal)
    {
        using namespace arc_dijkstras;

        int64_t from_node = g.getNodeAt(0, start);
        int64_t goal_node = g.getNodeAt(0, goal);

        const auto edge_check_fun = [&obs](GraphD &g, GraphEdge &e)
            {
                return checkEdge(g, e, obs);
            };


        const auto distance_function = [&] (const GraphD& search_graph, 
                                            const GraphEdge& edge)
            {
                UNUSED(search_graph);
                return edge.getWeight();
            };

        const auto heuristic_function = [&g] (const std::vector<double> &n1,
                                              const std::vector<double> &n2)
            {
                // DepthNode d1(n1);
                // DepthNode d2(n2);
                // return EigenHelpers::Distance(d1.q, d2.q) * std::pow(2, d1.depth);

                return g.distanceHeuristic(n1, n2);
            };

        return AstarLogging<std::vector<double>>::PerformLazyAstar(g, from_node, goal_node,
                                                                   edge_check_fun,
                                                                   distance_function,
                                                                   heuristic_function,
                                                                   // &depthDoublingDistance, 
                                                                   true);
    }
}

#endif
