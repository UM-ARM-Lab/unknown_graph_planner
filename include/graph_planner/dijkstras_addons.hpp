#ifndef DIJKSTRAS_ADDONS_HPP
#define DIJKSTRAS_ADDONS_HPP

#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/timing.hpp>
#include "a_star.hpp"

typedef arc_dijkstras::Graph<std::vector<double>> GraphD;

namespace arc_dijkstras
{
    typedef std::pair<int64_t, int64_t> HashableEdge;
    typedef std::map<HashableEdge, double> EvaluatedEdges;

    
    enum FORWARD_LAZY_CHECK_RESULT {EDGE_INVALID, EDGE_VALID, PATH_VALID};

    static inline HashableEdge getHashable(const GraphEdge& edge)
    {
        return std::make_pair(edge.getFromIndex(), edge.getToIndex());
    }

    static inline HashableEdge getSortedHashable(const GraphEdge& edge)
    {
        if(edge.getFromIndex() < edge.getToIndex())
        {
            return std::make_pair(edge.getFromIndex(), edge.getToIndex());
        }
        return std::make_pair(edge.getToIndex(), edge.getFromIndex());
    }


    /*
     *  Returns true iff all edges have same validity in g1 and g2
     *  Assumes topology is the same
     */
    static bool haveSameEdgeValidity(const GraphD& g1, const GraphD& g2)
    {
        for(size_t n_id = 0; n_id <g1.getNodes().size(); n_id++)
        {
            const auto &n1 = g1.getNode(n_id);
            const auto &n2 = g2.getNode(n_id);
            for(size_t e_id = 0; e_id < n1.getOutEdges().size(); e_id++)
            {
                if(n1.getOutEdges()[e_id].getValidity() !=
                   n2.getOutEdges()[e_id].getValidity())
                {
                    return false;
                }
            }
        }
        return true;
    }


    
    template<typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
    class LazySP
    {
    public:
        /**
         *  This performs a shortest path search using A* specific for the LazySP algortithm.
         *   The weights of the graph are assumed to be initialized to heuristic weights
         *   evalutedEdges is the true edge cost for any evaluated edges
         */
        static arc_helpers::AstarResult
        PerformAstarForLazySP(const Graph<NodeValueType, Allocator>& graph,
                              int64_t start_index,
                              int64_t goal_index,
                              const std::function<double(const NodeValueType&,
                                                         const NodeValueType&)>& heuristic_fn,
                              const bool limit_pqueue_duplicates,
                              const EvaluatedEdges &evaluatedEdges)
        {
            const auto edge_validity_check_function =
                [&] (const Graph<NodeValueType, Allocator>& search_graph,
                     const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    if(edge.getValidity() == EDGE_VALIDITY::INVALID)
                    {
                        return false;
                    }

                    return edge.getWeight() < std::numeric_limits<double>::infinity();
                };
    
            const auto distance_function =
                [&] (const Graph<NodeValueType, Allocator>& search_graph,
                     const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    if(evaluatedEdges.count(getHashable(edge)))
                    {
                        // std::cout << "Found already evaluted edge!\n";
                        return evaluatedEdges.at(getHashable(edge));
                    }
                    // std::cout << "Using heuristic weight\n";
                    return edge.getWeight();
                };

            return AstarLogging<NodeValueType>::PerformLazyAstar(
            // return SimpleGraphAstar<NodeValueType>::PerformLazyAstar(
                graph, start_index, goal_index, edge_validity_check_function, distance_function,
                heuristic_fn, limit_pqueue_duplicates);
        }



        static std::vector<int> ForwardSelector(std::vector<int64_t> path,
                                                Graph<NodeValueType, Allocator>& g,
                                                const EvaluatedEdges &evaluatedEdges)
        {
            int i=0;
            while(i < path.size() - 1 )
            {
                GraphEdge &e = g.getNode(path[i]).getEdgeTo(path[i+1]);
                if(evaluatedEdges.count(getHashable(e)) == 0)
                {
                    return std::vector<int>{i};
                }
                i++;
            }
            return std::vector<int>();
        }


        /*
         *  Checks unknown edges according to the forward selector
         *  Returns true if the fully evaluated path has the same cost as the partially evaluated path
         *  Returns early if an invalid edge is found or if an evaluated edge has higher cost 
         *  that the edge weight (heuristic)
         */
        static bool checkPath(const std::vector<int64_t> &path,
                              Graph<NodeValueType, Allocator>& g,
                              EvaluatedEdges &evaluated_edges,
                              const std::function<double(Graph<NodeValueType, Allocator>&,
                                                         GraphEdge&)>& eval_edge_fn)
        {
            bool path_could_be_optimal = true;

            while(path_could_be_optimal)
            {
                auto path_indicies_to_check = ForwardSelector(path, g, evaluated_edges);
                if(path_indicies_to_check.size() == 0)
                {
                    return true;
                }
            
                for(auto i:path_indicies_to_check)
                {
                    GraphEdge &e = g.getNode(path[i]).getEdgeTo(path[i+1]);
                    double evaluated_cost = eval_edge_fn(g, e);
                    evaluated_edges[getHashable(e)] = evaluated_cost;

                    if(e.getValidity() == EDGE_VALIDITY::INVALID || e.getWeight() < evaluated_cost)
                    {
                        path_could_be_optimal = false;
                    }
                }
            }
            return false;
        }
                              
                              


        static arc_helpers::AstarResult
        PerformLazySP(Graph<NodeValueType, Allocator>& g,
                      int64_t start_index,
                      int64_t goal_index,
                      const std::function<double(const NodeValueType&,
                                                 const NodeValueType&)>& heuristic_fn,
                      const std::function<double(Graph<NodeValueType, Allocator>&,
                                                 GraphEdge&)>& eval_edge_fn,
                      const bool limit_pqueue_duplicates)
        {
            EvaluatedEdges evaluated_edges;

            int num_astar_iters = 0;

            while(true)
            {
                PROFILE_START("lazy_sp a_star");
                auto prelim_result = PerformAstarForLazySP(g, start_index, goal_index,
                                                           heuristic_fn, limit_pqueue_duplicates,
                                                           evaluated_edges);
                PROFILE_RECORD("lazy_sp a_star");
                num_astar_iters++;
                
                auto path = prelim_result.first;

                if(checkPath(path, g, evaluated_edges, eval_edge_fn))
                {
                    PROFILE_RECORD_DOUBLE("lazysp astar iters", num_astar_iters);
                    return prelim_result;
                }
            }
        }
    };
}
#endif
