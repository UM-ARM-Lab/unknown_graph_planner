#ifndef DIJKSTRAS_ADDONS_HPP
#define DIJKSTRAS_ADDONS_HPP

#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/timing.hpp>
#include "a_star.hpp"
#include <utility>
#include "lpa_pqueue.hpp"

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





    /*****************************
     **    LPA*
     ****************************/
    typedef std::pair<double, double> LPAstarKey;

    struct LPAstarPQueueElement
    {
        int64_t node_id;

        LPAstarPQueueElement(int64_t node_id) :
            node_id(node_id){}
    };

    class CompareLPAstarKey
    {
    public:

        bool operator()(const LPAstarKey& lhs, const LPAstarKey& rhs) const
        {
            if(lhs.first == rhs.first)
            {
                return lhs.second <= rhs.second;
            }
            
            return lhs.first < rhs.first;
        }
    };
    
    template<typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
    class LPAstar
    {
    protected:
        const Graph<NodeValueType, Allocator>& graph;
        const int64_t start_index;
        const int64_t goal_index;
        const std::function<bool(const Graph<NodeValueType, Allocator>&,
                                 const GraphEdge&)>& edge_validity_check_fn;
        const std::function<double(const Graph<NodeValueType, Allocator>&,
                                   const GraphEdge&)>& distance_fn;
        const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn;

        // std::priority_queue<LPAstarPQueueElement,
        //                     std::vector<LPAstarPQueueElement>,
        //                     CompareLPAstarPQueueElementFn> queue;
        pqueue<LPAstarKey, LPAstarPQueueElement, CompareLPAstarKey> queue;

        std::unordered_map<int64_t, double> g_map;
        std::unordered_map<int64_t, double> rhs_map;

    protected:
        double g(int64_t node_id)
        {
            if(g_map.count(node_id) == 0)
            {
                return std::numeric_limits<double>::infinity();
            }
            return g_map[node_id];
        }
        
        double rhs(int64_t node_id)
        {
            if(rhs_map.count(node_id) == 0)
            {
                return std::numeric_limits<double>::infinity();
            }
            return rhs_map[node_id];
        }

        LPAstarKey calculateKey(int64_t node_id)
        {
            double k1 = std::min(g(node_id),rhs(node_id)) + heurisitic_fn(graph.getNode(node_id).getValue(),
                                                                          graph.getNode(goal_index).getValue());
            double k2 = std::min(g(node_id), rhs(node_id));
            return std::make_pair(k1, k2);
        }


        double cost(const GraphEdge &e)
        {
            if(!edge_validity_check_fn(graph, e))
            {
                return std::numeric_limits<double>::infinity();
            }
            return distance_fn(graph, e);
        }

        void updateVertex(int64_t u)
        {
            if(u != start_index)
            {
                double min_pred_cost = std::numeric_limits<double>::infinity();
                for(const GraphEdge &e: graph.getNode(u).getInEdges())
                {
                    int64_t s_prime = e.getFromIndex();
                    double val = g(s_prime) + cost(e);
                    if(val < min_pred_cost)
                    {
                        min_pred_cost = val;
                    }
                }
                rhs_map[u] = min_pred_cost;
            }

            queue.remove(u);
            if(g(u) != rhs(u))
            {
                queue.insert(u, calculateKey(u));
            }
        }

        arc_helpers::AstarResult findPath()
        {
            arc_helpers::AstarResult result;
            result.second = g(goal_index);
            if(result.second >= std::numeric_limits<double>::max())
            {
                return result;
            }
            
            std::vector<int64_t> path{goal_index};
            int64_t cur_index = goal_index;
            while(cur_index != start_index)
            {
                const auto& node = graph.getNode(current_index);
                for(const auto& e: node.getInEdges())
                {
                }
            }
            std::reverse(path.begin(), path.end());
            result.first = path;
            return result;
        }
    

    public:
        LPAstar(const Graph<NodeValueType, Allocator>& graph,
                const int64_t start_index,
                const int64_t goal_index,
                const std::function<bool(const Graph<NodeValueType, Allocator>&,
                                         const GraphEdge&)>& edge_validity_check_fn,
                const std::function<double(const Graph<NodeValueType, Allocator>&,
                                           const GraphEdge&)>& distance_fn,
                const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn) :
            graph(graph), start_index(start_index), goal_index(goal_index),
            edge_validity_check_fn(edge_validity_check_fn),
            distance_fn(distance_fn),
            heuristic_fn(heuristic_fn)
        {
            // Enforced sanity checks
            if ((start_index < 0) || (start_index >= (int64_t)graph.getNodes().size()))
            {
                throw std::invalid_argument("Start index out of range");
            }
            if ((goal_index < 0) || (goal_index >= (int64_t)graph.getNodes().size()))
            {
                throw std::invalid_argument("Goal index out of range");
            }
            if (start_index == goal_index)
            {
                throw std::invalid_argument("Start and goal indices must be different");
            }

            rhs_map[start_index] = 0;
            g_map[start_index] = 0;

            // queue.push(LPAstarKey(start_index,
            //                       std::make_pair(heuristic_fn(graph.getNode(start_index).getValue(),
            //                                                   graph.getNode(goal_index).getValue()),
            //                                      0)));
        }

        arc_helpers::AstarResult computeShortestPath()
        {
            while(!queue.isEmpty() &&
                  (CompareLPAstarKey()(queue.top().first, calculateKey(goal_index)) ||
                   rhs(goal_index) != g(goal_index)))
            {
                int64_t u = queue.top().second.node_id;
                queue.pop();
                if(g(u) > rhs(u))
                {
                    g(u) = rhs(u);
                    for(const GraphEdge &e: graph.getNode(u).getOutEdges())
                    {
                        updateVertex(e.getToIndex());
                    }
                }
                else
                {
                    g_map[u] = std::numeric_limits<double>::infinity();
                    for(const GraphEdge &e: graph.getNode(u).getOutEdges())
                    {
                        updateVertex(e.getToIndex());
                    }
                    updateVertex(u);
                }
            }
            //todo: return path
        }

        void updateEdgeCost(const GraphEdge &e)
        {
            updateVertex(e.getToIndex());
        }
    };
    
    


    /******************************
     **   Lazy SP
     *****************************/
    
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
                    if(e.getValidity() == EDGE_VALIDITY::INVALID)
                    {
                        evaluated_edges[getHashable(e)] = std::numeric_limits<double>::infinity();
                    }

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


        
        static arc_helpers::AstarResult
        PerformBiLazySP(Graph<NodeValueType, Allocator>& g,
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
            bool reversed = false;

            const auto edge_validity_check_fn =
                [&] (const Graph<NodeValueType, Allocator>& search_graph,
                     const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    if(edge.getValidity() == EDGE_VALIDITY::INVALID)
                    {
                        return false;
                    }
                    if(evaluated_edges.count(getHashable(edge)))
                    {
                        // std::cout << "Found already evaluted edge!\n";
                        return evaluated_edges.at(getHashable(edge)) < std::numeric_limits<double>::infinity();
                    }


                    return edge.getWeight() < std::numeric_limits<double>::infinity();
                };
    
            const auto distance_fn =
                [&] (const Graph<NodeValueType, Allocator>& search_graph,
                     const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    if(evaluated_edges.count(getHashable(edge)))
                    {
                        // std::cout << "Found already evaluted edge!\n";
                        return evaluated_edges.at(getHashable(edge));
                    }
                    // std::cout << "Using heuristic weight\n";
                    return edge.getWeight();
                };



            // RepeatedAstar<NodeValueType, Allocator>
            //     forwardAstar(g, start_index, goal_index, edge_validity_check_fn,
            //                  distance_fn, heuristic_fn, limit_pqueue_duplicates);
            // RepeatedAstar<NodeValueType, Allocator>
            //     reverseAstar(g, goal_index, start_index, edge_validity_check_fn,
            //                  distance_fn, heuristic_fn, limit_pqueue_duplicates);
            

            while(true)
            {
                PROFILE_START("lazy_sp a_star");
                auto prelim_result = PerformAstarForLazySP(g,
                                                           (!reversed ? start_index : goal_index),
                                                           (!reversed ? goal_index : start_index),
                                                           heuristic_fn, limit_pqueue_duplicates,
                                                           evaluated_edges);
                // auto prelim_result = !reversed ?
                //     forwardAstar.runAstarSearch() : reverseAstar.runAstarSearch();
                PROFILE_RECORD("lazy_sp a_star");
                num_astar_iters++;
                
                auto path = prelim_result.first;

                if(checkPath(path, g, evaluated_edges, eval_edge_fn))
                {
                    PROFILE_RECORD_DOUBLE("lazysp astar iters", num_astar_iters);
                    if(reversed)
                    {
                        std::reverse(prelim_result.first.begin(), prelim_result.first.end());
                    }
                    return prelim_result;
                }
                reversed = !reversed;
            }
        }

    };
}
#endif
