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

    /*
     *  See paper: http://idm-lab.org/bib/abstracts/papers/icaps05.pdf
     *  A generalized framework for lifelong planning A* search
     *
     *    Note - this is unfinished. Do not use without review
     */

    
    typedef std::pair<double, double> LPAstarKey;
    // typedef std::array<double, 3> LPAstarKey;

    // struct LPAstarPQueueElement
    // {
    //     int64_t node_id;

    //     LPAstarPQueueElement(int64_t node_id) :
    //         node_id(node_id){}
    // };
    typedef int64_t LPAstarPQueueElement;

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
        const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_consistent_fn;

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
            int64_t s = node_id;
            double h = heuristic_fn(graph.getNode(node_id).getValue(), graph.getNode(goal_index).getValue());
            double h_cons = heuristic_consistent_fn(graph.getNode(node_id).getValue(),
                                                    graph.getNode(goal_index).getValue());
                                                          
            if(g(node_id) < rhs(node_id))
            {
                return std::make_pair(g(s) + h_cons, g(s));
                                      
            }
            return std::make_pair(rhs(s) + h, rhs(s));
            
            
            // double k1 = std::min(g(node_id),rhs(node_id)) + heuristic_fn(graph.getNode(node_id).getValue(),
            //                                                              graph.getNode(goal_index).getValue());
            // double k2 = std::min(g(node_id), rhs(node_id));
            // return std::make_pair(k1, k2);
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
            // std::cout << "Updating vertex " << u << "\n";
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

            if(queue.contains(LPAstarPQueueElement(u)) && g(u) != rhs(u))
            {
                queue.update({calculateKey(u), LPAstarPQueueElement(u)});
            }
            else if(queue.contains(LPAstarPQueueElement(u)) && g(u) == rhs(u))
            {
                queue.remove(u);
            }
            else if(!queue.contains(LPAstarPQueueElement(u)) && g(u) != rhs(u))
            {
                queue.insert({calculateKey(u), LPAstarPQueueElement(u)});
            }
            // std::cout << "Vertex updated\n";
        }

        arc_helpers::AstarResult findPath()
        {
            // std::cout << "Returning path\n";
            arc_helpers::AstarResult result;

            result.second = g(goal_index);
            if(result.second >= std::numeric_limits<double>::max())
            {
                return result;
            }
            
            // std::cout << "Path cost " << result.second << "\n";
            
            std::vector<int64_t> path{goal_index};
            int64_t cur_index = goal_index;
            while(cur_index != start_index)
            {
                const auto& node = graph.getNode(cur_index);
                double min_val = std::numeric_limits<double>::infinity();
                for(const auto& e: node.getInEdges())
                {
                    double cc = g(e.getFromIndex()) + distance_fn(graph, e);
                    // std::cout << "  cost to " << e.getFromIndex() << " is " << cc << "\n";
                    if(cc < min_val)
                    {
                        min_val = cc;
                        cur_index = e.getFromIndex();
                    }
                }
                if(min_val >= std::numeric_limits<double>::max())
                {
                    throw std::logic_error("Best path has infinite cost");
                }
                // std::cout << "Node " << cur_index << " on path with cost " << min_val << "\n";
                path.push_back(cur_index);
            }
            std::reverse(path.begin(), path.end());
            result.first = path;
            // std::cout << "Found\n";
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
            LPAstar(graph, start_index, goal_index, edge_validity_check_fn, distance_fn,
                    heuristic_fn, heuristic_fn){}

        LPAstar(const Graph<NodeValueType, Allocator>& graph,
                const int64_t start_index,
                const int64_t goal_index,
                const std::function<bool(const Graph<NodeValueType, Allocator>&,
                                         const GraphEdge&)>& edge_validity_check_fn,
                const std::function<double(const Graph<NodeValueType, Allocator>&,
                                           const GraphEdge&)>& distance_fn,
                const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
                const std::function<double(const NodeValueType&,
                                           const NodeValueType&)>& heuristic_consistent_fn) :

            graph(graph), start_index(start_index), goal_index(goal_index),
            edge_validity_check_fn(edge_validity_check_fn),
            distance_fn(distance_fn),
            heuristic_fn(heuristic_fn),
            heuristic_consistent_fn(heuristic_consistent_fn)
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
            // g_map[start_index] = 0;

            queue.insert({calculateKey(start_index), LPAstarPQueueElement(start_index)});
        }

        arc_helpers::AstarResult computeShortestPath()
        {
            // std::cout << "Starting compute shortest path\n";
            while(!queue.isEmpty() &&
                  (CompareLPAstarKey()(queue.top().first, calculateKey(goal_index)) ||
                   rhs(goal_index) != g(goal_index)))
            {
                // std::cout << "Computing shortest path\n";
                int64_t u = queue.top().second;
                queue.pop();
                if(g(u) > rhs(u))
                {
                    g_map[u] = rhs(u);
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
                // std::cout << "Iteration complete\n";
            }
            // std::cout << "Path found...\n";
            return findPath();
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
                    if(edge.isInvalid())
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
                    // double planning_cost = 0;
                    // if(edge.getValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
                    // {
                        // PROFILE_START("astar_adding planning cost");
                        // planning_cost += 0.06;
                        // planning_cost += 1;
                        // PROFILE_RECORD("astar_adding planning cost");
                    // }
                    // else
                    // {
                    //     PROFILE_START("astar_not_adding_planning_cost");
                    //     PROFILE_RECORD("astar_not_adding_planning_cost");
                    // }

                    
                    // std::cout << "Using heuristic weight\n";
                    // return edge.getWeight() + planning_cost;
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
            while(i < (int)path.size() - 1 )
            {
                GraphEdge &e = g.getNode(path[i]).getEdgeTo(path[i+1]);
                if(e.getValidity() == EDGE_VALIDITY::INVALID)
                {
                    std::cout << "Forward selector encountered edge alredy known to be invalid: ( " << e.getFromIndex() << ", " << e.getToIndex() << ")\n";
                    assert(false);
                }
                
                // if(evaluatedEdges.count(getSortedHashable(e)) > 0 &&
                //    evaluatedEdges.at(getSortedHashable(e)) == std::numeric_limits<double>::infinity())
                // {
                //     std::cout << "Forward selector encountered edge with inf cost: ( " << e.getFromIndex() << ", " << e.getToIndex() << ")\n";
                //     assert(false);
                // }
                
                if(evaluatedEdges.count(getHashable(e)) == 0)
                {
                    return std::vector<int>{i};
                }
                i++;
            }
            return std::vector<int>();
        }


        // static bool checkPath(const std::vector<int64_t> &path,
        //                       Graph<NodeValueType, Allocator>& g,
        //                       EvaluatedEdges &evaluated_edges,
        //                       const std::function<double(Graph<NodeValueType, Allocator>&,
        //                                                  GraphEdge&)>& eval_edge_fn)
        // {
        //     return checkPath(path, g, evaluated_edges, eval_edge_fn, &ForwardSelector);
        // }
        

        static bool checkPath(const std::vector<int64_t> &path,
                              Graph<NodeValueType, Allocator>& g,
                              EvaluatedEdges &evaluated_edges,
                              const std::function<double(Graph<NodeValueType, Allocator>&,
                                                         GraphEdge&)>& eval_edge_fn,
                              const std::function<std::vector<int>(const std::vector<int64_t>&,
                                                    Graph<NodeValueType, Allocator>&,
                                                    EvaluatedEdges&)>& selector)
        {
            bool path_could_be_optimal = true;

            while(path_could_be_optimal)
            {
                auto path_indicies_to_check = selector(path, g, evaluated_edges);
                if(path_indicies_to_check.size() == 0)
                {
                    return true;
                }
            
                for(auto i:path_indicies_to_check)
                {
                    GraphEdge &e = g.getNode(path[i]).getEdgeTo(path[i+1]);
                    double evaluated_cost = eval_edge_fn(g, e);
                    evaluated_edges[getHashable(e)] = evaluated_cost;
                    // if(e.getValidity() == EDGE_VALIDITY::UNKNOWN)
                    // {
                    //     assert(false && "Edge was evaluated, should not be unknown");
                    // }
                    
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
                              
                              

        // static arc_helpers::AstarResult
        // PerformLazySP(Graph<NodeValueType, Allocator>& g,
        //               int64_t start_index,
        //               int64_t goal_index,
        //               const std::function<double(const NodeValueType&,
        //                                          const NodeValueType&)>& heuristic_fn,
        //               const std::function<double(Graph<NodeValueType, Allocator>&,
        //                                          GraphEdge&)>& eval_edge_fn,
        //               bool limit_pqueue_duplicates)
        // {
        //     return PerformLazySP(g, start_index, goal_index, heuristic_fn, eval_edge_fn,
        //                          &ForwardSelector);
        // }

        static arc_helpers::AstarResult
        PerformLazySP(Graph<NodeValueType, Allocator>& g,
                      int64_t start_index,
                      int64_t goal_index,
                      const std::function<double(const NodeValueType&,
                                                 const NodeValueType&)>& heuristic_fn,
                      const std::function<double(Graph<NodeValueType, Allocator>&,
                                                 GraphEdge&)>& eval_edge_fn,
                      const std::function<std::vector<int>(const std::vector<int64_t>&,
                                                           Graph<NodeValueType, Allocator>&,
                                                           EvaluatedEdges&)>& selector=&ForwardSelector)
        {
            EvaluatedEdges evaluated_edges;

            int num_astar_iters = 0;

            while(true)
            {
                PROFILE_START("lazy_sp a_star");
                auto prelim_result = PerformAstarForLazySP(g, start_index, goal_index,
                                                           heuristic_fn, false,
                                                           evaluated_edges);
                PROFILE_RECORD("lazy_sp a_star");
                num_astar_iters++;
                
                auto path = prelim_result.first;

                if(checkPath(path, g, evaluated_edges, eval_edge_fn, selector))
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
                        const std::function<std::vector<int>(const std::vector<int64_t>&,
                                                             Graph<NodeValueType, Allocator>&,
                                                             EvaluatedEdges&)>& selector=&ForwardSelector)
        {
            EvaluatedEdges evaluated_edges;

            int num_astar_iters = 0;
            bool reversed = false;

            while(true)
            {
                PROFILE_START("lazy_sp a_star");
                auto prelim_result = PerformAstarForLazySP(g,
                                                           (!reversed ? start_index : goal_index),
                                                           (!reversed ? goal_index : start_index),
                                                           heuristic_fn, true,
                                                           evaluated_edges);
                
                PROFILE_RECORD("lazy_sp a_star");
                num_astar_iters++;
                
                auto path = prelim_result.first;

                if(checkPath(path, g, evaluated_edges, eval_edge_fn, selector))
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
