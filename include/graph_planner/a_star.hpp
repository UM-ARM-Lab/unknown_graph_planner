#ifndef LOGGING_A_STAR_HPP
#define LOGGING_A_STAR_HPP

#include <arc_utilities/dijkstras.hpp>

/********
 *   This is a copy from arc_utilities/dijkstras.hpp, with added logging
 *
 *
 */


namespace arc_dijkstras
{
    template<typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
    class AstarLogging
    {
    protected:

        AstarLogging() {}

    public:

        static arc_helpers::AstarResult PerformLazyAstar(
                Graph<NodeValueType, Allocator>& graph,
                const int64_t start_index,
                const int64_t goal_index,
                const std::function<bool(Graph<NodeValueType, Allocator>&,
                                         GraphEdge&)>& edge_validity_check_fn,
                const std::function<double(const Graph<NodeValueType, Allocator>&,
                                           const GraphEdge&)>& distance_fn,
                const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
                const bool limit_pqueue_duplicates)
        {
            using namespace arc_helpers;
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
            // Make helper function
            const auto heuristic_function = [&] (const int64_t node_index)
            {
                return heuristic_fn(graph.getNode(node_index).getValue(), graph.getNode(goal_index).getValue());
            };
            // Setup
            std::priority_queue<AstarPQueueElement,
                                std::vector<AstarPQueueElement>,
                                CompareAstarPQueueElementFn> queue;
            
            // Optional map to reduce the number of duplicate items added to the pqueue
            // Key is the node index in the provided graph
            // Value is cost-to-come
            std::unordered_map<int64_t, double> queue_members_map;
            
            // Key is the node index in the provided graph
            // Value is a pair<backpointer, cost-to-come>
            // backpointer is the parent index in the provided graph
            std::unordered_map<int64_t, std::pair<int64_t, double>> explored;
            
            // Initialize
            queue.push(AstarPQueueElement(start_index, -1, 0.0, heuristic_function(start_index)));
            if (limit_pqueue_duplicates)
            {
                queue_members_map[start_index] = 0.0;
            }
            
            // Search
            while (queue.size() > 0)
            {
                // Get the top of the priority queue
                const arc_helpers::AstarPQueueElement n = queue.top();
                queue.pop();

                if (n.id() == goal_index)
                {
                    // Solution found
                    explored[n.id()] = std::make_pair(n.backpointer(), n.costToCome());
                    break;
                }

                if (limit_pqueue_duplicates)
                {
                    queue_members_map.erase(n.id());
                }
                
                if (explored.count(n.id()) && n.costToCome() >= explored[n.id()].second)
                {
                    continue;
                }
                
                // Add to the explored list
                explored[n.id()] = std::make_pair(n.backpointer(), n.costToCome());
                
                
                // Explore and add the children
                for(GraphEdge& current_out_edge: graph.getNode(n.id()).getOutEdges())
                {
                    // Get the next potential child node
                    const int64_t child_id = current_out_edge.getToIndex();

                    if (!edge_validity_check_fn(graph, current_out_edge))
                    {
                        continue;
                    }
                    
                    // Compute the cost-to-come for the new child
                    const double child_cost_to_come = n.costToCome() + distance_fn(graph, current_out_edge);

                    if(explored.count(child_id) &&
                       child_cost_to_come >= explored[child_id].second)
                    {
                        continue;
                    }

                    if (limit_pqueue_duplicates && queue_members_map.count(child_id) &&
                        child_cost_to_come >= queue_members_map[child_id])
                    {
                        continue;
                    }
                    
                    const double child_value = child_cost_to_come + heuristic_function(child_id);
                    queue.push(AstarPQueueElement(child_id, n.id(), child_cost_to_come, child_value));
                }
            }
            return ExtractAstarResult(explored, start_index, goal_index);
        }


        static arc_helpers::AstarResult PerformAstar(
                const Graph<NodeValueType, Allocator>& graph,
                const int64_t start_index,
                const int64_t goal_index,
                const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
                const bool limit_pqueue_duplicates)
        {
            const auto edge_validity_check_function = [&] (const Graph<NodeValueType,
                                                           Allocator>& search_graph, const GraphEdge& edge)
            {
                UNUSED(search_graph);
                if(edge.getValidity() == EDGE_VALIDITY::INVALID)
                {
                    return false;
                }

                return edge.getWeight() < std::numeric_limits<double>::infinity();
            };
            const auto distance_function = [&] (const Graph<NodeValueType, Allocator>& search_graph, 
                                                const GraphEdge& edge)
            {
                UNUSED(search_graph);
                return edge.getWeight();
            };
            return PerformLazyAstar(graph, start_index, goal_index, edge_validity_check_function,
                                    distance_function, heuristic_fn, limit_pqueue_duplicates);
        }
    };
}
#endif
