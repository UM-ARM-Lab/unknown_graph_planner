#ifndef MCTS_HPP
#define MCTS_HPP


#include "halton_graph.hpp"
#include "ctp.hpp"
#include "graph_visualization.hpp"

typedef CTP::CtpProblem<CTP::BctpGrid> State;

namespace MCTS{
    typedef int Action;


    class StateNode;

    class ActionNode
    {
    public:
        double summed_cost;
        int visit_count;
        std::vector<StateNode*> children;
        StateNode* parent;
        Action action;

        double getCostEstimate()
        {
            return summed_cost/visit_count;
        }
    };

    class StateNode
    {
    public:
        State state;
        std::vector<ActionNode*> children;
        int visit_count;
        ActionNode* parent; // not sure if we should use this

        StateNode(State s) : state(s){};
    };

    class Tree
    {
    public:
        
        StateNode* root;
        
        Tree(State b)
        {
            root = new StateNode(b);
            root->parent = nullptr;
            root->visit_count = 0;
        }

        ~Tree()
        {
            deleteSubtree(root);
        }

        void deleteSubtree(StateNode* root)
        {
            for(ActionNode* child:root->children)
            {
                deleteSubtree(child);
            }
            delete root;
        }

        void deleteSubtree(ActionNode* root)
        {
            for(StateNode* child:root->children)
                deleteSubtree(child);
            delete root;
        }

        ActionNode* addNode(StateNode* parent, Action a)
        {
            ActionNode* child = new ActionNode();
            parent->children.push_back(child);
            child->parent = parent;
            child->action = a;
            return child;
        }

        StateNode* addNode(ActionNode* parent, State s)
        {
            StateNode* child = new StateNode(s);
            parent->children.push_back(child);
            child->parent = parent;
            return child;
        }
    };
    
    class MCTS
    {
    public:
        Tree tree;
        GraphVisualizer &viz;
        // int expansion_count = 1;

        MCTS(State s, GraphVisualizer &viz): tree(s), viz(viz)
        {
            addActions(tree.root);
            tree.root->visit_count= 1;
        }
        
        virtual ActionNode* selectPromisingAction(StateNode* node) = 0;

        bool hasChild(ActionNode* n, const State& s)
        {
            for(StateNode* child:n->children)
            {
                if(child->state.isEquiv(s))
                {
                    return true;
                }
            }
            return false;
        }

        StateNode* getChild(ActionNode* n, const State& s)
        {
            for(StateNode* child:n->children)
            {
                if(child->state.isEquiv(s))
                {
                    return child;
                }
            }
            return nullptr;
        }

        void showChildrenValues(StateNode* parent)
        {
            std::vector<double> vals;
            std::vector<std::vector<double>> locs;
            int id = parent->state.agent.current_node;
            std::vector<double> pp = parent->state.true_graph.GetNodeImmutable(id).GetValueImmutable();

            for(ActionNode* child:parent->children)
            {
                // if(child->children.size() == 0)
                // {
                //     continue;
                // }
                // int agent_node_id = child->children[0]->state.agent.current_node;
                int agent_node_id = child->action;
                std::vector<double> pc = parent->state.true_graph.GetNodeImmutable(agent_node_id).GetValueImmutable();

                std::vector<double> loc(2);
                loc[0] = 0.3*pp[0] + 0.7*pc[0];
                loc[1] = 0.3*pp[1] + 0.7*pc[1];
                
                locs.push_back(loc);
                vals.push_back(child->getCostEstimate());
            }
            viz.vizDoubles(vals, locs);
        }


        void addActions(StateNode* n)
        {
            int cur_node = n->state.agent.current_node;
            for(Action action : n->state.getActions())
            {
                ActionNode* an = tree.addNode(n, action);
                int next_node = action;
                double action_cost = n->state.belief_graph.GetEdgeMutable(cur_node, next_node).GetWeight();
                an->summed_cost = action_cost;
                if(next_node != n->state.agent.goal_node)
                {
                    auto result = arc_dijkstras::SimpleGraphAstar<std::vector<double>>::PerformAstar(
                        n->state.belief_graph, next_node, n->state.agent.goal_node, &distanceHeuristic, true);
                    an->summed_cost += result.second;
                }
                an->visit_count++;
            }
        }

        StateNode* expand(ActionNode* parent, const State& s)
        {
            StateNode* child = tree.addNode(parent, s);
            addActions(child);
            return child;
        }

        Action fastPolicy(State ctp)
        {
            auto result = arc_dijkstras::SimpleGraphAstar<std::vector<double>>::PerformAstar(
                ctp.belief_graph, ctp.agent.current_node, ctp.agent.goal_node, &distanceHeuristic, true);

            
            return result.first[1];
        }

        void backprop(std::vector<double> costs, StateNode* state)
        {

            // std::cout << "costs size: " << costs.size() << "\n";
            double cost = 0;
            for(int i=(int)costs.size()-1; i>=0; i--)
            {
                cost += costs[i];
                std::cout << "backproping cost " << cost  << "\n";
                std::cout << "node id: " << state->state.agent.current_node << "\n";
                state->visit_count++;
                state->parent->visit_count++;
                state->parent->summed_cost += cost;
                state = state->parent->parent;
            }
            state->visit_count++;
            assert(state == tree.root);
        }

        void rollout()
        {
            State b = tree.root->state;
            std::vector<int64_t> path{b.agent.current_node};
            viz.vizPath(path, b.belief_graph, 1, "blue");
            std::vector<double> costs;
            b.sampleInstance();
            StateNode* node = tree.root;
            bool in_tree = true;
            while(b.inprogress && in_tree)
            {
                showChildrenValues(node);
                ActionNode* an = selectPromisingAction(node);
                costs.push_back(b.move(an->action));
                in_tree = hasChild(an, b);
                path.push_back(b.agent.current_node);
                   
                if(!in_tree)
                {
                    node = expand(an, b);
                    break;
                }
                node = getChild(an, b);
                viz.vizPath(path, b.belief_graph, 2, "blue");
                arc_helpers::WaitForInput();
            }
            viz.vizPath(path, b.belief_graph, 2, "blue");
            arc_helpers::WaitForInput();
            path.resize(0);
            path.push_back(b.agent.current_node);
            while(b.inprogress)
            {

                Action a = fastPolicy(b);
                costs.back() += b.move(a);
                path.push_back(b.agent.current_node);

            }
            
            viz.vizPath(path, b.belief_graph, 1, "purple");
            backprop(costs, node);
            arc_helpers::WaitForInput();
        }
    };

    class UCT : public MCTS
    {
    public:
        double exploration_const = 1.41;
        // double exploration_const = 0.000141;
        
    public:

        UCT(State s, GraphVisualizer &viz) : MCTS(s, viz){};
        
        
        ActionNode* selectPromisingAction(StateNode* node)
        {
            assert(node->children.size()>0);
            return findBestUctChild(node);
        }

        double uctValue(int parent_visits, double child_cost, double child_visits)
        {
            if(child_visits == 0)
            {
                return std::numeric_limits<double>::max();
            }
            return -child_cost +
                exploration_const * std::sqrt(std::log((double)parent_visits) / (double)child_visits);
        }

        ActionNode* findBestUctChild(StateNode* parent)
        {
            ActionNode* best = parent->children[0];
            double best_val = std::numeric_limits<double>::lowest();
            for(ActionNode* child:parent->children)
            { 
                double v = uctValue(parent->visit_count, child->getCostEstimate(), child->visit_count);
                std::cout << "action has value " << child->getCostEstimate() << ", uct: " << v << "\n";
                if(v > best_val)
                {
                    best_val = v;
                    best = child;
                }
            }
            std::cout << "best action " << best->action << "\n";
            return best;
        }

    };
        
}

#endif
