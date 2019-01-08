#ifndef MCTS_HPP
#define MCTS_HPP


#include "halton_graph.hpp"
#include "ctp.hpp"
// #include "graph_visualization.hpp"

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
        int expansion_count = 1;

        MCTS(State s): tree(s) 
        {
            addActions(tree.root);
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

        void addActions(StateNode* n)
        {
            for(Action action : n->state.getActions())
            {
                tree.addNode(n, action);
            }
        }

        void expand(ActionNode* parent, const State& s)
        {
            StateNode* child = tree.addNode(parent, s);
            addActions(child);
        }

        Action fastPolicy(State ctp)
        {
            auto result = arc_dijkstras::SimpleGraphAstar<std::vector<double>>::PerformAstar(
                ctp.belief_graph, ctp.agent.current_node, ctp.agent.goal_node, &distanceHeuristic, true);

            
            return result.first[1];
        }

        void backprop(std::vector<double> costs, StateNode* state)
        {
            double cost = 0;
            for(int i=(int)costs.size()-1; i>=0; i--)
            {
                cost += costs[i];
                state->visit_count++;
                state->parent->visit_count++;
                state->parent->summed_cost += cost;
                state = state->parent->parent;
            }
            assert(state = tree.root);
        }

        void rollout()
        {
            State b = tree.root->state;
            std::vector<double> costs;
            b.sampleInstance();
            StateNode* node = tree.root;
            bool in_tree = true;
            while(b.inprogress && in_tree)
            {
                ActionNode* an = selectPromisingAction(node);
                costs.push_back(b.move(an->action));
                in_tree = hasChild(an, b);
                   
                if(!in_tree)
                {
                    expand(an, b);
                    break;
                }
                node = getChild(an, b);
            }

            while(b.inprogress)
            {
                Action a = fastPolicy(b);
                costs.back() += b.move(a);
            }
            backprop(costs, node);
        }
    };

    class UCT : public MCTS
    {
    public:
        double exploration_const = 1.41;
        
    public:

        UCT(State s) : MCTS(s){};
        
        
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
            double best_val = std::numeric_limits<double>::min();
            for(ActionNode* child:parent->children)
            { 
                double v = uctValue(parent->visit_count, child->getCostEstimate(), child->visit_count);
                if(v > best_val)
                {
                    best_val = v;
                    best = child;
                }
            }
            return best;
        }
    };
        
}

#endif
