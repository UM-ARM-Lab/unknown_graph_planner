#ifndef MCTS_HPP
#define MCTS_HPP


#include "halton_graph.hpp"
#include "ctp.hpp"
typedef CTP::CtpProblem State;

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

        ActionNode* addNode(StateNode* parent)
        {
            ActionNode* child = new ActionNode();
            parent->children.push_back(child);
            child->parent = parent;
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
        
        virtual ActionNode* selectPromisingAction(StateNode* node) = 0;

        void rollout()
        {
            // State b = tree.root->state;
            
        }

        
    };

    class UCT : public MCTS
    {
    public:
        double exploration_const = 1.41;
        
    public:
        ActionNode* selectPromisingAction(StateNode* node)
        {
            if(node->children.size()>0)
            {
                return findBestUctChild(node);
            }
            return node->children[0]; //really should be fast policy
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
