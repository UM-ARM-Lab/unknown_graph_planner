#ifndef CTP_HEURISTICS_HPP
#define CTP_HEURISTICS_HPP

#include "ctp.hpp"
#include "dijkstras_addons.hpp"
#include "graph_visualization.hpp"

namespace CTP
{
    /*******************************
     *    Optimistic optimal path
     *******************************/
    template<typename BeliefGraph>
    Action OMT(NltpProblem<BeliefGraph> &ctp, GraphVisualizer &viz)
    {
        auto result = arc_dijkstras::SimpleGraphAstar<std::vector<double>>::PerformAstar(
            ctp.belief_graph, ctp.agent.current_node, ctp.agent.goal_node, &distanceHeuristic, true);
        viz.vizPath(result.first, ctp.true_graph);
        return result.first[1];
    }


    /******************************
     *     Pareto-Optimistic Cost
     *****************************/
    template<typename BeliefGraph>
    Action POMT(NltpProblem<BeliefGraph> &ctp, double alpha, GraphVisualizer &viz)
    {
        using namespace arc_dijkstras;
        BeliefGraph &bg = ctp.belief_graph;
        const auto eval_fun = [&bg, &alpha](GraphD &g, GraphEdge &e)
        {
            double p_cost = -std::log(bg.edge_probabilities[getHashable(e)]);
            double l_cost = e.getWeight();
            return l_cost + alpha * p_cost;
        };
        auto result = LazySP<std::vector<double>>::PerformLazySP(
            ctp.belief_graph, ctp.agent.current_node, ctp.agent.goal_node, &distanceHeuristic, eval_fun);
        viz.vizPath(result.first, ctp.true_graph);
        return result.first[1];
    }




    template<typename BeliefGraph>
    class ExplorationState;

    template<typename BeliefGraph>
    void sampleMostLikely(ExplorationState<BeliefGraph> &e, GraphVisualizer &viz);

    template<typename BeliefGraph>
    arc_helpers::AstarResult exploitationPath(BeliefGraph &g, Location start, Location goal);

    template<typename BeliefGraph>
    Path explorationPath(BeliefGraph &g, Location start, double max_cost, GraphVisualizer &viz);




    /*********************************************
     *   Hedged Shortest Path under Determinism
     *********************************************/
    template<typename BeliefGraph>
    Action HSPD(NltpProblem<BeliefGraph> &ctp, GraphVisualizer &viz)
    {
        auto exploitation = exploitationPath<BeliefGraph>(ctp.belief_graph, ctp.agent.current_node,
                                                          ctp.agent.goal_node);

        std::cout << "Exploitation path has cost " << exploitation.second << "\n";
        
        Path exploration_path = explorationPath<BeliefGraph>(ctp.belief_graph, ctp.agent.current_node,
                                                             exploitation.second, viz);

        viz.vizPath(exploitation.first, ctp.belief_graph, 3, "red");
        viz.vizPath(exploration_path, ctp.belief_graph, 2, "purple");
        if(exploration_path.size() > 0)
        {
            std::cout << "Running exploration path\n";
            return exploration_path[1];
        }
        std::cout << "Running exploitation path with cost " << exploitation.second << "\n";
        return exploitation.first[1];
    }













    
    /********************************************
     *    Hedged Shorted Path under Determinism Helpers
     ********************************************/
    template<typename BeliefGraph>
    class ExplorationState
    {
    public:
        BeliefGraph g;
        Path path;
        double prob;
        double cost;
        ExplorationState(BeliefGraph g, Path path, double cost, double prob):
            g(g), prob(prob), path(path), cost(cost){};
    };

    /** Currently only for CTP
     */
    template<typename BeliefGraph>
    void sampleMostLikely(ExplorationState<BeliefGraph> &e, GraphVisualizer &viz)
    {
        using namespace arc_dijkstras;
        Location loc = e.path.back();
        BeliefGraph &g = e.g;
        double p_sample = 1.0;

        for(auto &e:g.getNode(loc).getOutEdges())
        {
            if(e.getValidity() != EDGE_VALIDITY::UNKNOWN)
            {
                continue;
            }

            double p_edge = g.edge_probabilities[getHashable(e)];

            if(p_edge >= 0.5)
            {
                p_sample *= p_edge;
                e.setValidity(EDGE_VALIDITY::VALID);
            }
            else
            {
                p_sample *= 1 - p_edge;
                e.setValidity(EDGE_VALIDITY::INVALID);
            }
        }
        e.prob *= p_sample;

        
        // std::cout << "Cost " << e.cost << " prob: " << e.prob << "\n";
        // viz.vizGraph(g, "sample");
        // viz.vizPath(e.path, g);
        // viz.vizPoints(std::vector<Location>{loc}, g);
        // arc_helpers::WaitForInput();

    }

    template<typename BeliefGraph>
    arc_helpers::AstarResult exploitationPath(BeliefGraph &g, Location start, Location goal)
    {
        using namespace arc_dijkstras;
        // const auto eval_fun = [&g](GraphD &g, GraphEdge &e)
        // {
        //     double p = g.edge_probabilities[getHashable(e)];
        //     if(p <= 0.5 && e.getValidity() == EDGE_VALIDITY::UNKNOWN)
        //     {
        //         return std::numeric_limits<double>::max();
        //     }
        //     return e.getWeight();
        // };
        // I don't remember what this function was, but I had to change it as it was inconsistent with
        // other changes
        const auto eval_fun = [](GraphD &g, GraphEdge &e)
        {
            return e.getWeight();
        };
            
        return LazySP<std::vector<double>>::PerformLazySP(
            g, start, goal, &distanceHeuristic, eval_fun);
    }

    
    template<typename BeliefGraph>
    Path explorationPath(BeliefGraph &g, Location start, double max_cost, GraphVisualizer &viz)
    {
        ExplorationState<BeliefGraph> exp(g, std::vector<Location>{start}, 0, 1.0);

        auto cost_compare = [](const ExplorationState<BeliefGraph> &a,
                               const ExplorationState<BeliefGraph> &b){return a.cost < b.cost;};
        
        std::list<ExplorationState<BeliefGraph>> open_set{exp};

        while(!open_set.empty())
        {
            auto exp_it = std::min_element(open_set.begin(), open_set.end(), cost_compare);
            exp = *exp_it;
            open_set.erase(exp_it);

            sampleMostLikely<BeliefGraph>(exp, viz);

            if(exp.cost >= max_cost)
            {
                return Path();
            }

            if(exp.prob <= 0.5)
            {
                return exp.path;
            }

            Location loc = exp.path.back();
            for(const auto &e:exp.g.getNode(loc).getOutEdges())
            {
                if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::INVALID)
                {
                    continue;
                }
                Path p(exp.path);
                p.push_back(e.getToIndex());
                ExplorationState<BeliefGraph> new_exp(exp.g, p, exp.cost + e.getWeight(), exp.prob);
                open_set.push_back(new_exp);
            }
        }
        return Path();
    }
}


#endif
