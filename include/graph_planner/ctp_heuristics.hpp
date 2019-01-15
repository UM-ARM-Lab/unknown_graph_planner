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
            double l_cost = e.GetWeight();
            return l_cost + alpha * p_cost;
        };
        auto result = LazySP<std::vector<double>>::PerformLazySP(
            ctp.belief_graph, ctp.agent.current_node, ctp.agent.goal_node, &distanceHeuristic, eval_fun, true);
        viz.vizPath(result.first, ctp.true_graph);
        return result.first[1];
    }





    /********************************************
     *    Hedged Shorted Path under Determinism
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
    void sampleMostLikely(ExplorationState<BeliefGraph> &e)
    {
        using namespace arc_dijkstras;
        Location loc = e.path.back();
        BeliefGraph &g = e.g;
        double p_sample = 1.0;
        for(auto &e:g.GetNodeImmutable(loc).GetOutEdgesMutable())
        {
            if(e.GetValidity() != EDGE_VALIDITY::UNKNOWN)
            {
                continue;
            }

            double p_edge = g.edge_probabilities[getHashable(e)];

            if(p_edge >= 0.5)
            {
                p_sample *= p_edge;
                e.SetValidity(EDGE_VALIDITY::VALID);
            }
            else
            {
                p_sample *= 1 - p_edge;
                e.SetValidity(EDGE_VALIDITY::INVALID);
            }
        }
        e.prob *= p_sample;
    }

    
    template<typename BeliefGraph>
    Path explorationPath(BeliefGraph &g, Location start)
    {
        ExplorationState<BeliefGraph> exp(g, std::vector<Location>{start}, 0, 1.0);
        std::set<ExplorationState<BeliefGraph>> open_set{exp};

        while(!open_set.empty())
        {
            auto exp_it = std::min_element(open_set.begin(), open_set.end(),
                                           [](ExplorationState<BeliefGraph> &a,
                                              ExplorationState<BeliefGraph> &b){return a.cost < b.cost;});
            exp = *exp_it;
            open_set.erase(exp_it);

            sampleMostLikely<BeliefGraph>(exp);
            
            if(exp.prob <= 0.5)
            {
                return exp.path;
            }

            Location loc = exp.path.back();
            for(auto &e:g.GetNodeImmutable(loc).GetOutEdgesMutable())
            {
                Path p(exp.path);
                p.push_back(e.GetToIndex());
                ExplorationState<BeliefGraph> new_exp(exp.g, p, exp.cost + e.GetWeight(), exp.prob);
                open_set.insert(new_exp);
            }
        }
        return Path();
    }

}


#endif
