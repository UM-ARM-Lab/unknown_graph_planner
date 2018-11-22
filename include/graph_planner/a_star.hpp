#ifndef A_STAR_HPP
#define A_STAR_HPP

#include "graph.hpp"
#include <set>
#include <map>
#include <limits>
#include <algorithm>

double heuristic_cost_estimate(const Node &n1, const Node &n2)
{
    double sum = 0;
    for(int i=0; i<n1.q.size(); i++)
    {
        sum += (n1.q[i] - n2.q[i]) * (n1.q[i] - n2.q[i]);
    }
    return std::sqrt(sum);
}



void A_star(int start_ind, int goal_ind, Graph g)
{
    std::set<int> closed_set;
    std::set<int> open_set{start_ind};

    std::map<int, int> came_from;
    std::map<int, double> g_score;
    std::vector<double> f_score(g.V.size(), std::numeric_limits<double>::max());
    // std::map<int, double> f_score;

    g_score[start_ind] = 0;
    f_score[start_ind] = heuristic_cost_estimate(g.V[start_ind], g.V[goal_ind]);


    while(!open_set.empty())
    {
        int current = *std::min_element(open_set.begin(), open_set.end(),
                                        [&f_score](int a, int b){return f_score[a] < f_score[b];});
        if(current == goal_ind)
        {
            std::cout << "SHORTEST PATH FOUND";
            return;
        }

        open_set.erase(current);
        closed_set.insert(current);


        !!!!!!!!!!!!!Not finished
    }
    
}



#endif
