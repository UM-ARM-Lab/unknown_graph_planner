#ifndef A_STAR_HPP
#define A_STAR_HPP

#include "graph.hpp"
#include <set>
#include <map>
#include <limits>
#include <algorithm>

double heuristic_cost_estimate(const Node &n1, const Node &n2)
{
    // double sum = 0;
    // for(int i=0; i<n1.q.size(); i++)
    // {
    //     sum += (n1.q[i] - n2.q[i]) * (n1.q[i] - n2.q[i]);
    // }
    // return std::sqrt(sum);
    return distance(n1, n2);
}



bool set_contains(const std::set<int> &set, int val)
{
    for(int elem: set)
    {
        if(elem == val)
        {
            return true;
        }
    }
    return false;
}


std::vector<int> constructPath(const std::map<int, int> &came_from, int start_ind, int goal_ind)
{
    std::vector<int> path{goal_ind};
    int current_ind = goal_ind;
    while(current_ind != start_ind)
    {
        current_ind = came_from.at(current_ind);
        path.push_back(current_ind);
    }
    std::reverse(path.begin(), path.end());
    return path;
}


std::vector<int> A_star(int start_ind, int goal_ind, const Graph &g)
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
        int current_ind = *std::min_element(open_set.begin(), open_set.end(),
                                        [&f_score](int a, int b){return f_score[a] < f_score[b];});
        if(current_ind == goal_ind)
        {
            std::cout << "SHORTEST PATH FOUND with cost " << g_score[goal_ind] << " \n";
            return constructPath(came_from, start_ind, goal_ind);
        }

        open_set.erase(current_ind);
        closed_set.insert(current_ind);
        // std::cout << "    neighbors: ";
        for(int edge_ind:g.V[current_ind].edge_inds)
        {
            if(g.E[edge_ind].validity == EDGE_VALIDITY::INVALID)
                continue;

            int neighbor = g.E[edge_ind].v1_ind;

            if(neighbor == current_ind)
            {
                neighbor = g.E[edge_ind].v2_ind;
            }
            // std::cout << neighbor << ", ";

            if(set_contains(closed_set, neighbor))
            {
                continue;
            }

            double tentative_g_score = g_score[current_ind] +
                g.E[edge_ind].weight;
                // heuristic_cost_estimate(g.V[current_ind], g.V[neighbor]);

            if(!set_contains(open_set, neighbor))
            {
                open_set.insert(neighbor);
            }
            else if(g_score.count(neighbor) && tentative_g_score >= g_score[neighbor])
            {
                continue;
            }
            
            came_from[neighbor] = current_ind;
            g_score[neighbor] = tentative_g_score;
            f_score[neighbor] = g_score[neighbor] +
                heuristic_cost_estimate(g.V[neighbor], g.V[goal_ind]);
        }
        // std::cout << "\n";
        
    }
    std::cout << "NO SOLUTION\n";
    return std::vector<int>();
}



#endif
