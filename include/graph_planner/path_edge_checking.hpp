#ifndef PATH_EDGE_CHECKING_HPP
#define PATH_EDGE_CHECKING_HPP

// #include "a_star.hpp"
#include "2d_obstacles.hpp"
#include <arc_utilities/dijkstras.hpp>

// bool forwardLazyCheck(const std::vector<int> &path, Graph &g, const Obstacles2D::Obstacles &obs)
// {
//     // std::vector<int> path = A_star(start_ind, goal_ind, g);

//     std::cout << "Forward Check\n";
//     int i = 0;
//     while(i < path.size()-1)
//     {
//         Edge &e = g.getEdge(path[i], path[i+1]);
//         if(e.validity == EDGE_VALIDITY::VALID)
//         {
//             i++;
//             continue;
//         }

//         e.validity = obs.isValid(e, g) ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID;
        
//         std::cout << "Edge from point " << path[i] << " is " << 
//             ((e.validity == EDGE_VALIDITY::VALID) ? "valid" : "invalid") << "\n";
//         // std::cout << "Edge from point " << i << " is " << e.validity << "\n";
//         return false;
//     }
//     return true;
// }



// /***
//  *   Takes a single step on the path provided
//  *    Check and updated edge validity
//  */
// int forwardMove(const std::vector<int> &path, Graph &g, const Obstacles2D::Obstacles &obs)
// {
//     if(path.size() <= 1)
//     {
//         return path[0];
//     }
    
//     Edge &e = g.getEdge(path[0], path[1]);
//     assert(e.validity != EDGE_VALIDITY::INVALID);

//     if(e.validity == EDGE_VALIDITY::UNKNOWN)
//     {
//         e.validity = obs.isValid(e, g) ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID;
//     }

//     int robot_location = path[0];
//     if(e.validity == EDGE_VALIDITY::VALID)
//     {
//         robot_location = (path[0] == e.v1_ind) ? e.v2_ind : e.v1_ind;
//     }
//     return robot_location;
// }



// Arc Graph
bool forwardLazyCheck(const std::vector<int64_t> &path,
                      HaltonGraph &g, const Obstacles2D::Obstacles &obs)
{
    // std::vector<int> path = A_star(start_ind, goal_ind, g);

    std::cout << "Forward Check\n";
    int i = 0;
    while(i < path.size()-1)
    {
        
        arc_dijkstras::GraphEdge &e = g.getNode(path[i]).getEdgeTo(path[i+1]);
        if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::VALID)
        {
            i++;
            continue;
        }

        bool valid = obs.isValid(e, g);
        auto validity = valid ? arc_dijkstras::EDGE_VALIDITY::VALID : arc_dijkstras::EDGE_VALIDITY::INVALID;

        e.setValidity(validity);
        arc_dijkstras::GraphEdge &e2 = g.getNode(path[i+1]).getEdgeTo(path[i]);
        e2.setValidity(validity);
        
        std::cout << "Edge from point " << path[i] << " is " << 
            (valid ? "valid" : "invalid") << "\n";
        // std::cout << "Edge from point " << i << " is " << e.validity << "\n";
        return false;
    }
    return true;
}



/***
 *   Takes a single step on the path provided
 *    Check and updated edge validity
 */
int forwardMove(const std::vector<int64_t> &path, HaltonGraph &g,
                const Obstacles2D::Obstacles &obs)
{
    if(path.size() <= 1)
    {
        return path[0];
    }
    
    arc_dijkstras::GraphEdge &e = g.getNode(path[0]).getEdgeTo(path[1]);
    assert(e.getValidity() != arc_dijkstras::EDGE_VALIDITY::INVALID);

    if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
    {
        auto validity = obs.isValid(e, g) ? arc_dijkstras::EDGE_VALIDITY::VALID :
            arc_dijkstras::EDGE_VALIDITY::INVALID;
        arc_dijkstras::GraphEdge &e2 = g.getNode(path[1]).getEdgeTo(path[0]);
        e.setValidity(validity);
        e2.setValidity(validity);
    }

    int robot_location = path[0];
    if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::VALID)
    {
        robot_location = path[1];
    }
    return robot_location;
}


#endif
