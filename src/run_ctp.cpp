#include "halton_graph.hpp"
#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/timing.hpp>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "2d_obstacles.hpp"
#include "lazysp.hpp"
#include <utility>
#include "graph_visualization.hpp"
#include "ctp.hpp"

using namespace CTP;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;

    std::mt19937 rng;
    GraphVisualizer viz(n);
    ros::Rate r(0.5);


    std::string filepath = "/home/bradsaund/catkin_ws/src/graph_planner/graphs/CTP_2D_1000.graph";
    
    CtpGraph g(20, 0.5, 2, 0.5);
    // g.saveToFile(filepath);

    // HaltonGraph g(filepath);
    
    Agent agent(0, 5);
    CtpProblem<CtpGraph> ctp(g, g.sampleInstance(rng), agent);
    


    ros::Duration(1).sleep();
    
    viz.vizCtp(ctp);

    r.sleep();
    std::string unused;
    std::cout << "Waiting for user input...\n";
    std::getline(std::cin, unused);

    

    while(ros::ok() && !ctp.solved())
    {
        PROFILE_START("cycle");
        // std::vector<int> path = A_star(points[0], points[1], g);
        PROFILE_START("astar");
        
        auto result = arc_dijkstras::SimpleGraphAstar<std::vector<double>>::PerformAstar(
            ctp.belief_graph, ctp.agent.current_node, ctp.agent.goal_node, &distanceHeuristic, true);
        
        // double astar_time = PROFILE_RECORD("astar");
        // std::cout << "Astar took: " << astar_time << "\n";

        auto path = result.first;
        // forwardLazyCheck(path, g, obs);

        PROFILE_START("forward_move");
        ctp.move(path[1]);
        std::cout << "forward move took: " << PROFILE_RECORD("forward_move") << "\n";

        
        PROFILE_START("visualize");
        viz.vizCtp(ctp);
        viz.vizPath(path, ctp.true_graph);
        std::cout << "visualize took " << PROFILE_RECORD("visualize") << "\n";
        std::cout << "cycle took " << PROFILE_RECORD("cycle") << "\n";
        r.sleep();
        std::cout << "Waiting for user input...\n";
        std::getline(std::cin, unused);

    }

    
    
    return 0;
}
