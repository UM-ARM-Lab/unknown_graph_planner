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
#include "mcts.hpp"

using namespace CTP;



int main(int argc, char **argv)
{
    std::mt19937 rng;

    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;

    GraphVisualizer viz(n);
    ros::Rate r(0.5);

    std::string filepath = "/home/bradsaund/catkin_ws/src/graph_planner/graphs/BCTP_2D_10x10.graph";

    int rows=10;
    
    BctpGrid g(rows);
    // g.saveToFile(filepath);

    // HaltonGraph g(filepath);
    
    Agent agent(rows + 1, rows*(rows-1)-2);

    CtpProblem<BctpGrid> ctp(g, g.sampleInstance(rng), agent);
    


    ros::Duration(1).sleep();


    viz.vizCtp(ctp);
    viz.vizGraph(ctp.true_graph, "true_graph");

    r.sleep();
    arc_helpers::WaitForInput();

    while(ros::ok() && !ctp.solved())
    {
        MCTS::UCTH mcts(ctp, viz);
        for(int i=0; i<100; i++)
        {
            mcts.rollout();
        }

        PROFILE_START("cycle");
        // std::vector<int> path = A_star(points[0], points[1], g);
        PROFILE_START("astar");
        
        auto result = arc_dijkstras::SimpleGraphAstar<std::vector<double>>::PerformAstar(
            ctp.belief_graph, ctp.agent.current_node, ctp.agent.goal_node, &distanceHeuristic, true);
        
        // double astar_time = PROFILE_RECORD("astar");
        // std::cout << "Astar took: " << astar_time << "\n";

        auto path = result.first;
        // forwardLazyCheck(path, g, ctp.true_graph.storm);

        PROFILE_START("forward_move");
        ctp.move(path[1]);
        std::cout << "forward move took: " << PROFILE_RECORD("forward_move") << "\n";

        
        PROFILE_START("visualize");
        viz.vizCtp(ctp);
        std::cout << "visualize took " << PROFILE_RECORD("visualize") << "\n";
        std::cout << "cycle took " << PROFILE_RECORD("cycle") << "\n";
        
        // r.sleep();
        arc_helpers::WaitForInput();
    }

    
    
    return 0;
}
