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
    viz.vizTitle("True Graph");
    viz.vizText("Heuristic Cost = L - log(prob)", 1001, 1.2, 0.5);

    r.sleep();
    arc_helpers::WaitForInput();

    while(ros::ok() && !ctp.solved())
    {
        MCTS::UCTU mcts(ctp, viz);
        // for(int i=0; i<100; i++)
        // {
        //     mcts.rollout();
        // }
        int a = mcts.findAction();

        PROFILE_START("cycle");
        // std::vector<int> path = A_star(points[0], points[1], g);
        PROFILE_START("astar");
        
        // auto result = arc_dijkstras::SimpleGraphAstar<std::vector<double>>::PerformAstar(
        //     ctp.belief_graph, ctp.agent.current_node, ctp.agent.goal_node, &distanceHeuristic, true);
        
        // double astar_time = PROFILE_RECORD("astar");
        // std::cout << "Astar took: " << astar_time << "\n";

        // auto path = result.first;
        // forwardLazyCheck(path, g, ctp.true_graph.storm);

        PROFILE_START("forward_move");
        viz.vizCtp(ctp);
        viz.vizTitle("True Graph");
        viz.vizText("True Move", 1002, 1.1, 0.5);

        ctp.move(a);
        arc_helpers::WaitForInput();
        ros::Duration(2).sleep();


        viz.vizCtp(ctp);

        
        // r.sleep();
        arc_helpers::WaitForInput();
        ros::Duration(2).sleep();
        arc_helpers::WaitForInput();
    }

    
    
    return 0;
}
