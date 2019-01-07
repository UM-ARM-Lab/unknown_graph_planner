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

    ros::Publisher graph_valid_pub = n.advertise<visualization_msgs::Marker>("valid_graph", 10);
    ros::Publisher graph_unknown_pub = n.advertise<visualization_msgs::Marker>("unknown_graph", 10);
    ros::Publisher graph_invalid_pub = n.advertise<visualization_msgs::Marker>("invalid_graph", 10);
    ros::Publisher path_pub = n.advertise<visualization_msgs::Marker>("path", 10);
    ros::Publisher points_pub = n.advertise<visualization_msgs::Marker>("points", 10);
    ros::Publisher obs_pub = n.advertise<visualization_msgs::Marker>("ob", 10);
    ros::Rate r(0.5);


    std::string filepath = "/home/bradsaund/catkin_ws/src/graph_planner/graphs/BCTP_2D_10x10.graph";

    int rows=10;
    BctpGrid g(rows);
    // g.saveToFile(filepath);

    // HaltonGraph g(filepath);
    
    Agent agent(rows + 1, rows*(rows-1)-2);

    CtpProblem ctp(g, g.sampleInstance(), agent);
    


    ros::Duration(1).sleep();

    
    GraphMarker gm = toVisualizationMsg(ctp.true_graph);
    graph_valid_pub.publish(gm[0]);
    graph_unknown_pub.publish(gm[1]);
    // graph_invalid_pub.publish(gm[2]);
    points_pub.publish(toVisualizationMsg(ctp.agent, g));

    r.sleep();
    std::string unused;
    std::cout << "Waiting for user input...\n";
    std::getline(std::cin, unused);

    

    while(ros::ok() && !ctp.solved())
    {
        PROFILE_START("cycle");
        // std::vector<int> path = A_star(points[0], points[1], g);
        PROFILE_START("astar");
        
        // auto result = arc_dijkstras::SimpleGraphAstar<std::vector<double>>::PerformAstar(
        //     ctp.belief_graph, ctp.agent.current_node, ctp.agent.goal_node, &distanceHeuristic, true);
        
        // double astar_time = PROFILE_RECORD("astar");
        // std::cout << "Astar took: " << astar_time << "\n";

        // auto path = result.first;
        // forwardLazyCheck(path, g, obs);

        PROFILE_START("forward_move");
        // ctp.move(path[1]);
        std::cout << "forward move took: " << PROFILE_RECORD("forward_move") << "\n";

        
        PROFILE_START("visualize");

        // GraphMarker gm = toVisualizationMsg(g.sampleInstance());
        GraphMarker gm = toVisualizationMsg(ctp.belief_graph);
        obs_pub.publish(g.storm.toMarker());
        graph_valid_pub.publish(gm[0]);
        graph_unknown_pub.publish(gm[1]);
        graph_invalid_pub.publish(gm[2]);
        // path_pub.publish(toVisualizationMsg(path, g));
        points_pub.publish(toVisualizationMsg(ctp.agent, g));
        std::cout << "visualize took " << PROFILE_RECORD("visualize") << "\n";
        std::cout << "cycle took " << PROFILE_RECORD("cycle") << "\n";
        
        // r.sleep();
        std::cout << "Waiting for user input...\n";
        std::getline(std::cin, unused);

    }

    
    
    return 0;
}
