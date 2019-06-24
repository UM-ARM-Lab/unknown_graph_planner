#include "halton_graph.hpp"
#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/timing.hpp>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "2d_obstacles.hpp"
#include "path_edge_checking.hpp"
#include <utility>
#include "graph_visualization.hpp"



Obstacles2D::Obstacles makeObstacles()
{
    Obstacles2D::Obstacles o;
    o.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.1,0.5,0.9,0.8));
    return o;
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;

    ros::Publisher graph_valid_pub = n.advertise<visualization_msgs::Marker>("valid_graph", 10);
    ros::Publisher graph_unknown_pub = n.advertise<visualization_msgs::Marker>("unknown_graph", 10);
    ros::Publisher graph_invalid_pub = n.advertise<visualization_msgs::Marker>("invalid_graph", 10);
    ros::Publisher path_pub = n.advertise<visualization_msgs::Marker>("path", 10);
    ros::Publisher points_pub = n.advertise<visualization_msgs::Marker>("points", 10);
    ros::Publisher obs_pub = n.advertise<visualization_msgs::MarkerArray>("obs", 10);
    ros::Rate r(20);

    Obstacles2D::Obstacles obs = makeObstacles();

    std::string filepath = "/home/bradsaund/catkin_ws/src/graph_planner/graphs/2D_1000.graph";
    
    // HaltonGraph g(1000, 0.1);
    // g.saveToFile(filepath);

    HaltonGraph g(filepath);

    
    
    int start = 0;
    int end = 5;
    std::vector<int64_t> points{start, end};

    // validateEdges(g, obs);


    ros::Duration(1).sleep();
    
    GraphMarker gm = toVisualizationMsg(g);
    graph_valid_pub.publish(gm[0]);
    graph_unknown_pub.publish(gm[1]);
    points_pub.publish(pointsToVisualizationMsg(points, g));
    obs_pub.publish(obs.toMarkerArray());

    r.sleep();
    std::string unused;
    std::cout << "Waiting for user input...\n";
    std::getline(std::cin, unused);


    while(ros::ok() && (points[0] != points[1]))
    {
        PROFILE_START("cycle");
        // std::vector<int> path = A_star(points[0], points[1], g);
        PROFILE_START("astar");
        
        auto result = arc_dijkstras::SimpleGraphAstar<std::vector<double>>::PerformAstar(
            g, points[0], points[1], &distanceHeuristic, true);
        
        double astar_time = PROFILE_RECORD("astar");
        std::cout << "Astar took: " << astar_time << "\n";

        auto path = result.first;
        // forwardLazyCheck(path, g, obs);

        PROFILE_START("forward_move");
        points[0] = forwardMove(path, g, obs);
        std::cout << "forward move took: " << PROFILE_RECORD("forward_move") << "\n";

        PROFILE_START("visualize");
        GraphMarker gm = toVisualizationMsg(g);
        graph_valid_pub.publish(gm[0]);
        graph_unknown_pub.publish(gm[1]);
        graph_invalid_pub.publish(gm[2]);
        path_pub.publish(toVisualizationMsg(path, g));
        points_pub.publish(pointsToVisualizationMsg(points, g));
        obs_pub.publish(obs.toMarkerArray());
        std::cout << "visualize took " << PROFILE_RECORD("visualize") << "\n";
        std::cout << "cycle took " << PROFILE_RECORD("cycle") << "\n";
        // r.sleep();
    }

    
    
    return 0;
}
