#include "increasing_density_grid.hpp"
#include "increasing_density_planning.hpp"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "2d_obstacles.hpp"
#include <utility>
#include "graph_visualization.hpp"


using namespace Obstacles2D;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;

    GraphVisualizer viz(n);
    ros::Rate r(0.5);
    r.sleep();

    IncreasingDensityGrid g(5);

    std::cout << "Graph has " << g.getNodes().size() << " nodes\n";
    

    std::vector<double> start{0,0};
    std::vector<double> goal{1,1};
    // Rect r1(
    
    Obstacles obs;

    Rect r1(-0.1, 0.6, 0.4, 1.1);
    Rect r2(0.7, -0.1, 1.1, 0.6);
    Rect r3(0.4, 0.9, 0.6, 1.1);
    obs.obs.push_back(&r1);
    obs.obs.push_back(&r2);
    obs.obs.push_back(&r3);
    
    
    viz.vizGraph(g);
    viz.vizObstacles(obs);

    while(ros::ok())
    {
        std::cout << "Planning from start to goal\n";
        auto result = Plan(g, obs, start, goal);
        // auto result = AstarPlan(g, obs, start, goal);
        std::cout << "Plan complete\n";
        viz.vizGraph(g);
        viz.vizPath(result.first, g);
        viz.vizObstacles(obs, -1.0);
        std::cout << "Path: " << PrettyPrint::PrettyPrint(result.first) << "\n";
        std::cout << "Path cost: " << result.second << "\n";
        arc_helpers::WaitForInput();
    }
    
    return 0;
}
