#include "increasing_density_grid.hpp"
#include "increasing_density_planning.hpp"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "2d_obstacles.hpp"
#include <utility>
#include "graph_visualization.hpp"


using namespace Obstacles2D;
using namespace Eigen;
using namespace increasing_density_planning;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;

    GraphVisualizer viz(n);
    ros::Rate r(0.5);
    r.sleep();

    IncreasingDensityGrid g(5);
    IncreasingDensityGrid g_evaluated(g);
    IncreasingDensityGrid g_all_valid(g);


    std::cout << "Graph has " << g.getNodes().size() << " nodes\n";
    

    std::vector<double> start{0,0};
    std::vector<double> goal{1,1};
    
    Obstacles obs;
    checkAllEdges(g_all_valid, obs);

    Rect r1(-0.1, 0.6, 0.4, 1.1);
    Rect r2(0.7, -0.1, 1.1, 0.6);
    Rect r3(0.4, 0.9, 0.6, 1.1);
    ConvexPolygon cp1(std::vector<Vector2d>{Vector2d(.4,.6), Vector2d(.4,1.1), Vector2d(0.9,1.1)});
    obs.obs.push_back(&r1);
    obs.obs.push_back(&r2);
    obs.obs.push_back(&r3);
    obs.obs.push_back(&cp1);


    checkAllEdges(g_evaluated, obs);    
    // 
    
    viz.vizGraph(g);
    viz.vizGraph(g_evaluated, "evaluated");
    viz.vizGraph(g_all_valid, "all_edges");
    viz.vizObstacles(obs, -1.0);
    arc_helpers::WaitForInput();

    while(ros::ok())
    {
        std::cout << "Planning from start to goal\n";
        // auto result = Plan(g, obs, start, goal);
        auto result = AstarPlan(g, obs, start, goal);
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
