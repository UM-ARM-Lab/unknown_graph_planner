#include "increasing_density_grid.hpp"
#include "increasing_density_halton.hpp"
#include "increasing_density_planning.hpp"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "2d_obstacles.hpp"
#include <utility>
#include "graph_visualization.hpp"


using namespace Obstacles2D;
using namespace Eigen;
using namespace increasing_density_planning;


void setScene1(Obstacles &obs)
{
    obs.obs.push_back(std::make_shared<Rect>(-0.1, 0.6, 0.4, 1.1));
    obs.obs.push_back(std::make_shared<Rect>(0.7, -0.1, 1.1, 0.6));
    obs.obs.push_back(std::make_shared<Rect>(0.4, 0.9, 0.6, 1.1));

    auto cp1 = std::make_shared<ConvexPolygon>(std::vector<Vector2d>{
            Vector2d(.4,.6),
                Vector2d(.4,1.1),
                Vector2d(0.9,1.1)});
    obs.obs.push_back(cp1);
}

void setScene2(Obstacles &obs)
{
    obs.obs.push_back(std::make_shared<Rect>(-0.1, 0.05, 0.8, 0.1));
    // obs.obs.push_back(std::make_shared<Rect>(-0.1, 0.4, 0.3, 0.6));
    obs.obs.push_back(std::make_shared<Rect>(0.45, 0.4, 0.9, 0.45));
    obs.obs.push_back(std::make_shared<Rect>(0.85, 0.4, 0.9, 0.9));
    obs.obs.push_back(std::make_shared<Rect>(0.8, 0.8, 1.1, 0.9));
    // obs.obs.push_back(std::make_shared<Rect>(0.7, -0.1, 1.1, 0.6));
    // obs.obs.push_back(std::make_shared<Rect>(0.4, 0.9, 0.6, 1.1));

    auto cp1 = std::make_shared<ConvexPolygon>(std::vector<Vector2d>{
            Vector2d(-0.1, 0.4),
                Vector2d(-0.1, 0.45),
                Vector2d( 0.3, 0.45),
                Vector2d( 0.35, 0.425),
                Vector2d( 0.3, 0.4),
                });
    obs.obs.push_back(cp1);

    auto cp2 = std::make_shared<ConvexPolygon>(std::vector<Vector2d>{
            Vector2d(0.2, 0.6),
                Vector2d(0.45, 0.4),
                Vector2d( 0.45, 0.6),
                });
    obs.obs.push_back(cp2);
}


void setScene3(Obstacles &obs)
{
    auto cp1 = std::make_shared<ConvexPolygon>(std::vector<Vector2d>{
            Vector2d(0.1, 1.1),
                Vector2d(0.3, 0.45),
                Vector2d( 0.69, 0.71),
                });
    obs.obs.push_back(cp1);

    auto cp2 = std::make_shared<ConvexPolygon>(std::vector<Vector2d>{
            Vector2d(1.1, 0.1),
                Vector2d(0.45, 0.3),
                Vector2d( 0.71, 0.69),
                });
    obs.obs.push_back(cp2);
}

void setScene4(Obstacles &obs)
{
    auto cp1 = std::make_shared<ConvexPolygon>(std::vector<Vector2d>{
            Vector2d(0.8, 1.1),
                Vector2d(0.73, 0.78),
                Vector2d( 0.8, 0.85),
                });
    obs.obs.push_back(cp1);

    auto cp2 = std::make_shared<ConvexPolygon>(std::vector<Vector2d>{
            Vector2d(1.1, 0.8),
                Vector2d(0.78, 0.73),
                Vector2d( 0.85, 0.8),
                });
    obs.obs.push_back(cp2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;

    GraphVisualizer viz(n);
    ros::Rate r(0.5);
    r.sleep();

    int depth = 8;
    
    // DoublingSDG g(depth);
    // DoublingSDG g_evaluated(g);
    // DoublingSDG g_all_valid(g);
    ConicSDG g(depth);
    ConicSDG g_evaluated(g);
    ConicSDG g_all_valid(g);
    
    // SDHaltonGraph g(depth);
    // SDHaltonGraph g_evaluated(g);
    // SDHaltonGraph g_all_valid(g);
    // IterativeDeepeningHaltonGraph g(depth);
    // IterativeDeepeningHaltonGraph g_evaluated(g);
    // IterativeDeepeningHaltonGraph g_all_valid(g);


    std::cout << "Graph has " << g.getNodes().size() << " nodes and " << g.countEdges() << " edges\n";
    

    std::vector<double> start{0,0};
    std::vector<double> goal{1,1};
    
    Obstacles obs;
    checkAllEdges(g_all_valid, obs);

    setScene2(obs);

    checkAllEdges(g_evaluated, obs);    
    
    
    viz.vizGraph(g);
    viz.vizGraph(g_evaluated, "evaluated");
    viz.vizGraph(g_all_valid, "all_edges");
    viz.vizObstacles(obs, -(double)depth/5);
    arc_helpers::WaitForInput();

    bool record = true;

    while(ros::ok())
    {
        std::cout << "Planning from start to goal\n";
        // auto result = Plan(g, obs, start, goal);
        auto result = AstarPlan(g, obs, start, goal);
        std::cout << "Plan complete\n";
        viz.vizGraph(g);
        viz.vizPath(result.first, g);
        viz.vizGraph(g_evaluated, "evaluated");
        viz.vizGraph(g_all_valid, "all_edges");

        viz.vizObstacles(obs, -(double)depth/5);
        viz.vizText("Start", 0, 0, 0, "start");
        viz.vizText("Goal", 1, 1, 1, "goal");
        viz.vizText("Horizontal Edge Cost: (C-space) distance", 2, 0.5, -0.1, "edge cost");
        viz.vizText("Vertical Edge Cost: 0 ", 3, 0.5, -0.2, "vertical edge cost");
        viz.vizText("Heuristic: (C-space) distance * 2^depth", 4, 0.5, -0.3, "heuristic");
        std::cout << "Path: " << PrettyPrint::PrettyPrint(result.first) << "\n";
        std::cout << "Path cost: " << result.second << "\n";

        if(record)
        {
            record = false;
            std::string filename = "sim_timing_" + arc_helpers::GetCurrentTimeAsString();
            PROFILE_WRITE_SUMMARY_FOR_ALL(filename);
            PROFILE_WRITE_ALL_FEWER_THAN(filename, 10000);

        }
        
        arc_helpers::WaitForInput();
    }
    
    return 0;
}
