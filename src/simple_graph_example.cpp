#include "halton_graph.hpp"
#include "graph_visualization.hpp"
#include "2d_obstacles.hpp"

#include <ros/ros.h>


/*
 *   This is a simple example showing how to make a graph (manually), plan, and display the results
 *
 */


RDiscGraph makeGrid()
{
    RDiscGraph g(1.1/10);
    
    for(int i=0; i<=10; i++)
    {
        for(int j=0; j<=10; j++)
        {
            g.addVertexAndEdges(std::vector<double>{(double)i/10, (double)j/10});
        }
    }
    return g;
}

void setScene(Obstacles2D::Obstacles &obs)
{
    using namespace Obstacles2D;
    using namespace Eigen;

    obs.obs.push_back(std::make_shared<Rect>(-0.1, 0.05, 0.8, 0.1));
    obs.obs.push_back(std::make_shared<Rect>(0.45, 0.4, 0.9, 0.45));
    obs.obs.push_back(std::make_shared<Rect>(0.85, 0.4, 0.9, 0.9));
    obs.obs.push_back(std::make_shared<Rect>(0.8, 0.8, 1.1, 0.9));

    // auto cp1 = std::make_shared<ConvexPolygon>(std::vector<Vector2d>{
    //         Vector2d(-0.1, 0.4),
    //             Vector2d(-0.1, 0.45),
    //             Vector2d( 0.3, 0.45),
    //             Vector2d( 0.35, 0.425),
    //             Vector2d( 0.3, 0.4),
    //             });
    // obs.obs.push_back(cp1);

    auto cp2 = std::make_shared<ConvexPolygon>(std::vector<Vector2d>{
            Vector2d(0.2, 0.6),
                Vector2d(0.45, 0.4),
                Vector2d( 0.45, 0.6),
                });
    obs.obs.push_back(cp2);
}


bool checkEdge(GraphD &g,
               arc_dijkstras::GraphEdge &e, const Obstacles2D::Obstacles &obs)
{
    using namespace arc_dijkstras;
    if(e.getValidity() == EDGE_VALIDITY::VALID)
    {
        return true;;
    }
    if(e.getValidity() == EDGE_VALIDITY::INVALID)
    {
        return false;
    }
    
    std::vector<double> q1 = g.getNode(e.getFromIndex()).getValue();
    std::vector<double> q2 = g.getNode(e.getToIndex()).getValue();

    bool validity = obs.isValid(q1, q2);
    e.setValidity(validity ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID);
    return validity;
}


inline arc_helpers::AstarResult AstarPlan(RDiscGraph &g,
                                          const Obstacles2D::Obstacles &obs,
                                          const std::vector<double> &start,
                                          const std::vector<double> &goal)
{
    PROFILE_START("AStar plan");
    using namespace arc_dijkstras;

    int64_t from_node = g.getNearest(start);
    int64_t goal_node = g.getNearest(goal);

    const auto edge_check_fun = [&obs](GraphD &g, GraphEdge &e)
        {
            PROFILE_START("Edge Evaluation");
            auto result = checkEdge(g, e, obs);
            PROFILE_RECORD("Edge Evaluation");

            return result;
        };


    const auto distance_function = [&] (const GraphD& search_graph, 
                                        const GraphEdge& edge)
        {
            UNUSED(search_graph);
            return edge.getWeight();
        };

    const auto heuristic_function = [&g] (const std::vector<double> &q1,
                                          const std::vector<double> &q2)
        {
            return EigenHelpers::Distance(q1, q2);
        };

    auto result = AstarLogging<std::vector<double>>::PerformLazyAstar(g, from_node, goal_node,
                                                                      edge_check_fun,
                                                                      distance_function,
                                                                      heuristic_function,
                                                                      true);
    PROFILE_RECORD("AStar plan");
    return result;
}






void run(GraphVisualizer &viz)
{
    RDiscGraph graph = makeGrid();
    Obstacles2D::Obstacles obs;
    setScene(obs);

    std::vector<double> start{0,0};
    std::vector<double> goal{1,1};

    viz.vizGraph(graph, "graph");
    viz.vizObstacles(obs);
    
    arc_helpers::WaitForInput("Press enter to plan");
    auto result = AstarPlan(graph, obs, start, goal);
    viz.vizPath(result.first, graph);
    
}
 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;
    GraphVisualizer viz(n);
    ros::Duration(1.0).sleep();
    run(viz);
}
