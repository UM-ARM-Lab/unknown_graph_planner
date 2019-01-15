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
#include "ctp_worlds.hpp"
#include "ctp_heuristics.hpp"

using namespace CTP;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;

    std::mt19937 rng;
    rng.seed(std::random_device()());
    GraphVisualizer viz(n);
    ros::Rate r(0.5);

    std::string filepath = "/home/bradsaund/catkin_ws/src/graph_planner/graphs/CTP_2D_1000.graph";
    
    CtpPitfall g;
    // g.saveToFile(filepath);
    // HaltonGraph g(filepath);
    
    Agent agent(0, 1);
    int lookahead = 1;
    NltpProblem<CtpPitfall> ctp(g, g.sampleInstance(rng), agent, lookahead);
    


    ros::Duration(1).sleep();
    
    viz.vizCtp(ctp);
    auto msg = ctp.belief_graph.getEdgeMsgs();
    viz.vizTexts(msg.first, msg.second);

    arc_helpers::WaitForInput();

    double cost = 0.0;
    

    while(ros::ok() && !ctp.solved())
    {
        Action a = OMT(ctp, viz);
        // Action a = POMT(ctp, 100.0, viz);

        cost += ctp.move(a);

        viz.vizCtp(ctp);

        arc_helpers::WaitForInput();
    }
    viz.vizPath(std::vector<Location>{}, ctp.true_graph);

    std::cout << "Agent reached goal node " << ctp.agent.current_node << " with cost of " << cost << "\n";;
    
    return 0;
}
