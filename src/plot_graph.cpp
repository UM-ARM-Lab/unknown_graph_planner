#include "cspace_halton_graph.hpp"
#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/timing.hpp>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "2d_obstacles.hpp"
#include "lazysp.hpp"
#include <utility>

typedef std::vector<visualization_msgs::Marker> GraphMarker;

// visualization_msgs::Marker toVisualizationMsg(Graph g)
// {
//     visualization_msgs::Marker points;
//     points.header.frame_id = "/graph_frame";
//     points.type = visualization_msgs::Marker::POINTS;
//     points.pose.orientation.w = 1.0;
//     points.scale.x = 0.01;
//     points.scale.x = 0.01;
//     points.color.a = 1.0;
//     points.color.g = 1.0;

//     for(auto v:g.V)
//     {
//         geometry_msgs::Point p;
//         p.x = v.q[0];
//         p.y = v.q[1];
//         points.points.push_back(p);
//     }

//     return points;
// }

GraphMarker toVisualizationMsg(const CSpaceHaltonGraph &g)
{
    visualization_msgs::Marker valid_lines, invalid_lines, unknown_lines;
    valid_lines.header.frame_id = "/graph_frame";
    valid_lines.type = visualization_msgs::Marker::LINE_LIST;
    valid_lines.pose.orientation.w = 1.0;
    valid_lines.scale.x = 0.0015;
    valid_lines.color.a = 0.9;
    invalid_lines.header.frame_id = "/graph_frame";
    invalid_lines.type = visualization_msgs::Marker::LINE_LIST;
    invalid_lines.pose.orientation.w = 1.0;
    invalid_lines.scale.x = 0.0015;
    invalid_lines.color.a = 0.3;
    invalid_lines.color.r = 0.9;
    unknown_lines.header.frame_id = "/graph_frame";
    unknown_lines.type = visualization_msgs::Marker::LINE_LIST;
    unknown_lines.pose.orientation.w = 1.0;
    unknown_lines.scale.x = 0.0005;
    unknown_lines.color.a = 0.1;
    // lines.color.g = 1.0;

    for(const auto n:g.GetNodesImmutable())
    {
        for(const auto e:n.GetOutEdgesImmutable())
        {
            // if(e.validity == EDGE_VALIDITY::INVALID)
            // {
            //     continue;
            // }

        
            geometry_msgs::Point p1;
            
            const auto q1 = g.GetNodeImmutable(e.GetFromIndex()).GetValueImmutable();
            p1.x = q1[0];
            p1.y = q1[1];
        
            geometry_msgs::Point p2;
            const auto q2 = g.GetNodeImmutable(e.GetToIndex()).GetValueImmutable();
            // auto v2 = g.V[e.v2_ind];
            p2.x = q2[0];
            p2.y = q2[1];

            switch(e.GetValidity())
            {
            case EDGE_VALIDITY::UNKNOWN:
                unknown_lines.points.push_back(p1);
                unknown_lines.points.push_back(p2);
                break;
            case EDGE_VALIDITY::VALID:
                valid_lines.points.push_back(p1);
                valid_lines.points.push_back(p2);
                break;
            case EDGE_VALIDITY::INVALID:
                invalid_lines.points.push_back(p1);
                invalid_lines.points.push_back(p2);
                break;            
            }
        }

    }

    // visualization_msgs::MarkerArray graph_lines;
    
    return std::vector<visualization_msgs::Marker>{valid_lines, unknown_lines, invalid_lines};
    // graph_lines.markers.push_back(valid_lines);
    // graph_lines.markers.push_back(unknown_lines);
    // return graph_lines;
}


visualization_msgs::Marker toVisualizationMsg(std::vector<int64_t> path, const CSpaceHaltonGraph &g)
{
    visualization_msgs::Marker lines;
    lines.header.frame_id = "/graph_frame";
    lines.type = visualization_msgs::Marker::LINE_STRIP;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = 0.007;
    lines.color.a = 1.0;
    lines.color.b = 1.0;

    for(auto ind: path)
    {
        geometry_msgs::Point p1;
        const auto v = g.GetNodeImmutable(ind).GetValueImmutable();
        p1.x = v[0];
        p1.y = v[1];
        lines.points.push_back(p1);
    }
    return lines;
}


visualization_msgs::Marker pointsToVisualizationMsg(std::vector<int> ps, const CSpaceHaltonGraph &g)
{
    visualization_msgs::Marker points;
    points.header.frame_id = "/graph_frame";
    points.type = visualization_msgs::Marker::POINTS;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.03;
    points.scale.x = 0.03;
    points.color.a = 1.0;
    points.color.b = 1.0;

    for(auto ind:ps)
    {
        const auto v = g.GetNodeImmutable(ind).GetValueImmutable();
        geometry_msgs::Point p;
        p.x = v[0];
        p.y = v[1];
        points.points.push_back(p);
    }

    return points;
}



Obstacles2D::Obstacles makeObstacles()
{
    Obstacles2D::Obstacles o;
    Obstacles2D::Rect* r = new Obstacles2D::Rect(0.1,0.5,0.9,0.8);
    o.obs.push_back(r);
    return o;
}


void validateEdges(Graph &g, Obstacles2D::Obstacles &obs)
{
    for(auto &e: g.E)
    {
        e.validity = obs.isValid(e, g) ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID;
    }
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

    // Graph g(1000);
    CSpaceHaltonGraph g(1000, 0.1);
    int start = 0;
    int end = 5;
    std::vector<int> points{start, end};

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
