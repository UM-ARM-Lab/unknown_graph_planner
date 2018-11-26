#include "graph.hpp"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "a_star.hpp"
#include "2d_obstacles.hpp"
#include "lazysp.hpp"
#include <utility>

typedef std::pair<visualization_msgs::Marker, visualization_msgs::Marker> graph_marker;

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

graph_marker toVisualizationMsg(Graph g)
{
    visualization_msgs::Marker valid_lines;
    visualization_msgs::Marker unknown_lines;
    valid_lines.header.frame_id = "/graph_frame";
    valid_lines.type = visualization_msgs::Marker::LINE_LIST;
    valid_lines.pose.orientation.w = 1.0;
    valid_lines.scale.x = 0.0015;
    valid_lines.color.a = 0.9;
    unknown_lines.header.frame_id = "/graph_frame";
    unknown_lines.type = visualization_msgs::Marker::LINE_LIST;
    unknown_lines.pose.orientation.w = 1.0;
    unknown_lines.scale.x = 0.0005;
    unknown_lines.color.a = 0.1;
    // lines.color.g = 1.0;

    for(auto e:g.E)
    {
        // if(e.validity == EDGE_VALIDITY::INVALID)
        // {
        //     continue;
        // }

        
        geometry_msgs::Point p1;
        auto v1 = g.V[e.v1_ind];
        p1.x = v1.q[0];
        p1.y = v1.q[1];
        
        geometry_msgs::Point p2;
        auto v2 = g.V[e.v2_ind];
        p2.x = v2.q[0];
        p2.y = v2.q[1];

        if(e.validity == EDGE_VALIDITY::UNKNOWN)
        {
            unknown_lines.points.push_back(p1);
            unknown_lines.points.push_back(p2);
        }
        if(e.validity == EDGE_VALIDITY::VALID)
        {
            valid_lines.points.push_back(p1);
            valid_lines.points.push_back(p2);
        }

    }

    visualization_msgs::MarkerArray graph_lines;
    return std::make_pair(valid_lines, unknown_lines);
    // graph_lines.markers.push_back(valid_lines);
    // graph_lines.markers.push_back(unknown_lines);
    // return graph_lines;
}


visualization_msgs::Marker toVisualizationMsg(std::vector<int> path, Graph g)
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
        auto v = g.V[ind];
        p1.x = v.q[0];
        p1.y = v.q[1];
        lines.points.push_back(p1);
    }
    return lines;
}


visualization_msgs::Marker pointsToVisualizationMsg(std::vector<int> ps, Graph g)
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
        auto v = g.V[ind];
        geometry_msgs::Point p;
        p.x = v.q[0];
        p.y = v.q[1];
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
    ros::Publisher path_pub = n.advertise<visualization_msgs::Marker>("path", 10);
    ros::Publisher points_pub = n.advertise<visualization_msgs::Marker>("points", 10);
    ros::Publisher obs_pub = n.advertise<visualization_msgs::MarkerArray>("obs", 10);
    ros::Rate r(10);

    Obstacles2D::Obstacles obs = makeObstacles();

    Graph g(1000);
    int start = 0;
    int end = 5;
    std::vector<int> points{start, end};

    // validateEdges(g, obs);


    while(ros::ok())
    {
        std::vector<int> path = A_star(points[0], points[1], g);
        
        // forwardLazyCheck(path, g, obs);
        points[0] = forwardMove(path, g, obs);
        
        graph_marker gm = toVisualizationMsg(g);
        graph_valid_pub.publish(gm.first);
        graph_unknown_pub.publish(gm.second);
        path_pub.publish(toVisualizationMsg(path, g));
        points_pub.publish(pointsToVisualizationMsg(points, g));
        obs_pub.publish(obs.toMarkerArray());
        // r.sleep();
    }

    
    
    return 0;
}
