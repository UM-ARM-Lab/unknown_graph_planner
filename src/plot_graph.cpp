#include "graph.hpp"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "a_star.hpp"


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

visualization_msgs::Marker toVisualizationMsg(Graph g)
{
    visualization_msgs::Marker lines;
    lines.header.frame_id = "/graph_frame";
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = 0.001;
    lines.color.a = 0.5;
    // lines.color.g = 1.0;

    for(auto e:g.E)
    {
        if(e.validity == EDGE_VALIDITY::INVALID)
        {
            continue;
        }
        geometry_msgs::Point p1;
        auto v1 = g.V[e.v1_ind];
        p1.x = v1.q[0];
        p1.y = v1.q[1];
        lines.points.push_back(p1);
        
        geometry_msgs::Point p2;
        auto v2 = g.V[e.v2_ind];
        p2.x = v2.q[0];
        p2.y = v2.q[1];
        lines.points.push_back(p2);
    }

    return lines;
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




int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;

    ros::Publisher graph_pub = n.advertise<visualization_msgs::Marker>("graph_points", 10);
    ros::Publisher path_pub = n.advertise<visualization_msgs::Marker>("path", 10);
    ros::Publisher points_pub = n.advertise<visualization_msgs::Marker>("points", 10);
    ros::Rate r(30);

    Graph g(1000);
    int start = 0;
    int end = 5;
    std::vector<int> points{start, end};
    std::vector<int> path = A_star(start, end, g);

    while(ros::ok())
    {
        graph_pub.publish(toVisualizationMsg(g));
        path_pub.publish(toVisualizationMsg(path, g));
        points_pub.publish(pointsToVisualizationMsg(points, g));
        r.sleep();
    }

    
    
    return 0;
}
