#ifndef GRAPH_VISUALIZATION
#define GRAPH_VISUALIZATION

#include "visualization_msgs/Marker.h"
#include <arc_utilities/dijkstras.hpp>
#include "ctp.hpp"


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

GraphMarker toVisualizationMsg(const GraphD &g)
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


visualization_msgs::Marker toVisualizationMsg(std::vector<int64_t> path, const GraphD &g)
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


visualization_msgs::Marker pointsToVisualizationMsg(std::vector<int> ps, const GraphD &g)
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


visualization_msgs::Marker toVisualizationMsg(CTP::Agent &a, const GraphD &g)
{
    std::vector<int> p{a.current_node, a.goal_node};
    return pointsToVisualizationMsg(p, g);
}

#endif
