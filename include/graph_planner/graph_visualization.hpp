#ifndef GRAPH_VISUALIZATION
#define GRAPH_VISUALIZATION

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <arc_utilities/dijkstras.hpp>
#include <iomanip>
#include "ctp.hpp"
#include "ctp_worlds.hpp"
#include "increasing_density_grid.hpp"
#include "2d_obstacles.hpp"


typedef std::vector<visualization_msgs::Marker> GraphMarker;



static inline std_msgs::ColorRGBA colorLookup(std::string color)
{
    std_msgs::ColorRGBA cm;
    cm.a = 1.0;
    if(color=="blue")
    {
        cm.b=1.0;
    }
    else if(color=="red")
    {
        cm.r=1.0;
    }
    else if(color=="green")
    {
        cm.g = 1.0;
    }
    else if(color=="purple")
    {
        cm.b = 1.0; cm.r=1.0;
    }
    else if(color=="clear blue")
    {
        cm.b=1.0;
        cm.a=0.3;
    }
    else if(color=="clear red")
    {
        cm.r = 1.0;
        cm.a = 0.05;
    }
    else if (color=="black")
    {
        cm.r = 0.0; cm.g = 0.0; cm.b = 0.0;
    }
    return cm;
}


static inline geometry_msgs::Point to3DPoint(const GraphD &g, int64_t node_ind)
{
    const std::vector<double> &node = g.getNode(node_ind).getValue();
    geometry_msgs::Point p;
    p.x = node[0];
    p.y = node[1];
    return p;
}

static inline geometry_msgs::Point to3DPoint(const SelectiveDensificationGraph &g, int64_t node_ind)
{
    DepthNode node = g.getNodeValue(node_ind);
    geometry_msgs::Point p;
    p.x = node.q[0];
    p.y = node.q[1];
    p.z = -(double)node.depth / 3.0;
    // p.z = 1.0 / std::pow(2, (double)node.depth) - 1;
    return p;
}



template <typename T>
static inline GraphMarker toVisualizationMsg(const T &g, std::string name="graph")
{
    visualization_msgs::Marker valid_lines, invalid_lines, unknown_lines;
    valid_lines.header.frame_id = "/graph_frame";
    valid_lines.type = visualization_msgs::Marker::LINE_LIST;
    valid_lines.ns = name;
    valid_lines.pose.orientation.w = 1.0;
    valid_lines.scale.x = 0.002;
    valid_lines.color.a = 0.9;
    invalid_lines.header.frame_id = "/graph_frame";
    invalid_lines.type = visualization_msgs::Marker::LINE_LIST;
    invalid_lines.ns = name;
    invalid_lines.pose.orientation.w = 1.0;
    invalid_lines.scale.x = 0.0055;
    invalid_lines.color.a = 0.3;
    invalid_lines.color.r = 0.9;
    unknown_lines.header.frame_id = "/graph_frame";
    unknown_lines.type = visualization_msgs::Marker::LINE_LIST;
    unknown_lines.ns = name;
    unknown_lines.pose.orientation.w = 1.0;
    unknown_lines.scale.x = 0.0002;
    unknown_lines.color.a = 0.1;
    // lines.color.g = 1.0;

    for(const auto n:g.getNodes())
    {
        for(const auto e:n.getOutEdges())
        {
            geometry_msgs::Point p1 = to3DPoint(g, e.getFromIndex());
            geometry_msgs::Point p2 = to3DPoint(g, e.getToIndex());

            switch(e.getValidity())
            {
            case arc_dijkstras::EDGE_VALIDITY::UNKNOWN:
                unknown_lines.points.push_back(p1);
                unknown_lines.points.push_back(p2);
                break;
            case arc_dijkstras::EDGE_VALIDITY::VALID:
                valid_lines.points.push_back(p1);
                valid_lines.points.push_back(p2);
                break;
            case arc_dijkstras::EDGE_VALIDITY::INVALID:
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


template <typename T>
static inline visualization_msgs::Marker toVisualizationMsg(std::vector<int64_t> path, const T &g,
                                                           int id=0, std::string color="blue")
{
    visualization_msgs::Marker lines;
    lines.header.frame_id = "/graph_frame";
    lines.type = visualization_msgs::Marker::LINE_STRIP;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = 0.010;
    lines.color = colorLookup(color);
    lines.id = id;

    for(auto ind: path)
    {
        lines.points.push_back(to3DPoint(g, ind));
    }
    return lines;
}


static inline visualization_msgs::Marker pointsToVisualizationMsg(std::vector<CTP::Location> ps, const GraphD &g)
{
    visualization_msgs::Marker points;
    points.header.frame_id = "/graph_frame";
    points.type = visualization_msgs::Marker::POINTS;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.015;
    points.scale.x = 0.03;
    points.color.a = 1.0;
    points.color.b = 1.0;

    for(auto ind:ps)
    {
        const auto v = g.getNode(ind).getValue();
        geometry_msgs::Point p;
        p.x = v[0];
        p.y = v[1];
        points.points.push_back(p);
    }

    return points;
}


static inline visualization_msgs::Marker toVisualizationMsg(CTP::Agent &a, const GraphD &g)
{
    std::vector<CTP::Location> p{a.current_node, a.goal_node};
    return pointsToVisualizationMsg(p, g);
}

static inline visualization_msgs::MarkerArray toVisualizationMsg(std::vector<std::string> texts,
                                                                 std::vector<std::vector<double>> loc)
{
    visualization_msgs::MarkerArray tms;
    for(size_t i=0; i<40; i++)
    {
        visualization_msgs::Marker tm;
        tm.id = i;
        tm.header.frame_id = "/graph_frame";
        tm.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
        tm.text = "";
        tm.pose.position.x = 1000;
        tm.pose.position.y = 1000;
        
        if(i < texts.size())
        {
            tm.text = texts[i];
            tm.pose.position.x = loc[i][0];
            tm.pose.position.y = loc[i][1];
        }
        tm.scale.z = 0.03;
        tm.color.a = 1.0;

        tms.markers.push_back(tm);
    }
    return tms;
}


class GraphVisualizer
{
public:
    ros::Publisher graph_valid_pub;
    ros::Publisher graph_unknown_pub;
    ros::Publisher graph_invalid_pub;
    ros::Publisher path_pub;
    ros::Publisher points_pub;
    ros::Publisher obs_pub;
    ros::Publisher text_pub;
    
    GraphVisualizer(ros::NodeHandle &n)
    {
        graph_valid_pub = n.advertise<visualization_msgs::Marker>("valid_graph", 10);
        graph_unknown_pub = n.advertise<visualization_msgs::Marker>("unknown_graph", 10);
        graph_invalid_pub = n.advertise<visualization_msgs::Marker>("invalid_graph", 10);
        path_pub= n.advertise<visualization_msgs::Marker>("path", 10);
        points_pub = n.advertise<visualization_msgs::Marker>("points", 10);
        obs_pub = n.advertise<visualization_msgs::MarkerArray>("obstacles", 10);
        text_pub = n.advertise<visualization_msgs::MarkerArray>("text", 10);
    }


    template <typename T>
    void vizGraph(const T &g, std::string name="graph")
    {
        GraphMarker gm = toVisualizationMsg(g, name);
        graph_valid_pub.publish(gm[0]);
        graph_unknown_pub.publish(gm[1]);
        graph_invalid_pub.publish(gm[2]);
    }

    void vizAgent(CTP::Agent &a, const GraphD &g)
    {
        points_pub.publish(toVisualizationMsg(a, g));
    }

    void vizPoints(const std::vector<CTP::Location> &p, const GraphD &g)
    {
        points_pub.publish(pointsToVisualizationMsg(p, g));
    }


    template <typename BeliefGraph>
    void vizCtp(CTP::NltpProblem<BeliefGraph> &ctp)
    {
        vizGraph(ctp.belief_graph);
        vizAgent(ctp.agent, ctp.belief_graph);
        vizGraph(ctp.true_graph, "true_graph");
    }

    void vizCtp(CTP::CtpProblem<CTP::BctpGrid> &ctp)
    {
        vizCtp<CTP::BctpGrid>(ctp);
        obs_pub.publish(ctp.belief_graph.getObstacle().toMarkerArray());
    }


    void vizObstacles(const Obstacles2D::Obstacles &obs, double z_scale = 0.01, std::string ns="",
                      std_msgs::ColorRGBA color = colorLookup("red"))
    {
        obs_pub.publish(obs.toMarkerArray(z_scale, ns, color));
    }


    template <typename T>
    void vizPath(const std::vector<int64_t> &path, const T &g, int id = 0,
                 std::string color = "clear blue")
    {
        path_pub.publish(toVisualizationMsg(path, g, id, color));
    }

    void vizTitle(std::string text)
    {
        vizText(text, 1000, -0.1, 0.5);
    }

    void vizText(std::string text, int id, double x, double y, std::string ns = "text")
    {
        visualization_msgs::Marker m;
        m.id = id;
        m.ns = ns;
        m.header.frame_id = "/graph_frame";
        m.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.text = text;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.scale.z = 0.05;
        m.color.a = 1.0;
        visualization_msgs::MarkerArray ma;
        ma.markers.push_back(m);
        text_pub.publish(ma);
    }

    void vizTexts(std::vector<std::string> texts, std::vector<std::vector<double>> points)
    {
        text_pub.publish(toVisualizationMsg(texts, points));
    }
            
    void vizDoubles(std::vector<double> vals, std::vector<std::vector<double>> points)
    {
        std::vector<std::string> texts;
        for(double d: vals)
        {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << d;
            texts.push_back(ss.str());
            std::cout << "Visualizing " << texts.back() << "\n";
        }
        for(auto p:points)
        {
            std::cout << "(" << p[0] << ", " << p[1] << "), ";
        }
        std::cout << "\n";
        
        text_pub.publish(toVisualizationMsg(texts, points));
    }
};


#endif
