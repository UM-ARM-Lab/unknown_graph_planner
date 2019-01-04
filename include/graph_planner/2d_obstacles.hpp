#ifndef OBSTACLES2D_HPP
#define OBSTACLES2D_HPP


// #include "graph.hpp"
#include "halton_graph.hpp"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <arc_utilities/eigen_helpers.hpp>


#define EDGE_DISCRETIZATION 0.001

namespace Obstacles2D
{
    
    class Obstacle
    {
    public:
        virtual bool isValid(std::vector<double> q) const = 0;
        virtual visualization_msgs::Marker toMarker() const = 0;
    };
    
    class Rect : public Obstacle
    {
    public:
        double x1, y1, x2, y2;
        
        Rect(double x1_, double y1_, double x2_, double y2_):
            x1(x1_), y1(y1_), x2(x2_), y2(y2_)
        {
        };

        bool isValid(std::vector<double> q) const
        {
            return !(q[0] > x1 && q[0] < x2 && q[1] > y1 && q[1] < y2);
        };

        visualization_msgs::Marker toMarker() const
        {
            visualization_msgs::Marker cube;
            cube.header.frame_id = "/graph_frame";
            cube.type = visualization_msgs::Marker::CUBE;
            cube.pose.orientation.w = 1;
            double w = x2-x1;
            double h = y2-y1;
            cube.scale.x = w;
            cube.scale.y = h;
            cube.scale.z = 0.01;
            cube.pose.position.x = x1+w/2;
            cube.pose.position.y = y1+h/2;
            cube.color.a = 0.9;
            cube.color.r = 1.0;
            return cube;
        }
    };

    class Obstacles
    {
    public:
        Obstacles() {};

        std::vector<Obstacle*> obs;

        visualization_msgs::MarkerArray toMarkerArray() const
        {
            visualization_msgs::MarkerArray m;
            for(auto ob: obs)
            {
                m.markers.push_back(ob->toMarker());
            }
            return m;
        };

        bool isValid(std::vector<double> q) const
        {
            for(auto ob: obs)
            {
                if(!ob->isValid(q))
                {
                    return false;
                }
            }
            return true;
        }


        bool isValid(std::vector<double> q1, std::vector<double> q2) const
        {
            std::vector<double> q = q1;
            double d = EigenHelpers::Distance(q1, q2);
            // double d = distance(q1, q2);
            int num_point_checks = (int)(d/EDGE_DISCRETIZATION) + 1;
            double dx = (q2[0] - q1[0])/num_point_checks;
            double dy = (q2[1] - q1[1])/num_point_checks;
            for(int i=0; i<= num_point_checks; i++)
            {
                if(!isValid(q))
                {
                    return false;
                }
                q[0] += dx;
                q[1] += dy;
            }
            return true;
        }

        // bool isValid(Edge e, const Graph &g) const
        // {
        //     std::vector<double> q1 = g.V[e.v1_ind].q;
        //     std::vector<double> q2 = g.V[e.v2_ind].q;
        //     return isValid(q1, q2);
        // }

        bool isValid(arc_dijkstras::GraphEdge e, const HaltonGraph &g) const
        {
            std::vector<double> q1 = g.GetNodeImmutable(e.GetFromIndex()).GetValueImmutable();
            std::vector<double> q2 = g.GetNodeImmutable(e.GetToIndex()).GetValueImmutable();
            // std::vector<double> q2 = g.V[e.v2_ind].q;
            return isValid(q1, q2);
        }

        /*
         *  Returns true if there is a valid edge in $g$ between
         *   vertices at inds v1_ind to v2_ind
         */
        // bool isValid(int v1_ind, int v2_ind, Graph &g)
        // {
        //     Node v1 = g.V[v1_ind];
        //     for(int edge_ind: v1.edge_inds)
        //     {
        //         Edge e = g.E[edge_ind];
        //         if(e.v1_ind != v2_ind && e.v2_ind != v2_ind)
        //         {
        //             continue;
        //         }

        //         if(e.validity == EDGE_VALIDITY::INVALID)
        //         {
        //             return false;
        //         }

        //         if(e.validity == EDGE_VALIDITY::VALID)
        //         {
        //             return true;
        //         }
                
        //         e.validity = isValid(e, g) ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID;
        //         return e.validity == EDGE_VALIDITY::VALID;
        //     }
        //     return false;
        // }
    };
}

#endif
