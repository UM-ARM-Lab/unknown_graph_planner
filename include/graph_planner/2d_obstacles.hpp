#ifndef OBSTACLES2D_HPP
#define OBSTACLES2D_HPP


// #include "graph.hpp"
#include "halton_graph.hpp"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <arc_utilities/eigen_helpers.hpp>
#define DEFAULT_COLOR rosColor(1.0, 0, 0, 0.9)


#define EDGE_DISCRETIZATION 0.001


inline std_msgs::ColorRGBA rosColor(double r, double g, double b, double a)
{
    std_msgs::ColorRGBA c;
    c.r = r;
    c.b = b;
    c.g = g;
    c.a = a;
    return c;
}


namespace Obstacles2D
{
    class Obstacle
    {
    public:
        Obstacle()
        {}
        
        virtual bool isValid(const std::vector<double> &q) const = 0;
        virtual visualization_msgs::MarkerArray toMarkerArray(double z_scale, std::string ns="",
                                                              std_msgs::ColorRGBA color=DEFAULT_COLOR) const = 0;


    };


    /**
     * Hack class to make rviz markers disappear
     */
    class Empty : public Obstacle
    {
        bool isValid(const std::vector<double> &q) const override
        {
            return true;
        };
        
        visualization_msgs::MarkerArray
        toMarkerArray(double z_scale = 0.01,  std::string ns="",
                      std_msgs::ColorRGBA color=DEFAULT_COLOR) const override
        {
            visualization_msgs::Marker cube;
            cube.header.frame_id = "/graph_frame";
            cube.type = visualization_msgs::Marker::CUBE;
            cube.pose.orientation.w = 1;
            cube.ns = ns;
            cube.scale.x = 0.001;
            cube.scale.y = 0.001;
            cube.scale.z = 0.001;
            cube.pose.position.x = 10000;
            cube.pose.position.y = 10000;


            visualization_msgs::MarkerArray arr;
            arr.markers.push_back(cube);
            return arr;
        }
    };
    
    class Rect : public Obstacle
    {
    public:
        double x1, y1, x2, y2;
        
        Rect(double x1, double y1, double x2, double y2):
            x1(x1), y1(y1), x2(x2), y2(y2)
        {
        };

        Rect(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2) : x1(p1.x()), y1(p1.y()),
                                                                     x2(p2.x()), y2(p2.y()) {};

        bool isValid(const std::vector<double> &q) const override
        {
            return !(q[0] > x1 && q[0] < x2 && q[1] > y1 && q[1] < y2);
        };

        visualization_msgs::MarkerArray
        toMarkerArray(double z_scale = 0.01,  std::string ns="",
                      std_msgs::ColorRGBA color=DEFAULT_COLOR) const override
        {
            visualization_msgs::Marker cube;
            cube.header.frame_id = "/graph_frame";
            cube.type = visualization_msgs::Marker::CUBE;
            cube.pose.orientation.w = 1;
            double w = x2-x1;
            double h = y2-y1;
            cube.scale.x = w;
            cube.scale.y = h;
            cube.scale.z = std::abs(z_scale);
            cube.pose.position.x = x1+w/2;
            cube.pose.position.y = y1+h/2;
            cube.pose.position.z = z_scale / 2;
            cube.color = color;
            cube.ns = ns;

            visualization_msgs::MarkerArray arr;
            arr.markers.push_back(cube);
            return arr;
        }
    };

    class ConvexPolygon : public Obstacle
    {
    public:
        std::vector<Eigen::Vector2d> points;
        ConvexPolygon(const std::vector<Eigen::Vector2d> &points): points(points)
        {
            if(!EigenHelpers::CloseEnough(points.front(), points.back(),
                                          EDGE_DISCRETIZATION/2))
            {
                this->points.push_back(points.front());
            }
            assert(this->points.size() >= 4 && "Polygon must have 3 distinct vertices");
        };

        bool isValid(const std::vector<double> &q) const override
        {
            auto isRightSide = [](const std::vector<double> &q_test,
                                  const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
            {
                auto a = Eigen::Vector2d(q_test[0], q_test[1]) -  p1;
                auto b = p2 - p1;
                return (a[0]*b[1] - a[1]*b[0]) > 0;
            };
            
            bool first_side_is_right = isRightSide(q, points[0], points[1]);
            for(size_t i=1; i<points.size()-1; i++)
            {
                if(first_side_is_right != isRightSide(q, points[i], points[i+1]))
                {
                    return true;
                }
            }
            return false;
        }

        
        visualization_msgs::Marker edgeToMarker(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2,
                                                double z_scale, std_msgs::ColorRGBA color) const
        {
            visualization_msgs::Marker cube;
            cube.header.frame_id = "/graph_frame";
            cube.type = visualization_msgs::Marker::CUBE;

            cube.scale.x = (p2-p1).norm();;
            cube.scale.y = 0.01;
            cube.scale.z = std::abs(z_scale);
            cube.pose.position.x = (p1.x() + p2.x())/2;
            cube.pose.position.y = (p1.y() + p2.y())/2;
            cube.pose.position.z = z_scale / 2;

            Eigen::Vector2d vec = p2 - p1;
            double cos_theta = vec.x()/vec.norm();
            double w = std::sqrt((1 - cos_theta)/2);
            if(vec.y() > 0)
            {
                w *= -1;
            }
            cube.pose.orientation.w = w;
            cube.pose.orientation.z = std::sqrt(1 - w*w);
            cube.color = color;
            return cube;
        }
        
        visualization_msgs::MarkerArray
        toMarkerArray(double z_scale = 0.01, std::string ns="",
                      std_msgs::ColorRGBA color=DEFAULT_COLOR) const override
        {

            visualization_msgs::MarkerArray arr;
            for(size_t i=0; i<points.size()-1; i++)
            {
                arr.markers.push_back(edgeToMarker(points[i], points[i+1], z_scale, color));
                arr.markers.back().ns=ns;
            }

            return arr;
        }

        
    };

    class Obstacles
    {
    public:
        Obstacles() {};

        std::vector<std::shared_ptr<Obstacle>> obs;

        visualization_msgs::MarkerArray
        toMarkerArray(double z_scale = 0.01, std::string ns="", std_msgs::ColorRGBA color=DEFAULT_COLOR) const
        {
            visualization_msgs::MarkerArray all_obs;
            int id = 0;
            for(auto ob: obs)
            {
                for(const visualization_msgs::Marker &m:ob->toMarkerArray(z_scale, ns, color).markers)
                {
                    all_obs.markers.push_back(m);
                    all_obs.markers.back().id=id++;
                }
            }
            return all_obs;
        };

        bool isValid(const std::vector<double> &q) const
        {
            for(auto &ob: obs)
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

        double fractionUntilCollision(std::vector<double> q1, std::vector<double> q2) const
        {
            std::vector<double> q = q1;
            double d = EigenHelpers::Distance(q1, q2);
            int num_point_checks = (int)(d/EDGE_DISCRETIZATION) + 1;
            double dx = (q2[0] - q1[0])/num_point_checks;
            double dy = (q2[1] - q1[1])/num_point_checks;
            for(int i=0; i<= num_point_checks; i++)
            {
                if(!isValid(q))
                {
                    if(i==0)
                    {
                        return 0;
                    }
                    return (double)(i-1) / (double)num_point_checks;
                }
                q[0] += dx;
                q[1] += dy;
            }
            return 1.0;
        }

        // bool isValid(Edge e, const Graph &g) const
        // {
        //     std::vector<double> q1 = g.V[e.v1_ind].q;
        //     std::vector<double> q2 = g.V[e.v2_ind].q;
        //     return isValid(q1, q2);
        // }

        bool isValid(arc_dijkstras::GraphEdge e, const HaltonGraph &g) const
        {
            std::vector<double> q1 = g.getNode(e.getFromIndex()).getValue();
            std::vector<double> q2 = g.getNode(e.getToIndex()).getValue();
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
