#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "gcopter/trajectory.hpp"

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
using namespace std;


// Visualizer for the planner
class Visualizer
{
private:
    // config contains the scale for some markers
    ros::NodeHandle nh;

    // These are publishers for path, waypoints on the trajectory,
    // the entire trajectory, the mesh of free-space polytopes,
    // the edge of free-space polytopes, and spheres for safety radius
    ros::Publisher routePub;
    ros::Publisher wayPointsPub;
    ros::Publisher trajectoryPub;
    ros::Publisher meshPub;
    ros::Publisher edgePub;
    ros::Publisher spherePub;
    ros::Publisher listsPub;
    ros::Publisher arrowsPub;
    ros::Publisher selectedPointPub;

public:
    ros::Publisher speedPub;
    ros::Publisher thrPub;
    ros::Publisher tiltPub;
    ros::Publisher bdrPub;

public:
    Visualizer(ros::NodeHandle &nh_)
        : nh(nh_)
    {
        routePub = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
        wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);
        trajectoryPub = nh.advertise<visualization_msgs::Marker>("/visualizer/trajectory", 10);
        meshPub = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
        edgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);   
        spherePub = nh.advertise<visualization_msgs::Marker>("/visualizer/spheres", 1000);
        listsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/lists", 2);   
        arrowsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/arrows", 2); 
        selectedPointPub = nh.advertise<visualization_msgs::Marker>("/visualizer/selectedPoint", 1000); 
        speedPub = nh.advertise<std_msgs::Float64>("/visualizer/speed", 1000);
        thrPub = nh.advertise<std_msgs::Float64>("/visualizer/total_thrust", 1000);
        tiltPub = nh.advertise<std_msgs::Float64>("/visualizer/tilt_angle", 1000);
        bdrPub = nh.advertise<std_msgs::Float64>("/visualizer/body_rate", 1000);
        
    }

    // Visualize the trajectory and its front-end path
    void displayMarkerList(const std::vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id, bool show_sphere = true, bool show_line = true)
    {
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "odom";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id + 1000;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = color(0);
        sphere.color.g = line_strip.color.g = color(1);
        sphere.color.b = line_strip.color.b = color(2);
        sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
        sphere.scale.x = scale;
        sphere.scale.y = scale;
        sphere.scale.z = scale;
        line_strip.scale.x = scale / 2;
        geometry_msgs::Point pt;
        // cout << "list size\t" << list.size() << "\t" << int(list.size()) << "\t" << line_strip.points.size()<< endl;
        for (int i = 0; i < int(list.size()); i++)
        {
            // cout << "i\t" << i << "\t" << list[i](0) << "\t" << list[i](1) << "\t" << list[i](2) << endl;
            pt.x = list[i](0);
            pt.y = list[i](1);
            pt.z = list[i](2);
            if (show_sphere)
            {
                sphere.points.push_back(pt);
            }
            if (show_line)
            {
                line_strip.points.push_back(pt);
            }
        }
        if (show_sphere)
        {
            // cout << "llllllll"<< endl;
            listsPub.publish(sphere);
            ros::Duration(0.001).sleep();
        }
        if (show_line)
        {
            listsPub.publish(line_strip);
            ros::Duration(0.001).sleep();
        }
    }

    inline void displayVectorArrow(const vector<Eigen::Vector3d> &list, Eigen::Vector4d color, const Eigen::Vector3d xyz, double scale, int id)
    {
        // cout <<"ABCDEF" << endl;
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "odom";
        arrow.header.stamp = ros::Time::now();
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;

        arrow.color.r = color(0);
        arrow.color.g = color(1);
        arrow.color.b = color(2);
        arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
        arrow.scale.x = xyz(0) * scale;
        arrow.scale.y = xyz(1) * scale;
        arrow.scale.z = xyz(2) * scale;
        arrow.pose.orientation.x = 0.0;
        arrow.pose.orientation.y = 0.0;
        arrow.pose.orientation.z = 0.0;
        arrow.pose.orientation.w = 1.0;

        geometry_msgs::Point start, end;
        for (int i = 0; i < int(list.size() / 2); i++)
        {
            start.x = list[2 * i](0);
            start.y = list[2 * i](1);
            start.z = list[2 * i](2);
            end.x = list[2 * i + 1](0);
            end.y = list[2 * i + 1](1);
            end.z = list[2 * i + 1](2);
            arrow.points.clear();
            arrow.points.push_back(start);
            arrow.points.push_back(end);
            arrow.id = i + id;

            arrowsPub.publish(arrow);
            ros::Duration(0.001).sleep();
        }
        return ;
    }

    inline void displaySelectedPos(Eigen::Vector3d pos, Eigen::Vector4d color, const double scale, int id)
    {
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "odom";
        sphere.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.id = id;

        sphere.pose.orientation.w = 1.0;
        sphere.color.r = color(0);
        sphere.color.g = color(1);
        sphere.color.b = color(2);
        sphere.color.a = color(3);
        sphere.scale.x = scale;
        sphere.scale.y = scale;
        sphere.scale.z = scale;
        sphere.pose.position.x = pos(0);
        sphere.pose.position.y = pos(1);
        sphere.pose.position.z = pos(2);

        selectedPointPub.publish(sphere);
        ros::Duration(0.001).sleep();
    }

    // Visualize all spheres with centers sphs and the same radius
    inline void visualizeSphere(const Eigen::Vector3d &center,
                                const double &radius)
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 0.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 1.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETE;

        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        spherePub.publish(sphereDeleter);
        spherePub.publish(sphereMarkers);
    }

    inline void visualizeStartGoal(const Eigen::Vector3d &center,
                                   const double &radius,
                                   const int sg)
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = sg;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "StartGoal";
        sphereMarkers.color.r = 1.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 0.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETEALL;

        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        if (sg == 0)
        {
            spherePub.publish(sphereDeleter);
            ros::Duration(1.0e-9).sleep();
            sphereMarkers.header.stamp = ros::Time::now();
        }
        spherePub.publish(sphereMarkers);
    }
};

#endif