#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "gcopter/trajectory.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std;


// Visualizer for the planner
class Visualizer
{
private:
    // config contains the scale for some markers
    rclcpp::Node::SharedPtr node_;

    // These are publishers for path, waypoints on the trajectory,
    // the entire trajectory, the mesh of free-space polytopes,
    // the edge of free-space polytopes, and spheres for safety radius
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr routePub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wayPointsPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectoryPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr meshPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edgePub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spherePub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr listsPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrowsPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr selectedPointPub;

public:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speedPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tiltPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bdrPub;

public:
    explicit Visualizer(const rclcpp::Node::SharedPtr &node)
        : node_(node)
    {
        routePub = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/route", 10);
        wayPointsPub = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/waypoints", 10);
        trajectoryPub = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/trajectory", 10);
        meshPub = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/mesh", 1000);
        edgePub = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/edge", 1000);
        spherePub = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/spheres", 1000);
        listsPub = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/lists", 2);
        arrowsPub = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/arrows", 2);
        selectedPointPub = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/selectedPoint", 1000);
        speedPub = node_->create_publisher<std_msgs::msg::Float64>("/visualizer/speed", 1000);
        thrPub = node_->create_publisher<std_msgs::msg::Float64>("/visualizer/total_thrust", 1000);
        tiltPub = node_->create_publisher<std_msgs::msg::Float64>("/visualizer/tilt_angle", 1000);
        bdrPub = node_->create_publisher<std_msgs::msg::Float64>("/visualizer/body_rate", 1000);
    }

    // Visualize the trajectory and its front-end path
    void displayMarkerList(const std::vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id, bool show_sphere = true, bool show_line = true)
    {
        visualization_msgs::msg::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "odom";
        sphere.header.stamp = line_strip.header.stamp = node_->now();
        sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
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
        geometry_msgs::msg::Point pt;
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
            listsPub->publish(sphere);
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }
        if (show_line)
        {
            listsPub->publish(line_strip);
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }
    }

    inline void displayVectorArrow(const vector<Eigen::Vector3d> &list, Eigen::Vector4d color, const Eigen::Vector3d xyz, double scale, int id)
    {
        // cout <<"ABCDEF" << endl;
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "odom";
        arrow.header.stamp = node_->now();
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

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

        geometry_msgs::msg::Point start, end;
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

            arrowsPub->publish(arrow);
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }
        return;
    }

    inline void displaySelectedPos(Eigen::Vector3d pos, Eigen::Vector4d color, const double scale, int id)
    {
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = "odom";
        sphere.header.stamp = node_->now();
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
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

        selectedPointPub->publish(sphere);
        rclcpp::sleep_for(std::chrono::milliseconds(1));
    }

    // Visualize all spheres with centers sphs and the same radius
    inline void visualizeSphere(const Eigen::Vector3d &center,
                                const double &radius)
    {
        visualization_msgs::msg::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = node_->now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::msg::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 0.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 1.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::msg::Marker::DELETE;

        geometry_msgs::msg::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        spherePub->publish(sphereDeleter);
        spherePub->publish(sphereMarkers);
    }

    inline void visualizeStartGoal(const Eigen::Vector3d &center,
                                   const double &radius,
                                   const int sg)
    {
        visualization_msgs::msg::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = sg;
        sphereMarkers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = node_->now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::msg::Marker::ADD;
        sphereMarkers.ns = "StartGoal";
        sphereMarkers.color.r = 1.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 0.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::msg::Marker::DELETEALL;

        geometry_msgs::msg::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        if (sg == 0)
        {
            spherePub->publish(sphereDeleter);
            rclcpp::sleep_for(std::chrono::nanoseconds(1));
            sphereMarkers.header.stamp = node_->now();
        }
        spherePub->publish(sphereMarkers);
    }
};

#endif
