/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : Examples
 * Purpose : Example of a minimal ROS2-Node class 
 *           which inherits from rclcpp::Node
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <cstdio>

#include <rclcpp/rclcpp.hpp>
// #include <ros_core/default.h>

#include <point_transformation/srv/pixel_to_point.hpp>
#include <point_transformation_lib/Transformation.h>

using PixelToPoint = point_transformation::srv::PixelToPoint;

class PointTransformationNode : public rclcpp::Node
{
public:
    PointTransformationNode();

private:
    rclcpp::Service<PixelToPoint>::SharedPtr service_;
};