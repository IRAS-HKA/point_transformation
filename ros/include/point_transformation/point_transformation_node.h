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
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>

#include <point_transformation/srv/pixel_to_point.hpp>
#include <point_transformation_lib/Transformation.h>

using PixelToPoint = point_transformation::srv::PixelToPoint;

class PointTransformationNode : public rclcpp::Node
{
public:
    PointTransformationNode();

private:
    rclcpp::Service<PixelToPoint>::SharedPtr service_;
    double opening_angle_horizontal_;
    double opening_angle_vertical_;
    double focal_factor_;
    int width_;
    int height_;
    double default_depth_;
    int max_pixel_range_for_depth_matching_;
    double get_depth_from_image_(const std::shared_ptr<PixelToPoint::Request> request);
    bool set_depth_if_not_nan_(cv_bridge::CvImagePtr &cv_ptr, int x, int y, double &depth);
};