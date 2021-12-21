#include <point_transformation/point_transformation_node.h>

PointTransformationNode::PointTransformationNode() : Node("point_transformation_node")
{
    this->declare_parameter("opening_angle_horizontal");
    this->declare_parameter("opening_angle_vertical");
    this->declare_parameter("focal_factor");
    this->declare_parameter("width");
    this->declare_parameter("height");
    this->declare_parameter("default_depth");

    opening_angle_horizontal_ = this->get_parameter("opening_angle_horizontal").as_double();
    opening_angle_vertical_ = this->get_parameter("opening_angle_vertical").as_double();
    focal_factor_ = this->get_parameter("focal_factor").as_double();
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    default_depth_ = this->get_parameter("default_depth").as_double();

    service_ = this->create_service<PixelToPoint>(
        "point_transformation_node/pixel_to_point", [&](const std::shared_ptr<PixelToPoint::Request> request, std::shared_ptr<PixelToPoint::Response> response)
        {
            Transformation t;

            // If width or heigth has default value of 0, then use the value from the parameter file
            if (request->width != 0 && request->height != 0)
            {
                width_ = request->width;
                height_ = request->height;
            }

            t.init(std::vector<double>{opening_angle_horizontal_ * focal_factor_, opening_angle_vertical_ * focal_factor_}, width_, height_);

            double depth = default_depth_;

            // If no depth image given, use default from param file
            if (!request->depth_image.data.empty())
            {

                depth = get_depth_from_image_(request);
            }

            std::vector<double> point = t.pixel_to_point(std::vector<int>{(int)request->pixel.x, (int)request->pixel.y}, depth);

            response->point.x = point[0];
            response->point.y = point[1];
            response->point.z = point[2];

            RCLCPP_INFO(get_logger(), "Service sending back response...");
        });

    RCLCPP_INFO(get_logger(), "Node started");
}

double PointTransformationNode::get_depth_from_image_(const std::shared_ptr<PixelToPoint::Request> request)
{
    double x_ratio = (double)request->depth_image.width / request->width;
    double y_ratio = (double)request->depth_image.height / request->height;

    int depth_pixel_x = std::round(request->pixel.x * x_ratio);
    int depth_pixel_y = std::round(request->pixel.y * y_ratio);

    cv_bridge::CvImagePtr cv_ptr;

    cv_ptr = cv_bridge::toCvCopy(request->depth_image, request->depth_image.encoding);

    double depth = cv_ptr->image.at<double>(depth_pixel_x, depth_pixel_y);

    RCLCPP_INFO(get_logger(), std::to_string(depth_pixel_x));
    RCLCPP_INFO(get_logger(), std::to_string(cv_ptr->image.cols));
    RCLCPP_INFO(get_logger(), std::to_string(depth));

    // Warum ist depth value z=2.247117487993712e+307?
    // Nochmal ausprobieren in python tiefenwerte auszulesen

    return depth;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointTransformationNode>());
    rclcpp::shutdown();

    return 0;
}
