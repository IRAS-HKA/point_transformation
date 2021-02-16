#include <point_transformation/point_transformation_node.h>

PointTransformationNode::PointTransformationNode() : Node("point_transformation_node")
{
    service_ = this->create_service<PixelToPoint>(
        "point_transformation_node/pixel_to_point", [&](const std::shared_ptr<PixelToPoint::Request> request, std::shared_ptr<PixelToPoint::Response> response) {
            Transformation t;

            // opening angles from roboception: horizontal 61°, vertical 48°
            // different focal_factor for changed lenses
            float focal_factor = 6. / 8.;

            t.init(std::vector<double>{61. * focal_factor, 48. * focal_factor}, request->depth_image.width, request->depth_image.height);

            // t.depth_from_pixel();
            // fixed depth of 1m
            std::vector<double> point = t.pixel_to_point(std::vector<int>{(int)request->pixel.x, (int)request->pixel.y}, 1);

            response->point.x = point[0];
            response->point.y = point[1];
            response->point.z = point[2];

            RCLCPP_INFO(get_logger(), "Service sending back response...");
        });

    RCLCPP_INFO(get_logger(), "Node started");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointTransformationNode>());
    rclcpp::shutdown();

    return 0;
}
