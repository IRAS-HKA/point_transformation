#include <point_transformation/point_transformation_node.h>

PointTransformationNode::PointTransformationNode() : Node("point_transformation_node")
{
    this->declare_parameter("opening_angle_horizontal", 61.0);
    this->declare_parameter("opening_angle_vertical", 48.0);
    this->declare_parameter("focal_factor", 0.75);
    this->declare_parameter("width", 1920);
    this->declare_parameter("height", 1080);
    this->declare_parameter("default_depth", 1.0);
    this->declare_parameter("max_pixel_range_for_depth_matching", 5);

    opening_angle_horizontal_ = this->get_parameter("opening_angle_horizontal").as_double();
    opening_angle_vertical_ = this->get_parameter("opening_angle_vertical").as_double();
    focal_factor_ = this->get_parameter("focal_factor").as_double();
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    default_depth_ = this->get_parameter("default_depth").as_double();
    max_pixel_range_for_depth_matching_ = this->get_parameter("max_pixel_range_for_depth_matching").as_int();

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

            std::vector<double> depth_list;
            // If no depth image given, use default from param file
            if (!request->depth_image.data.empty())
            {
                depth_list = get_depth_from_image_(request);
            }
            else
            {
                for (size_t i = 0; i < request->pixels.size(); i++)
                {
                    depth_list.push_back(default_depth_);
                }
            }

            for (size_t i = 0; i < depth_list.size(); i++)
            {

                if (depth_list[i] == 0)
                {
                    geometry_msgs::msg::Point point_msg;
                    response->points.push_back(point_msg);
                }
                else
                {
                    std::vector<double> point = t.pixel_to_point(std::vector<int>{(int)request->pixels[i].x, (int)request->pixels[i].y}, depth_list[i]);

                    geometry_msgs::msg::Point point_msg;
                    point_msg.x = point[0];
                    point_msg.y = point[1];
                    point_msg.z = point[2];
                    response->points.push_back(point_msg);
                }
            }
            RCLCPP_INFO(get_logger(), "Service sending back response..."); });

    RCLCPP_INFO(get_logger(), "Node started");
}

std::vector<double> PointTransformationNode::get_depth_from_image_(const std::shared_ptr<PixelToPoint::Request> request)
{
    // x from openpose is from top left to the right = cv/column cv(row, col)
    // y from openpose is from top left downwards = cv/row cv(row, col)

    // datatype for roboception is "float", for realsense is "u_int16_t"
    camera_type_ = request->camera_type;

    double x_ratio = (double)request->depth_image.width / request->width;
    double y_ratio = (double)request->depth_image.height / request->height;

    cv_bridge::CvImagePtr cv_ptr;

    cv_ptr = cv_bridge::toCvCopy(request->depth_image, request->depth_image.encoding);

    // cv::Mat image_32f;
    // cv_ptr->image.convertTo(image_32f, CV_32F);
    // cv_ptr->image = image_32f;

    // RCLCPP_INFO(get_logger(), std::to_string(x_ratio) + " " + std::to_string(y_ratio) + " " + std::to_string(depth_pixel_row) + " " + std::to_string(depth_pixel_col) + " " + std::to_string(request->depth_image.width) + " " + std::to_string(request->depth_image.height));

    // for (int i = 1; i < 480; i++)
    // {
    //     RCLCPP_INFO(get_logger(), std::to_string(cv_ptr->image.at<float>(i, i)));
    //     cv_ptr->image.at<float>(i, i) = 20;
    // }

    // cv_ptr->image.at<u_int32_t>(0, 0) = 100;
    // cv::imshow("Display window", cv_ptr->image);
    // cv::waitKey(0);

    if (camera_type_ == "roboception")
    {
        cv::patchNaNs(cv_ptr->image, 0.0);
    }

    std::vector<double> depth_list;

    for (size_t i = 0; i < request->pixels.size(); i++)
    {
        double depth = 0.0;

        int depth_pixel_row = std::round(request->pixels[i].y * y_ratio);
        int depth_pixel_col = std::round(request->pixels[i].x * x_ratio);

        // cv_ptr->image.at<float>(depth_pixel_row, depth_pixel_col + 1) = 20;
        // cv_ptr->image.at<float>(depth_pixel_row, 0) = 20;
        // cv_ptr->image.at<float>(0, depth_pixel_col) = 20;

        if (!set_depth_if_not_nan_(cv_ptr, depth_pixel_row, depth_pixel_col, depth))
        {
            for (int i = 1; i < max_pixel_range_for_depth_matching_; i++)
            {
                for (int j = 0; j < i + 1; j++)
                {
                    // left row
                    if (set_depth_if_not_nan_(cv_ptr, depth_pixel_row - i, depth_pixel_col + j, depth))
                    {
                        goto break_loops;
                    }
                    if (set_depth_if_not_nan_(cv_ptr, depth_pixel_row - i, depth_pixel_col - j, depth))
                    {
                        goto break_loops;
                    }

                    // right row
                    if (set_depth_if_not_nan_(cv_ptr, depth_pixel_row + i, depth_pixel_col + j, depth))
                    {
                        goto break_loops;
                    }
                    if (set_depth_if_not_nan_(cv_ptr, depth_pixel_row + i, depth_pixel_col - j, depth))
                    {
                        goto break_loops;
                    }

                    // bottom row
                    if (set_depth_if_not_nan_(cv_ptr, depth_pixel_row - j, depth_pixel_col - i, depth))
                    {
                        goto break_loops;
                    }
                    if (set_depth_if_not_nan_(cv_ptr, depth_pixel_row + j, depth_pixel_col - i, depth))
                    {
                        goto break_loops;
                    }

                    // top row
                    if (set_depth_if_not_nan_(cv_ptr, depth_pixel_row - j, depth_pixel_col + i, depth))
                    {
                        goto break_loops;
                    }
                    if (set_depth_if_not_nan_(cv_ptr, depth_pixel_row + j, depth_pixel_col + i, depth))
                    {
                        goto break_loops;
                    }
                }
            }
            RCLCPP_INFO(get_logger(), "found no valid depth pixel in range");
        }

    break_loops:

        if (camera_type_ == "roboception")
        {
            depth_list.push_back(depth); // float already in m
        }
        else // if (camera_type_ == "realsense")
        {
            depth_list.push_back(depth / 1000); // convert mm in m
        }

        // RCLCPP_INFO(get_logger(), "depth[m]: " + std::to_string(depth));

        // cv::imshow("Display window", cv_ptr->image);
        // cv::waitKey(100);
    }

    return depth_list;
}

bool PointTransformationNode::set_depth_if_not_nan_(cv_bridge::CvImagePtr &cv_ptr, int row, int col, double &depth)
{
    if (camera_type_ == "roboception")
    {
        if (cv_ptr->image.at<float>(row, col) >= 0.001)
        {
            depth = cv_ptr->image.at<float>(row, col);
            // cv_ptr->image.at<float>(row, col) = 60000.0;
            return true;
        }
    }
    else // if (camera_type_ == "realsense")
    {
        if (cv_ptr->image.at<u_int16_t>(row, col) != 0.0)
        {
            depth = cv_ptr->image.at<u_int16_t>(row, col);
            // cv_ptr->image.at<u_int16_t>(row, col) = 60000.0;
            return true;
        }
    }
    return false;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointTransformationNode>());
    rclcpp::shutdown();

    return 0;
}
