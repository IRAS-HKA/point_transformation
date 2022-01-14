#include <point_transformation/point_transformation_node.h>

PointTransformationNode::PointTransformationNode() : Node("point_transformation_node")
{
    this->declare_parameter("opening_angle_horizontal");
    this->declare_parameter("opening_angle_vertical");
    this->declare_parameter("focal_factor");
    this->declare_parameter("width");
    this->declare_parameter("height");
    this->declare_parameter("default_depth");
    this->declare_parameter("max_pixel_range_for_depth_matching");

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

            double depth = default_depth_;

            // If no depth image given, use default from param file
            if (!request->depth_image.data.empty())
            {
                depth = get_depth_from_image_(request);

                if (depth == 0)
                {
                    response->depth_found = false;
                }
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
    // x from openpose is from top left to the right = cv/column cv(row, col)
    // y from openpose is from top left downwards = cv/row cv(row, col)

    // datatype für roboception is "float", for realsense is "u_int16_t"
    camera_type_ = request->camera_type;

    double x_ratio = (double)request->depth_image.width / request->width;
    double y_ratio = (double)request->depth_image.height / request->height;

    int depth_pixel_row = std::round(request->pixel.y * y_ratio);
    int depth_pixel_col = std::round(request->pixel.x * x_ratio);

    // RCLCPP_INFO(get_logger(), std::to_string(x_ratio) + " " + std::to_string(y_ratio) + " " + std::to_string(depth_pixel_row) + " " + std::to_string(depth_pixel_col) + " " + std::to_string(request->depth_image.width) + " " + std::to_string(request->depth_image.height));

    double depth = 0.0;

    cv_bridge::CvImagePtr cv_ptr;

    cv_ptr = cv_bridge::toCvCopy(request->depth_image, request->depth_image.encoding);

    // cv::Mat image_32f;
    // cv_ptr->image.convertTo(image_32f, CV_32F);
    // cv_ptr->image = image_32f;

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

    // cv_ptr->image.at<float>(depth_pixel_row, depth_pixel_col + 1) = 20;
    // cv_ptr->image.at<float>(depth_pixel_row, 0) = 20;
    // cv_ptr->image.at<float>(0, depth_pixel_col) = 20;

    if (!set_depth_if_not_nan_(cv_ptr, depth_pixel_row, depth_pixel_col, depth))
    {
        for (int i = 1; i < max_pixel_range_for_depth_matching_; i++)
        {
            for (int j = 0; j < i + 1; j++)
            {
                //left row
                if (set_depth_if_not_nan_(cv_ptr, depth_pixel_row - i, depth_pixel_col + j, depth))
                {
                    goto break_loops;
                }
                if (set_depth_if_not_nan_(cv_ptr, depth_pixel_row - i, depth_pixel_col - j, depth))
                {
                    goto break_loops;
                }

                //right row
                if (set_depth_if_not_nan_(cv_ptr, depth_pixel_row + i, depth_pixel_col + j, depth))
                {
                    goto break_loops;
                }
                if (set_depth_if_not_nan_(cv_ptr, depth_pixel_row + i, depth_pixel_col - j, depth))
                {
                    goto break_loops;
                }

                //bottom row
                if (set_depth_if_not_nan_(cv_ptr, depth_pixel_row - j, depth_pixel_col - i, depth))
                {
                    goto break_loops;
                }
                if (set_depth_if_not_nan_(cv_ptr, depth_pixel_row + j, depth_pixel_col - i, depth))
                {
                    goto break_loops;
                }

                //top row
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
        depth = depth; //float already in m
    }
    else // if (camera_type_ == "realsense")
    {
        depth = depth / 1000; //convert mm in m
    }

    RCLCPP_INFO(get_logger(), "depth[m]: " + std::to_string(depth));

    // cv::imshow("Display window", cv_ptr->image);
    // cv::waitKey(100);

    return depth;
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
    else //if (camera_type_ == "realsense")
    {
        if (cv_ptr->image.at<u_int16_t>(row, col) != 0.0)
        {
            depth = cv_ptr->image.at<u_int16_t>(row, col);
            // cv_ptr->image.at<u_int16_t>(row, col) = 60000.0;
            return true;
        }
    }

    // RCLCPP_INFO(get_logger(), std::to_string(row) + " " + std::to_string(col));
    // cv_ptr->image.at<float>(row, col) = 0.0001;
    return false;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointTransformationNode>());
    rclcpp::shutdown();

    return 0;
}
