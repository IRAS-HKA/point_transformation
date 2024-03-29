#pragma once

#include <math.h>
#include <vector>

// #include <opencv2/core/types.hpp>

// #include <aip/common/default.h>
// #include <aip/common/types/Pose.h>
// #include <aip/comd/types/TransformationMatrix.h>

class Transformation
{
public:
    static Transformation global_instance;

    Transformation();
    ~Transformation();

    void init(std::vector<double> opening_angle, int width_pixel, int height_pixel);
    // void init(float focal_length, float stereo_baseline);

    std::vector<int> size_pixel() const { return {half_size_pixel_[0] * 2, half_size_pixel_[1] * 2}; }

    std::vector<double> pixel_to_point(std::vector<int> pixel, double depth) const;
    std::vector<int> point_to_pixel(std::vector<double> point) const;

    // cv::Point3d pixelToPoint_D(cv::Point2i point, float disparity) const;
    // cv::Point2i pointToPixel_D(cv::Point3d point) const;

    // cv::Point3d transform(Pose pose, cv::Point3d point) const;
    // cv::Point3d transform(TransformationMatrix matrix, cv::Point3d point) const;

private:
    double pixel_to_point(std::vector<int> &position, double depth, int index) const;
    int point_to_pixel(std::vector<double> &position, int index) const;

    std::vector<int> half_size_pixel_;
    std::vector<double> size_real_;

    // float focal_length_;
    // float stereo_baseline_;
};