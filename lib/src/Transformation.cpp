#include <point_transformation_lib/Transformation.h>

Transformation Transformation::global_instance = Transformation();

Transformation::Transformation()
{
}

Transformation::~Transformation()
{
}

void Transformation::init(std::vector<double> opening_angle, int width_pixel, int height_pixel)
{
    half_size_pixel_ = {width_pixel / 2,
                        height_pixel / 2};

    size_real_ = {tan(opening_angle[0] * M_PI / 360) / half_size_pixel_[0],
                  tan(opening_angle[1] * M_PI / 360) / half_size_pixel_[1]};
}

void Transformation::init(float focal_length, float stereo_baseline)
{
    focal_length_ = focal_length;
    stereo_baseline_ = stereo_baseline;
}

cv::Point3d Transformation::pixelToPoint(cv::Point2i point, double depth) const
{
    std::vector<int> positions{point.x, point.y};

    return {pixelToPoint(positions, depth, 0),
            pixelToPoint(positions, depth, 1),
            depth};
}

cv::Point2i Transformation::pointToPixel(cv::Point3d point) const
{
    std::vector<double> positions{point.x, point.y, point.z};

    return {pointToPixel(positions, 0),
            pointToPixel(positions, 1)};
}

double Transformation::pixelToPoint(std::vector<int> &position, double depth, int index) const
{
    return (position[index] - half_size_pixel_[index]) * depth * size_real_[index];
}

int Transformation::pointToPixel(std::vector<double> &position, int index) const
{
    return position[index] / position[2] / size_real_[index] + half_size_pixel_[index];
}

// https://doc.rc-visard.com/latest/en/stereo_matching.html

cv::Point3d Transformation::pixelToPoint_D(cv::Point2i point, float disparity) const
{
    float factor = stereo_baseline_ / disparity;

    return {point.x * factor,
            point.y * factor,
            focal_length_ * factor};
}

cv::Point2i Transformation::pointToPixel_D(cv::Point3d point) const
{
    float factor = focal_length_ / point.z;

    return {(int)std::round(point.x * factor),
            (int)std::round(point.y * factor)};
}

// cv::Point3d Transformation::transform(Pose pose, cv::Point3d point) const
// {
//     TransformationMatrix matrix;

//     matrix.translation = {pose.x, pose.y, pose.z};

//     double sin_a = sin(pose.a);
//     double cos_a = cos(pose.a);
//     double sin_b = sin(pose.b);
//     double cos_b = cos(pose.b);
//     double sin_c = sin(pose.c);
//     double cos_c = cos(pose.c);

//     // https://tams.informatik.uni-hamburg.de/lehre/2010ss/vorlesung/Introduction_to_robotics/folien/Vorlesung2_druck4to1.pdf

//     matrix.rotation = {{cos_a * cos_b, cos_a * sin_b * sin_c - sin_a * cos_c, cos_a * sin_b * cos_c + sin_a * sin_c},
//                        {sin_a * cos_b, sin_a * sin_b * sin_c + cos_a * cos_c, sin_a * sin_b * cos_c - cos_a * cos_c},
//                        {-sin_b, cos_b * sin_c, cos_b * cos_c}};

//     return transform(matrix, point);
// }

// cv::Point3d Transformation::transform(TransformationMatrix matrix, cv::Point3d point) const
// {
//     return {point.x * matrix.rotation[0][0] + point.x * matrix.rotation[0][1] + point.x * matrix.rotation[0][2] + matrix.translation[0],
//             point.y * matrix.rotation[1][0] + point.y * matrix.rotation[1][1] + point.y * matrix.rotation[1][2] + matrix.translation[1],
//             point.z * matrix.rotation[2][0] + point.z * matrix.rotation[2][1] + point.z * matrix.rotation[2][2] + matrix.translation[2]};
// }