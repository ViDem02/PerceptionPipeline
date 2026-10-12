//
// Created by Vi De Matteis on 10/11/25.
//

#ifndef RACINGLINE_RACINGLINE_NLOPT_H
#define RACINGLINE_RACINGLINE_NLOPT_H
#include <array>
#include <optional>
#include <vector>
#include <pcl/point_cloud.h>

namespace pcl
{
    struct PointXYZ;
}


// Optimised racing line on the ground plane (x forward, z right).
struct RacingLine {
    std::vector<std::array<double, 2>> points; // p_i = mid_i + d_i * normal_i, one per gate
    std::vector<double> offsets;               // d_i
    std::vector<double> half_width;            // bound on |d_i|
    int status = 0;                            // nlopt_result; points is empty if < 0
    double cost = 0.0;
};

// Gates are formed by pairing left[i] with right[i] ({x, z} each); needs at least
// two gates, otherwise returns an empty RacingLine. `start`, if given, is a fixed point
// the line comes from (the car), included in the curvature cost.
RacingLine optimise_racing_line(
    const std::vector<std::vector<double>>& left,
    const std::vector<std::vector<double>>& right,
    std::optional<std::array<double, 2>> start = std::nullopt);

// Racing line drawn as a polyline from the car (origin) through every optimised point.
pcl::PointCloud<pcl::PointXYZ>::Ptr
get_racing_line_point_cloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_left,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_right,
    int nr_points_lin_space = 100,
    float y_constant = 0
    );


#endif //RACINGLINE_RACINGLINE_NLOPT_H
