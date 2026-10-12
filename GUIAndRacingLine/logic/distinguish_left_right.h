#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <limits>

namespace cones {

struct Result {
    pcl::PointCloud<pcl::PointXYZ>::Ptr left;
    pcl::PointCloud<pcl::PointXYZ>::Ptr right;
};

// Splits detected cone centres into the left and right track boundaries, each ordered
// from the car outwards. Points are in the viewer frame (x forward, y up, z right);
// only the ground-plane coordinates (x, z) are used. (robot_x, robot_z) is the car
// position on the ground plane, the car is assumed to head along +x.
Result distinguishLeftRight(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cones,
    double robot_x, double robot_z,
    double angle_low_deg = -45.0, double angle_high_deg = 45.0,
    double max_distance = std::numeric_limits<double>::infinity()
    );

}
