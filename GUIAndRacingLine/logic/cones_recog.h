//
// Created by Vi De Matteis on 11/11/25.
//

#ifndef SECOND_PRJ_SETUP_CONES_REGOC_H
#define SECOND_PRJ_SETUP_CONES_REGOC_H
#include <vector>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <Eigen/Core>

// ICP alignment of a cluster onto the cone model (both centred on their centroid).
struct ConeAlignment {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();  // centred cluster -> model
    double fitness = 0.0;     // mean squared distance [m^2]
    float yaw_deg = 0.0f;     // rotation about the vertical axis y
    float tilt_deg = 0.0f;    // angle between the vertical and the rotated vertical
    bool converged = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned;  // centred cluster after the transform
};

// cone_model must already be centred (detectCones does it).
ConeAlignment alignClusterToConeModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster,
                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cone_model, int icp_max_iter);

std::vector<int> detectCones(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<pcl::PointIndices> &cluster_indices,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cone_model,
    int icp_max_iter = 40,
    double fitness_threshold = 0.01,
    double height_threshold = -0.01, // <-- adjust as needed (meters)
    double min_cone_height = 0.3, // <-- minimum expected cone height in meters
    double max_cone_height = 0.5,
    float max_yaw_deg = 20.0f,
    bool visualize = true,
    bool verbose = false);

#endif //SECOND_PRJ_SETUP_CONES_REGOC_H