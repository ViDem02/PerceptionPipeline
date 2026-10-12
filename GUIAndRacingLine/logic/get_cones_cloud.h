

#ifndef SECOND_PRJ_SETUP_GET_CONES_CLOUD_H
#define SECOND_PRJ_SETUP_GET_CONES_CLOUD_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct ConeDetectionResults
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr centers_of_mass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cone_cloud;
};


ConeDetectionResults
get_cones_cloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cones_cloud,
    double initial_sor_std_dev_thr,
    float cut_bottom,
    float cut_top, double detect_cones_fitness_detection, bool verbose, const pcl::PointCloud<pcl::
    PointXYZ>::Ptr &ideal_cone_model, bool visualize_cone_detection = false
);

#endif //SECOND_PRJ_SETUP_GET_CONES_CLOUD_H