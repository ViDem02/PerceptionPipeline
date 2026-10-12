#include "odometry.h"

#include <cmath>
#include <pcl/common/transforms.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>

Eigen::Affine3f levelling_transform(double pitch_deg)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(static_cast<float>(pitch_deg * M_PI / 180), Eigen::Vector3f::UnitY()));
    return transform;
}

Cloud::Ptr level_and_filter(const Cloud::ConstPtr& raw, double pitch_deg)
{
    Cloud::Ptr cloud(new Cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(raw);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-1, 10);
    pass.filter(*cloud);

    pcl::transformPointCloud(*cloud, *cloud, levelling_transform(pitch_deg));

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);
    return cloud;
}

Eigen::Vector3d displacement_from_T21(const Eigen::Matrix4d& T_21)
{
    const Eigen::Matrix3d R = T_21.block<3,3>(0,0);
    const Eigen::Vector3d t = T_21.block<3,1>(0,3);
    return -R.transpose() * t;
}

MotionEstimate estimate_motion(const Cloud::Ptr& first, const Cloud::Ptr& second, int max_iterations,
                               double max_correspondence_distance)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(first);
    icp.setInputTarget(second);
    icp.setMaximumIterations(max_iterations);
    if (std::isfinite(max_correspondence_distance))
        icp.setMaxCorrespondenceDistance(max_correspondence_distance);

    Cloud aligned;
    pcl::console::TicToc time;
    time.tic();
    icp.align(aligned);

    MotionEstimate m;
    m.ms = time.toc();
    m.converged = icp.hasConverged();
    m.convergence_state = icp.getConvergeCriteria()->getConvergenceState();
    m.T_21 = icp.getFinalTransformation().cast<double>();
    m.displacement = displacement_from_T21(m.T_21);
    m.fitness = std::isfinite(max_correspondence_distance) ? icp.getFitnessScore(max_correspondence_distance)
                                                           : icp.getFitnessScore();
    return m;
}
