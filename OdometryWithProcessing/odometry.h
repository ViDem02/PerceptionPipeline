#pragma once
// LiDAR odometry between two scans with point-to-point ICP.
//
// Frames: a raw scan has x forward, y left, z up. level_and_filter() rotates it into the
// levelled frame used everywhere else: x forward, y up, z right.

#include <Eigen/Dense>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

// ICP settings of the program. Correspondences farther than 1 m are rejected: with PCL's
// default (unbounded) every point is matched, and on synthetic motions with known ground
// truth (tests/test_odometry.cpp) the estimate is off by ~0.8 m and ~10 deg.
constexpr int kIterations = 70;
constexpr double kMaxCorrespondenceDistance = 1.0;

// Rotation from the raw frame to the levelled frame, with a hand-tuned pitch correction.
Eigen::Affine3f levelling_transform(double pitch_deg);

// Pass-through on the raw forward axis (x in [-1, 10] m), levelling, statistical outlier removal.
Cloud::Ptr level_and_filter(const Cloud::ConstPtr& raw, double pitch_deg);

struct MotionEstimate {
    // ICP result. PCL maps the source (first) into the target (second):
    // p_second = T_21 * p_first.
    Eigen::Matrix4d T_21 = Eigen::Matrix4d::Identity();
    // Pose of the vehicle at the second scan, in the frame of the first: T_12 = T_21^-1.
    // Its translation -R^T t is the displacement (x forward, y up, z right).
    Eigen::Vector3d displacement = Eigen::Vector3d::Zero();
    double fitness = 0.0;           // mean squared distance of the pairs within max_correspondence_distance [m^2]
    int convergence_state = 0;      // pcl::registration::DefaultConvergenceCriteria::ConvergenceState
    bool converged = false;
    double ms = 0.0;
};

// Displacement of the vehicle (frame of the first scan) from the ICP transform first -> second.
Eigen::Vector3d displacement_from_T21(const Eigen::Matrix4d& T_21);

// Runs ICP with `first` as source and `second` as target. A max_correspondence_distance of
// infinity keeps PCL's default (every point is matched, however far).
MotionEstimate estimate_motion(const Cloud::Ptr& first, const Cloud::Ptr& second, int max_iterations,
                               double max_correspondence_distance = std::numeric_limits<double>::infinity());
