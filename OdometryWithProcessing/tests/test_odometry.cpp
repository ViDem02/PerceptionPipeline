// Ground-truth test of the ICP odometry (odometry.h).
//
// A synthetic second scan is made from the first one with a known vehicle motion M
// (pose of the sensor at scan 2 in the raw frame of scan 1): every point p of the first
// scan is seen at M^-1 p from the second pose. To make the pair realistic, the two scans
// are sampled independently (70% of the points each), get 1 cm of noise, and the second
// scan only keeps what is in front of the moved sensor (partial overlap).
// Both scans are levelled with the same pitch, so the expected results are
//   T_21 = L M^-1 L^-1   and   displacement = R_L t_M   (L = levelling transform).
//
// Usage: test_odometry first.pcd   (exit code 1 if a check fails)

#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/default_convergence_criteria.h>

#include "odometry.h"

namespace {

int failures = 0;

void check(bool ok, const std::string& what)
{
    std::printf("  [%s] %s\n", ok ? "pass" : "FAIL", what.c_str());
    if (!ok) ++failures;
}

Cloud::Ptr sample(const Cloud& in, const Eigen::Affine3f& T, std::mt19937& rng, bool keep_front_only)
{
    std::uniform_real_distribution<float> uni(0.f, 1.f);
    std::normal_distribution<float> noise(0.f, 0.01f);
    Cloud::Ptr out(new Cloud);
    for (const auto& p : in) {
        if (uni(rng) > 0.7f) continue;
        const Eigen::Vector3f q = T * p.getVector3fMap();
        if (keep_front_only && q.x() < 0.5f) continue;  // forward-looking sensor
        out->push_back(pcl::PointXYZ(q.x() + noise(rng), q.y() + noise(rng), q.z() + noise(rng)));
    }
    return out;
}

double rotation_angle_deg(const Eigen::Matrix3d& R)
{
    return std::acos(std::clamp((R.trace() - 1) / 2, -1.0, 1.0)) * 180 / M_PI;
}

struct Motion {
    double forward, left, yaw_deg;
};

}

int main(int argc, char** argv)
{
    if (argc < 2) { std::fprintf(stderr, "usage: %s first.pcd\n", argv[0]); return 2; }
    Cloud::Ptr raw(new Cloud);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *raw) < 0) return 2;

    std::printf("Displacement from a known transform\n");
    {
        // If M is the pose at scan 2 in the frame of scan 1, points map first -> second with
        // T_21 = M^-1, and the displacement must be the translation of M.
        Eigen::Affine3d M = Eigen::Affine3d::Identity();
        M.translate(Eigen::Vector3d(1.0, 0.0, -0.5));
        M.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
        const Eigen::Vector3d d = displacement_from_T21(M.inverse().matrix());
        check((d - Eigen::Vector3d(1.0, 0.0, -0.5)).norm() < 1e-12, "displacement_from_T21(M^-1) = translation of M");
    }

    const double pitch = -10.0;
    const Eigen::Affine3f L = levelling_transform(pitch);
    const std::vector<Motion> motions = {
        {0.5, 0.0, 0.0},
        {0.5, 0.2, 2.0},
        {0.3, -0.1, -3.0},
        {1.0, 0.0, 5.0},
    };
    // PCL default (unbounded, the previous setting of the program) for comparison.
    const std::vector<double> max_dist = {std::numeric_limits<double>::infinity(), kMaxCorrespondenceDistance};

    std::printf("\n%-28s %-12s %9s %9s %9s %9s %10s %6s\n", "motion (fwd, left, yaw)", "max corr.",
                "err pos", "err rot", "est fwd", "est right", "state", "ms");
    for (const auto& mo : motions) {
        // Pose of the sensor at scan 2 in the raw frame of scan 1 (x forward, y left, z up).
        Eigen::Affine3f M = Eigen::Affine3f::Identity();
        M.translate(Eigen::Vector3f(mo.forward, mo.left, 0));
        M.rotate(Eigen::AngleAxisf(mo.yaw_deg * M_PI / 180, Eigen::Vector3f::UnitZ()));

        std::mt19937 rng(42);
        const auto raw1 = sample(*raw, Eigen::Affine3f::Identity(), rng, false);
        const auto raw2 = sample(*raw, M.inverse(), rng, true);
        const auto first = level_and_filter(raw1, pitch);
        const auto second = level_and_filter(raw2, pitch);

        const Eigen::Vector3d disp_gt = (L.linear() * M.translation()).cast<double>();
        const Eigen::Matrix3d R_gt = (L * M.inverse() * L.inverse()).linear().cast<double>();

        for (double md : max_dist) {
            const auto est = estimate_motion(first, second, kIterations, md);
            const double err_pos = (est.displacement - disp_gt).norm();
            const double err_rot = rotation_angle_deg(est.T_21.block<3,3>(0,0) * R_gt.transpose());
            char label[64], mdl[16];
            std::snprintf(label, sizeof label, "(%.1f m, %+.1f m, %+.0f deg)", mo.forward, mo.left, mo.yaw_deg);
            std::snprintf(mdl, sizeof mdl, std::isfinite(md) ? "%.1f m" : "unbounded", md);
            using CS = pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState;
            const char* state = est.convergence_state == CS::CONVERGENCE_CRITERIA_ITERATIONS ? "iter. cap" :
                                est.convergence_state == CS::CONVERGENCE_CRITERIA_NOT_CONVERGED ? "not conv." :
                                "tolerance";
            std::printf("%-28s %-12s %8.3fm %8.2fd %8.3fm %8.3fm %10s %6.0f\n", label, mdl, err_pos, err_rot,
                        est.displacement.x(), est.displacement.z(), state, est.ms);
            if (md == kMaxCorrespondenceDistance) {
                // the program's settings: position within 5 cm and rotation within 0.5 deg
                check(err_pos < 0.05 && err_rot < 0.5,
                      std::string("program settings recover ") + label + " (expected fwd " +
                      std::to_string(disp_gt.x()) + ", right " + std::to_string(disp_gt.z()) + ")");
            }
        }
    }

    std::printf(failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", failures);
    return failures ? 1 : 0;
}
