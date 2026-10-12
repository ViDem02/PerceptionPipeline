// Verification of the ICP step of the cone recogniser (alignClusterToConeModel).
//
//  1. Known rotations of the full model: the transform maps the cluster onto the model,
//     so it must be the inverse of the applied rotation; tilt is recovered.
//  2. A rotation about the cone axis (yaw) leaves the surface unchanged, so ICP cannot
//     recover it: the yaw it reports is not a property of the object.
//  3. Single simulated views of true cones and of cone-sized non-cones at several ranges
//     and bearings: yaw, tilt and fitness, and how each gate separates the two groups.
//
// Usage: test_cone_icp ideal_cone.ply   (exit code 1 if a check fails)

#include <cmath>
#include <cstdio>
#include <tuple>
#include <string>
#include <vector>

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include "lidar_sim.h"
#include "logic/cones_recog.h"

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
int failures = 0;

void check(bool ok, const std::string& what)
{
    std::printf("  [%s] %s\n", ok ? "pass" : "FAIL", what.c_str());
    if (!ok) ++failures;
}

double angle_deg(const Eigen::Matrix3f& R)
{
    return std::acos(std::clamp((R.trace() - 1.0f) / 2.0f, -1.0f, 1.0f)) * 180.0 / M_PI;
}

struct View {
    std::string object;
    double range, bearing_deg;
    ConeAlignment a;
    int points;
};

}

int main(int argc, char** argv)
{
    if (argc < 2) { std::fprintf(stderr, "usage: %s ideal_cone.ply\n", argv[0]); return 2; }
    Cloud::Ptr model(new Cloud);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(argv[1], *model) == -1) return 2;
    Eigen::Vector4f c;
    pcl::compute3DCentroid(*model, c);
    for (auto& p : *model) { p.x -= c[0]; p.y -= c[1]; p.z -= c[2]; }  // as detectCones does

    std::printf("1. Known rotations of the full model\n");
    for (const auto& [name, axis, deg] : std::vector<std::tuple<std::string, Eigen::Vector3f, float>>{
             {"tilt 8 deg about x", Eigen::Vector3f::UnitX(), 8.f},
             {"tilt 8 deg about z", Eigen::Vector3f::UnitZ(), 8.f}}) {
        const Eigen::Matrix3f G = Eigen::AngleAxisf(deg * M_PI / 180, axis).toRotationMatrix();
        Eigen::Affine3f T = Eigen::Affine3f::Identity();
        T.linear() = G;
        T.translation() << 3.0f, -0.6f, 0.4f;
        Cloud::Ptr cluster(new Cloud);
        pcl::transformPointCloud(*model, *cluster, T);
        const auto a = alignClusterToConeModel(cluster, model, 40);
        const float err = angle_deg(a.transform.block<3, 3>(0, 0) * G);
        char buf[200];
        std::snprintf(buf, sizeof buf, "%s: transform = inverse of the applied rotation (error %.2f deg), tilt %.2f deg",
                      name.c_str(), err, a.tilt_deg);
        check(a.converged && err < 1.0f && std::abs(a.tilt_deg - deg) < 1.0f, buf);
    }

    std::printf("2. Rotation about the cone axis\n");
    {
        const Eigen::Matrix3f G = Eigen::AngleAxisf(30 * M_PI / 180, Eigen::Vector3f::UnitY()).toRotationMatrix();
        Eigen::Affine3f T = Eigen::Affine3f::Identity();
        T.linear() = G;
        Cloud::Ptr cluster(new Cloud);
        pcl::transformPointCloud(*model, *cluster, T);
        const auto a = alignClusterToConeModel(cluster, model, 40);
        char buf[200];
        std::snprintf(buf, sizeof buf, "model turned 30 deg about its axis: reported yaw %.2f deg instead of -30, "
                      "fitness %.5f (the turn is invisible: yaw is ICP drift)", a.yaw_deg, a.fitness);
        check(a.converged && std::abs(a.yaw_deg + 30) > 15 && a.fitness < 1e-3, buf);
    }

    std::printf("3. Simulated single views (sensor 0.82 m above the ground)\n");
    std::vector<View> views;
    for (double range : {2.5, 3.5, 4.5, 5.5}) {
        for (double bearing : {-30.0, -15.0, 0.0, 15.0, 30.0}) {
            const double x = range * std::cos(bearing * M_PI / 180), z = range * std::sin(bearing * M_PI / 180);
            for (int kind = 0; kind < 3; ++kind) {
                sim::Scene scene;
                std::string name;
                if (kind == 0) { scene.cones.push_back({x, z}); name = "cone"; }
                if (kind == 1) { scene.boxes.push_back({x - 0.15, z - 0.15, x + 0.15, z + 0.15, 0.3}); name = "box 0.3 m"; }
                if (kind == 2) { scene.cylinders.push_back({x, z, 0.15, 0.35}); name = "bucket"; }
                const auto cloud = sim::scan(scene, sim::SensorSpec{}, 7);
                Cloud::Ptr cluster(new Cloud);
                for (const auto& p : *cloud) if (p.y > -scene.sensor_height + 0.02) cluster->push_back(p);
                if (cluster->size() < 10) continue;  // the detector skips clusters below 10 points
                views.push_back({name, range, bearing, alignClusterToConeModel(cluster, model, 40),
                                 static_cast<int>(cluster->size())});
            }
        }
    }
    std::printf("\n  %-10s %5s %8s %6s %8s %8s %10s\n", "object", "range", "bearing", "points", "|yaw|", "tilt", "fitness");
    for (const auto& v : views)
        std::printf("  %-10s %5.1f %8.0f %6d %8.1f %8.1f %10.5f\n", v.object.c_str(), v.range, v.bearing_deg, v.points,
                    std::abs(v.a.yaw_deg), v.a.tilt_deg, v.a.fitness);

    auto share = [&](const std::string& object, auto accepted) {
        int n = 0, k = 0;
        for (const auto& v : views) if (v.object == object) { ++n; k += accepted(v.a); }
        return n ? 100.0 * k / n : 0.0;
    };
    auto yaw_gate = [](const ConeAlignment& a) { return a.fitness < 0.005 && std::abs(a.yaw_deg) < 13; };
    std::printf("\n  accepted by the current gate (fitness < 0.005, |yaw| < 13 deg): cones %.0f%%, boxes %.0f%%, buckets %.0f%%\n",
                share("cone", yaw_gate), share("box 0.3 m", yaw_gate), share("bucket", yaw_gate));

    std::printf(failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", failures);
    return failures ? 1 : 0;
}
