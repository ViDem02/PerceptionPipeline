#include "distinguish_left_right.h"

#include <algorithm>
#include <vector>
#include <cmath>
#include <limits>

/*
 Reference frame (the one produced by the transform in PCLViewer::loadButtonPressed,
 checked on external_files/cones.pcd):
   x : forward, direction of travel of the car (LiDAR only sees x >= 0)
   y : up       (ground at y ~ -0.8, cones are kept below y = -0.2)
   z : right
 The track lies on the ground plane (x, z). y is a height and is ignored when measuring
 distances and angles. The output clouds contain the input points unchanged, so they are
 expressed in the same frame as the input.
*/

namespace cones{

namespace {

// Projection of a point on the ground plane.
struct Ground {
    double x;
    double z;
};

Ground ground(const pcl::PointXYZ& p) { return {p.x, p.z}; }

double dist_sq(const Ground& a, const Ground& b) {
    const double dx = a.x - b.x;
    const double dz = a.z - b.z;
    return dx*dx + dz*dz;
}

double wrapToPi(double a) {
    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}

double deg(double rad) { return rad * 180.0 / M_PI; }

// Signed turn angle between prev->curr and curr->next (degrees, positive = turning right).
double turn_angle_deg(const Ground& prev, const Ground& curr, const Ground& next) {
    const double a1 = std::atan2(curr.z - prev.z, curr.x - prev.x);
    const double a2 = std::atan2(next.z - curr.z, next.x - curr.x);
    return deg(wrapToPi(a2 - a1));
}

}

Result distinguishLeftRight(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cones,
    double robot_x, double robot_z,
    double angle_low_deg, double angle_high_deg,
    double max_distance)
{
    Result res{pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(),
               pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>()};

    const Ground robot{robot_x, robot_z};

    // Clamp negative distances to zero to avoid unintended inclusion.
    const double md = std::max(0.0, max_distance);
    const double max_dist_sq = md * md;

    // Cones within range: ground projection + index in src_cones.
    std::vector<Ground> cones;
    std::vector<int> src_idx;
    for (int i = 0; i < static_cast<int>(src_cones->size()); ++i)
    {
        const Ground p = ground(src_cones->points[i]);
        if (dist_sq(p, robot) <= max_dist_sq) {
            cones.push_back(p);
            src_idx.push_back(i);
        }
    }

    const int N = static_cast<int>(cones.size());

    // Seeds: closest cone on each side of the car. The car heads along +x,
    // so cones with z < robot_z are on its left.
    int seed_l = -1, seed_r = -1;
    double best_l = std::numeric_limits<double>::infinity();
    double best_r = std::numeric_limits<double>::infinity();
    for (int i = 0; i < N; ++i) {
        const double d = dist_sq(cones[i], robot);
        if (cones[i].z < robot.z) {
            if (d < best_l) { best_l = d; seed_l = i; }
        } else {
            if (d < best_r) { best_r = d; seed_r = i; }
        }
    }

    std::vector<int> path_l_idx;
    std::vector<int> path_r_idx;
    if (seed_l >= 0) path_l_idx.push_back(seed_l);
    if (seed_r >= 0) path_r_idx.push_back(seed_r);

    auto collect = [&]{
        res.left->reserve(path_l_idx.size());
        res.right->reserve(path_r_idx.size());
        for (int i : path_l_idx) res.left->push_back(src_cones->points[src_idx[i]]);
        for (int i : path_r_idx) res.right->push_back(src_cones->points[src_idx[i]]);
        return res;
    };

    // Both boundaries are needed to grow the paths.
    if (seed_l < 0 || seed_r < 0) return collect();

    std::vector<char> remaining(N, 1);
    remaining[seed_l] = remaining[seed_r] = 0;

    // Last direction of `path` (unit vector), or of `other` if `path` has a single cone
    // (the two boundaries run parallel), or the car heading (+x) if both have a single cone.
    auto direction = [&](const std::vector<int>& path, const std::vector<int>& other)->Ground{
        const std::vector<int>& p = path.size() >= 2 ? path : other;
        if (p.size() < 2) return {1.0, 0.0};
        const Ground& end = cones[p.back()];
        const Ground& prev = cones[p[p.size()-2]];
        const double n = std::hypot(end.x - prev.x, end.z - prev.z);
        return {(end.x - prev.x) / n, (end.z - prev.z) / n};
    };

    // Signed distance of `p` from the line through the last cone of `path`, along the path's
    // direction (positive = right).
    auto lateral = [&](const std::vector<int>& path, const std::vector<int>& other, const Ground& p)->double{
        const Ground& end = cones[path.back()];
        const Ground d = direction(path, other);
        return d.x * (p.z - end.z) - d.z * (p.x - end.x);
    };

    // A cone fits the left path if it lies on the left of the right boundary and closer to the
    // left boundary than to the right one (both extrapolated along their last direction);
    // symmetrically for the right path. The two conditions are mutually exclusive, so a path
    // cannot jump across the track when the nearest cone is on the opposite boundary
    // (tight curves, missing cones).
    auto fits_left = [&](int j){
        const double lat_l = lateral(path_l_idx, path_r_idx, cones[j]);
        const double lat_r = lateral(path_r_idx, path_l_idx, cones[j]);
        return lat_r < 0 && std::abs(lat_l) < std::abs(lat_r);
    };
    auto fits_right = [&](int j){
        const double lat_l = lateral(path_l_idx, path_r_idx, cones[j]);
        const double lat_r = lateral(path_r_idx, path_l_idx, cones[j]);
        return lat_l > 0 && std::abs(lat_r) < std::abs(lat_l);
    };

    // The path must not turn more than the given limits.
    auto turn_ok = [&](const std::vector<int>& path, int j){
        if (path.size() < 2) return true;
        const double a = turn_angle_deg(cones[path[path.size()-2]], cones[path.back()], cones[j]);
        return a >= angle_low_deg && a <= angle_high_deg;
    };

    // nearest remaining cone to the end of `path` that fits its side and turn limits
    auto next_cone = [&](const std::vector<int>& path, auto fits)->int{
        int best = -1; double bestd = std::numeric_limits<double>::infinity();
        for (int j = 0; j < N; ++j) if (remaining[j] && fits(j) && turn_ok(path, j)) {
            double d = dist_sq(cones[j], cones[path.back()]);
            if (d < bestd) { bestd = d; best = j; }
        }
        return best;
    };

    // Grow both paths one cone at a time until neither can be extended.
    while (true) {
        const int candL = next_cone(path_l_idx, fits_left);
        const int candR = next_cone(path_r_idx, fits_right);
        if (candL < 0 && candR < 0) break;
        if (candL >= 0) { path_l_idx.push_back(candL); remaining[candL] = 0; }
        if (candR >= 0) { path_r_idx.push_back(candR); remaining[candR] = 0; }
    }

    return collect();
}

}
