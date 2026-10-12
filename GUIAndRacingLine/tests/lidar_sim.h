#pragma once
// Minimal LiDAR simulator: casts a grid of rays into a scene made of a flat ground,
// cones, boxes and vertical cylinders, and returns the first hit of every ray.
// Frame: the viewer frame (x forward, y up, z right), sensor at the origin.

#include <cmath>
#include <limits>
#include <optional>
#include <random>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace sim {

struct Vec { double x, y, z; };

inline Vec operator+(Vec a, Vec b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec operator-(Vec a, Vec b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec operator*(double s, Vec a) { return {s * a.x, s * a.y, s * a.z}; }
inline double dot(Vec a, Vec b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// Formula Student small cone, standing on the ground at (x, z).
struct Cone { double x, z, base_radius = 0.114, height = 0.325; };
// Axis-aligned box from the ground up.
struct Box { double x0, z0, x1, z1, height; };
// Vertical cylinder from the ground up (poles, people).
struct Cylinder { double x, z, radius, height; };

struct Scene {
    double sensor_height = 0.82;  // ground at y = -sensor_height (as in cones.pcd)
    std::vector<Cone> cones;
    std::vector<Box> boxes;
    std::vector<Cylinder> cylinders;
};

struct SensorSpec {
    int n_azimuth = 200, n_elevation = 120;      // 24,000 rays, like cones.pcd
    double azimuth_deg = 40;                      // +- horizontal half field of view
    double elevation_min_deg = -35, elevation_max_deg = 20;
    double max_range = 40;
    double range_noise = 0.01;                    // [m], gaussian
};

namespace detail {

constexpr double kNone = std::numeric_limits<double>::infinity();

inline double smallest_positive(double a, double b, double c, auto valid)
{
    double best = kNone;
    if (std::abs(a) < 1e-12) {
        if (std::abs(b) > 1e-12) { const double t = -c / b; if (t > 0 && valid(t)) best = t; }
        return best;
    }
    const double disc = b * b - 4 * a * c;
    if (disc < 0) return best;
    const double sq = std::sqrt(disc);
    for (double t : {(-b - sq) / (2 * a), (-b + sq) / (2 * a)})
        if (t > 0 && t < best && valid(t)) best = t;
    return best;
}

inline double hit_cone(Vec o, Vec d, const Cone& c, double ground)
{
    const Vec apex{c.x, ground + c.height, c.z};
    const Vec v{0, -1, 0};  // axis, apex -> base
    const double cos2 = 1.0 / (1.0 + std::pow(c.base_radius / c.height, 2));
    const Vec w = o - apex;
    const double dv = dot(d, v), wv = dot(w, v);
    const double a = dv * dv - cos2 * dot(d, d);
    const double b = 2 * (dv * wv - cos2 * dot(w, d));
    const double cc = wv * wv - cos2 * dot(w, w);
    return smallest_positive(a, b, cc, [&](double t) {
        const double h = dot(w + t * d, v);
        return h >= 0 && h <= c.height;
    });
}

inline double hit_cylinder(Vec o, Vec d, const Cylinder& c, double ground)
{
    const double ox = o.x - c.x, oz = o.z - c.z;
    const double a = d.x * d.x + d.z * d.z;
    const double b = 2 * (ox * d.x + oz * d.z);
    const double cc = ox * ox + oz * oz - c.radius * c.radius;
    return smallest_positive(a, b, cc, [&](double t) {
        const double y = o.y + t * d.y;
        return y >= ground && y <= ground + c.height;
    });
}

inline double hit_box(Vec o, Vec d, const Box& bx, double ground)
{
    double t0 = 0, t1 = kNone;
    const double lo[3] = {bx.x0, ground, bx.z0}, hi[3] = {bx.x1, ground + bx.height, bx.z1};
    const double oo[3] = {o.x, o.y, o.z}, dd[3] = {d.x, d.y, d.z};
    for (int k = 0; k < 3; ++k) {
        if (std::abs(dd[k]) < 1e-12) {
            if (oo[k] < lo[k] || oo[k] > hi[k]) return kNone;
            continue;
        }
        double ta = (lo[k] - oo[k]) / dd[k], tb = (hi[k] - oo[k]) / dd[k];
        if (ta > tb) std::swap(ta, tb);
        t0 = std::max(t0, ta);
        t1 = std::min(t1, tb);
        if (t0 > t1) return kNone;
    }
    return t0 > 0 ? t0 : kNone;
}

}

// cone_of_point, if given, receives for every point the index of the cone it hit (-1 otherwise).
inline pcl::PointCloud<pcl::PointXYZ>::Ptr scan(const Scene& scene, const SensorSpec& s, unsigned seed,
                                                std::vector<int>* cone_of_point = nullptr)
{
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, s.range_noise);
    std::uniform_real_distribution<double> jitter(-0.5, 0.5);
    const double ground = -scene.sensor_height;
    const Vec o{0, 0, 0};

    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const double daz = 2 * s.azimuth_deg / s.n_azimuth;
    const double del = (s.elevation_max_deg - s.elevation_min_deg) / s.n_elevation;
    for (int i = 0; i < s.n_azimuth; ++i) {
        for (int j = 0; j < s.n_elevation; ++j) {
            const double az = (-s.azimuth_deg + (i + 0.5 + jitter(rng)) * daz) * M_PI / 180;
            const double el = (s.elevation_min_deg + (j + 0.5 + jitter(rng)) * del) * M_PI / 180;
            const Vec d{std::cos(el) * std::cos(az), std::sin(el), std::cos(el) * std::sin(az)};

            double t = d.y < 0 ? (ground - o.y) / d.y : detail::kNone;
            int cone = -1;
            for (size_t k = 0; k < scene.cones.size(); ++k) {
                const double tc = detail::hit_cone(o, d, scene.cones[k], ground);
                if (tc < t) { t = tc; cone = static_cast<int>(k); }
            }
            const double t_cone = t;
            for (const auto& c : scene.cylinders) t = std::min(t, detail::hit_cylinder(o, d, c, ground));
            for (const auto& b : scene.boxes) t = std::min(t, detail::hit_box(o, d, b, ground));
            if (!(t < s.max_range)) continue;

            if (t < t_cone) cone = -1;  // a box or cylinder is in front of the cone
            t += noise(rng);
            cloud->push_back(pcl::PointXYZ(t * d.x, t * d.y, t * d.z));
            if (cone_of_point) cone_of_point->push_back(cone);
        }
    }
    return cloud;
}

}
