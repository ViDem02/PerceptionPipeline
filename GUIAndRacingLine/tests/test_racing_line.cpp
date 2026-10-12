// Ground-truth tests for the racing line (logic/racingline_nlopt.cpp).
//
//  * Known optimum: on a straight the line is the centreline; on a constant-radius arc
//    the minimum-curvature line hugs the outer edge (largest radius). With few gates the
//    discrete optimum pulls the interior points a few cm inside the bound (its cost is
//    lower than with every point on the bound), hence the 5% tolerance.
//  * Start at the car: the turn at the car is part of the cost, so the line leaves the
//    car smoothly instead of jumping to the outer edge.
//  * Interpolation: the drawn polyline starts at the car, passes through every optimised
//    point in order, and every drawn point lies on its segment.
//  * Sanity on every synthetic track: offsets within the track, no reversal, progress
//    along the track.
//  * Degenerate inputs (no cones, one gate) return an empty line without crashing.
//
// Exit code 1 if any check fails.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "logic/racingline_nlopt.h"
#include "synthetic_tracks.h"

namespace {

int failures = 0;

void check(bool ok, const std::string& what)
{
    std::printf("  [%s] %s\n", ok ? "pass" : "FAIL", what.c_str());
    if (!ok) ++failures;
}

struct Boundaries {
    std::vector<std::vector<double>> left, right;  // {x, z}, ordered along the track
};

Boundaries boundaries(const std::vector<synth::Cone>& cones)
{
    Boundaries b;
    for (const auto& c : cones) {
        auto& side = c.side == synth::Side::Left ? b.left : b.right;
        side.push_back({c.x, c.z});
    }
    return b;  // the generator emits cones in track order
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_of(const std::vector<std::vector<double>>& pts)
{
    auto c = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (const auto& p : pts) c->push_back(pcl::PointXYZ(p[0], -0.6f, p[1]));
    return c;
}

double max_abs(const std::vector<double>& v, double ref = 0.0)
{
    double m = 0;
    for (double x : v) m = std::max(m, std::abs(x - ref));
    return m;
}

// Turning angle at every interior vertex of the polyline car -> p_0 -> ... (degrees).
std::vector<double> turning_angles(const RacingLine& rl)
{
    std::vector<std::array<double, 2>> v = {{0.0, 0.0}};
    v.insert(v.end(), rl.points.begin(), rl.points.end());
    std::vector<double> out;
    for (size_t i = 1; i + 1 < v.size(); ++i) {
        const double ax = v[i][0] - v[i-1][0], az = v[i][1] - v[i-1][1];
        const double bx = v[i+1][0] - v[i][0], bz = v[i+1][1] - v[i][1];
        out.push_back(std::atan2(ax*bz - az*bx, ax*bx + az*bz) * 180.0 / M_PI);
    }
    return out;
}

void check_interpolation(const RacingLine& rl, const pcl::PointCloud<pcl::PointXYZ>& drawn, int n_per_segment)
{
    std::vector<std::array<double, 2>> v = {{0.0, 0.0}};
    v.insert(v.end(), rl.points.begin(), rl.points.end());
    const size_t segments = v.size() - 1;

    check(drawn.size() == segments * (n_per_segment - 1) + 1,
          "drawn points = segments x (n - 1) + 1 = " + std::to_string(segments * (n_per_segment - 1) + 1) +
          " (got " + std::to_string(drawn.size()) + ")");
    if (drawn.empty()) return;

    // vertex k of the polyline is drawn point k * (n - 1)
    double vertex_err = 0;
    for (size_t k = 0; k < v.size(); ++k) {
        const size_t idx = k * (n_per_segment - 1);
        if (idx >= drawn.size()) { vertex_err = 1e9; break; }
        vertex_err = std::max(vertex_err, std::hypot(drawn[idx].x - v[k][0], drawn[idx].z - v[k][1]));
    }
    check(vertex_err < 1e-4, "starts at the car and passes through every optimised point, in order (max err " +
          std::to_string(vertex_err) + " m)");

    // every drawn point on its segment
    double off_segment = 0;
    for (size_t i = 0; i < drawn.size(); ++i) {
        const size_t s = std::min(i / (n_per_segment - 1), segments - 1);
        const double ax = v[s][0], az = v[s][1], bx = v[s+1][0], bz = v[s+1][1];
        const double px = drawn[i].x, pz = drawn[i].z;
        const double len = std::hypot(bx - ax, bz - az);
        const double dist = len > 0 ? std::abs((bx - ax)*(pz - az) - (bz - az)*(px - ax)) / len
                                    : std::hypot(px - ax, pz - az);
        off_segment = std::max(off_segment, dist);
    }
    check(off_segment < 1e-4, "every drawn point lies on its segment (max dist " + std::to_string(off_segment) + " m)");
}

}

int main()
{
    constexpr int n = 20;

    std::printf("Straight track: optimum is the centreline\n");
    {
        synth::TrackSpec spec{"straight", "", {{20, 0}}};
        const auto b = boundaries(synth::generate(spec, {}, 0));
        const auto rl = optimise_racing_line(b.left, b.right);
        check(rl.status > 0 && rl.points.size() == b.left.size(), "optimiser succeeded, one point per gate");
        check(max_abs(rl.offsets) < 0.01, "all offsets ~0 (max |d| = " + std::to_string(max_abs(rl.offsets)) + " m)");
        double max_z = 0;
        for (const auto& p : rl.points) max_z = std::max(max_z, std::abs(p[1]));
        check(max_z < 0.01, "points on the centreline z = 0 (max |z| = " + std::to_string(max_z) + " m)");
    }

    for (const double curvature : {-1.0 / 10, 1.0 / 10}) {
        const bool left_turn = curvature < 0;
        std::printf("Constant %s arc, R = 10 m: optimum hugs the outer (%s) edge\n",
                    left_turn ? "left" : "right", left_turn ? "right" : "left");
        synth::TrackSpec spec{"arc", "", {{0.5, 0}, {20, curvature}}};
        spec.first_cone_s = 0.0;
        const auto b = boundaries(synth::generate(spec, {}, 0));
        const auto rl = optimise_racing_line(b.left, b.right);
        check(rl.status > 0, "optimiser succeeded");
        // d > 0 is towards the right boundary (normal = tangent rotated towards +z).
        const double outer = left_turn ? 1.0 : -1.0;
        double worst = 0;
        for (size_t i = 0; i < rl.offsets.size(); ++i)
            worst = std::max(worst, (rl.half_width[i] - outer * rl.offsets[i]) / rl.half_width[i]);
        check(worst < 0.05, "every offset within 5% of the outer bound (max gap " +
              std::to_string(100 * worst) + "% of the half width)");
        check(std::abs(rl.offsets.front()) == rl.half_width.front() && std::abs(rl.offsets.back()) == rl.half_width.back(),
              "first and last offsets exactly on the outer bound");
    }

    std::printf("Start at the car (left curve R = 15 m, first gate 1 m ahead)\n");
    {
        const auto spec = synth::scenarios()[1];
        const auto b = boundaries(synth::generate(spec, {}, 0));
        const auto free_start = optimise_racing_line(b.left, b.right);
        const auto car_start = optimise_racing_line(b.left, b.right, std::array<double, 2>{0.0, 0.0});
        const double turn_free = std::abs(turning_angles(free_start).front());
        const double turn_car = std::abs(turning_angles(car_start).front());
        char buf[160];
        std::snprintf(buf, sizeof buf, "turn at the car: %.1f deg with the car in the cost, %.1f deg without",
                      turn_car, turn_free);
        check(car_start.status > 0 && turn_car < turn_free / 2, buf);
    }

    std::printf("Interpolation of the drawn line\n");
    {
        synth::TrackSpec spec{"s", "", {{3, 0}, {14, 0.1}, {14, -0.1}}};
        const auto b = boundaries(synth::generate(spec, {}, 0));
        const auto rl = optimise_racing_line(b.left, b.right, std::array<double, 2>{0.0, 0.0});
        const auto drawn = get_racing_line_point_cloud(cloud_of(b.left), cloud_of(b.right), n, 0);
        check_interpolation(rl, *drawn, n);
    }

    std::printf("Every synthetic track (clean, car as start): bounds, no reversal, progress\n");
    for (const auto& spec : synth::scenarios()) {
        auto b = boundaries(synth::generate(spec, {}, 0));
        // only the cones the car can see
        auto in_range = [&](std::vector<std::vector<double>>& side) {
            std::vector<std::vector<double>> out;
            for (const auto& p : side) {
                if (std::hypot(p[0], p[1]) > spec.max_distance) break;
                out.push_back(p);
            }
            side = out;
        };
        in_range(b.left);
        in_range(b.right);
        const auto rl = optimise_racing_line(b.left, b.right, std::array<double, 2>{0.0, 0.0});
        double bound_violation = 0;
        for (size_t i = 0; i < rl.offsets.size(); ++i)
            bound_violation = std::max(bound_violation, std::abs(rl.offsets[i]) - rl.half_width[i]);
        double max_turn = 0;
        for (double a : turning_angles(rl)) max_turn = std::max(max_turn, std::abs(a));
        bool progress = true;  // each point is further along the gate sequence than the previous
        for (size_t i = 1; i < rl.points.size(); ++i) {
            const double tx = (b.left[i][0] + b.right[i][0] - b.left[i-1][0] - b.right[i-1][0]) / 2;
            const double tz = (b.left[i][1] + b.right[i][1] - b.left[i-1][1] - b.right[i-1][1]) / 2;
            progress &= (rl.points[i][0] - rl.points[i-1][0]) * tx + (rl.points[i][1] - rl.points[i-1][1]) * tz > 0;
        }
        char buf[200];
        std::snprintf(buf, sizeof buf, "%-22s %2zu gates, max |d| - w/2 = %+.3f m, max turn %5.1f deg, progress %s",
                      spec.name.c_str(), rl.points.size(), bound_violation, max_turn, progress ? "yes" : "NO");
        check(rl.status > 0 && bound_violation <= 1e-9 && max_turn < 90 && progress, buf);
    }

    std::printf("Degenerate inputs\n");
    {
        auto empty = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        check(get_racing_line_point_cloud(empty, empty, n, 0)->empty(), "no cones -> empty line");
        const auto one = cloud_of({{3.0, -1.5}});
        const auto two = cloud_of({{3.0, 1.5}, {6.0, 1.5}});
        check(get_racing_line_point_cloud(one, two, n, 0)->empty(), "one gate -> empty line, no crash");
        const auto two_l = cloud_of({{3.0, -1.5}, {6.0, -1.5}});
        check(get_racing_line_point_cloud(two_l, two, n, 0)->size() == 2 * (n - 1) + 1, "two gates -> line drawn");
    }

    std::printf(failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", failures);
    return failures ? 1 : 0;
}
