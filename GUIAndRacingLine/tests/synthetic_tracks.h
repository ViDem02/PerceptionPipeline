#pragma once
// Synthetic cone tracks with ground-truth labels, in the viewer frame used by
// cones::distinguishLeftRight (x forward, y up, z right; car at the origin heading +x).

#include <cmath>
#include <random>
#include <string>
#include <vector>

namespace synth {

enum class Side { Left, Right };

struct Cone {
    double x;
    double z;
    Side side;
    int order;  // index along its boundary, 0 = closest to the start
};

// Piece of centreline with constant curvature. curvature > 0 turns right, < 0 turns left,
// 0 is a straight.
struct Segment {
    double length;
    double curvature;
};

struct TrackSpec {
    std::string name;
    std::string description;
    std::vector<Segment> segments;
    double width = 3.0;          // distance between left and right cones
    double spacing = 4.0;        // distance between consecutive cone pairs along the centreline
    double first_cone_s = 1.0;   // arclength of the first pair, measured from the car
    double car_lateral = 0.0;    // car offset from the centreline (+ = right)
    double car_yaw_deg = 0.0;    // car heading relative to the track (+ = pointing right)
    double max_distance = 20.0;  // passed to distinguishLeftRight
};

struct Perturbation {
    double noise_sigma = 0.0;    // gaussian noise on x and z [m]
    double dropout = 0.0;        // probability of losing each cone
};

inline std::vector<Cone> generate(const TrackSpec& spec, const Perturbation& pert, unsigned seed)
{
    // Integrate the centreline in track coordinates: start at the origin heading +x.
    std::vector<Cone> cones;
    const double ds = 0.005;
    double x = 0, z = 0, heading = 0, s = 0;
    double next_pair = spec.first_cone_s;
    int order = 0;
    for (const auto& seg : spec.segments) {
        for (double t = 0; t < seg.length; t += ds) {
            if (s >= next_pair) {
                // left normal of a direction (cos h, sin h) in (x, z) with z pointing right
                const double nx = std::sin(heading), nz = -std::cos(heading);
                const double hw = spec.width / 2;
                cones.push_back({x + hw*nx, z + hw*nz, Side::Left, order});
                cones.push_back({x - hw*nx, z - hw*nz, Side::Right, order});
                ++order;
                next_pair += spec.spacing;
            }
            x += ds * std::cos(heading);
            z += ds * std::sin(heading);
            heading += ds * seg.curvature;
            s += ds;
        }
    }

    // Express the cones in the car frame.
    const double yaw = spec.car_yaw_deg * M_PI / 180.0;
    const double cx = 0.0, cz = spec.car_lateral;
    for (auto& c : cones) {
        const double dx = c.x - cx, dz = c.z - cz;
        c.x =  std::cos(yaw)*dx + std::sin(yaw)*dz;
        c.z = -std::sin(yaw)*dx + std::cos(yaw)*dz;
    }

    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, pert.noise_sigma);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    std::vector<Cone> out;
    for (auto c : cones) {
        if (pert.dropout > 0 && uni(rng) < pert.dropout) continue;
        if (pert.noise_sigma > 0) { c.x += noise(rng); c.z += noise(rng); }
        out.push_back(c);
    }
    return out;
}

inline double radius_curv(double r) { return 1.0 / r; }

inline std::vector<TrackSpec> scenarios()
{
    std::vector<TrackSpec> v;
    // Formula Student-like geometry: 3 m wide, pairs every 4 m.
    v.push_back({"straight", "Straight, 3 m wide, pairs every 4 m",
                 {{40, 0}}});
    v.push_back({"left_R15", "Constant left curve, R = 15 m",
                 {{3, 0}, {40, -radius_curv(15)}}});
    v.push_back({"right_R15", "Constant right curve, R = 15 m",
                 {{3, 0}, {40, radius_curv(15)}}});
    v.push_back({"straight_then_left_R9", "Straight into a left curve, R = 9 m",
                 {{10, 0}, {30, -radius_curv(9)}}});
    v.push_back({"s_curve", "S-curve: right R = 10 m, then left R = 10 m",
                 {{3, 0}, {14, radius_curv(10)}, {14, -radius_curv(10)}}});
    v.push_back({"hairpin_right_R6", "Hairpin, right, R = 6 m (inner R = 4.5 m), pairs every 2.5 m",
                 {{4, 0}, {19, radius_curv(6)}, {8, 0}}, 3.0, 2.5});
    v.push_back({"straight_offset_yaw", "Straight, car 0.8 m right of centre and yawed 15 deg left",
                 {{40, 0}}, 3.0, 4.0, 2.0, 0.8, -15});
    v.push_back({"left_R15_spacing5", "Left curve R = 15 m, pairs every 5 m (FS max on straights)",
                 {{3, 0}, {40, -radius_curv(15)}}, 3.0, 5.0});
    // Small-scale geometry like external_files/cones.pcd (~1.3 m wide), GUI max distance 6 m.
    v.push_back({"lab_straight", "Lab scale: straight, 1.3 m wide, pairs every 1.2 m, max dist 6 m",
                 {{8, 0}}, 1.3, 1.2, 2.8, 0, 0, 6.0});
    v.push_back({"lab_right_R4", "Lab scale: right curve R = 4 m, 1.3 m wide, pairs every 1.2 m, max dist 6 m",
                 {{1, 0}, {10, radius_curv(4)}}, 1.3, 1.2, 2.8, 0, 0, 6.0});
    return v;
}

}
