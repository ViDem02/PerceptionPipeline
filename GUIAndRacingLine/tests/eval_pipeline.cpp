// End-to-end ground-truth evaluation of the cone pipeline on simulated scans.
//
// Each scene (tests/lidar_sim.h) has cones at known positions and sides, walls and
// obstacles. The scan goes through the same steps and default sliders as the GUI:
// get_cones_cloud -> distinguishLeftRight -> get_racing_line_point_cloud. Metrics:
//   detection  precision, recall (on cones hit by >= 10 rays: the detector discards
//              smaller clusters), centroid error on the ground plane
//   left/right side accuracy of the detected cones
//   racing line share of the line (after the first gate) inside the true track
// Writes an HTML report for visual review. Exit code 1 if a check fails.
//
// Usage: eval_pipeline --model ideal_cone.ply [--report out.html] [--seeds N]

#include <algorithm>
#include <array>
#include <chrono>
#include <tuple>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/io/ply_io.h>

#include "lidar_sim.h"
#include "logic/distinguish_left_right.h"
#include "logic/get_cones_cloud.h"
#include "logic/racingline_nlopt.h"
#include "report_html.h"
#include "synthetic_tracks.h"

namespace {

// GUI defaults (PCLViewer::modelParameterSetting)
constexpr double kSor = 1.1, kCutBottom = -10.0, kCutTop = -0.2, kFitness = 0.005, kLeftRightMaxDist = 6.0;
constexpr double kMatchRadius = 0.3;  // detection <-> true cone association [m]
constexpr int kMinRays = 10;

struct SceneSpec {
    std::string name, description;
    synth::TrackSpec track;
};

struct GtCone {
    double x, z;
    synth::Side side;
    int order;
    int rays = 0;
};

struct Result {
    std::vector<GtCone> gt;
    std::vector<std::array<double, 2>> detections;
    std::vector<int> det_match;    // per detection: index in gt or -1 (false positive)
    std::vector<int> det_label;    // per detection: 0 left, 1 right, -1 unassigned
    std::vector<int> gt_detected;  // per gt cone: detection index or -1
    std::vector<std::array<double, 2>> line;
    std::vector<std::array<double, 2>> object_points;  // above-ground points, for the drawing
    // metrics
    int visible = 0, tp = 0, fp = 0, fn_visible = 0;
    double err_sum = 0, err_max = 0;
    int side_correct = 0, side_wrong = 0, side_unassigned = 0;
    int line_points = 0, line_inside = 0;
    double ms = 0;
};

sim::Scene make_scene(const SceneSpec& spec, std::vector<GtCone>& gt)
{
    sim::Scene scene;
    for (const auto& c : synth::generate(spec.track, {}, 0)) {
        if (c.x > 8.5 || std::abs(c.z) > 4.5) continue;  // inside the walls
        scene.cones.push_back({c.x, c.z});
        gt.push_back({c.x, c.z, c.side, c.order});
    }
    // walls: left, right, back (3.5 m tall, like the real scene)
    scene.boxes.push_back({0, -5.2, 9.2, -5.0, 3.5});
    scene.boxes.push_back({0, 5.0, 9.2, 5.2, 3.5});
    scene.boxes.push_back({9.0, -5.2, 9.2, 5.2, 3.5});
    // obstacles, skipped if they would stand on the track
    auto clear = [&](double x, double z, double r) {
        for (const auto& c : gt) if (std::hypot(c.x - x, c.z - z) < r + 0.6) return false;
        return true;
    };
    const std::vector<sim::Box> boxes = {
        {4.0, 3.0, 4.6, 3.6, 1.0},     // crate
        {5.0, -3.0, 5.3, -2.7, 0.3},   // cone-sized box: hard negative
    };
    for (const auto& b : boxes)
        if (clear((b.x0 + b.x1) / 2, (b.z0 + b.z1) / 2, 0.5)) scene.boxes.push_back(b);
    const std::vector<sim::Cylinder> cyl = {
        {6.5, -3.5, 0.05, 2.0},  // pole
        {7.0, 3.2, 0.25, 1.7},   // person
        {3.5, -2.2, 0.15, 0.4},  // bucket, cone height: hard negative
    };
    for (const auto& c : cyl)
        if (clear(c.x, c.z, c.radius)) scene.cylinders.push_back(c);
    return scene;
}

// Lateral distance of p from the polyline `mid` and whether p projects past its first point.
std::pair<double, bool> distance_to_centreline(const std::array<double, 2>& p, const std::vector<std::array<double, 2>>& mid)
{
    double best = 1e9;
    bool past_start = false;
    for (size_t i = 0; i + 1 < mid.size(); ++i) {
        const double ax = mid[i][0], az = mid[i][1], bx = mid[i+1][0], bz = mid[i+1][1];
        const double L2 = (bx - ax)*(bx - ax) + (bz - az)*(bz - az);
        double t = ((p[0] - ax)*(bx - ax) + (p[1] - az)*(bz - az)) / L2;
        const bool before = (i == 0 && t < 0);
        t = std::clamp(t, 0.0, 1.0);
        const double d = std::hypot(p[0] - ax - t*(bx - ax), p[1] - az - t*(bz - az));
        if (d < best) { best = d; past_start = !before; }
    }
    return {best, past_start};
}

Result run(const SceneSpec& spec, const pcl::PointCloud<pcl::PointXYZ>::Ptr& model, unsigned seed)
{
    Result r;
    const sim::Scene scene = make_scene(spec, r.gt);
    std::vector<int> cone_of_point;
    auto cloud = sim::scan(scene, sim::SensorSpec{}, seed, &cone_of_point);
    for (int c : cone_of_point) if (c >= 0) ++r.gt[c].rays;
    for (const auto& p : *cloud)
        if (p.y > -scene.sensor_height + 0.05 && (std::abs(p.z) < 4.9 && p.x < 8.9)) r.object_points.push_back({p.x, p.z});

    auto cone_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const auto t0 = std::chrono::steady_clock::now();
    auto [centers, cones_pts] = get_cones_cloud(cloud, cone_cloud, kSor, kCutBottom, kCutTop, kFitness, false, model);
    const auto lr = cones::distinguishLeftRight(centers, 0, 0, -45, 45, kLeftRightMaxDist);
    const auto line = get_racing_line_point_cloud(lr.left, lr.right, 20, 0);
    r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    for (const auto& p : *centers) r.detections.push_back({p.x, p.z});
    for (const auto& p : *line) r.line.push_back({p.x, p.z});

    // greedy association, closest pairs first
    r.det_match.assign(r.detections.size(), -1);
    r.gt_detected.assign(r.gt.size(), -1);
    std::vector<std::tuple<double, int, int>> pairs;
    for (size_t d = 0; d < r.detections.size(); ++d)
        for (size_t g = 0; g < r.gt.size(); ++g) {
            const double dist = std::hypot(r.detections[d][0] - r.gt[g].x, r.detections[d][1] - r.gt[g].z);
            if (dist < kMatchRadius) pairs.emplace_back(dist, d, g);
        }
    std::sort(pairs.begin(), pairs.end());
    for (auto [dist, d, g] : pairs) {
        if (r.det_match[d] >= 0 || r.gt_detected[g] >= 0) continue;
        r.det_match[d] = g;
        r.gt_detected[g] = d;
        r.err_sum += dist;
        r.err_max = std::max(r.err_max, dist);
    }

    for (const auto& g : r.gt) r.visible += g.rays >= kMinRays;
    for (size_t g = 0; g < r.gt.size(); ++g) {
        if (r.gt_detected[g] >= 0) ++r.tp;
        else if (r.gt[g].rays >= kMinRays) ++r.fn_visible;
    }
    r.fp = static_cast<int>(std::count(r.det_match.begin(), r.det_match.end(), -1));

    // sides: the classifier returns the input points, match them back by coordinates
    r.det_label.assign(r.detections.size(), -1);
    for (int side = 0; side < 2; ++side)
        for (const auto& p : side == 0 ? *lr.left : *lr.right)
            for (size_t d = 0; d < centers->size(); ++d)
                if ((*centers)[d].x == p.x && (*centers)[d].z == p.z) r.det_label[d] = side;
    for (size_t d = 0; d < r.detections.size(); ++d) {
        if (r.det_match[d] < 0) continue;
        if (std::hypot(r.detections[d][0], r.detections[d][1]) > kLeftRightMaxDist) continue;
        const int truth = r.gt[r.det_match[d]].side == synth::Side::Left ? 0 : 1;
        if (r.det_label[d] < 0) ++r.side_unassigned;
        else if (r.det_label[d] == truth) ++r.side_correct;
        else ++r.side_wrong;
    }

    // racing line inside the true track (centreline from the true gates)
    std::vector<std::array<double, 2>> mid;
    double half_width = spec.track.width / 2;
    for (int k = 0;; ++k) {
        const GtCone *l = nullptr, *rr = nullptr;
        for (const auto& g : r.gt) if (g.order == k) (g.side == synth::Side::Left ? l : rr) = &g;
        if (!l || !rr) break;
        mid.push_back({(l->x + rr->x) / 2, (l->z + rr->z) / 2});
    }
    for (const auto& p : r.line) {
        auto [dist, past_start] = distance_to_centreline(p, mid);
        if (!past_start) continue;
        ++r.line_points;
        r.line_inside += dist <= half_width;
    }
    return r;
}

std::string fmt(double v, int prec = 2)
{
    std::ostringstream o;
    o.setf(std::ios::fixed);
    o.precision(prec);
    o << v;
    return o.str();
}

std::string render(const Result& r)
{
    const double min_x = -0.5, max_x = 9.5, min_z = -5.5, max_z = 5.5;
    const double scale = 40;
    const double W = (max_z - min_z) * scale, H = (max_x - min_x) * scale;
    auto px = [&](double z) { return fmt((z - min_z) * scale); };
    auto py = [&](double x) { return fmt((max_x - x) * scale); };
    std::ostringstream s;
    s << "<svg viewBox='0 0 " << fmt(W, 0) << " " << fmt(H, 0) << "' width='" << fmt(W, 0) << "' height='" << fmt(H, 0)
      << "' xmlns='http://www.w3.org/2000/svg'><rect width='100%' height='100%' fill='var(--plot)'/>";
    s << "<circle cx='" << px(0) << "' cy='" << py(0) << "' r='" << fmt(kLeftRightMaxDist * scale)
      << "' fill='none' stroke='var(--muted)' stroke-dasharray='4 4'/>";
    for (size_t i = 0; i < r.object_points.size(); i += 2)
        s << "<circle cx='" << px(r.object_points[i][1]) << "' cy='" << py(r.object_points[i][0]) << "' r='0.8' fill='var(--muted)'/>";
    // true cones: hollow, coloured by true side; thick red if visible but missed
    for (size_t g = 0; g < r.gt.size(); ++g) {
        const auto& c = r.gt[g];
        const bool missed = r.gt_detected[g] < 0 && c.rays >= kMinRays;
        s << "<circle cx='" << px(c.z) << "' cy='" << py(c.x) << "' r='7' fill='none' stroke='"
          << (missed ? "var(--bad)" : c.side == synth::Side::Left ? "var(--left)" : "var(--right)")
          << "' stroke-width='" << (missed ? 3 : 1.5) << "'><title>true " << (c.side == synth::Side::Left ? "left" : "right")
          << " cone, " << c.rays << " rays" << (missed ? ", MISSED" : "") << "</title></circle>";
    }
    // racing line
    s << "<polyline fill='none' stroke='var(--line)' stroke-width='2' points='";
    for (const auto& p : r.line) s << px(p[1]) << "," << py(p[0]) << " ";
    s << "'/>";
    // detections: filled by predicted side; false positives as red crosses
    for (size_t d = 0; d < r.detections.size(); ++d) {
        const auto& p = r.detections[d];
        if (r.det_match[d] < 0) {
            s << "<path d='M" << fmt((p[1] - min_z) * scale - 6) << " " << fmt((max_x - p[0]) * scale - 6) << " l12 12 m0 -12 l-12 12'"
              << " stroke='var(--bad)' stroke-width='3'><title>false positive</title></path>";
            continue;
        }
        const int truth = r.gt[r.det_match[d]].side == synth::Side::Left ? 0 : 1;
        const bool wrong = r.det_label[d] >= 0 && r.det_label[d] != truth;
        s << "<circle cx='" << px(p[1]) << "' cy='" << py(p[0]) << "' r='4' fill='"
          << (r.det_label[d] == 0 ? "var(--left)" : r.det_label[d] == 1 ? "var(--right)" : "var(--muted)")
          << "' stroke='" << (wrong ? "var(--bad)" : "var(--ink)") << "' stroke-width='" << (wrong ? 2 : 0.5)
          << "'><title>detection, " << (r.det_label[d] == 0 ? "left" : r.det_label[d] == 1 ? "right" : "unassigned")
          << (wrong ? " (WRONG SIDE)" : "") << "</title></circle>";
    }
    s << "<polygon points='" << px(0) << "," << fmt((max_x) * scale - 9) << " " << fmt(-min_z * scale - 6) << ","
      << fmt(max_x * scale + 6) << " " << fmt(-min_z * scale + 6) << "," << fmt(max_x * scale + 6) << "' fill='var(--ink)'/>";
    s << "</svg>";
    return s.str();
}

}

int main(int argc, char** argv)
{
    std::string model_path, report = "pipeline_report.html";
    int seeds = 3;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&] { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
        if (a == "--model") model_path = next();
        else if (a == "--report") report = next();
        else if (a == "--seeds") seeds = std::stoi(next());
        else { std::cerr << "unknown argument " << a << "\n"; return 2; }
    }
    auto model = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (model_path.empty() || pcl::io::loadPLYFile<pcl::PointXYZ>(model_path, *model) == -1) {
        std::cerr << "usage: " << argv[0] << " --model ideal_cone.ply [--report out.html] [--seeds N]\n";
        return 2;
    }

    // Lab-scale tracks like cones.pcd (1.3 m wide, pairs every 1.2 m from 2.8 m) and an FS one.
    auto lab = [](std::vector<synth::Segment> seg) {
        synth::TrackSpec t{"", "", seg, 1.3, 1.2, 2.8};
        return t;
    };
    const std::vector<SceneSpec> scenes = {
        {"lab_straight", "Straight, 1.3 m wide, pairs every 1.2 m", lab({{8, 0}})},
        {"lab_left_R5", "Left curve R = 5 m", lab({{1, 0}, {10, -0.2}})},
        {"lab_right_R5", "Right curve R = 5 m", lab({{1, 0}, {10, 0.2}})},
        {"lab_s_curve", "S-curve, R = 4 m right then left", lab({{1, 0}, {2.5, 0.25}, {2.5, -0.25}, {4, 0}})},
        {"fs_straight", "Formula Student: 3 m wide, pairs every 4 m", synth::TrackSpec{"", "", {{9, 0}}, 3.0, 4.0, 1.0}},
    };

    std::ofstream f(report);
    f << html_head("Pipeline Report") << R"HTML(
<style>:root{--line:#ea580c}@media (prefers-color-scheme: dark){:root{--line:#fb923c}}</style>
<h1>Cone pipeline on simulated scans</h1>
<p>Each scene is a simulated scan (24,000 rays, sensor 0.82 m above the ground) with cones at known positions,
walls, a crate, a pole, a person and two cone-sized hard negatives. Same steps and default sliders as the GUI.</p>
<p class="legend"><span><i class="dot" style="background:var(--plot);border:2px solid var(--left)"></i>true left cone</span>
<span><i class="dot" style="background:var(--plot);border:2px solid var(--right)"></i>true right cone</span>
<span><i class="dot" style="background:var(--plot);border:3px solid var(--bad)"></i>missed (visible)</span>
<span><i class="dot" style="background:var(--left)"></i>/<i class="dot" style="background:var(--right)"></i>detected, predicted side</span>
<span style="color:var(--bad)">✕ false positive</span><span style="color:var(--line)">— racing line</span>
<span>grey: scan points above the ground; dashed circle: left/right max distance</span></p>
)HTML";

    std::printf("%-14s %4s %8s %8s %9s %9s %9s %9s %7s\n", "scene", "seed", "precis.", "recall", "err mean",
                "err max", "side ok", "line in", "ms");
    int total_tp = 0, total_fp = 0, total_fn = 0, total_side_ok = 0, total_side_wrong = 0, total_side_un = 0;
    int total_line = 0, total_line_in = 0;
    double total_err = 0;
    f << "<h2>Scenes</h2><div class='grid'>";
    for (size_t i = 0; i < scenes.size(); ++i) {
        for (int s = 1; s <= seeds; ++s) {
            const Result r = run(scenes[i], model, s);
            const double precision = r.tp + r.fp ? double(r.tp) / (r.tp + r.fp) : 1.0;
            const double recall = r.tp + r.fn_visible ? double(r.tp) / (r.tp + r.fn_visible) : 1.0;
            const int sides = r.side_correct + r.side_wrong + r.side_unassigned;
            const std::string line_in = r.line_points ? fmt(100.0 * r.line_inside / r.line_points, 0) + "%" : "no line";
            std::printf("%-14s %4d %7.0f%% %7.0f%% %8.3fm %8.3fm %4d/%-4d %9s %7.0f\n", scenes[i].name.c_str(), s,
                        100 * precision, 100 * recall, r.tp ? r.err_sum / r.tp : 0.0, r.err_max, r.side_correct, sides,
                        line_in.c_str(), r.ms);
            total_tp += r.tp; total_fp += r.fp; total_fn += r.fn_visible; total_err += r.err_sum;
            total_side_ok += r.side_correct; total_side_wrong += r.side_wrong; total_side_un += r.side_unassigned;
            total_line += r.line_points; total_line_in += r.line_inside;
            if (s == 1) {
                f << "<div class='card'><h3><span class='id'>P" << i + 1 << "</span> " << scenes[i].name << "</h3><div class='sub'>"
                  << scenes[i].description << "<br>" << r.tp << " detected of " << r.visible << " visible (" << r.gt.size()
                  << " in the scene), " << r.fp << " false positives, centroid error " << fmt(r.tp ? 100 * r.err_sum / r.tp : 0, 1)
                  << " cm mean; sides " << r.side_correct << " correct, " << r.side_wrong << " wrong, " << r.side_unassigned
                  << " unassigned; "
                  << (r.line_points ? "racing line " + fmt(100.0 * r.line_inside / r.line_points, 0) + "% inside the track"
                                    : std::string("no racing line (it needs two cones on each side)"))
                  << "</div>" << render(r) << "</div>";
            }
        }
    }
    f << "</div>";

    const double precision = double(total_tp) / std::max(1, total_tp + total_fp);
    const double recall = double(total_tp) / std::max(1, total_tp + total_fn);
    const double side_acc = double(total_side_ok) / std::max(1, total_side_ok + total_side_wrong + total_side_un);
    const double line_in = double(total_line_in) / std::max(1, total_line);
    std::printf("\nOverall: precision %.1f%%, recall %.1f%% (visible cones), centroid error %.1f cm mean,\n"
                "         sides %.1f%% correct (%d wrong, %d unassigned), racing line %.1f%% inside the track (where one was drawn)\n",
                100 * precision, 100 * recall, 100 * total_err / std::max(1, total_tp), 100 * side_acc,
                total_side_wrong, total_side_un, 100 * line_in);
    f << "</body></html>\n";
    std::cout << "Report written to " << report << "\n";
    return 0;
}
