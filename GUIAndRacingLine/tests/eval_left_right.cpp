// Evaluation of cones::distinguishLeftRight.
//
//  * Automatic: synthetic tracks with ground truth (tests/synthetic_tracks.h), clean and
//    perturbed (noise, missing cones). Prints metrics; exit code 1 if a clean track is
//    not classified perfectly.
//  * Human in the loop: writes an HTML report with a top-down view of every clean track,
//    of the worst perturbed run of each track and, with --pcd, of the real point cloud
//    run through the same pipeline as the GUI.
//
// Usage: eval_left_right [--report out.html] [--seeds N]
//                        [--pcd cones.pcd --model ideal_cone.ply] [--max-distance 6]

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "logic/distinguish_left_right.h"
#include "logic/get_cones_cloud.h"
#include "report_html.h"
#include "synthetic_tracks.h"

namespace {

constexpr int UNASSIGNED = -1, LEFT = 0, RIGHT = 1;

struct Metrics {
    int in_range = 0;      // cones expected to be classified, plus any other cone that was
    int correct = 0;
    int wrong_side = 0;
    int unassigned = 0;
    int order_errors = 0;  // consecutive cones of a path that go backwards along the track
    bool perfect() const { return wrong_side == 0 && unassigned == 0 && order_errors == 0; }
    double accuracy() const { return in_range ? double(correct) / in_range : 1.0; }
};

struct Run {
    std::vector<synth::Cone> cones;
    std::vector<int> label;               // per cone: LEFT, RIGHT or UNASSIGNED
    std::vector<int> left_path, right_path;
    Metrics m;
};

// Runs the classifier on `cones` and maps the output points back to the input cones.
Run classify(const std::vector<synth::Cone>& cones, double max_distance, unsigned seed)
{
    Run r;
    r.cones = cones;
    r.label.assign(cones.size(), UNASSIGNED);

    // Cone centres are at a random height: the classification must not depend on y.
    std::mt19937 rng(seed + 12345);
    std::uniform_real_distribution<float> height(-0.8f, -0.5f);
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::map<std::pair<float, float>, int> index_of;
    for (int i = 0; i < static_cast<int>(cones.size()); ++i) {
        pcl::PointXYZ p(static_cast<float>(cones[i].x), height(rng), static_cast<float>(cones[i].z));
        cloud->push_back(p);
        index_of[{p.x, p.z}] = i;
    }

    const auto res = cones::distinguishLeftRight(cloud, 0, 0, -45, 45, max_distance);

    auto collect = [&](const pcl::PointCloud<pcl::PointXYZ>& side, int lab, std::vector<int>& path) {
        for (const auto& p : side) {
            auto it = index_of.find({p.x, p.z});
            if (it == index_of.end()) { std::cerr << "output point not in input\n"; std::exit(2); }
            r.label[it->second] = lab;
            path.push_back(it->second);
        }
    };
    collect(*res.left, LEFT, r.left_path);
    collect(*res.right, RIGHT, r.right_path);

    // A cone is expected to be classified if it, and every cone before it on its boundary,
    // is within range: a boundary that leaves the range and comes back is not reachable.
    std::vector<char> expected(cones.size(), 0);
    for (auto side : {synth::Side::Left, synth::Side::Right}) {
        std::vector<int> b;
        for (int i = 0; i < static_cast<int>(cones.size()); ++i) if (cones[i].side == side) b.push_back(i);
        std::sort(b.begin(), b.end(), [&](int a, int c) { return cones[a].order < cones[c].order; });
        for (int i : b) {
            if (cones[i].x*cones[i].x + cones[i].z*cones[i].z > max_distance*max_distance) break;
            expected[i] = 1;
        }
    }

    for (int i = 0; i < static_cast<int>(cones.size()); ++i) {
        const auto& c = cones[i];
        if (!expected[i] && r.label[i] == UNASSIGNED) continue;
        ++r.m.in_range;
        const int truth = c.side == synth::Side::Left ? LEFT : RIGHT;
        if (r.label[i] == UNASSIGNED) ++r.m.unassigned;
        else if (r.label[i] == truth) ++r.m.correct;
        else ++r.m.wrong_side;
    }
    auto count_order_errors = [&](const std::vector<int>& path, int lab) {
        int prev = -1;
        for (int i : path) {
            const int truth = cones[i].side == synth::Side::Left ? LEFT : RIGHT;
            if (truth != lab) continue;
            if (prev >= 0 && cones[i].order <= cones[prev].order) ++r.m.order_errors;
            prev = i;
        }
    };
    count_order_errors(r.left_path, LEFT);
    count_order_errors(r.right_path, RIGHT);
    return r;
}

// ---------------------------------------------------------------- SVG rendering

std::string fmt(double v, int prec = 2)
{
    std::ostringstream o;
    o.setf(std::ios::fixed);
    o.precision(prec);
    o << v;
    return o.str();
}

// Top-down view: forward (+x) points up, right (+z) points right, as seen by the driver.
// has_truth = false for real data (no ground truth to compare with).
std::string render_svg(const Run& r, double max_distance, bool has_truth,
                       const std::vector<std::pair<double, double>>& extra_points = {})
{
    double min_x = 0, max_x = 0, min_z = 0, max_z = 0;
    for (const auto& c : r.cones) {
        min_x = std::min(min_x, c.x); max_x = std::max(max_x, c.x);
        min_z = std::min(min_z, c.z); max_z = std::max(max_z, c.z);
    }
    const double margin = 1.0;
    min_x -= margin; max_x += margin; min_z -= margin; max_z += margin;
    const double w_m = max_z - min_z, h_m = max_x - min_x;
    const double scale = std::min(420.0 / w_m, 420.0 / h_m);
    const double W = w_m * scale, H = h_m * scale;
    auto px = [&](double z) { return (z - min_z) * scale; };
    auto py = [&](double x) { return (max_x - x) * scale; };
    const double cone_r = std::clamp(0.18 * scale, 3.0, 7.0);

    std::ostringstream s;
    s << "<svg viewBox='0 0 " << fmt(W, 0) << " " << fmt(H, 0) << "' width='" << fmt(W, 0)
      << "' height='" << fmt(H, 0) << "' xmlns='http://www.w3.org/2000/svg'>";
    s << "<rect width='100%' height='100%' fill='var(--plot)'/>";
    // range of the classifier
    if (std::isfinite(max_distance))
        s << "<circle cx='" << fmt(px(0)) << "' cy='" << fmt(py(0)) << "' r='" << fmt(max_distance * scale)
          << "' fill='none' stroke='var(--muted)' stroke-dasharray='4 4'/>";
    // car longitudinal axis
    s << "<line x1='" << fmt(px(0)) << "' y1='0' x2='" << fmt(px(0)) << "' y2='" << fmt(H)
      << "' stroke='var(--grid)'/>";
    for (const auto& p : extra_points)
        s << "<circle cx='" << fmt(px(p.second)) << "' cy='" << fmt(py(p.first)) << "' r='1' fill='var(--muted)'/>";
    // ground-truth boundaries (thin, dashed)
    if (has_truth) {
        for (auto side : {synth::Side::Left, synth::Side::Right}) {
            std::vector<const synth::Cone*> b;
            for (const auto& c : r.cones) if (c.side == side) b.push_back(&c);
            std::sort(b.begin(), b.end(), [](auto* a, auto* c) { return a->order < c->order; });
            s << "<polyline fill='none' stroke='var(--muted)' stroke-width='1' stroke-dasharray='2 3' points='";
            for (auto* c : b) s << fmt(px(c->z)) << "," << fmt(py(c->x)) << " ";
            s << "'/>";
        }
    }
    // predicted paths, in the order returned by the classifier
    auto path_line = [&](const std::vector<int>& path, const char* color) {
        s << "<polyline fill='none' stroke='" << color << "' stroke-width='2' points='";
        for (int i : path) s << fmt(px(r.cones[i].z)) << "," << fmt(py(r.cones[i].x)) << " ";
        s << "'/>";
    };
    path_line(r.left_path, "var(--left)");
    path_line(r.right_path, "var(--right)");
    // cones
    for (size_t i = 0; i < r.cones.size(); ++i) {
        const auto& c = r.cones[i];
        const char* fill = r.label[i] == LEFT ? "var(--left)" : r.label[i] == RIGHT ? "var(--right)" : "var(--plot)";
        const int truth = c.side == synth::Side::Left ? LEFT : RIGHT;
        const bool wrong = has_truth && r.label[i] != UNASSIGNED && r.label[i] != truth;
        const bool missed = r.label[i] == UNASSIGNED;
        s << "<circle cx='" << fmt(px(c.z)) << "' cy='" << fmt(py(c.x)) << "' r='" << fmt(cone_r)
          << "' fill='" << fill << "' stroke='" << (wrong ? "var(--bad)" : missed ? "var(--muted)" : "var(--ink)")
          << "' stroke-width='" << (wrong ? 3 : 1) << "'><title>#" << i << " x=" << fmt(c.x) << " z=" << fmt(c.z)
          << (has_truth ? (truth == LEFT ? " truth=L" : " truth=R") : "") << "</title></circle>";
    }
    // position in each path
    auto path_numbers = [&](const std::vector<int>& path) {
        for (size_t k = 0; k < path.size(); ++k) {
            const auto& c = r.cones[path[k]];
            s << "<text x='" << fmt(px(c.z) + cone_r + 2) << "' y='" << fmt(py(c.x) + 4)
              << "' font-size='10' fill='var(--ink)'>" << k << "</text>";
        }
    };
    path_numbers(r.left_path);
    path_numbers(r.right_path);
    // car
    s << "<polygon points='" << fmt(px(0)) << "," << fmt(py(0) - 9) << " " << fmt(px(0) - 6) << "," << fmt(py(0) + 6)
      << " " << fmt(px(0) + 6) << "," << fmt(py(0) + 6) << "' fill='var(--ink)'/>";
    s << "</svg>";
    return s.str();
}

std::string metrics_line(const Metrics& m)
{
    return std::to_string(m.correct) + "/" + std::to_string(m.in_range) + " correct, " +
           std::to_string(m.wrong_side) + " wrong side, " + std::to_string(m.unassigned) + " unassigned, " +
           std::to_string(m.order_errors) + " order errors";
}

struct Panel {
    std::string id, title, subtitle, svg;
    bool ok;
    bool has_truth;
};

struct SummaryRow {
    std::string track, perturbation;
    int runs = 0, perfect = 0;
    double accuracy = 0, wrong = 0, unassigned = 0, order = 0;
};

void write_report(const std::string& path, const std::vector<SummaryRow>& rows, const std::vector<Panel>& panels)
{
    std::ofstream f(path);
    f << html_head("Cone Side Report") << R"HTML(
<h1>Left/right cone classification</h1>
<p>Top-down views: the car (triangle) is at the origin and points up (+x); right is +z. Numbers are each cone's position along its predicted path.</p>
<p class="legend"><span><i class="dot" style="background:var(--left)"></i>predicted left</span><span><i class="dot" style="background:var(--right)"></i>predicted right</span><span><i class="dot" style="background:var(--plot);border-color:var(--muted)"></i>unassigned</span><span><i class="dot" style="background:var(--plot);border:3px solid var(--bad)"></i>wrong side (synthetic only)</span><span>dashed lines: true boundaries; dashed circle: max distance</span></p>
)HTML";
    f << "<h2>Summary over random seeds</h2><div class='wrap'><table><tr><th>Track</th><th>Perturbation</th><th>Runs</th>"
         "<th>Perfect runs</th><th>Mean accuracy</th><th>Wrong side / run</th><th>Unassigned / run</th><th>Order errors / run</th></tr>";
    for (const auto& r : rows)
        f << "<tr><td>" << r.track << "</td><td>" << r.perturbation << "</td><td>" << r.runs << "</td><td class='"
          << (r.perfect == r.runs ? "ok" : "ko") << "'>" << fmt(100.0 * r.perfect / r.runs, 0) << "%</td><td>"
          << fmt(100.0 * r.accuracy, 1) << "%</td><td>" << fmt(r.wrong) << "</td><td>" << fmt(r.unassigned)
          << "</td><td>" << fmt(r.order) << "</td></tr>";
    f << "</table></div>";

    std::string section;
    for (const auto& p : panels) {
        const std::string sec = p.id.substr(0, 1) == "R" ? "Real data (no ground truth: judge by eye)"
                              : p.id.substr(0, 1) == "C" ? "Clean synthetic tracks"
                                                         : "Worst perturbed run of each track";
        if (sec != section) {
            if (!section.empty()) f << "</div>";
            f << "<h2>" << sec << "</h2><div class='grid'>";
            section = sec;
        }
        f << "<div class='card'><h3><span class='id'>" << p.id << "</span> " << p.title;
        if (p.has_truth) f << " <span class='" << (p.ok ? "ok" : "ko") << "'>" << (p.ok ? "✓" : "✗") << "</span>";
        f << "</h3><div class='sub'>" << p.subtitle << "</div>" << p.svg << "</div>";
    }
    if (!section.empty()) f << "</div>";
    f << "</body></html>\n";
}

// Same loading and detection pipeline as PCLViewer (load + compute buttons, default sliders).
bool real_data_panel(const std::string& pcd, const std::string& model_path, double max_distance, Panel& panel)
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto model = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cone_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(model_path, *model) == -1) return false;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd, *cloud) == -1) return false;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.0, 0.0, 0.1;
    transform.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(-M_PI / 1, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud, *cloud, transform);

    auto [centers, cones_pts] = get_cones_cloud(cloud, cone_cloud, 1.1, -10.0f, -0.2f, 0.005, false, model);
    const auto res = cones::distinguishLeftRight(centers, 0, 0, -45, 45, max_distance);

    Run r;
    std::map<std::pair<float, float>, int> index_of;
    for (const auto& p : *centers) {
        index_of[{p.x, p.z}] = static_cast<int>(r.cones.size());
        r.cones.push_back({p.x, p.z, synth::Side::Left, 0});
    }
    r.label.assign(r.cones.size(), UNASSIGNED);
    for (const auto& p : *res.left) { int i = index_of.at({p.x, p.z}); r.label[i] = LEFT; r.left_path.push_back(i); }
    for (const auto& p : *res.right) { int i = index_of.at({p.x, p.z}); r.label[i] = RIGHT; r.right_path.push_back(i); }

    std::cout << "\nReal data: " << pcd << " (" << centers->size() << " cones detected)\n";
    for (size_t i = 0; i < r.cones.size(); ++i)
        std::cout << "  cone " << i << "  x=" << fmt(r.cones[i].x) << "  z=" << fmt(r.cones[i].z) << "  -> "
                  << (r.label[i] == LEFT ? "left" : r.label[i] == RIGHT ? "right" : "unassigned") << "\n";

    std::vector<std::pair<double, double>> pts;
    for (const auto& p : *cones_pts) pts.push_back({p.x, p.z});
    panel = {"R1", pcd.substr(pcd.find_last_of('/') + 1),
             std::to_string(centers->size()) + " cones detected with the GUI default sliders, max distance " +
                 fmt(max_distance, 1) + " m. Grey dots: points of the detected cone clusters.",
             render_svg(r, max_distance, false, pts), true, false};
    return true;
}

}

int main(int argc, char** argv)
{
    std::string report = "left_right_report.html", pcd, model;
    int seeds = 100;
    double real_max_distance = 6.0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&] { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
        if (a == "--report") report = next();
        else if (a == "--seeds") seeds = std::stoi(next());
        else if (a == "--pcd") pcd = next();
        else if (a == "--model") model = next();
        else if (a == "--max-distance") real_max_distance = std::stod(next());
        else { std::cerr << "unknown argument " << a << "\n"; return 2; }
    }

    const std::vector<std::pair<std::string, synth::Perturbation>> perturbations = {
        {"noise 5 cm", {0.05, 0.0}},
        {"noise 15 cm", {0.15, 0.0}},
        {"10% cones missing", {0.0, 0.1}},
        {"noise 10 cm + 10% missing", {0.10, 0.1}},
    };

    std::vector<SummaryRow> rows;
    std::vector<Panel> clean_panels, worst_panels;
    bool clean_ok = true;

    std::printf("%-24s %-26s %7s %9s %9s\n", "track", "perturbation", "perfect", "accuracy", "unassig.");
    int n = 0;
    for (const auto& spec : synth::scenarios()) {
        ++n;
        const Run clean = classify(synth::generate(spec, {}, 0), spec.max_distance, 0);
        clean_ok &= clean.m.perfect();
        std::printf("%-24s %-26s %7s %8.1f%% %9d\n", spec.name.c_str(), "none",
                    clean.m.perfect() ? "yes" : "NO", 100 * clean.m.accuracy(), clean.m.unassigned);
        clean_panels.push_back({"C" + std::to_string(n), spec.name, spec.description + "<br>" + metrics_line(clean.m),
                                render_svg(clean, spec.max_distance, true), clean.m.perfect(), true});

        Run worst;
        std::string worst_label;
        unsigned worst_seed = 0;
        for (const auto& [label, pert] : perturbations) {
            SummaryRow row{spec.name, label};
            for (int s = 1; s <= seeds; ++s) {
                Run r = classify(synth::generate(spec, pert, s), spec.max_distance, s);
                ++row.runs;
                row.perfect += r.m.perfect();
                row.accuracy += r.m.accuracy();
                row.wrong += r.m.wrong_side;
                row.unassigned += r.m.unassigned;
                row.order += r.m.order_errors;
                const auto badness = [](const Metrics& m) { return m.wrong_side * 3 + m.unassigned + m.order_errors; };
                if (worst.cones.empty() || badness(r.m) > badness(worst.m)) {
                    worst = r; worst_label = label; worst_seed = s;
                }
            }
            row.accuracy /= row.runs; row.wrong /= row.runs; row.unassigned /= row.runs; row.order /= row.runs;
            std::printf("%-24s %-26s %6.0f%% %8.1f%% %9.2f\n", "", label.c_str(), 100.0 * row.perfect / row.runs,
                        100 * row.accuracy, row.unassigned);
            rows.push_back(row);
        }
        worst_panels.push_back({"W" + std::to_string(n), spec.name,
                                worst_label + ", seed " + std::to_string(worst_seed) + "<br>" + metrics_line(worst.m),
                                render_svg(worst, spec.max_distance, true), worst.m.perfect(), true});
    }

    std::vector<Panel> panels;
    if (!pcd.empty()) {
        Panel p;
        if (!real_data_panel(pcd, model, real_max_distance, p)) { std::cerr << "cannot load " << pcd << " / " << model << "\n"; return 2; }
        panels.push_back(p);
    }
    panels.insert(panels.end(), clean_panels.begin(), clean_panels.end());
    panels.insert(panels.end(), worst_panels.begin(), worst_panels.end());
    write_report(report, rows, panels);

    std::cout << "\nReport written to " << report << "\n";
    std::cout << (clean_ok ? "All clean tracks classified perfectly.\n" : "FAIL: some clean tracks are misclassified.\n");
    return clean_ok ? 0 : 1;
}
