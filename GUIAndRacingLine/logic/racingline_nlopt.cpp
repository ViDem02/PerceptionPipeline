#include <nlopt.h>
#include <array>
#include <optional>
#include <utility>
#include <vector>
#include <iostream>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h> // for PointCloud
#include "racingline_nlopt.h"


/*
 Transliteration of pathplanning/racingline.py using NLOPT.

 We minimize curvature^2 + 0.1 * sum (d[i+1]-d[i])^2 subject
 to -half_width[i] <= d[i] <= half_width[i].

 The curvature at p_i is the turning angle between p_{i-1}->p_i and p_i->p_{i+1}
 divided by the mean length of the two segments. If a start point is given (the car),
 it is a fixed p_{-1}, so the line also has to leave the car smoothly.
*/

template <typename T>
std::vector<T> linspace(T a, T b, size_t N) {
    T h = (b - a) / static_cast<T>(N-1);
    std::vector<T> xs(N);
    typename std::vector<T>::iterator x;
    T val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        *x = val;
    return xs;
}

struct TrackData {
    std::vector<std::vector<double>> left {};
    std::vector<std::vector<double>> right{};
    int N = 0;
    std::vector<std::vector<double>> mid; // midpoint
    std::vector<double> half_width;
    std::vector<std::vector<double>> tangent; // unit tangent
    std::vector<std::vector<double>> normal;  // normal (rotated tangent)
    std::optional<std::array<double, 2>> start; // fixed point before the first gate

    TrackData(std::vector<std::vector<double>> arg_left, std::vector<std::vector<double>> arg_right) {
        left = std::move(arg_left);
        right = std::move(arg_right);
        N = std::min(left.size(), right.size());
        mid.resize(N, std::vector<double>(2));
        half_width.resize(N);
        for (int i=0;i<N;++i) {
            mid[i][0] = 0.5*(left[i][0] + right[i][0]);
            mid[i][1] = 0.5*(left[i][1] + right[i][1]);
            double dx = right[i][0] - left[i][0];
            double dy = right[i][1] - left[i][1];
            half_width[i] = 0.5 * std::sqrt(dx*dx + dy*dy);
        }
        tangent.resize(N, std::vector<double>(2,0.0));
        // central differences for interior, forward/backward at ends
        for (int i=1;i<N-1;++i) {
            tangent[i][0] = mid[i+1][0] - mid[i-1][0];
            tangent[i][1] = mid[i+1][1] - mid[i-1][1];
        }
        tangent[0][0] = mid[1][0] - mid[0][0];
        tangent[0][1] = mid[1][1] - mid[0][1];
        tangent[N-1][0] = mid[N-1][0] - mid[N-2][0];
        tangent[N-1][1] = mid[N-1][1] - mid[N-2][1];
        normal.resize(N, std::vector<double>(2,0.0));
        for (int i=0;i<N;++i) {
            double norm = std::sqrt(tangent[i][0]*tangent[i][0] + tangent[i][1]*tangent[i][1]);
            if (norm < 1e-12) norm = 1e-12;
            tangent[i][0] /= norm;
            tangent[i][1] /= norm;
            // rotate 90 degrees ccw
            normal[i][0] = -tangent[i][1];
            normal[i][1] = tangent[i][0];
        }
    }
};



// Objective callback for NLOPT C API
// x: vector d[0..N-1]
// returns curvature cost + smoothness cost.
static double objective_func(unsigned n, const double* x, double* grad, void* data) {
    auto* td = static_cast<TrackData*>(data);
    int N = td->N;
    const auto& mid = td->mid;
    const auto& normal = td->normal;

    // Racing line points, preceded by the fixed start point if any
    std::vector<std::array<double, 2>> pts;
    if (td->start) pts.push_back(*td->start);
    for (int i=0;i<N;++i) {
        pts.push_back({mid[i][0] + x[i]*normal[i][0],
                       mid[i][1] + x[i]*normal[i][1]});
    }
    double cost = 0.0;
    // curvature squared
    for (size_t i=1;i+1<pts.size();++i) {
        double dx1 = pts[i][0] - pts[i-1][0];
        double dy1 = pts[i][1] - pts[i-1][1];
        double dx2 = pts[i+1][0] - pts[i][0];
        double dy2 = pts[i+1][1] - pts[i][1];
        // signed turning angle: unlike the cross product alone, a U-turn costs pi, not 0
        double angle = std::atan2(dx1*dy2 - dy1*dx2, dx1*dx2 + dy1*dy2);
        double mean_len = 0.5 * (std::hypot(dx1, dy1) + std::hypot(dx2, dy2)) + 1e-6;
        double curvature = angle / mean_len;
        cost += curvature * curvature;
    }
    // smoothness term
    for (int i=0;i<N-1;++i) {
        double diff = x[i+1]-x[i];
        cost += 0.1 * diff * diff;
    }
    // No analytical gradient provided; derivative-free algorithm ignores this.
    if (grad) {
        for (unsigned i = 0; i < n; ++i) grad[i] = 0.0;
    }
    return cost;
}


RacingLine optimise_racing_line(
    const std::vector<std::vector<double>>& left,
    const std::vector<std::vector<double>>& right,
    std::optional<std::array<double, 2>> start)
{
    RacingLine rl;

    // TrackData needs at least two gates (tangents use mid[1] - mid[0]).
    if (std::min(left.size(), right.size()) < 2) return rl;

    TrackData td(left, right);
    td.start = start;
    const int N = td.N;

    // Choose an algorithm: LN_BOBYQA (derivative-free, bound-constrained)
    nlopt_opt opt = nlopt_create(NLOPT_LN_BOBYQA, N);

    // Bounds
    std::vector<double> lb(N), ub(N);
    for (int i=0;i<N;++i) {
        lb[i] = -td.half_width[i];
        ub[i] =  td.half_width[i];
    }
    nlopt_set_lower_bounds(opt, lb.data());
    nlopt_set_upper_bounds(opt, ub.data());

    nlopt_set_min_objective(opt, objective_func, &td);
    nlopt_set_xtol_rel(opt, 1e-6);
    nlopt_set_maxeval(opt, 1000);

    std::vector<double> x(N, 0.0); // initial guess centered
    double minf;
    rl.status = nlopt_optimize(opt, x.data(), &minf);
    nlopt_destroy(opt);
    if (rl.status < 0) {
        std::cerr << "Optimization failed with code: " << rl.status << std::endl;
        return rl;
    }

    rl.cost = minf;
    rl.offsets = x;
    rl.half_width = td.half_width;
    for (int i=0;i<N;++i) {
        rl.points.push_back({td.mid[i][0] + x[i]*td.normal[i][0],
                             td.mid[i][1] + x[i]*td.normal[i][1]});
    }
    return rl;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
get_racing_line_point_cloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_left,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_right,
    int nr_points_lin_space,
    float y_constant
    )
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr racing_line(new pcl::PointCloud<pcl::PointXYZ>);

    // Ground-plane coordinates (x forward, z right).
    std::vector<std::vector<double>> left = {};
    std::vector<std::vector<double>> right = {};

    for (auto& point : *src_left)
    {
        left.push_back({point.x, point.z});
    }

    for (auto& point : *src_right)
    {
        right.push_back({point.x, point.z});
    }

    // The car is at the origin of the frame.
    const RacingLine rl = optimise_racing_line(left, right, std::array<double, 2>{0.0, 0.0});
    if (rl.points.empty()) return racing_line;

    // Polyline car (origin) -> p_0 -> p_1 -> ... -> p_{N-1}, each segment sampled with
    // nr_points_lin_space points; the first point of a segment is skipped after the first
    // segment, as it is the last point of the previous one.
    std::vector<double> X = {0.0};
    std::vector<double> Z = {0.0};
    for (const auto& p : rl.points) {
        X.push_back(p[0]);
        Z.push_back(p[1]);
    }

    for (size_t s=1;s<X.size();++s)
    {
        auto x_vect = linspace(X[s-1], X[s], nr_points_lin_space);
        auto z_vect = linspace(Z[s-1], Z[s], nr_points_lin_space);

        for (int j=(s == 1 ? 0 : 1);j<nr_points_lin_space;++j)
        {
            pcl::PointXYZ pt;
            pt.x = x_vect[j];
            pt.y = y_constant;
            pt.z = z_vect[j];

            racing_line->push_back(pt);
        }
    }

    return racing_line;
}





/*

int main() {

    constexpr int nr_points_lin_space = 30;
    constexpr float y_constant = 0;

    const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cones_left(new pcl::PointCloud<pcl::PointXYZ>);
    const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cones_right(new pcl::PointCloud<pcl::PointXYZ>);
    const pcl::PointCloud<pcl::PointXYZ>::Ptr racing_line(new pcl::PointCloud<pcl::PointXYZ>);

    source_cones_left->push_back(pcl::PointXYZ(0,0,0));
    source_cones_left->push_back(pcl::PointXYZ(0,0,3));
    source_cones_left->push_back(pcl::PointXYZ(2,0,5));
    source_cones_left->push_back(pcl::PointXYZ(3,0,5));

    source_cones_right->push_back(pcl::PointXYZ(0,0,0));
    source_cones_right->push_back(pcl::PointXYZ(1,0,2));
    source_cones_right->push_back(pcl::PointXYZ(3,0,4));
    source_cones_right->push_back(pcl::PointXYZ(4,0,4));

    std::vector<std::vector<double>> left = {};
    std::vector<std::vector<double>> right = {};

    for (auto& point : *source_cones_left)
    {
        left.push_back({point.x, point.z});
    }

    for (auto& point : *source_cones_right)
    {
        right.push_back({point.x, point.z});
    }

    TrackData td(
        left,
        right
        );

    int N = td.N;

    // Choose an algorithm: LN_BOBYQA (derivative-free, bound-constrained)
    nlopt_opt opt = nlopt_create(NLOPT_LN_BOBYQA, N);

    // Bounds
    std::vector<double> lb(N), ub(N);
    for (int i=0;i<N;++i) {
        lb[i] = -td.half_width[i];
        ub[i] =  td.half_width[i];
    }
    nlopt_set_lower_bounds(opt, lb.data());
    nlopt_set_upper_bounds(opt, ub.data());

    nlopt_set_min_objective(opt, objective_func, &td);
    nlopt_set_xtol_rel(opt, 1e-6);
    nlopt_set_maxeval(opt, 1000);

    std::vector<double> x(N, 0.0); // initial guess centered
    double minf;
    nlopt_result result = nlopt_optimize(opt, x.data(), &minf);
    if (result < 0) {
        std::cerr << "Optimization failed with code: " << result << std::endl;
        nlopt_destroy(opt);
        return 1;
    }

    std::cout << "Status: " << result << "\nMinimum cost: " << minf << "\nOptimal d:";
    for (double v : x) std::cout << " " << v;
    std::cout << "\nRacing line (x,y):\n";

    for (int i=0;i<N;++i) {
        double X = td.mid[i][0] + x[i]*td.normal[i][0];
        double Y = td.mid[i][1] + x[i]*td.normal[i][1];
        std::cout << X << ", " << Y << "\n";
    }

    if (td.mid.size() > 2)
    {
        for (int i=1;i<(N-1);++i)
        {
            pcl::PointXYZ pt;

            float X1 = td.mid[i-1][0] + x[i]*td.normal[i-1][0];
            float Z1 = td.mid[i-1][1] + x[i]*td.normal[i-1][1];

            float X2 = td.mid[i][0] + x[i]*td.normal[i][0];
            float Z2 = td.mid[i][1] + x[i]*td.normal[i][1];

            auto x_vect = linspace(X1, X2, nr_points_lin_space);
            auto z_vect = linspace(Z1, Z2, nr_points_lin_space);

            for (int j=0;j<nr_points_lin_space;++j)
            {
                pt.x = x_vect[j];
                pt.y = y_constant;
                pt.z = z_vect[j];

                racing_line->push_back(pt);
            }
        }
    }

    std::cout << "[";
    for (const auto point : *racing_line)
    {
        std::cout << "[" << point.x << "," << point.z << "]" << ",";
    }
    std::cout << "]";

    nlopt_destroy(opt);
    return 0;
}


*/