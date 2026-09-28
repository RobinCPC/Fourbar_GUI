#include "linkage.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fourbar {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kD2R = kPi / 180.0;
constexpr double kR2D = 180.0 / kPi;

Vec2 polar(double r, double deg) { return {r * std::cos(deg * kD2R), r * std::sin(deg * kD2R)}; }
Vec2 polar_rad(double r, double rad) { return {r * std::cos(rad), r * std::sin(rad)}; }
Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }

// min/max of a member over all poses
template <typename Pose, typename Get>
void min_max(const std::vector<Pose>& poses, Get get, double& lo, double& hi) {
    lo = std::numeric_limits<double>::infinity();
    hi = -std::numeric_limits<double>::infinity();
    for (const Pose& q : poses) {
        const double v = get(q);
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
}

}  // namespace

std::vector<double> fourbar_crank_angles(double step) {
    // MATLAB: a1 = 180:-nn:0; a2 = -1:-nn:-179; theta2 = [a1 a2]
    std::vector<double> out;
    for (double a = 180.0; a >= 0.0 - 1e-9; a -= step) out.push_back(a);
    for (double a = -step; a >= -179.0 - 1e-9; a -= step) out.push_back(a);
    return out;
}

std::vector<double> eightbar_crank_angles(double step) {
    // MATLAB: theta2 = 0:nn:359
    std::vector<double> out;
    for (double a = 0.0; a <= 359.0 + 1e-9; a += step) out.push_back(a);
    return out;
}

FourBarResult solve_fourbar(const FourBarParams& p, const std::vector<double>& crank_angles) {
    FourBarResult res;
    res.poses.reserve(crank_angles.size());

    const Vec2 C = polar(p.r1, p.theta1);
    const double sign = p.mode >= 0 ? 1.0 : -1.0;

    for (double th2 : crank_angles) {
        const double th1r = p.theta1 * kD2R;
        const double th2r = th2 * kD2R;
        // Loop equation coefficients (rm = r2, rj = r3 in the MATLAB code)
        const double A = 2 * p.r1 * p.r4 * std::cos(th1r) - 2 * p.r2 * p.r4 * std::cos(th2r);
        const double B = 2 * p.r1 * p.r4 * std::sin(th1r) - 2 * p.r2 * p.r4 * std::sin(th2r);
        const double Cc = p.r1 * p.r1 + p.r2 * p.r2 + p.r4 * p.r4 - p.r3 * p.r3 -
                          2 * p.r1 * p.r2 * (std::cos(th1r) * std::cos(th2r) + std::sin(th1r) * std::sin(th2r));
        const double disc2 = B * B + A * A - Cc * Cc;
        if (disc2 < 0) {
            res.valid = false;
            res.poses.clear();
            return res;
        }
        if (disc2 == 0) res.repeated_root = true;
        const double disc = std::sqrt(disc2);

        FourBarPose q;
        q.theta2 = th2;
        q.theta4 = 2 * std::atan2(-B + sign * disc, Cc - A) * kR2D;
        if (q.theta4 > 0) q.theta4 -= 360.0;

        q.O = {0.0, 0.0};
        q.A = polar(p.r2, th2);
        q.C = C;
        q.B = C + polar(p.r4, q.theta4);
        const Vec2 AB = q.B - q.A;
        q.theta3 = std::atan2(AB.y, AB.x) * kR2D;  // same as MATLAB theta5 / theta3x
        q.P = q.A + polar(p.r6, p.beta + q.theta3);

        // Moment arm of P about the crank pivot (MATLAB "force arm" section)
        const double dOP = std::hypot(q.P.x, q.P.y);
        const double alpha = std::acos((p.r2 * p.r2 + dOP * dOP - p.r6 * p.r6) / (2 * p.r2 * dOP)) * kR2D;
        const double gamma = (std::fabs(th2) + q.theta3 - alpha) * kD2R;
        q.moment_arm = dOP * std::cos(gamma);
        q.force = p.torque / q.moment_arm;

        res.poses.push_back(q);
    }

    res.valid = !res.poses.empty();
    if (!res.valid) return res;

    FourBarSummary& s = res.summary;
    min_max(res.poses, [](const FourBarPose& q) { return q.P.x; }, s.x_min, s.x_max);
    min_max(res.poses, [](const FourBarPose& q) { return q.P.y; }, s.y_min, s.y_max);
    min_max(res.poses, [](const FourBarPose& q) { return q.theta4; }, s.rocker_min, s.rocker_max);
    min_max(res.poses, [](const FourBarPose& q) { return q.theta3; }, s.coupler_min, s.coupler_max);
    s.x_range = s.x_max - s.x_min;
    s.y_range = s.y_max - s.y_min;
    s.xy_ratio = s.x_range / s.y_range;
    s.rocker_range = s.rocker_max - s.rocker_min;
    s.coupler_range = s.coupler_max - s.coupler_min;
    return res;
}

EightBarResult solve_eightbar(const EightBarParams& p, const std::vector<double>& crank_angles) {
    EightBarResult res;
    res.poses.reserve(crank_angles.size());

    const double th1 = p.theta1 * kD2R;
    const double al = p.alpha * kD2R;
    const double be = p.beta * kD2R;
    const double ga = p.gamma * kD2R;
    const double r12 = 55.0;
    const double th13 = 0.0;
    const double th14 = -90.0 * kD2R;

    for (double th2deg : crank_angles) {
        const double th2 = th2deg * kD2R;

        // Equation 1: slider r1 and coupler angle th3
        const double th4 = th1 - 90.0 * kD2R;
        const double A = 2 * p.r4 * (std::cos(th1) * std::cos(th4) + std::sin(th1) * std::sin(th4)) -
                         2 * p.r2 * (std::cos(th1) * std::cos(th2) + std::sin(th1) * std::sin(th2));
        const double B = p.r2 * p.r2 + p.r4 * p.r4 - p.r3 * p.r3 -
                         2 * p.r2 * p.r4 * (std::cos(th2) * std::cos(th4) + std::sin(th2) * std::sin(th4));
        const double d1 = A * A - 4 * B;
        const double r1 = (-A + std::sqrt(d1)) / 2;
        const double th3 = std::atan2(r1 * std::sin(th1) + p.r4 * std::sin(th4) - p.r2 * std::sin(th2),
                                      r1 * std::cos(th1) + p.r4 * std::cos(th4) - p.r2 * std::cos(th2));
        const double th9 = th3 - al;
        const double th11 = th3 + ga;

        // Equation 2: th10, th5, th6
        const double th12 = th3 + kPi;
        const double r13 = r1 + 6;
        const double r14 = p.r4 + 59;
        double C1 = r12 * std::cos(th12) + r13 * std::cos(th13) + r14 * std::cos(th14);
        double C2 = r12 * std::sin(th12) + r13 * std::sin(th13) + r14 * std::sin(th14);
        double C3 = p.r5 * p.r5 - C1 * C1 - C2 * C2 - p.r10 * p.r10 - 2 * p.r10 * C1;
        double C4 = 4 * C2 * p.r10;
        double C5 = p.r5 * p.r5 - C1 * C1 - C2 * C2 - p.r10 * p.r10 + 2 * C1 * p.r10;
        const double d2 = C4 * C4 - 4 * C3 * C5;
        const double th10 = 2 * std::atan2(-C4 - std::sqrt(d2), 2 * C3);
        const double th5 = std::atan2(C2 - p.r10 * std::sin(th10), C1 - p.r10 * std::cos(th10));
        const double th6 = th10 + be;

        // Equation 4: th7, th8
        C1 = (r1 * std::cos(th1) + p.r4 * std::cos(th4) + r12 * std::cos(th12) + p.r6 * std::cos(th6)) -
             (p.r2 * std::cos(th2) + p.r10 * std::cos(th10) + p.r11 * std::cos(th11));
        C2 = (r1 * std::sin(th1) + p.r4 * std::sin(th4) + r12 * std::sin(th12) + p.r6 * std::sin(th6)) -
             (p.r2 * std::sin(th2) + p.r10 * std::sin(th10) + p.r11 * std::sin(th11));
        C3 = p.r8 * p.r8 - C1 * C1 - C2 * C2 - p.r7 * p.r7 - 2 * C1 * p.r7;
        C4 = 4 * C2 * p.r7;
        C5 = p.r8 * p.r8 - C1 * C1 - C2 * C2 - p.r7 * p.r7 + 2 * C1 * p.r7;
        const double d3 = C4 * C4 - 4 * C3 * C5;
        const double th7 = 2 * std::atan2(-C4 - std::sqrt(d3), 2 * C3);
        const double th8 = std::atan2(C2 - p.r7 * std::sin(th7), C1 - p.r7 * std::cos(th7));

        if (d1 < 0 || d2 < 0 || d3 < 0) {
            res.valid = false;
            res.poses.clear();
            return res;
        }

        EightBarPose q;
        q.theta2 = th2deg;
        q.r1 = r1;
        q.theta3 = th3 * kR2D;
        q.theta4 = th4 * kR2D;
        q.theta5 = th5 * kR2D;
        q.theta6 = th6 * kR2D;
        q.theta7 = th7 * kR2D;
        q.theta8 = th8 * kR2D;
        q.theta9 = th9 * kR2D;
        q.theta10 = th10 * kR2D;
        q.theta11 = th11 * kR2D;
        q.theta12 = th12 * kR2D;

        // Joint positions, same construction as the MATLAB animation loop
        q.O = {0.0, 0.0};
        q.A = polar_rad(p.r2, th2);
        q.B = q.A + polar_rad(p.r3, th3);
        q.E = q.B + polar_rad(r12, th12);
        q.D = q.E - polar_rad(p.r10, th10);
        q.C = q.D - polar_rad(p.r5, th5);
        q.F = q.D + polar_rad(p.r6, th6);
        q.H = q.A + polar_rad(p.r11, th11);
        q.G = q.H + polar_rad(p.r7, th7);
        q.P = q.H + polar_rad(0.5 * p.r7, th7);
        res.poses.push_back(q);
    }

    res.valid = !res.poses.empty();
    if (!res.valid) return res;

    EightBarSummary& s = res.summary;
    min_max(res.poses, [](const EightBarPose& q) { return q.P.x; }, s.x_min, s.x_max);
    min_max(res.poses, [](const EightBarPose& q) { return q.P.y; }, s.y_min, s.y_max);
    min_max(res.poses, [](const EightBarPose& q) { return q.theta5; }, s.r_min, s.r_max);
    s.x_range = s.x_max - s.x_min;
    s.y_range = s.y_max - s.y_min;
    s.xy_ratio = s.x_range / s.y_range;
    s.r_range = s.r_max - s.r_min;
    return res;
}

}  // namespace fourbar
