// Unit tests for the linkage kinematics.
//
// Reference values were produced by running the original MATLAB scripts
// (FourAnalysis.m, EightbarAnalysis.m) in GNU Octave with their default
// inputs, plotting and input() stubbed out.
#include "linkage.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace {

int g_failures = 0;
int g_checks = 0;

void check_near(double actual, double expected, double tol, const std::string& what) {
    ++g_checks;
    if (!(std::fabs(actual - expected) <= tol)) {
        ++g_failures;
        std::printf("FAIL %s: got %.12g, expected %.12g (tol %g)\n", what.c_str(), actual, expected, tol);
    }
}

void check_true(bool cond, const std::string& what) {
    ++g_checks;
    if (!cond) {
        ++g_failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

double dist(fourbar::Vec2 a, fourbar::Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

const fourbar::FourBarPose* find_pose(const fourbar::FourBarResult& r, double theta2) {
    for (const auto& q : r.poses)
        if (std::fabs(q.theta2 - theta2) < 1e-9) return &q;
    return nullptr;
}

const fourbar::EightBarPose* find_pose(const fourbar::EightBarResult& r, double theta2) {
    for (const auto& q : r.poses)
        if (std::fabs(q.theta2 - theta2) < 1e-9) return &q;
    return nullptr;
}

void test_crank_angles() {
    const auto f = fourbar::fourbar_crank_angles();
    check_true(f.size() == 360, "four-bar sweep has 360 angles");
    check_near(f.front(), 180, 0, "four-bar sweep starts at 180");
    check_near(f[180], 0, 0, "four-bar sweep passes 0");
    check_near(f.back(), -179, 0, "four-bar sweep ends at -179");

    const auto e = fourbar::eightbar_crank_angles();
    check_true(e.size() == 360, "eight-bar sweep has 360 angles");
    check_near(e.front(), 0, 0, "eight-bar sweep starts at 0");
    check_near(e.back(), 359, 0, "eight-bar sweep ends at 359");
}

void test_fourbar_matches_matlab() {
    fourbar::FourBarParams p;  // FourAnalysis.m defaults
    const auto r = fourbar::solve_fourbar(p);
    check_true(r.valid, "four-bar defaults assemble");
    check_true(!r.repeated_root, "four-bar defaults have two distinct roots");

    // MATLAB E vector
    const double E[13] = {33.52568942144764,  1.503360538648728,  6.579508429746248, -16.03857076346915,
                          32.02232888279891,  22.61807919321539,  1.415784630040753, -75.14983928783408,
                          -107.9340593096414, 32.78422002180736, -0.1227746472559236, -31.78517884585346,
                          31.66240419859754};
    const auto& s = r.summary;
    const double got[13] = {s.x_max,      s.x_min,      s.y_max,         s.y_min,         s.x_range,
                            s.y_range,    s.xy_ratio,   s.rocker_max,    s.rocker_min,    s.rocker_range,
                            s.coupler_max, s.coupler_min, s.coupler_range};
    for (int i = 0; i < 13; ++i) check_near(got[i], E[i], 1e-9, "four-bar E[" + std::to_string(i + 1) + "]");

    // theta2, X, Y, Flen, F, theta41, theta31
    const double samples[6][7] = {
        {180, 1.53992308815, -4.04364910219, 2.40895725497, 1.24535210984, -107.386876847, -12.982131502},
        {135, 5.0462324745, 3.80688950929, 6.17409421563, 0.48590123429, -106.609015604, -24.6481975329},
        {90, 15.3252761211, 6.55881830428, 16.488171306, 0.181948619063, -98.6407607168, -31.6352688898},
        {0, 33.4817597162, -4.28813213715, 31.4962222782, 0.0952495182915, -75.5485710005, -13.7820712766},
        {-90, 17.9999586748, -16.0385707635, 18.0342851231, 0.166349815339, -89.8006863342, -0.122774647256},
        {-179, 1.56015546284, -4.24490592197, 2.45703642652, 1.22098311919, -107.322961743, -12.7275267045},
    };
    for (const auto& row : samples) {
        const std::string tag = "four-bar theta2=" + std::to_string(static_cast<int>(row[0]));
        const auto* q = find_pose(r, row[0]);
        check_true(q != nullptr, tag + " present");
        if (!q) continue;
        check_near(q->P.x, row[1], 1e-9, tag + " P.x");
        check_near(q->P.y, row[2], 1e-9, tag + " P.y");
        check_near(q->moment_arm, row[3], 1e-9, tag + " moment arm");
        check_near(q->force, row[4], 1e-9, tag + " force");
        check_near(q->theta4, row[5], 1e-8, tag + " theta4");
        check_near(q->theta3, row[6], 1e-8, tag + " theta3");
    }
}

void test_fourbar_closure_both_modes() {
    for (int mode : {1, -1}) {
        fourbar::FourBarParams p;
        p.r2 = 18;  // fourbarGUI.fig default crank length
        p.mode = mode;
        const auto r = fourbar::solve_fourbar(p);
        const std::string tag = "four-bar mode " + std::to_string(mode);
        check_true(r.valid, tag + " assembles");
        double worst = 0;
        for (const auto& q : r.poses) {
            worst = std::fmax(worst, std::fabs(dist(q.O, q.A) - p.r2));
            worst = std::fmax(worst, std::fabs(dist(q.A, q.B) - p.r3));
            worst = std::fmax(worst, std::fabs(dist(q.C, q.B) - p.r4));
            worst = std::fmax(worst, std::fabs(dist(q.O, q.C) - p.r1));
            worst = std::fmax(worst, std::fabs(dist(q.A, q.P) - p.r6));
        }
        check_near(worst, 0, 1e-9, tag + " link lengths preserved");
    }
}

void test_fourbar_unassemblable() {
    fourbar::FourBarParams p;
    p.r3 = 200;  // coupler longer than the other three links combined
    const auto r = fourbar::solve_fourbar(p);
    check_true(!r.valid, "four-bar r3=200 reports no solution");
    check_true(r.poses.empty(), "four-bar r3=200 returns no poses");
}

void test_eightbar_matches_matlab() {
    fourbar::EightBarParams p;  // EightbarAnalysis.m defaults
    const auto r = fourbar::solve_eightbar(p);
    check_true(r.valid, "eight-bar defaults solve");

    const double E[10] = {104.81914684,  57.5340121237,  6.34813115571,  -16.2315671268, 47.2851347164,
                          22.5796982825, 2.09414378017, -69.0872885511, -97.3095252199, 28.2222366688};
    const auto& s = r.summary;
    const double got[10] = {s.x_max,   s.x_min,    s.y_max, s.y_min, s.x_range,
                            s.y_range, s.xy_ratio, s.r_max, s.r_min, s.r_range};
    for (int i = 0; i < 10; ++i) check_near(got[i], E[i], 1e-8, "eight-bar E[" + std::to_string(i + 1) + "]");

    // theta2, X, Y, r1, theta5, theta7
    const double samples[5][6] = {
        {0, 104.770563413, -5.31785993538, 155.00950536, -69.1231327961, -344.630670908},
        {90, 78.1811972178, 6.3459128624, 123.729543764, -84.0303877303, -335.903178912},
        {180, 57.5971683837, -4.75884085808, 108.00950536, -97.2969043895, -340.903921562},
        {270, 81.5604027002, -16.2276941073, 134.818396371, -82.8752105613, -346.213232911},
        {359, 104.794684234, -5.51638465976, 155.100371448, -69.1072718447, -344.72795877},
    };
    for (const auto& row : samples) {
        const std::string tag = "eight-bar theta2=" + std::to_string(static_cast<int>(row[0]));
        const auto* q = find_pose(r, row[0]);
        check_true(q != nullptr, tag + " present");
        if (!q) continue;
        check_near(q->P.x, row[1], 1e-8, tag + " P.x");
        check_near(q->P.y, row[2], 1e-8, tag + " P.y");
        check_near(q->r1, row[3], 1e-8, tag + " r1");
        check_near(q->theta5, row[4], 1e-8, tag + " theta5");
        check_near(q->theta7, row[5], 1e-8, tag + " theta7");
    }
}

void test_eightbar_closure() {
    fourbar::EightBarParams p;
    const auto r = fourbar::solve_eightbar(p);
    double worst = 0;
    for (const auto& q : r.poses) {
        worst = std::fmax(worst, std::fabs(dist(q.A, q.B) - p.r3));
        worst = std::fmax(worst, std::fabs(dist(q.D, q.C) - p.r5));
        worst = std::fmax(worst, std::fabs(dist(q.D, q.F) - p.r6));
        worst = std::fmax(worst, std::fabs(dist(q.H, q.G) - p.r7));
        worst = std::fmax(worst, std::fabs(dist(q.G, q.F) - p.r8));  // loop closed by equation 4
    }
    check_near(worst, 0, 1e-9, "eight-bar link lengths preserved");
}

}  // namespace

int main() {
    test_crank_angles();
    test_fourbar_matches_matlab();
    test_fourbar_closure_both_modes();
    test_fourbar_unassemblable();
    test_eightbar_matches_matlab();
    test_eightbar_closure();
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
