// Kinematics of the elliptical-trainer linkages, ported from the MATLAB
// scripts in the repository root (FourAnalysis.m, fourbarGUI.m,
// EightbarAnalysis.m). Angles in the parameter structs are in degrees;
// lengths are in the same unit as the MATLAB code (cm).
#pragma once

#include <vector>

namespace fourbar {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

// ---------------------------------------------------------------------------
// Four-bar linkage (FourAnalysis.m / fourbarGUI.m)
//
//   O = crank pivot (origin), A = crank tip, B = coupler/rocker joint,
//   C = rocker pivot, P = pedal point on the coupler (offset r6, angle beta).
// ---------------------------------------------------------------------------
struct FourBarParams {
    double r1 = 73.0;      // frame length  |OC|
    double r2 = 16.0;      // crank length  |OA|
    double r3 = 60.0;      // coupler length |AB|
    double r4 = 58.0;      // rocker length |CB|
    double r6 = 18.0;      // coupler point radius |AP|
    double theta1 = 35.0;  // frame angle (deg)
    double beta = 0.0;     // coupler point angle relative to AB (deg)
    double torque = 3.0;   // crank moment (N*m) used by the force analysis
    int mode = 1;          // assembly mode: +1 or -1
};

struct FourBarPose {
    double theta2 = 0.0;   // crank angle (deg)
    double theta3 = 0.0;   // coupler angle (deg)
    double theta4 = 0.0;   // rocker angle (deg), wrapped to (-360, 0]
    Vec2 O, A, B, C, P;
    double moment_arm = 0.0;  // "force arm" of P about O (MATLAB Flen)
    double force = 0.0;       // minimum pedal force = torque / moment_arm
};

// Summary values; same order as the MATLAB E vector:
// (Xmax, Xmin, Ymax, Ymin, Xd, Yd, X/Y, Rmax, Rmin, Rran, th3max, th3min, th3ran)
struct FourBarSummary {
    double x_max = 0, x_min = 0, y_max = 0, y_min = 0;
    double x_range = 0, y_range = 0, xy_ratio = 0;
    double rocker_max = 0, rocker_min = 0, rocker_range = 0;
    double coupler_max = 0, coupler_min = 0, coupler_range = 0;
};

struct FourBarResult {
    bool valid = false;          // false: linkage cannot be assembled at some crank angle
    bool repeated_root = false;  // some crank angle has a double root
    std::vector<FourBarPose> poses;  // one per crank angle
    FourBarSummary summary;
};

// Crank angles used by the MATLAB code: 180, 179, ..., 0, -1, ..., -179 (deg).
std::vector<double> fourbar_crank_angles(double step = 1.0);

// Solve the loop equation for every crank angle in `crank_angles`.
FourBarResult solve_fourbar(const FourBarParams& p, const std::vector<double>& crank_angles);
inline FourBarResult solve_fourbar(const FourBarParams& p) {
    return solve_fourbar(p, fourbar_crank_angles());
}

// ---------------------------------------------------------------------------
// Eight-bar linkage (EightbarAnalysis.m)
// ---------------------------------------------------------------------------
struct EightBarParams {
    double r2 = 23.5;   // crank
    double r3 = 135.0;
    double r4 = 30.5;
    double r5 = 95.0;
    double r6 = 90.0;
    double r7 = 18.0;
    double r8 = 10.0;
    double r10 = 74.5;
    double r11 = 73.0;
    double theta1 = 0.0;  // deg
    double alpha = 0.0;   // deg
    double beta = 2.0;    // deg
    double gamma = 7.0;   // deg
    // r9 is read by the MATLAB script but never used; the fixed values
    // r12 = 55, r13 = r1 + 6, r14 = r4 + 59 are hard-coded there as well.
};

struct EightBarPose {
    double theta2 = 0.0;  // crank angle (deg)
    double r1 = 0.0;      // slider length
    // Link angles in degrees, indexed like the MATLAB `sol` columns 3..12.
    double theta3 = 0, theta4 = 0, theta5 = 0, theta6 = 0, theta7 = 0;
    double theta8 = 0, theta9 = 0, theta10 = 0, theta11 = 0, theta12 = 0;
    Vec2 O, A, B, C, D, E, F, G, H, P;
};

// Same order as the MATLAB E vector: (Xmax, Xmin, Ymax, Ymin, Xd, Yd, X/Y, Rmax, Rmin, Rran)
// where R is theta5 (the MATLAB code takes column 4 of sol(:,2:12)).
struct EightBarSummary {
    double x_max = 0, x_min = 0, y_max = 0, y_min = 0;
    double x_range = 0, y_range = 0, xy_ratio = 0;
    double r_max = 0, r_min = 0, r_range = 0;
};

struct EightBarResult {
    bool valid = false;  // false: some crank angle has no real solution
    std::vector<EightBarPose> poses;
    EightBarSummary summary;
};

// Crank angles used by the MATLAB code: 0, 1, ..., 359 (deg).
std::vector<double> eightbar_crank_angles(double step = 1.0);

EightBarResult solve_eightbar(const EightBarParams& p, const std::vector<double>& crank_angles);
inline EightBarResult solve_eightbar(const EightBarParams& p) {
    return solve_eightbar(p, eightbar_crank_angles());
}

}  // namespace fourbar
