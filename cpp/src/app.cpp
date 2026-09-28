#include "app.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "imgui.h"
#include "implot.h"
#include "linkage.h"

namespace app {

namespace {

// ---------------------------------------------------------------------------
// Styling helpers
// ---------------------------------------------------------------------------
const ImVec4 kFrameColor(0.60f, 0.60f, 0.60f, 1.0f);
const ImVec4 kLinkColor(0.30f, 0.65f, 1.00f, 1.0f);
const ImVec4 kLink2Color(0.35f, 0.85f, 0.45f, 1.0f);
const ImVec4 kPathPColor(1.00f, 0.35f, 0.35f, 1.0f);
const ImVec4 kPathBColor(0.85f, 0.45f, 0.95f, 1.0f);
const ImVec4 kPointColor(1.00f, 0.85f, 0.20f, 1.0f);
const ImVec4 kCursorColor(1.00f, 0.85f, 0.20f, 0.8f);

ImPlotSpec LineSpec(const ImVec4& col, float weight, ImPlotItemFlags flags = ImPlotItemFlags_None) {
    ImPlotSpec s;
    s.LineColor = col;
    s.LineWeight = weight;
    s.Flags = flags;
    return s;
}

ImPlotSpec JointSpec(const ImVec4& col, float size, ImPlotItemFlags flags = ImPlotItemFlags_None) {
    ImPlotSpec s = LineSpec(col, 2.0f, flags);
    s.Marker = ImPlotMarker_Circle;
    s.MarkerSize = size;
    return s;
}

// Plots a polyline through the given joints.
void PlotChain(const char* label, std::initializer_list<fourbar::Vec2> pts, const ImPlotSpec& spec) {
    std::vector<double> xs, ys;
    for (const auto& p : pts) {
        xs.push_back(p.x);
        ys.push_back(p.y);
    }
    ImPlot::PlotLine(label, xs.data(), ys.data(), static_cast<int>(xs.size()), spec);
}

void PlotLabel(const char* text, fourbar::Vec2 p) { ImPlot::PlotText(text, p.x, p.y, ImVec2(10, -10)); }

// Bounding box of every joint over the whole crank revolution, plotted as an
// invisible item so that fitting the axes keeps the full motion in view.
struct Bounds {
    double xs[2] = {0, 0};
    double ys[2] = {0, 0};
    bool empty = true;

    void Reset() { empty = true; }
    void Add(fourbar::Vec2 p) {
        if (empty) {
            xs[0] = xs[1] = p.x;
            ys[0] = ys[1] = p.y;
            empty = false;
            return;
        }
        xs[0] = std::min(xs[0], p.x);
        xs[1] = std::max(xs[1], p.x);
        ys[0] = std::min(ys[0], p.y);
        ys[1] = std::max(ys[1], p.y);
    }
    // Sets axis limits that contain the whole box (plus a margin) while keeping
    // x and y at the same scale for a plot of the given pixel size.
    void SetupFit(ImVec2 plot_size) const {
        if (empty) return;
        const double cx = 0.5 * (xs[0] + xs[1]), cy = 0.5 * (ys[0] + ys[1]);
        double w = (xs[1] - xs[0]) * 1.12 + 1.0, h = (ys[1] - ys[0]) * 1.12 + 1.0;
        // Approximate plot area: frame minus axis labels and ticks
        const double pw = std::max(50.0, static_cast<double>(plot_size.x) - 70.0);
        const double ph = std::max(50.0, static_cast<double>(plot_size.y) - 60.0);
        if (w / h < pw / ph)
            w = h * pw / ph;
        else
            h = w * ph / pw;
        ImPlot::SetupAxesLimits(cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2, ImPlotCond_Always);
    }
    void Plot() const {
        if (empty) return;
        ImPlotSpec spec = LineSpec(ImVec4(0, 0, 0, 0), 0.0f, ImPlotItemFlags_NoLegend);
        ImPlot::PlotScatter("##bounds", xs, ys, 2, spec);
    }
};

// Shared animation state (crank position within the sweep)
struct Playback {
    bool playing = true;
    float speed = 60.0f;  // crank degrees per second
    float index = 0.0f;   // fractional position in the pose list

    int Current(int count) const {
        if (count <= 0) return 0;
        return std::clamp(static_cast<int>(index), 0, count - 1);
    }

    void Advance(float dt, int count, double step_deg) {
        if (!playing || count <= 0) return;
        index += static_cast<float>(speed * dt / step_deg);
        index = std::fmod(index, static_cast<float>(count));
        if (index < 0) index += static_cast<float>(count);
    }

    // Play/pause, speed and crank-position controls. Returns nothing; edits in place.
    void Controls(int count, double current_angle) {
        if (ImGui::Button(playing ? "Pause" : "Play", ImVec2(70, 0))) playing = !playing;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::SliderFloat("##speed", &speed, 5.0f, 360.0f, "speed %.0f deg/s", ImGuiSliderFlags_Logarithmic);
        int i = Current(count);
        ImGui::SetNextItemWidth(-FLT_MIN);
        char fmt[64];
        std::snprintf(fmt, sizeof(fmt), "crank %.0f deg", current_angle);
        if (ImGui::SliderInt("##crank", &i, 0, std::max(count - 1, 0), fmt)) {
            index = static_cast<float>(i);
            playing = false;
        }
    }
};

bool InputLength(const char* label, double* v, double lo = 0.1) {
    ImGui::SetNextItemWidth(-FLT_MIN);
    bool changed = ImGui::DragScalar(label, ImGuiDataType_Double, v, 0.1f, &lo, nullptr, "%.2f");
    if (*v < lo) *v = lo;
    return changed;
}

bool InputAngle(const char* label, double* v) {
    const double lo = -180.0, hi = 180.0;
    ImGui::SetNextItemWidth(-FLT_MIN);
    return ImGui::DragScalar(label, ImGuiDataType_Double, v, 0.2f, &lo, &hi, "%.1f deg");
}

void ParamRow(const char* name, const char* help) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextUnformatted(name);
    if (help && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", help);
    ImGui::TableSetColumnIndex(1);
}

void SummaryRow(const char* name, double value, const char* unit = "") {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextUnformatted(name);
    ImGui::TableSetColumnIndex(1);
    ImGui::Text("%.3f %s", value, unit);
}

// ---------------------------------------------------------------------------
// Four-bar tab
// ---------------------------------------------------------------------------
struct FourBarTab {
    fourbar::FourBarParams params;
    fourbar::FourBarResult result;
    Playback playback;
    bool dirty = true;
    bool show_info = false;

    // Cached plot series
    std::vector<double> crank, arm, force, px, py, bx, by;
    Bounds bounds;
    bool fit_view = true;

    FourBarTab() { params.r2 = 18.0; }  // fourbarGUI.fig default crank length

    void Recompute() {
        result = fourbar::solve_fourbar(params);
        crank.clear(); arm.clear(); force.clear();
        px.clear(); py.clear(); bx.clear(); by.clear();
        for (const auto& q : result.poses) {
            crank.push_back(q.theta2);
            arm.push_back(q.moment_arm);
            force.push_back(q.force);
            px.push_back(q.P.x);
            py.push_back(q.P.y);
            bx.push_back(q.B.x);
            by.push_back(q.B.y);
        }
        bounds.Reset();
        for (const auto& q : result.poses)
            for (auto pt : {q.O, q.A, q.B, q.C, q.P}) bounds.Add(pt);
        fit_view = true;
        dirty = false;
    }

    void ParamsPanel() {
        ImGui::SeparatorText("Links");
        if (ImGui::BeginTable("fb_params", 2, ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("name", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
            ParamRow("Frame r1", "Distance between the crank pivot O and the rocker pivot C");
            dirty |= InputLength("##r1", &params.r1);
            ParamRow("Crank r2", "Crank length |OA|");
            dirty |= InputLength("##r2", &params.r2);
            ParamRow("Coupler r3", "Coupler length |AB|");
            dirty |= InputLength("##r3", &params.r3);
            ParamRow("Rocker r4", "Rocker length |CB|");
            dirty |= InputLength("##r4", &params.r4);
            ParamRow("Coupler radius r6", "Distance from A to the pedal point P");
            dirty |= InputLength("##r6", &params.r6);
            ParamRow("Frame angle", "Angle of the frame OC (theta1)");
            dirty |= InputAngle("##th1", &params.theta1);
            ParamRow("Beta angle", "Angle of AP relative to the coupler AB");
            dirty |= InputAngle("##beta", &params.beta);
            ParamRow("Crank torque", "Crank moment used for the minimum pedal force");
            dirty |= InputLength("##torque", &params.torque, 0.0);
            ImGui::EndTable();
        }
        ImGui::SeparatorText("Assembly mode");
        dirty |= ImGui::RadioButton("+1", &params.mode, 1);
        ImGui::SameLine();
        dirty |= ImGui::RadioButton("-1", &params.mode, -1);
        ImGui::SameLine();
        if (ImGui::SmallButton("Defaults")) {
            params = fourbar::FourBarParams();
            params.r2 = 18.0;
            dirty = true;
        }
        ImGui::SameLine();
        ImGui::Checkbox("Info", &show_info);

        if (dirty) Recompute();

        ImGui::SeparatorText("Status");
        if (!result.valid) {
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "No solution: the linkage cannot be\nassembled for every crank angle.");
        } else if (result.repeated_root) {
            ImGui::TextUnformatted("One real root (repeated) at some angle.");
        } else {
            ImGui::TextUnformatted("Two distinct real roots.");
        }

        if (result.valid) {
            ImGui::SeparatorText("Summary (MATLAB E vector)");
            const auto& s = result.summary;
            if (ImGui::BeginTable("fb_summary", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
                SummaryRow("P x max / min", s.x_max);
                ImGui::SameLine(); ImGui::Text("/ %.3f", s.x_min);
                SummaryRow("P y max / min", s.y_max);
                ImGui::SameLine(); ImGui::Text("/ %.3f", s.y_min);
                SummaryRow("Stride X / Y", s.x_range);
                ImGui::SameLine(); ImGui::Text("/ %.3f", s.y_range);
                SummaryRow("Ratio X/Y", s.xy_ratio);
                SummaryRow("Rocker range", s.rocker_range, "deg");
                SummaryRow("Coupler range", s.coupler_range, "deg");
                ImGui::EndTable();
            }
        }
    }

    void InfoWindow() {
        if (!show_info) return;
        ImGui::SetNextWindowSize(ImVec2(420, 0), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Four-bar info", &show_info)) {
            ImGui::TextWrapped(
                "O is the crank pivot at the origin, C the rocker pivot at distance r1 and angle theta1.\n"
                "The crank OA (r2) turns a full circle; the coupler AB (r3) and rocker CB (r4) close the loop.\n"
                "P is the pedal point on the coupler, r6 from A at angle beta relative to AB.\n\n"
                "Assembly mode picks one of the two solutions of the loop equation (open or crossed).\n\n"
                "Moment arm and minimum force follow FourAnalysis.m: force = torque / moment arm.\n\n"
                "Plots: drag to pan, scroll to zoom, double-click to fit.");
        }
        ImGui::End();
    }

    void LinkageView(int cur) {
        const ImVec2 size = ImGui::GetContentRegionAvail();
        if (!ImPlot::BeginPlot("##fourbar_linkage", ImVec2(-1, -1), ImPlotFlags_Equal)) return;
        ImPlot::SetupAxes("x", "y");
        if (fit_view) {
            bounds.SetupFit(size);
            fit_view = false;
        }
        ImPlot::SetupLegend(ImPlotLocation_NorthWest);
        if (result.valid) {
            const auto& q = result.poses[cur];
            bounds.Plot();
            ImPlot::PlotLine("Path of P", px.data(), py.data(), static_cast<int>(px.size()), LineSpec(kPathPColor, 1.5f));
            ImPlot::PlotLine("Path of B", bx.data(), by.data(), static_cast<int>(bx.size()), LineSpec(kPathBColor, 1.0f));
            PlotChain("Frame", {q.O, q.C}, LineSpec(kFrameColor, 2.0f, ImPlotItemFlags_NoLegend));
            PlotChain("Linkage", {q.O, q.A, q.B, q.C}, JointSpec(kLinkColor, 4.0f));
            PlotChain("##AP", {q.A, q.P}, LineSpec(kLinkColor, 2.0f, ImPlotItemFlags_NoLegend));
            PlotChain("Pedal point P", {q.P}, JointSpec(kPointColor, 6.0f));
            PlotLabel("O", q.O);
            PlotLabel("A", q.A);
            PlotLabel("B", q.B);
            PlotLabel("C", q.C);
            PlotLabel("P", q.P);
        }
        ImPlot::EndPlot();
    }

    void AnalysisPlots(int cur) {
        if (!result.valid) return;
        const double x = crank[cur];
        const float h = (ImGui::GetContentRegionAvail().y - ImGui::GetStyle().ItemSpacing.y) / 2;
        if (ImPlot::BeginPlot("Moment arm vs crank angle", ImVec2(-1, h))) {
            ImPlot::SetupAxes("crank angle (deg)", "moment arm", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::PlotLine("moment arm", crank.data(), arm.data(), static_cast<int>(crank.size()),
                             LineSpec(kLinkColor, 2.0f, ImPlotItemFlags_NoLegend));
            ImPlot::PlotInfLines("##cur", &x, 1, LineSpec(kCursorColor, 1.0f, ImPlotItemFlags_NoLegend));
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("Minimum pedal force vs crank angle", ImVec2(-1, h))) {
            ImPlot::SetupAxes("crank angle (deg)", "force", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_None);
            // The force goes to infinity where the moment arm crosses zero; default to a readable range.
            ImPlot::SetupAxisLimits(ImAxis_Y1, -2.0 * params.torque, 2.0 * params.torque, ImPlotCond_Once);
            ImPlot::PlotLine("force", crank.data(), force.data(), static_cast<int>(crank.size()),
                             LineSpec(kPathPColor, 2.0f, ImPlotItemFlags_NoLegend));
            ImPlot::PlotInfLines("##cur", &x, 1, LineSpec(kCursorColor, 1.0f, ImPlotItemFlags_NoLegend));
            ImPlot::EndPlot();
        }
    }

    void Render(float dt) {
        if (dirty) Recompute();
        const int count = static_cast<int>(result.poses.size());
        playback.Advance(dt, count, 1.0);
        const int cur = playback.Current(count);

        if (ImGui::BeginTable("fb_layout", 3, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV, ImVec2(0, -1))) {
            ImGui::TableSetupColumn("params", ImGuiTableColumnFlags_WidthFixed, 290.0f);
            ImGui::TableSetupColumn("linkage", ImGuiTableColumnFlags_WidthStretch, 1.2f);
            ImGui::TableSetupColumn("plots", ImGuiTableColumnFlags_WidthStretch, 1.0f);
            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);
            if (ImGui::BeginChild("fb_left")) {
                ParamsPanel();
                ImGui::SeparatorText("Animation");
                playback.Controls(count, count ? result.poses[cur].theta2 : 0.0);
            }
            ImGui::EndChild();

            ImGui::TableSetColumnIndex(1);
            LinkageView(playback.Current(static_cast<int>(result.poses.size())));

            ImGui::TableSetColumnIndex(2);
            if (ImGui::BeginChild("fb_right")) AnalysisPlots(playback.Current(static_cast<int>(result.poses.size())));
            ImGui::EndChild();
            ImGui::EndTable();
        }
        InfoWindow();
    }
};

// ---------------------------------------------------------------------------
// Eight-bar tab
// ---------------------------------------------------------------------------
struct EightBarTab {
    fourbar::EightBarParams params;
    fourbar::EightBarResult result;
    Playback playback;
    bool dirty = true;
    std::vector<double> px, py, crank, theta5;
    Bounds bounds;
    bool fit_view = true;

    void Recompute() {
        result = fourbar::solve_eightbar(params);
        px.clear(); py.clear(); crank.clear(); theta5.clear();
        for (const auto& q : result.poses) {
            px.push_back(q.P.x);
            py.push_back(q.P.y);
            crank.push_back(q.theta2);
            theta5.push_back(q.theta5);
        }
        bounds.Reset();
        for (const auto& q : result.poses)
            for (auto pt : {q.O, q.A, q.B, q.C, q.D, q.E, q.F, q.G, q.H, q.P}) bounds.Add(pt);
        fit_view = true;
        dirty = false;
    }

    void ParamsPanel() {
        ImGui::SeparatorText("Links");
        if (ImGui::BeginTable("eb_params", 2, ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("name", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
            struct { const char* name; const char* id; double* v; } lengths[] = {
                {"r2 (crank)", "##r2", &params.r2}, {"r3", "##r3", &params.r3},   {"r4", "##r4", &params.r4},
                {"r5", "##r5", &params.r5},         {"r6", "##r6", &params.r6},   {"r7", "##r7", &params.r7},
                {"r8", "##r8", &params.r8},         {"r10", "##r10", &params.r10}, {"r11", "##r11", &params.r11},
            };
            for (auto& l : lengths) {
                ParamRow(l.name, nullptr);
                dirty |= InputLength(l.id, l.v);
            }
            ParamRow("theta1", nullptr);
            dirty |= InputAngle("##th1", &params.theta1);
            ParamRow("alpha", nullptr);
            dirty |= InputAngle("##alpha", &params.alpha);
            ParamRow("beta", nullptr);
            dirty |= InputAngle("##beta", &params.beta);
            ParamRow("gamma", nullptr);
            dirty |= InputAngle("##gamma", &params.gamma);
            ImGui::EndTable();
        }
        if (ImGui::SmallButton("Defaults")) {
            params = fourbar::EightBarParams();
            dirty = true;
        }
        ImGui::TextDisabled("r12 = 55, r13 = r1 + 6, r14 = r4 + 59\n(fixed, as in EightbarAnalysis.m)");

        if (dirty) Recompute();

        ImGui::SeparatorText("Status");
        if (!result.valid)
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "No real solution for some crank angle.");
        else
            ImGui::TextUnformatted("Solved for the full crank revolution.");

        if (result.valid) {
            ImGui::SeparatorText("Summary (MATLAB E vector)");
            const auto& s = result.summary;
            if (ImGui::BeginTable("eb_summary", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
                SummaryRow("P x max / min", s.x_max);
                ImGui::SameLine(); ImGui::Text("/ %.3f", s.x_min);
                SummaryRow("P y max / min", s.y_max);
                ImGui::SameLine(); ImGui::Text("/ %.3f", s.y_min);
                SummaryRow("Stride X / Y", s.x_range);
                ImGui::SameLine(); ImGui::Text("/ %.3f", s.y_range);
                SummaryRow("Ratio X/Y", s.xy_ratio);
                SummaryRow("theta5 range", s.r_range, "deg");
                ImGui::EndTable();
            }
        }
    }

    void LinkageView(int cur) {
        const float h = ImGui::GetContentRegionAvail().y * 0.68f;
        const ImVec2 size(ImGui::GetContentRegionAvail().x, h);
        if (ImPlot::BeginPlot("##eightbar_linkage", ImVec2(-1, h), ImPlotFlags_Equal)) {
            ImPlot::SetupAxes("x", "y");
            if (fit_view) {
                bounds.SetupFit(size);
                fit_view = false;
            }
            ImPlot::SetupLegend(ImPlotLocation_NorthWest);
            if (result.valid) {
                const auto& q = result.poses[cur];
                bounds.Plot();
                ImPlot::PlotLine("Path of P", px.data(), py.data(), static_cast<int>(px.size()), LineSpec(kPathPColor, 1.5f));
                PlotChain("Crank and coupler", {q.O, q.A, q.B}, JointSpec(kLinkColor, 4.0f));
                PlotChain("Links", {q.C, q.D, q.E}, JointSpec(kLink2Color, 4.0f));
                PlotChain("##links2", {q.A, q.H, q.G, q.F, q.E}, JointSpec(kLink2Color, 4.0f, ImPlotItemFlags_NoLegend));
                PlotChain("Pedal point P", {q.P}, JointSpec(kPointColor, 6.0f));
                const char* names[] = {"O", "A", "B", "C", "D", "E", "F", "G", "H", "P"};
                const fourbar::Vec2 pts[] = {q.O, q.A, q.B, q.C, q.D, q.E, q.F, q.G, q.H, q.P};
                for (int i = 0; i < 10; ++i) PlotLabel(names[i], pts[i]);
            }
            ImPlot::EndPlot();
        }
        if (result.valid && ImPlot::BeginPlot("theta5 vs crank angle", ImVec2(-1, -1))) {
            ImPlot::SetupAxes("crank angle (deg)", "theta5 (deg)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::PlotLine("theta5", crank.data(), theta5.data(), static_cast<int>(crank.size()),
                             LineSpec(kLink2Color, 2.0f, ImPlotItemFlags_NoLegend));
            const double x = crank[cur];
            ImPlot::PlotInfLines("##cur", &x, 1, LineSpec(kCursorColor, 1.0f, ImPlotItemFlags_NoLegend));
            ImPlot::EndPlot();
        }
    }

    void Render(float dt) {
        if (dirty) Recompute();
        const int count = static_cast<int>(result.poses.size());
        playback.Advance(dt, count, 1.0);
        const int cur = playback.Current(count);

        if (ImGui::BeginTable("eb_layout", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV, ImVec2(0, -1))) {
            ImGui::TableSetupColumn("params", ImGuiTableColumnFlags_WidthFixed, 290.0f);
            ImGui::TableSetupColumn("linkage", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            if (ImGui::BeginChild("eb_left")) {
                ParamsPanel();
                ImGui::SeparatorText("Animation");
                playback.Controls(count, count ? result.poses[cur].theta2 : 0.0);
            }
            ImGui::EndChild();
            ImGui::TableSetColumnIndex(1);
            if (ImGui::BeginChild("eb_right")) LinkageView(playback.Current(static_cast<int>(result.poses.size())));
            ImGui::EndChild();
            ImGui::EndTable();
        }
    }
};

FourBarTab* g_fourbar = nullptr;
EightBarTab* g_eightbar = nullptr;

}  // namespace

void Init() {
    static FourBarTab fourbar_tab;
    static EightBarTab eightbar_tab;
    g_fourbar = &fourbar_tab;
    g_eightbar = &eightbar_tab;
}

void RenderUI(float dt) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::Begin("Fourbar GUI", nullptr, flags);
    ImGui::Text("Elliptical trainer linkage analysis  v%s", FOURBAR_APP_VERSION);
    if (ImGui::BeginTabBar("mechanisms")) {
        if (ImGui::BeginTabItem("Four-bar")) {
            g_fourbar->Render(dt);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Eight-bar")) {
            g_eightbar->Render(dt);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

}  // namespace app
