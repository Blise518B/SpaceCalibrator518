// The "Live" tab (fork, docs/DESIGN.md section 13): what the guard and the solver see right now, from
// LiveStats (live_stats.h). Every graph explains itself: hover the plot (values at the cursor plus what
// the graph means) or the info icon next to its title. Pause freezes a copy; while paused all time
// graphs pan and zoom together.

#include "guard_ui.h"

#include "IconsMaterialSymbols.h"
#include "blackbox_format.h"
#include "calibration.h"
#include "guard_config.h"
#include "imgui.h"
#include "imgui_extensions.h"
#include "implot.h"
#include "live_stats.h"
#include "localisation.h"
#include "trust/trust_manager.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <fmt/format.h>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace spacecal::guard {

namespace {
    constexpr double k_WINDOWS[] = { 10.0, 60.0, 300.0 };
    constexpr const char* k_WINDOW_KEYS[] = { "live_window_10s", "live_window_1min", "live_window_5min" };
    constexpr size_t k_MAX_LINE_POINTS = 2400;
    constexpr double k_NAN = std::numeric_limits<double>::quiet_NaN();

    const ImVec4 k_GOOD(0.35f, 0.6f, 0.4f, 1.0f);
    const ImVec4 k_WARN(0.85f, 0.65f, 0.2f, 1.0f);
    const ImVec4 k_BAD(0.8f, 0.3f, 0.3f, 1.0f);
    const ImVec4 k_RECOVER(0.3f, 0.5f, 0.85f, 1.0f);
    const ImVec4 k_GREY(0.5f, 0.5f, 0.5f, 1.0f);
    const ImVec4 k_ACCENT(0.45f, 0.7f, 1.0f, 1.0f);
    const ImVec4 k_SHIFT(0.95f, 0.55f, 0.2f, 1.0f);
    const ImVec4 k_MARKER(0.75f, 0.55f, 0.95f, 1.0f);
    const ImVec4 k_RECENTER(0.4f, 0.75f, 0.85f, 1.0f);

    // LOCALE_FORMAT binds its arguments by reference and refuses temporaries
    template <typename... Args>
    std::string lf(const char* key, const Args&... args)
    {
        return fmt::vformat(LOCALE_GET(key), fmt::make_format_args(args...));
    }

    struct LiveUiState {
        int window = 1;
        bool paused = false;
        double pausedAt = 0.0;
        LiveStats snapshot;
        double xMin = -60.0, xMax = 0.0; // linked time axis of every time graph, seconds relative to ref
    };

    LiveUiState& ui()
    {
        static LiveUiState s;
        return s;
    }

    // what every graph needs
    struct Ctx {
        const LiveStats& L;
        double ref; // the "0 s" of the time axes (now, or the pause moment)
        double liveNow; // the overlay clock now (for wall clock times)
        std::time_t wallNow;
        double rOk; // cm
        double rHigh; // cm
        double tFrom, tTo; // visible time range (absolute)
    };

    ImVec4 alpha(ImVec4 c, float a)
    {
        c.w = a;
        return c;
    }

    ImVec4 stateColor(uint8_t s)
    {
        switch (static_cast<trust::State>(s)) {
        case trust::State::TRUSTED: return k_GOOD;
        case trust::State::SUSPECT: return k_WARN;
        case trust::State::UNTRUSTED: return k_BAD;
        case trust::State::RECOVERING: return k_RECOVER;
        default: return k_GREY;
        }
    }

    ImVec4 errorColor(double cm, const Ctx& c)
    {
        return cm < c.rOk ? k_GOOD : (cm < c.rHigh ? k_WARN : k_BAD);
    }

    std::string num(double v, int decimals = 1)
    {
        return fmt::format("{:.{}f}", v, decimals);
    }

    std::string duration(double seconds)
    {
        if (seconds < 90.0)
            return fmt::format("{:.0f} s", std::max(0.0, seconds));
        if (seconds < 5400.0)
            return fmt::format("{:.0f} min", seconds / 60.0);
        return fmt::format("{:.1f} h", seconds / 3600.0);
    }

    std::string wallClock(const Ctx& c, double t)
    {
        const std::time_t when = c.wallNow - static_cast<std::time_t>(std::llround(c.liveNow - t));
        std::tm tm {};
#if defined(_WIN32)
        localtime_s(&tm, &when);
#else
        localtime_r(&when, &tm);
#endif
        return fmt::format("{:02}:{:02}:{:02}", tm.tm_hour, tm.tm_min, tm.tm_sec);
    }

    std::string atText(const Ctx& c, double t)
    {
        return fmt::format("{:+.1f} s ({})", t - c.ref, wallClock(c, t));
    }

    std::string outcomeText(uint8_t outcome)
    {
        using O = blackbox::CalibrationOutcome;
        switch (static_cast<O>(outcome)) {
        case O::APPLIED: return LOCALE_GET("live_outcome_applied");
        case O::FORCED: return LOCALE_GET("live_outcome_forced");
        case O::SKIPPED: return LOCALE_GET("live_outcome_skipped");
        case O::CORRECTED: return LOCALE_GET("live_outcome_corrected");
        default: return LOCALE_GET("live_outcome_rejected");
        }
    }

    bool outcomeChanged(uint8_t outcome)
    {
        using O = blackbox::CalibrationOutcome;
        return outcome == static_cast<uint8_t>(O::APPLIED) || outcome == static_cast<uint8_t>(O::FORCED) || outcome == static_cast<uint8_t>(O::CORRECTED);
    }

    ImVec4 outcomeColor(uint8_t outcome)
    {
        using O = blackbox::CalibrationOutcome;
        switch (static_cast<O>(outcome)) {
        case O::APPLIED: return k_GOOD;
        case O::FORCED: return k_RECOVER;
        case O::CORRECTED: return k_SHIFT;
        default: return k_GREY;
        }
    }

    std::string reasonText(const LiveSolve& s)
    {
        if (s.errorKey.empty())
            return s.errorText;
        const std::string localized = LOCALE_GET(s.errorKey);
        return localized.empty() || localized == s.errorKey ? s.errorText : localized;
    }

    std::string solveDetails(const Ctx& c, const LiveSolve& s)
    {
        std::string text = fmt::format("{} · {} · {}", atText(c, s.t), blackbox::calibrationTriggerName(static_cast<blackbox::CalibrationTrigger>(s.trigger)), outcomeText(s.outcome));
        const std::string reason = reasonText(s);
        if (!reason.empty() && !outcomeChanged(s.outcome))
            text += ": " + reason;
        text += "\n";
        if (std::isfinite(s.rmsMm))
            text += lf("live_solve_rms", num(s.rmsMm), std::isfinite(s.currentRmsMm) ? num(s.currentRmsMm) : std::string("-")) + "\n";
        text += lf("live_solve_change", num(s.changeCm), num(s.changeDeg, 2), fmt::format("{:.5f}", s.axisVariance));
        return text;
    }

    // the explanation of a graph, below the values at the cursor
    void helpTooltip(const std::string& values, const std::string& help)
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.0f);
        if (!values.empty()) {
            ImGui::TextUnformatted(values.c_str());
            ImGui::Separator();
        }
        ImGui::TextDisabled("%s", help.c_str());
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }

    void graphTitle(const char* titleKey, const std::string& help)
    {
        ImGui::Spacing();
        ImGui::TextHeading("%s", LOCALE_GET(titleKey).c_str());
        ImGui::SameLine();
        ImGui::TextDisabled("%s", ICON_MS_INFO);
        if (ImGui::IsItemHovered())
            helpTooltip("", help);
    }

    // while live the graphs ignore the mouse (the wheel scrolls the page; hover and legend clicks
    // still work); paused, they pan and zoom together
    ImPlotFlags timePlotFlags()
    {
        return ImPlotFlags_NoMouseText | (ui().paused ? ImPlotFlags_None : ImPlotFlags_NoInputs);
    }

    void setupLegend()
    {
        ImPlot::SetupLegend(ImPlotLocation_North, ImPlotLegendFlags_Outside | ImPlotLegendFlags_Horizontal);
    }

    void setupTimeAxis()
    {
        LiveUiState& st = ui();
        ImPlot::SetupAxis(ImAxis_X1, nullptr, ImPlotAxisFlags_None);
        ImPlot::SetupAxisLinks(ImAxis_X1, &st.xMin, &st.xMax);
        ImPlot::SetupAxisFormat(ImAxis_X1, "%g s");
    }

    // vertical ticks for solves, universe shifts, re-centers and markers
    void eventTicks(const Ctx& c, bool legend)
    {
        std::vector<double> applied, shifts, recenters, markers;
        for (const LiveSolve& s : c.L.solves())
            if (s.t >= c.tFrom && s.t <= c.tTo && outcomeChanged(s.outcome) && s.outcome != static_cast<uint8_t>(blackbox::CalibrationOutcome::CORRECTED))
                applied.push_back(s.t - c.ref);
        for (const LiveEvent& e : c.L.events()) {
            if (e.t < c.tFrom || e.t > c.tTo)
                continue;
            (e.kind == LiveEventKind::PLAYSPACE_SHIFT ? shifts : (e.kind == LiveEventKind::REFERENCE_JUMP ? recenters : markers)).push_back(e.t - c.ref);
        }
        auto draw = [legend](const char* key, const std::vector<double>& xs, ImVec4 color) {
            ImPlotSpec spec;
            spec.LineColor = alpha(color, 0.7f);
            spec.LineWeight = 1.5f;
            if (!legend)
                spec.Flags = ImPlotItemFlags_NoLegend;
            const double none = -1e9; // keeps the legend entry when nothing is in view
            const std::string label = LOCALE_GET(key);
            if (xs.empty())
                ImPlot::PlotInfLines(label.c_str(), &none, 1, spec);
            else
                ImPlot::PlotInfLines(label.c_str(), xs.data(), static_cast<int>(xs.size()), spec);
        };
        draw("live_s_applied", applied, k_GOOD);
        draw("live_s_shift", shifts, k_SHIFT);
        draw("live_s_recenter", recenters, k_RECENTER);
        draw("live_s_marker", markers, k_MARKER);
    }

    void horizontalLine(const char* id, double y, ImVec4 color, bool legend = false, ImAxis yAxis = ImAxis_Y1)
    {
        ImPlot::SetAxes(ImAxis_X1, yAxis);
        ImPlotSpec spec;
        spec.LineColor = alpha(color, 0.75f);
        spec.LineWeight = 1.0f;
        spec.Flags = ImPlotInfLinesFlags_Horizontal | (legend ? 0 : ImPlotItemFlags_NoLegend);
        ImPlot::PlotInfLines(id, &y, 1, spec);
        ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
    }

    template <typename T, typename F>
    const T* nearestInTime(const std::deque<T>& d, double t, F accept)
    {
        const T* best = nullptr;
        double bestDt = 1e18;
        // the deque is in time order: binary search, then look around
        auto it = std::lower_bound(d.begin(), d.end(), t, [](const T& s, double v) { return s.t < v; });
        auto consider = [&](typename std::deque<T>::const_iterator i) {
            for (int k = 0; k < 64 && i != d.end(); k++, ++i) {
                if (accept(*i)) {
                    const double dt = std::abs(i->t - t);
                    if (dt < bestDt) {
                        bestDt = dt;
                        best = &*i;
                    }
                    return;
                }
            }
        };
        consider(it);
        if (it != d.begin()) {
            auto back = it;
            for (int k = 0; k < 64 && back != d.begin(); k++) {
                --back;
                if (accept(*back)) {
                    const double dt = std::abs(back->t - t);
                    if (dt < bestDt) {
                        bestDt = dt;
                        best = &*back;
                    }
                    break;
                }
            }
        }
        return best;
    }

    // ---- tiles --------------------------------------------------------------------------------

    template <typename F>
    void tile(const char* captionKey, ImVec4 valueColor, const std::string& value, const std::string& line2, const std::string& line3, F help)
    {
        ImGui::BeginGroup();
        ImGui::TextDisabled("%s", LOCALE_GET(captionKey).c_str());
        ImGui::PushStyleColor(ImGuiCol_Text, valueColor);
        ImGui::TextTitle("%s", value.c_str());
        ImGui::PopStyleColor();
        if (!line2.empty())
            ImGui::TextWrappedDisabled(line2.c_str());
        if (!line3.empty())
            ImGui::TextWrappedDisabled(line3.c_str());
        ImGui::EndGroup();
        if (ImGui::IsItemHovered())
            helpTooltip("", help());
    }

    void drawTiles(const Ctx& c)
    {
        const auto& head = c.L.head();
        const LiveHeadSample* now = nullptr;
        for (auto it = head.rbegin(); it != head.rend(); ++it) {
            if (it->t <= c.ref + 1e-6) {
                now = &*it;
                break;
            }
        }
        if (!ImGui::BeginTable("##live_tiles", 4, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchSame))
            return;
        ImGui::TableNextRow();

        // 1. head-tracker error
        ImGui::TableNextColumn();
        {
            double max10 = -1.0;
            for (auto it = head.rbegin(); it != head.rend() && it->t >= c.ref - 10.0; ++it)
                if (it->t <= c.ref + 1e-6 && it->valid)
                    max10 = std::max(max10, static_cast<double>(it->errorCm));
            const bool valid = now && now->valid;
            tile("live_tile_error", valid ? errorColor(now->errorCm, c) : k_GREY, valid ? num(now->errorCm) + " cm" : "-",
                valid ? lf("live_tile_error_max", num(max10)) : LOCALE_GET("live_tile_error_none"), "",
                [&c] { return lf("live_help_tile_error", num(c.rOk, 0), num(c.rHigh, 0)); });
        }

        // 2. guard
        ImGui::TableNextColumn();
        {
            std::string value = "-", line2, line3;
            ImVec4 color = k_GREY;
            if (now) {
                value = trust::stateName(static_cast<trust::State>(now->state));
                color = stateColor(now->state);
                double since = now->t;
                for (auto it = head.rbegin(); it != head.rend(); ++it) {
                    if (it->t > now->t)
                        continue;
                    if (it->state != now->state)
                        break;
                    since = it->t;
                }
                line2 = lf("live_tile_guard_for", duration(now->t - since) + (since <= head.front().t + 0.5 ? "+" : ""));
                const bool holdOn = GuardConfigManager::getInstance() && GuardConfigManager::getInstance()->get().trust.hold_enabled;
                line3 = now->hold ? LOCALE_GET("live_tile_guard_hold") : (holdOn ? LOCALE_GET("live_tile_guard_follow") : LOCALE_GET("live_tile_guard_observe"));
            }
            tile("live_tile_guard", color, value, line2, line3, [] { return LOCALE_GET("live_help_tile_guard"); });
        }

        // 3. calibration
        ImGui::TableNextColumn();
        {
            const LiveSolve* last = nullptr;
            unsigned applied = 0, rejected = 0;
            std::map<std::string, unsigned> reasons;
            for (const LiveSolve& s : c.L.solves()) {
                if (s.t > c.ref + 1e-6 || s.t < c.ref - 300.0)
                    continue;
                if (outcomeChanged(s.outcome)) {
                    applied++;
                    last = &s;
                } else if (s.outcome == static_cast<uint8_t>(blackbox::CalibrationOutcome::REJECTED)) {
                    rejected++;
                    reasons[reasonText(s)]++;
                }
            }
            std::string top;
            unsigned topN = 0;
            for (const auto& [r, n] : reasons)
                if (n > topN) {
                    top = r;
                    topN = n;
                }
            const std::string value = last ? lf("live_tile_cal_last", duration(c.ref - last->t)) : LOCALE_GET("live_tile_cal_none");
            tile("live_tile_cal", last ? (c.ref - last->t < 10.0 ? k_ACCENT : ImGui::GetStyleColorVec4(ImGuiCol_Text)) : k_GREY, value,
                lf("live_tile_cal_counts", applied, rejected), top.empty() ? "" : lf("live_tile_cal_top", top),
                [] { return LOCALE_GET("live_help_tile_cal"); });
        }

        // 4. universe shifts
        ImGui::TableNextColumn();
        {
            const LiveEvent* last = nullptr;
            for (const LiveEvent& e : c.L.events())
                if (e.kind == LiveEventKind::PLAYSPACE_SHIFT && e.t <= c.ref + 1e-6)
                    last = &e;
            const unsigned total = c.L.playspaceShiftsTotal();
            tile("live_tile_shifts", last && c.ref - last->t < 10.0 ? k_SHIFT : ImGui::GetStyleColorVec4(ImGuiCol_Text), lf("live_tile_shifts_count", total),
                last ? lf("live_tile_shifts_last", num(last->valueCm), duration(c.ref - last->t)) : LOCALE_GET("live_tile_shifts_none"), "",
                [] { return LOCALE_GET("live_help_tile_shifts"); });
        }
        ImGui::EndTable();
    }

    // ---- 1. head-tracker error ------------------------------------------------------------------

    void drawErrorGraph(const Ctx& c)
    {
        const std::string help = lf("live_help_error", num(c.rOk, 0), num(c.rHigh, 0));
        graphTitle("live_g_error", help);

        std::vector<double> xs, ys, dx, dy;
        double maxVisible = 0.0;
        struct Band {
            double x0, x1;
            uint8_t state;
        };
        std::vector<Band> bands;
        for (const LiveHeadSample& s : c.L.head()) {
            if (s.t < c.tFrom - 1.0 || s.t > c.tTo)
                continue;
            const double x = s.t - c.ref;
            xs.push_back(x);
            ys.push_back(s.valid ? s.errorCm : k_NAN);
            if (s.valid)
                maxVisible = std::max(maxVisible, static_cast<double>(s.errorCm));
            if (s.state != static_cast<uint8_t>(trust::State::TRUSTED)) {
                if (!bands.empty() && bands.back().state == s.state && x - bands.back().x1 < 0.5)
                    bands.back().x1 = x;
                else
                    bands.push_back({ x, x, s.state });
            }
        }
        decimateMinMax(xs, ys, k_MAX_LINE_POINTS, dx, dy);
        const double yMax = std::max(c.rHigh * 1.25, std::min(maxVisible * 1.1, 50.0));

        if (ImPlot::BeginPlot("##live_error", ImVec2(-1, 220), timePlotFlags())) {
            setupTimeAxis();
            ImPlot::SetupAxis(ImAxis_Y1, "cm");
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, yMax, ImPlotCond_Always);
            setupLegend();

            ImPlot::PushPlotClipRect();
            ImDrawList* draw = ImPlot::GetPlotDrawList();
            for (const Band& b : bands) {
                const ImVec2 p0 = ImPlot::PlotToPixels(b.x0, yMax);
                const ImVec2 p1 = ImPlot::PlotToPixels(std::max(b.x1, b.x0 + 0.05), 0.0);
                draw->AddRectFilled(p0, p1, ImGui::GetColorU32(alpha(stateColor(b.state), 0.2f)));
            }
            ImPlot::PopPlotClipRect();

            horizontalLine("##r_ok", c.rOk, k_WARN);
            horizontalLine("##r_high", c.rHigh, k_BAD);
            eventTicks(c, true);

            ImPlotSpec line;
            line.LineColor = k_ACCENT;
            line.LineWeight = 1.5f;
            line.Flags = ImPlotItemFlags_NoLegend;
            if (!dx.empty())
                ImPlot::PlotLine("##error", dx.data(), dy.data(), static_cast<int>(dx.size()), line);

            if (ImPlot::IsPlotHovered()) {
                const double t = c.ref + ImPlot::GetPlotMousePos().x;
                const LiveHeadSample* s = nearestInTime(c.L.head(), t, [](const LiveHeadSample&) { return true; });
                std::string values;
                if (s)
                    values = fmt::format("{}\n{} · {}", atText(c, s->t), s->valid ? num(s->errorCm) + " cm" : LOCALE_GET("live_tile_error_none"),
                        trust::stateName(static_cast<trust::State>(s->state)));
                helpTooltip(values, help);
            }
            ImPlot::EndPlot();
        }
    }

    // ---- 2. bullseye + error by direction -------------------------------------------------------

    void circle(const char* id, double r, ImVec4 color)
    {
        static std::vector<double> cx, cy;
        cx.clear();
        cy.clear();
        for (int i = 0; i <= 64; i++) {
            const double a = i * 2.0 * 3.14159265358979323846 / 64.0;
            cx.push_back(r * std::cos(a));
            cy.push_back(r * std::sin(a));
        }
        ImPlotSpec spec;
        spec.LineColor = alpha(color, 0.75f);
        spec.Flags = ImPlotItemFlags_NoLegend;
        ImPlot::PlotLine(id, cx.data(), cy.data(), static_cast<int>(cx.size()), spec);
    }

    // Three views of where the head tracker sits compared to where it should be (the centre), all on the
    // same scale: from above (right / forward), from the side (forward / up) and from behind (right / up).
    enum class View { ABOVE, SIDE, BEHIND };

    double viewX(View v, const LiveHeadSample& s)
    {
        return v == View::SIDE ? static_cast<double>(s.forwardCm) : static_cast<double>(s.sideCm);
    }

    double viewY(View v, const LiveHeadSample& s)
    {
        return v == View::ABOVE ? static_cast<double>(s.forwardCm) : static_cast<double>(s.upCm);
    }

    void drawHeadView(const Ctx& c, View v, const std::vector<const LiveHeadSample*>& pts, double lim, float size, const std::string& help)
    {
        const char* titleKey = v == View::ABOVE ? "live_bullseye_top" : (v == View::SIDE ? "live_bullseye_side" : "live_bullseye_back");
        const char* xKey = v == View::SIDE ? "live_axis_forward_cm" : "live_axis_right_cm";
        const char* yKey = v == View::ABOVE ? "live_axis_forward_cm" : "live_axis_up_cm";
        const std::string title = LOCALE_GET(titleKey) + "##view" + std::to_string(static_cast<int>(v));
        if (!ImPlot::BeginPlot(title.c_str(), ImVec2(size, size + ImGui::GetTextLineHeightWithSpacing()), ImPlotFlags_Equal | ImPlotFlags_NoMouseText | ImPlotFlags_NoLegend | ImPlotFlags_NoInputs))
            return;
        ImPlot::SetupAxes(LOCALE_GET(xKey).c_str(), LOCALE_GET(yKey).c_str());
        ImPlot::SetupAxesLimits(-lim, lim, -lim, lim, ImPlotCond_Always);
        const double zero = 0.0;
        ImPlotSpec cross;
        cross.LineColor = alpha(k_GREY, 0.4f);
        cross.Flags = ImPlotItemFlags_NoLegend;
        ImPlot::PlotInfLines("##cx", &zero, 1, cross);
        cross.Flags = ImPlotItemFlags_NoLegend | ImPlotInfLinesFlags_Horizontal;
        ImPlot::PlotInfLines("##cy", &zero, 1, cross);
        circle("##ring_ok", c.rOk, k_WARN);
        circle("##ring_high", c.rHigh, k_BAD);

        // older dots fainter: four age bands across the visible window
        const size_t stride = std::max<size_t>(1, pts.size() / 2400);
        const double span = std::max(1e-3, c.tTo - c.tFrom);
        std::vector<double> bx, by;
        for (int band = 0; band < 4; band++) {
            bx.clear();
            by.clear();
            for (size_t i = 0; i < pts.size(); i += stride) {
                const double age = (c.tTo - pts[i]->t) / span;
                const int b = std::min(3, static_cast<int>(age * 4.0));
                if (3 - b != band)
                    continue;
                bx.push_back(viewX(v, *pts[i]));
                by.push_back(viewY(v, *pts[i]));
            }
            if (bx.empty())
                continue;
            ImPlotSpec dots;
            dots.Marker = ImPlotMarker_Circle;
            dots.MarkerSize = 2.0f;
            const float a = 0.15f + 0.25f * band;
            dots.MarkerFillColor = alpha(k_ACCENT, a);
            dots.MarkerLineColor = alpha(k_ACCENT, a);
            dots.Flags = ImPlotItemFlags_NoLegend;
            const std::string id = fmt::format("##age{}", band);
            ImPlot::PlotScatter(id.c_str(), bx.data(), by.data(), static_cast<int>(bx.size()), dots);
        }
        if (!pts.empty()) {
            const double nx = viewX(v, *pts.back()), ny = viewY(v, *pts.back());
            ImPlotSpec nowDot;
            nowDot.Marker = ImPlotMarker_Circle;
            nowDot.MarkerSize = 6.0f;
            nowDot.MarkerFillColor = errorColor(pts.back()->errorCm, c);
            nowDot.MarkerLineColor = ImVec4(1, 1, 1, 1);
            nowDot.Flags = ImPlotItemFlags_NoLegend;
            ImPlot::PlotScatter("##now", &nx, &ny, 1, nowDot);
        }
        if (ImPlot::IsPlotHovered()) {
            const ImPlotPoint m = ImPlot::GetPlotMousePos();
            const char* atKey = v == View::ABOVE ? "live_bullseye_at_top" : (v == View::SIDE ? "live_bullseye_at_side" : "live_bullseye_at_back");
            std::string values = lf(atKey, num(m.x), num(m.y), num(std::hypot(m.x, m.y)));
            if (!pts.empty())
                values += "\n" + lf("live_bullseye_now", num(pts.back()->forwardCm), num(pts.back()->sideCm), num(pts.back()->upCm), num(pts.back()->errorCm));
            helpTooltip(values, help);
        }
        ImPlot::EndPlot();
    }

    void drawHeadViews(const Ctx& c)
    {
        const std::string help = lf("live_help_bullseye", num(c.rOk, 0), num(c.rHigh, 0));
        graphTitle("live_g_bullseye", help);
        std::vector<const LiveHeadSample*> pts;
        for (const LiveHeadSample& s : c.L.head())
            if (s.valid && s.t >= c.tFrom && s.t <= c.tTo)
                pts.push_back(&s);
        // one scale for all three views, so they can be compared
        double maxAbs = 0.0;
        for (const LiveHeadSample* s : pts)
            maxAbs = std::max({ maxAbs, std::abs(static_cast<double>(s->forwardCm)), std::abs(static_cast<double>(s->sideCm)), std::abs(static_cast<double>(s->upCm)) });
        const double lim = std::max(c.rHigh * 1.25, std::min(maxAbs * 1.15, 60.0));
        if (!ImGui::BeginTable("##live_views", 3, ImGuiTableFlags_SizingStretchSame))
            return;
        ImGui::TableNextRow();
        for (View v : { View::ABOVE, View::SIDE, View::BEHIND }) {
            ImGui::TableNextColumn();
            ImGui::PushID(static_cast<int>(v));
            // square, at most 360 px, centred in its column
            const float column = ImGui::GetContentRegionAvail().x;
            const float size = std::min(360.0f, column);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (column - size) * 0.5f);
            drawHeadView(c, v, pts, lim, size, help);
            ImGui::PopID();
        }
        ImGui::EndTable();
    }

    void drawDirection(const Ctx& c, float height)
    {
        const std::string help = LOCALE_GET("live_help_direction");
        graphTitle("live_g_direction", help);
        std::vector<double> xs, f, r, u;
        double maxAbs = 0.0;
        for (const LiveHeadSample& s : c.L.head()) {
            if (s.t < c.tFrom - 1.0 || s.t > c.tTo)
                continue;
            xs.push_back(s.t - c.ref);
            f.push_back(s.valid ? s.forwardCm : k_NAN);
            r.push_back(s.valid ? s.sideCm : k_NAN);
            u.push_back(s.valid ? s.upCm : k_NAN);
            if (s.valid)
                maxAbs = std::max({ maxAbs, std::abs(static_cast<double>(s.forwardCm)), std::abs(static_cast<double>(s.sideCm)), std::abs(static_cast<double>(s.upCm)) });
        }
        const double lim = std::max(c.rOk * 2.0, std::min(maxAbs * 1.15, 50.0));
        if (ImPlot::BeginPlot("##live_direction", ImVec2(-1, height), timePlotFlags())) {
            setupTimeAxis();
            ImPlot::SetupAxis(ImAxis_Y1, "cm");
            ImPlot::SetupAxisLimits(ImAxis_Y1, -lim, lim, ImPlotCond_Always);
            setupLegend();
            horizontalLine("##zero", 0.0, k_GREY);
            const char* keys[] = { "live_s_forward", "live_s_right", "live_s_up" };
            const std::vector<double>* series[] = { &f, &r, &u };
            std::vector<double> dx, dy;
            for (int i = 0; i < 3; i++) {
                decimateMinMax(xs, *series[i], k_MAX_LINE_POINTS, dx, dy);
                if (dx.empty())
                    continue;
                ImPlotSpec spec;
                spec.LineColor = ImPlot::GetColormapColor(i);
                spec.LineWeight = 1.25f;
                ImPlot::PlotLine(LOCALE_GET(keys[i]).c_str(), dx.data(), dy.data(), static_cast<int>(dx.size()), spec);
            }
            if (ImPlot::IsPlotHovered()) {
                const double t = c.ref + ImPlot::GetPlotMousePos().x;
                const LiveHeadSample* s = nearestInTime(c.L.head(), t, [](const LiveHeadSample& h) { return h.valid; });
                std::string values;
                if (s)
                    values = fmt::format("{}\n{}", atText(c, s->t), lf("live_direction_values", num(s->forwardCm), num(s->sideCm), num(s->upCm)));
                helpTooltip(values, help);
            }
            ImPlot::EndPlot();
        }
    }

    // ---- 3. calibration: applied vs proposed ----------------------------------------------------

    void drawCalibrationGraph(const Ctx& c)
    {
        const std::string help = LOCALE_GET("live_help_cal");
        graphTitle("live_g_cal", help);
        const auto& cal = c.L.calibration();
        const LiveCalibrationSample* refCal = nullptr;
        for (auto it = cal.rbegin(); it != cal.rend(); ++it)
            if (it->t <= c.ref + 1e-6) {
                refCal = &*it;
                break;
            }
        if (!refCal) {
            ImGui::TextDisabled("%s", LOCALE_GET("live_no_cal").c_str());
            return;
        }
        // lines relative to the calibration at the right edge of the graph
        std::vector<double> xs, ys[4];
        const LiveCalibrationSample* before = nullptr;
        for (const LiveCalibrationSample& s : cal) {
            if (s.t < c.tFrom) {
                before = &s;
                continue;
            }
            if (s.t > c.tTo)
                break;
            if (xs.empty() && before) { // the value in effect at the left edge
                xs.push_back(c.tFrom - c.ref);
                ys[0].push_back(before->xCm - refCal->xCm);
                ys[1].push_back(before->yCm - refCal->yCm);
                ys[2].push_back(before->zCm - refCal->zCm);
                ys[3].push_back(before->yawDeg - refCal->yawDeg);
            }
            xs.push_back(s.t - c.ref);
            ys[0].push_back(s.xCm - refCal->xCm);
            ys[1].push_back(s.yCm - refCal->yCm);
            ys[2].push_back(s.zCm - refCal->zCm);
            ys[3].push_back(s.yawDeg - refCal->yawDeg);
        }
        if (!xs.empty()) { // hold the last value to the right edge
            xs.push_back(std::min(c.tTo, c.L.now()) - c.ref);
            for (auto& y : ys)
                y.push_back(y.back());
        }
        std::vector<const LiveSolve*> solves;
        for (const LiveSolve& s : c.L.solves())
            if (s.t >= c.tFrom && s.t <= c.tTo)
                solves.push_back(&s);

        double maxCm = 2.0, maxDeg = 1.0;
        for (int i = 0; i < 3; i++)
            for (double v : ys[i])
                maxCm = std::max(maxCm, std::abs(v) * 1.15);
        for (double v : ys[3])
            maxDeg = std::max(maxDeg, std::abs(v) * 1.15);
        for (const LiveSolve* s : solves) {
            maxCm = std::max({ maxCm, std::abs(s->xCm - refCal->xCm) * 1.15, std::abs(s->yCm - refCal->yCm) * 1.15, std::abs(s->zCm - refCal->zCm) * 1.15 });
            maxDeg = std::max(maxDeg, std::abs(s->yawDeg - refCal->yawDeg) * 1.15);
        }
        maxCm = std::min(maxCm, 50.0);
        maxDeg = std::min(maxDeg, 30.0);

        if (ImPlot::BeginPlot("##live_cal", ImVec2(-1, 230), timePlotFlags())) {
            setupTimeAxis();
            ImPlot::SetupAxis(ImAxis_Y1, "cm");
            ImPlot::SetupAxisLimits(ImAxis_Y1, -maxCm, maxCm, ImPlotCond_Always);
            ImPlot::SetupAxis(ImAxis_Y2, LOCALE_GET("live_axis_deg").c_str(), ImPlotAxisFlags_AuxDefault);
            ImPlot::SetupAxisLimits(ImAxis_Y2, -maxDeg, maxDeg, ImPlotCond_Always);
            setupLegend();
            horizontalLine("##zero", 0.0, k_GREY);

            const char* keys[] = { "live_s_dx", "live_s_dy", "live_s_dz", "live_s_dyaw" };
            for (int i = 0; i < 4; i++) {
                ImPlot::SetAxes(ImAxis_X1, i == 3 ? ImAxis_Y2 : ImAxis_Y1);
                const ImVec4 color = ImPlot::GetColormapColor(i);
                if (!xs.empty()) {
                    ImPlotSpec spec;
                    spec.LineColor = color;
                    spec.LineWeight = 1.5f;
                    ImPlot::PlotStairs(LOCALE_GET(keys[i]).c_str(), xs.data(), ys[i].data(), static_cast<int>(xs.size()), spec);
                }
                // proposals: filled when it changed the calibration, hollow when rejected
                for (int filled = 0; filled < 2; filled++) {
                    std::vector<double> px, py;
                    for (const LiveSolve* s : solves) {
                        if ((outcomeChanged(s->outcome) ? 1 : 0) != filled || s->outcome == static_cast<uint8_t>(blackbox::CalibrationOutcome::SKIPPED))
                            continue;
                        px.push_back(s->t - c.ref);
                        const float v[4] = { s->xCm - refCal->xCm, s->yCm - refCal->yCm, s->zCm - refCal->zCm, s->yawDeg - refCal->yawDeg };
                        py.push_back(v[i]);
                    }
                    if (px.empty())
                        continue;
                    ImPlotSpec dots;
                    dots.Marker = i == 3 ? ImPlotMarker_Diamond : ImPlotMarker_Circle;
                    dots.MarkerSize = 3.5f;
                    dots.MarkerLineColor = color;
                    dots.MarkerFillColor = filled ? color : alpha(color, 0.0f);
                    dots.Flags = ImPlotItemFlags_NoLegend;
                    const std::string id = fmt::format("##prop{}_{}", i, filled);
                    ImPlot::PlotScatter(id.c_str(), px.data(), py.data(), static_cast<int>(px.size()), dots);
                }
            }
            ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);

            if (ImPlot::IsPlotHovered()) {
                const ImPlotPoint m = ImPlot::GetPlotMousePos();
                const double t = c.ref + m.x;
                const double nearSeconds = (c.tTo - c.tFrom) / 80.0;
                const LiveSolve* s = nearestInTime(c.L.solves(), t, [](const LiveSolve&) { return true; });
                std::string values;
                if (s && std::abs(s->t - t) <= nearSeconds) {
                    values = solveDetails(c, *s);
                } else {
                    const LiveCalibrationSample* k = nearestInTime(cal, t, [t](const LiveCalibrationSample& x) { return x.t <= t; });
                    if (k)
                        values = fmt::format("{}\n{}", atText(c, t), lf("live_cal_values", num(k->xCm - refCal->xCm), num(k->yCm - refCal->yCm), num(k->zCm - refCal->zCm), num(k->yawDeg - refCal->yawDeg, 2)));
                }
                helpTooltip(values, help);
            }
            ImPlot::EndPlot();
        }
    }

    // ---- 4. glitch meter + verdict per device ---------------------------------------------------

    struct DeviceRow {
        uint8_t index;
        std::string label;
        int order;
    };

    std::vector<DeviceRow> visibleDevices(const Ctx& c)
    {
        std::vector<DeviceRow> rows;
        for (uint8_t i = 0; i < 64; i++) {
            const LiveDeviceInfo& info = c.L.deviceInfo(i);
            const auto& d = c.L.device(i);
            if (!info.known || d.empty() || d.back().t < c.tFrom || d.front().t > c.tTo)
                continue;
            std::string label = info.label;
            int order = info.isTarget ? 0 : (info.isTracker ? 1 : (info.isReference ? 3 : 2));
            if (info.isTracker) {
                // which body part: the typical height over the last minutes
                std::vector<float> h;
                const size_t stride = std::max<size_t>(1, d.size() / 400);
                for (size_t k = 0; k < d.size(); k += stride)
                    if (d[k].tracking)
                        h.push_back(d[k].heightM);
                if (h.size() > 20) {
                    std::nth_element(h.begin(), h.begin() + static_cast<std::ptrdiff_t>(h.size() / 2), h.end());
                    const float median = h[h.size() / 2];
                    if (median < 0.45f)
                        label += " (" + LOCALE_GET("live_guess_foot") + ")";
                    else if (median < 1.15f)
                        label += " (" + LOCALE_GET("live_guess_hip") + ")";
                }
            }
            rows.push_back({ i, label, order });
        }
        std::stable_sort(rows.begin(), rows.end(), [](const DeviceRow& a, const DeviceRow& b) { return a.order < b.order; });
        return rows;
    }

    void drawGlitchMeter(const Ctx& c)
    {
        const std::string help = LOCALE_GET("live_help_glitch");
        graphTitle("live_g_glitch", help);
        const std::vector<DeviceRow> rows = visibleDevices(c);
        if (rows.empty()) {
            ImGui::TextDisabled("%s", LOCALE_GET("live_no_devices").c_str());
            return;
        }
        double maxVisible = 0.0;
        std::vector<float> limits;
        for (const DeviceRow& row : rows)
            for (const LiveDeviceSample& s : c.L.device(row.index))
                if (s.t >= c.tFrom && s.t <= c.tTo) {
                    maxVisible = std::max(maxVisible, static_cast<double>(s.unexplainedCm));
                    if (s.limitCm > 0.0f)
                        limits.push_back(s.limitCm);
                }
        double limit = 3.0;
        if (!limits.empty()) {
            std::nth_element(limits.begin(), limits.begin() + static_cast<std::ptrdiff_t>(limits.size() / 2), limits.end());
            limit = limits[limits.size() / 2];
        }
        const double yMax = std::max(limit * 2.0, std::min(maxVisible * 1.1, 30.0));

        if (ImPlot::BeginPlot("##live_glitch", ImVec2(-1, 230), timePlotFlags())) {
            setupTimeAxis();
            ImPlot::SetupAxis(ImAxis_Y1, "cm");
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, yMax, ImPlotCond_Always);
            setupLegend();
            horizontalLine(LOCALE_GET("live_s_limit").c_str(), limit, k_BAD, true);
            std::vector<double> xs, ys, dx, dy;
            for (size_t k = 0; k < rows.size(); k++) {
                xs.clear();
                ys.clear();
                for (const LiveDeviceSample& s : c.L.device(rows[k].index)) {
                    if (s.t < c.tFrom - 1.0 || s.t > c.tTo)
                        continue;
                    xs.push_back(s.t - c.ref);
                    ys.push_back(s.tracking ? s.unexplainedCm : k_NAN);
                }
                decimateMinMax(xs, ys, k_MAX_LINE_POINTS, dx, dy);
                if (dx.empty())
                    continue;
                ImPlotSpec spec;
                spec.LineColor = ImPlot::GetColormapColor(static_cast<int>(k));
                spec.LineWeight = rows[k].order == 0 ? 1.75f : 1.0f;
                ImPlot::PlotLine(rows[k].label.c_str(), dx.data(), dy.data(), static_cast<int>(dx.size()), spec);
            }
            if (ImPlot::IsPlotHovered()) {
                const double t = c.ref + ImPlot::GetPlotMousePos().x;
                std::string values = atText(c, t);
                for (const DeviceRow& row : rows) {
                    const LiveDeviceSample* s = nearestInTime(c.L.device(row.index), t, [](const LiveDeviceSample&) { return true; });
                    if (s && std::abs(s->t - t) < 0.5)
                        values += fmt::format("\n{}: {}", row.label,
                            s->tracking ? fmt::format("{} cm · {}", num(s->unexplainedCm), trust::stateName(static_cast<trust::State>(s->state))) : LOCALE_GET("live_not_tracking"));
                }
                helpTooltip(values, help);
            }
            ImPlot::EndPlot();
        }

        // the verdict per device, same time axis
        const std::string helpStates = LOCALE_GET("live_help_states");
        graphTitle("live_g_states", helpStates);
        const int n = static_cast<int>(rows.size());
        std::vector<double> ticks;
        std::vector<const char*> labels;
        for (int k = 0; k < n; k++) {
            ticks.push_back(k);
            labels.push_back(rows[static_cast<size_t>(k)].label.c_str());
        }
        if (ImPlot::BeginPlot("##live_states", ImVec2(-1, 34.0f + 20.0f * n), timePlotFlags() | ImPlotFlags_NoLegend)) {
            setupTimeAxis();
            ImPlot::SetupAxis(ImAxis_Y1, nullptr, ImPlotAxisFlags_Invert | ImPlotAxisFlags_NoGridLines);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -0.5, n - 0.5, ImPlotCond_Always);
            ImPlot::SetupAxisTicks(ImAxis_Y1, ticks.data(), n, labels.data());
            ImPlot::PushPlotClipRect();
            ImDrawList* draw = ImPlot::GetPlotDrawList();
            for (int k = 0; k < n; k++) {
                const auto& d = c.L.device(rows[static_cast<size_t>(k)].index);
                double segStart = 0.0;
                ImVec4 segColor;
                bool open = false;
                auto flush = [&](double x1) {
                    if (!open)
                        return;
                    const ImVec2 p0 = ImPlot::PlotToPixels(segStart, k - 0.38);
                    const ImVec2 p1 = ImPlot::PlotToPixels(std::max(x1, segStart + 0.02), k + 0.38);
                    draw->AddRectFilled(ImVec2(p0.x, std::min(p0.y, p1.y)), ImVec2(p1.x, std::max(p0.y, p1.y)), ImGui::GetColorU32(segColor));
                };
                double lastX = 0.0;
                for (const LiveDeviceSample& s : d) {
                    if (s.t < c.tFrom - 1.0 || s.t > c.tTo)
                        continue;
                    const double x = s.t - c.ref;
                    const ImVec4 color = !s.tracking ? alpha(k_GREY, 0.6f) : (s.state == 0 ? alpha(k_GOOD, 0.35f) : stateColor(s.state));
                    if (!open || color.x != segColor.x || color.y != segColor.y || color.z != segColor.z || color.w != segColor.w || x - lastX > 0.5) {
                        flush(lastX);
                        segStart = x;
                        segColor = color;
                        open = true;
                    }
                    lastX = x;
                }
                flush(lastX);
            }
            ImPlot::PopPlotClipRect();
            // an invisible item so the plot has something to fit and hover
            const double none = k_NAN;
            ImPlotSpec hidden;
            hidden.Flags = ImPlotItemFlags_NoLegend;
            ImPlot::PlotLine("##none", &none, &none, 1, hidden);
            if (ImPlot::IsPlotHovered()) {
                const ImPlotPoint m = ImPlot::GetPlotMousePos();
                const int k = static_cast<int>(std::lround(m.y));
                std::string values;
                if (k >= 0 && k < n) {
                    const DeviceRow& row = rows[static_cast<size_t>(k)];
                    const LiveDeviceSample* s = nearestInTime(c.L.device(row.index), c.ref + m.x, [](const LiveDeviceSample&) { return true; });
                    if (s)
                        values = fmt::format("{}\n{}: {}", atText(c, s->t), row.label,
                            s->tracking ? std::string(trust::stateName(static_cast<trust::State>(s->state))) : LOCALE_GET("live_not_tracking"));
                }
                helpTooltip(values, helpStates);
            }
            ImPlot::EndPlot();
        }
    }

    // ---- 5. solver health ----------------------------------------------------------------------

    void drawSolverHealth(const Ctx& c)
    {
        std::vector<const LiveSolve*> solves;
        for (const LiveSolve& s : c.L.solves())
            if (s.t >= c.tFrom && s.t <= c.tTo && s.outcome != static_cast<uint8_t>(blackbox::CalibrationOutcome::CORRECTED)
                && s.outcome != static_cast<uint8_t>(blackbox::CalibrationOutcome::SKIPPED))
                solves.push_back(&s);

        auto hoverSolve = [&c, &solves](const std::string& help) {
            const double t = c.ref + ImPlot::GetPlotMousePos().x;
            const LiveSolve* best = nullptr;
            for (const LiveSolve* s : solves)
                if (!best || std::abs(s->t - t) < std::abs(best->t - t))
                    best = s;
            helpTooltip(best && std::abs(best->t - t) < (c.tTo - c.tFrom) / 40.0 ? solveDetails(c, *best) : atText(c, t), help);
        };
        auto scatterByOutcome = [&solves, &c](bool rms) {
            const uint8_t outcomes[] = { static_cast<uint8_t>(blackbox::CalibrationOutcome::APPLIED), static_cast<uint8_t>(blackbox::CalibrationOutcome::FORCED),
                static_cast<uint8_t>(blackbox::CalibrationOutcome::REJECTED) };
            for (uint8_t o : outcomes) {
                std::vector<double> xs, ys;
                for (const LiveSolve* s : solves) {
                    if (s->outcome != o)
                        continue;
                    const double v = rms ? s->rmsMm : s->axisVariance;
                    if (!std::isfinite(v) || (!rms && v <= 0.0))
                        continue;
                    xs.push_back(s->t - c.ref);
                    ys.push_back(v);
                }
                ImPlotSpec dots;
                dots.Marker = ImPlotMarker_Circle;
                dots.MarkerSize = 3.5f;
                dots.MarkerFillColor = outcomeColor(o);
                dots.MarkerLineColor = outcomeColor(o);
                const std::string label = outcomeText(o);
                const double none = k_NAN;
                if (xs.empty())
                    ImPlot::PlotScatter(label.c_str(), &none, &none, 1, dots);
                else
                    ImPlot::PlotScatter(label.c_str(), xs.data(), ys.data(), static_cast<int>(xs.size()), dots);
            }
        };

        if (!ImGui::BeginTable("##live_solver", 2, ImGuiTableFlags_SizingStretchSame))
            return;
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        {
            const std::string help = LOCALE_GET("live_help_rms");
            graphTitle("live_g_rms", help);
            double maxRms = 5.0;
            for (const LiveSolve* s : solves) {
                if (std::isfinite(s->rmsMm))
                    maxRms = std::max(maxRms, static_cast<double>(s->rmsMm) * 1.15);
                if (std::isfinite(s->currentRmsMm))
                    maxRms = std::max(maxRms, static_cast<double>(s->currentRmsMm) * 1.15);
            }
            maxRms = std::min(maxRms, 100.0);
            if (ImPlot::BeginPlot("##live_rms", ImVec2(-1, 210), timePlotFlags())) {
                setupTimeAxis();
                ImPlot::SetupAxis(ImAxis_Y1, "mm");
                ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, maxRms, ImPlotCond_Always);
                setupLegend();
                std::vector<double> xs, ys;
                for (const LiveSolve* s : solves)
                    if (std::isfinite(s->currentRmsMm)) {
                        xs.push_back(s->t - c.ref);
                        ys.push_back(s->currentRmsMm);
                    }
                ImPlotSpec line;
                line.LineColor = alpha(k_ACCENT, 0.9f);
                line.LineWeight = 1.25f;
                const std::string label = LOCALE_GET("live_s_current_rms");
                if (!xs.empty())
                    ImPlot::PlotLine(label.c_str(), xs.data(), ys.data(), static_cast<int>(xs.size()), line);
                scatterByOutcome(true);
                if (ImPlot::IsPlotHovered())
                    hoverSolve(help);
                ImPlot::EndPlot();
            }
        }
        ImGui::TableNextColumn();
        {
            const std::string help = LOCALE_GET("live_help_axis");
            graphTitle("live_g_axis", help);
            if (ImPlot::BeginPlot("##live_axis", ImVec2(-1, 210), timePlotFlags())) {
                setupTimeAxis();
                ImPlot::SetupAxis(ImAxis_Y1, nullptr, ImPlotAxisFlags_AutoFit);
                ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
                setupLegend();
                horizontalLine(LOCALE_GET("live_s_axis_limit").c_str(), k_MAX_AXIS_VARIANCE_THRESHOLD, k_BAD, true);
                scatterByOutcome(false);
                if (ImPlot::IsPlotHovered())
                    hoverSolve(help);
                ImPlot::EndPlot();
            }
        }
        ImGui::EndTable();

        // what the solver did in this window, in words
        unsigned applied = 0, rejected = 0;
        std::map<std::string, unsigned> reasons;
        for (const LiveSolve* s : solves) {
            if (outcomeChanged(s->outcome))
                applied++;
            else {
                rejected++;
                reasons[reasonText(*s)]++;
            }
        }
        if (solves.empty()) {
            ImGui::TextDisabled("%s", LOCALE_GET("live_solves_none").c_str());
        } else {
            std::string text = lf("live_solves_summary", applied, rejected);
            if (!reasons.empty()) {
                std::vector<std::pair<unsigned, std::string>> sorted;
                for (const auto& [r, k] : reasons)
                    sorted.push_back({ k, r });
                std::sort(sorted.rbegin(), sorted.rend());
                std::string list;
                for (const auto& [k, r] : sorted)
                    list += (list.empty() ? "" : ", ") + fmt::format("{} {}x", r, k);
                text += " " + lf("live_solves_reasons", list);
            }
            ImGui::TextWrappedDisabled(text.c_str());
        }
    }
}

void page_live(double currentTime)
{
    ImGui::TextTitle("%s", LOCALE_GET("live_title").c_str());
    draw_mark_event_row(); // a glitch seen here can be marked and described right away
    ImGui::TextWrappedDisabled(LOCALE_GET("live_intro").c_str());
    ImGui::Separator();

    LiveStats* live = LiveStats::getInstance();
    trust::TrustManager* tm = trust::TrustManager::getInstance();
    if (!live || !tm)
        return;
    LiveUiState& st = ui();
    if (g_liveDemo) {
        live_demo_control(currentTime);
        live_demo_tick(currentTime);
        ImGui::TextColored(k_WARN, "%s", "--live-demo: made-up data");
    }

    // window and pause
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(LOCALE_GET("live_window").c_str());
    for (int i = 0; i < 3; i++) {
        ImGui::SameLine();
        if (ImGui::RadioButton(LOCALE_GET(k_WINDOW_KEYS[i]).c_str(), st.window == i)) {
            st.window = i;
            st.xMin = -k_WINDOWS[i];
            st.xMax = 0.0;
        }
    }
    ImGui::SameLine(0.0f, ImGui::GetFontSize() * 2.0f);
    const std::time_t wallNow = std::time(nullptr);
    if (!st.paused) {
        if (ImGui::IconButton(ICON_MS_PAUSE, LOCALE_GET("live_pause").c_str())) {
            st.paused = true;
            st.pausedAt = currentTime;
            st.snapshot = *live;
        }
        if (ImGui::IsItemHovered())
            helpTooltip("", LOCALE_GET("live_help_pause"));
    } else {
        if (ImGui::IconButtonPrimary(ICON_MS_PLAY_ARROW, LOCALE_GET("live_resume").c_str())) {
            st.paused = false;
            st.snapshot.clear();
        }
        ImGui::SameLine();
        const Ctx clockCtx { st.snapshot, st.pausedAt, currentTime, wallNow, 0.0, 0.0, 0.0, 0.0 };
        ImGui::TextDisabled("%s", lf("live_paused_hint", wallClock(clockCtx, st.pausedAt)).c_str());
    }
    if (!st.paused) {
        st.xMin = -k_WINDOWS[st.window];
        st.xMax = 0.0;
    }

    const LiveStats& L = st.paused ? st.snapshot : *live;
    if (L.now() < 0.0) {
        ImGui::Spacing();
        ImGui::TextWrapped("%s", LOCALE_GET("live_no_data").c_str());
        return;
    }
    const trust::ConsensusParams& params = tm->consensus().params();
    const double ref = st.paused ? st.pausedAt : currentTime;
    const Ctx c { L, ref, currentTime, wallNow, params.r_ok * 100.0, params.r_high * 100.0, ref + st.xMin, ref + st.xMax };

    ImGui::Spacing();
    drawTiles(c);
    drawErrorGraph(c);

    drawHeadViews(c);
    drawDirection(c, 220.0f);

    drawCalibrationGraph(c);
    drawGlitchMeter(c);
    drawSolverHealth(c);
}

} // namespace spacecal::guard
