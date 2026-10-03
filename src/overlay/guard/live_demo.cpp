// Development aid (fork): `SpaceCalibrator.exe --live-demo` fills the Live tab with made-up data, so
// its layout and explanations can be checked without SteamVR or a headset. Never runs otherwise.

#include "guard_ui.h"

#include "blackbox_format.h"
#include "imgui.h"
#include "live_stats.h"
#include "util.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>

namespace spacecal::guard {

bool g_liveDemo = false;

namespace {
    double noise(uint32_t& seed)
    {
        seed = seed * 1664525u + 1013904223u;
        return static_cast<double>(seed >> 8) / static_cast<double>(1u << 24) - 0.5; // -0.5 .. 0.5
    }
}

// `%APPDATA%\space-calibrator\live_demo.txt` with lines `scroll=<px>` and `mouse=<x>,<y>` scrolls the
// page and places the mouse, so screenshots can show every graph and its hover explanation
// without touching the real mouse. Read twice a second; a missing file changes nothing.
void live_demo_control(double t)
{
    static double nextRead = 0.0;
    static bool hasScroll = false, hasMouse = false;
    static float scroll = 0.0f, mx = 0.0f, my = 0.0f;
    if (!g_liveDemo)
        return;
    if (t >= nextRead) {
        nextRead = t + 0.5;
        hasScroll = hasMouse = false;
        std::ifstream f(util::getSpaceCalibratorConfigDir() / "live_demo.txt");
        std::string line;
        while (std::getline(f, line)) {
            if (std::sscanf(line.c_str(), "scroll=%f", &scroll) == 1)
                hasScroll = true;
            else if (std::sscanf(line.c_str(), "mouse=%f,%f", &mx, &my) == 2)
                hasMouse = true;
        }
    }
    if (hasScroll)
        ImGui::SetScrollY(scroll);
    if (hasMouse)
        ImGui::GetIO().AddMousePosEvent(mx, my);
}

void live_demo_tick(double t)
{
    LiveStats* live = LiveStats::getInstance();
    static double last = -1.0;
    static uint32_t seed = 518;
    if (!g_liveDemo || !live || t - last < 1.0 / 60.0)
        return;
    const double dt = last < 0.0 ? 0.0 : t - last;
    last = t;

    // head tracker: a small wobble with the head turns, a glitch every 47 s that lasts 4 s
    const double cycle = std::fmod(t, 47.0);
    const bool glitch = cycle > 30.0 && cycle < 34.0;
    LiveHeadSample h;
    h.t = t;
    h.valid = true;
    h.forwardCm = static_cast<float>(0.8 + 0.6 * std::sin(t * 0.9) + 0.2 * noise(seed) + (glitch ? 14.0 : 0.0));
    h.sideCm = static_cast<float>(0.4 * std::sin(t * 1.7) + 0.2 * noise(seed) + (glitch ? -6.0 : 0.0));
    h.upCm = static_cast<float>(-0.5 + 0.3 * std::cos(t * 0.6) + 0.15 * noise(seed));
    h.errorCm = static_cast<float>(std::sqrt(h.forwardCm * h.forwardCm + h.sideCm * h.sideCm + h.upCm * h.upCm));
    h.state = glitch ? (cycle < 30.5 ? 1 : 2) : (cycle >= 34.0 && cycle < 37.0 ? 3 : 0);
    live->pushHead(h);

    struct Dev {
        uint8_t index;
        const char* label;
        bool tracker;
        float height;
        double jitter;
    };
    static const Dev devs[] = { { 20, "Head tracker", false, 1.15f, 0.3 }, { 24, "Tracker 1A2B3C4D", true, 0.65f, 0.5 }, { 13, "Tracker 5E6F7A8B", true, 0.12f, 0.8 },
        { 23, "Tracker 9C0D1E2F", true, 0.13f, 0.8 }, { 21, "Index right", false, 1.0f, 1.0 }, { 22, "Index left", false, 1.0f, 1.0 }, { 0, "Headset", false, 1.2f, 0.2 } };
    for (const Dev& d : devs) {
        LiveDeviceInfo info;
        info.known = true;
        info.label = d.label;
        info.isTarget = d.index == 20;
        info.isReference = d.index == 0;
        info.isTracker = d.tracker;
        LiveDeviceSample s;
        s.t = t;
        s.unexplainedCm = static_cast<float>(std::abs(noise(seed)) * d.jitter);
        if (d.index == 13 && std::fmod(t, 23.0) < dt)
            s.unexplainedCm = 6.5f; // a foot jumps every 23 s
        if (d.index == 20 && glitch && cycle < 30.0 + dt)
            s.unexplainedCm = 15.0f;
        s.limitCm = static_cast<float>(100.0 * (1.0 * (1.0 / 60.0) + 0.03));
        s.heightM = d.height + static_cast<float>(0.05 * noise(seed));
        s.state = d.index == 20 ? h.state : (d.index == 13 && std::fmod(t, 23.0) < 3.5 ? 2 : 0);
        s.tracking = !(d.index == 23 && std::fmod(t, 61.0) > 55.0);
        live->pushDevice(d.index, info, s);
    }

    // calibration: steps when a solve is applied
    static float cx = 102.0f, cy = -3.0f, cz = 51.0f, cyaw = 31.5f;
    static double nextSolve = 0.0;
    if (t >= nextSolve) {
        nextSolve = t + 3.0;
        LiveSolve s;
        s.t = t;
        s.trigger = static_cast<uint8_t>(blackbox::CalibrationTrigger::CONTINUOUS);
        const bool apply = std::fmod(t, 12.0) < 3.0;
        s.outcome = static_cast<uint8_t>(apply ? blackbox::CalibrationOutcome::APPLIED : blackbox::CalibrationOutcome::REJECTED);
        if (!apply) {
            const bool still = std::fmod(t, 9.0) < 3.0;
            s.errorKey = still ? "calibration_error_lack_of_translational_variance" : "calibration_error_worse_rms_than_last";
            s.errorText = still ? "lack of translational variance" : "worse RMS error than the last attempt";
        }
        s.currentRmsMm = static_cast<float>(4.0 + 1.5 * std::sin(t * 0.05));
        s.rmsMm = s.currentRmsMm + static_cast<float>(apply ? -0.6 : 1.5 + 2.0 * std::abs(noise(seed)));
        s.axisVariance = static_cast<float>(apply ? 0.004 + 0.002 * std::abs(noise(seed)) : 0.0004 + 0.003 * std::abs(noise(seed)));
        s.xCm = cx + static_cast<float>(noise(seed) * (apply ? 1.5 : 6.0));
        s.yCm = cy + static_cast<float>(noise(seed) * (apply ? 1.0 : 4.0));
        s.zCm = cz + static_cast<float>(noise(seed) * (apply ? 1.5 : 6.0));
        s.yawDeg = cyaw + static_cast<float>(noise(seed) * (apply ? 0.6 : 2.0));
        s.changeCm = static_cast<float>(std::sqrt((s.xCm - cx) * (s.xCm - cx) + (s.yCm - cy) * (s.yCm - cy) + (s.zCm - cz) * (s.zCm - cz)));
        s.changeDeg = std::abs(s.yawDeg - cyaw);
        if (apply) {
            cx = s.xCm;
            cy = s.yCm;
            cz = s.zCm;
            cyaw = s.yawDeg;
        }
        live->pushSolve(s);
    }
    LiveCalibrationSample c;
    c.t = t;
    c.xCm = cx;
    c.yCm = cy;
    c.zCm = cz;
    c.yawDeg = cyaw;
    live->pushCalibration(c);

    // a universe shift every 80 s, a marker every 50 s
    if (std::fmod(t, 80.0) < dt)
        live->pushEvent({ t, LiveEventKind::PLAYSPACE_SHIFT, 12.4f, "corrected" });
    if (std::fmod(t, 50.0) < dt)
        live->pushEvent({ t, LiveEventKind::MARKER, 0.0f, "hotkey" });
}

} // namespace spacecal::guard
