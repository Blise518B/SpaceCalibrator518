#pragma once

// SteamVR moving base stations, per device (fork, docs/DESIGN.md "Universe shifts").
//
// Every lighthouse pose arrives as raw = W * P: W the WorldFromDriver, the frame of the base
// station the device is tracked from (SteamVR's map of that station), P the pose relative to that
// station. When SteamVR moves a station on its map (vrserver.txt: "Moving base X ... because of
// relationship with Y"; a universe tilt turns all of them), W changes and P does not: the device
// jumps in the raw world while nothing physically moved. A device's own movement never changes W.
// 2026-10-02: every one of 163 "Moving base" lines matched such a change; body devices got 137
// frame changes that moved them up to 24 cm, and trackers on different stations get different
// changes (one station moved 21 cm on the map under two trackers while the rest stayed on another).
//
// Per device a correction Q (raw -> stable), shown = Q * raw:
//  * W changes within the same station's frame family (a refinement): Q <- Q * W_old * W_new^-1.
//    Exact: the device does not move.
//  * W switches to another station's frame (metres away): SteamVR re-expresses P in the new frame,
//    and its map of the two stations rarely agrees exactly (a few cm; 22 cm during a move). Q
//    absorbs the discontinuity between the last sample before and the first after, up to
//    switch_pos / switch_deg. Never for the head tracker, whose jumps the trust layer must see.
//  * Q slides back to SteamVR's map at slide_mps / slide_dps, measured at the device, so
//    corrections never pile up. The device's own motion always passes through unchanged.
//  * Back from a loss of tracking (> lost_after): Q is SteamVR's map again.
// SteamVR's map is pinned at the reference family, the station frame the head tracker is tracked
// from: G = S_r * W_ref^-1, W_ref the head tracker's own version of that frame (not the base
// station's record, which runs ahead and may never be adopted by any device, and not another
// device's version: the solver calibrates against the head tracker's raw poses, so its pin must
// stay identity). G is the calibration fix (C_new = C_old * delta^-1 when the head tracker's
// version moves by delta), and the driver adds pin(d) = G^-1 * Q_d on top of the calibration for
// each device; a device on another version of the head tracker's station is held in its version.
//
// Pure logic (Eigen only): the overlay feeds it from the driver's pose buffer, the replay tool and
// the tests from recordings. Validated on a four-hour recording of 2026-10-02: frame
// change steps above 2 cm went from 129 to 0, ordinary steps are unchanged.

#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

namespace spacecal::trust {

struct FrameParams {
    // a WorldFromDriver within this of the previous one is the same station's frame. Stations differ by
    // metres and tens of degrees (closest pair seen: 1.9 m, 46 deg); SteamVR's moves and
    // universe tilts stay below 0.3 m and 4.3 deg (2026-10-02)
    double family_pos = 1.0; // m, at the device
    double family_deg = 10.0;
    double same_pos = 0.0005; // m: below this (and same_deg) the frame did not change (float noise)
    double same_deg = 0.005;
    double switch_pos = 0.25; // m at the device: switch discontinuities up to this are absorbed
    double switch_deg = 10.0;
    double sample_gap = 0.05; // s: a switch is absorbed only with tracking samples this close around it
    double lost_after = 0.5; // s without tracking: back on SteamVR's map when it tracks again
    double slide_mps = 0.015; // m/s at the device: corrections slide back to SteamVR's map (0 = never)
    double slide_dps = 0.5; // deg/s
    bool absorb_switches = true;
};

struct FrameSample {
    uint8_t index = 255;
    bool tracking = false;
    bool reference = false; // the calibration's target device (the head tracker)
    Eigen::Isometry3d wfd = Eigen::Isometry3d::Identity(); // driver -> raw world, as reported with the pose
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity(); // the pose in driver space
};

enum class FrameEventKind : uint8_t {
    REFERENCE_MOVED, // SteamVR moved the head tracker's station: the calibration fix
    DEVICE_MOVED, // SteamVR moved the station under a device on another station: held, sliding back
    SWITCH_ABSORBED, // a device changed stations and the two did not agree: held, sliding back
    SWITCH_SHOWN, // a station change too big to absorb (or of the head tracker): shown as it is
    REFERENCE_SWITCHED, // the head tracker is tracked from another station now
};

struct FrameEvent {
    FrameEventKind kind = FrameEventKind::REFERENCE_MOVED;
    double t = 0.0;
    uint8_t index = 255; // the device that showed it
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity(); // REFERENCE_MOVED: raw_new = delta * raw_old on that family
    double move = 0.0; // m at the device
    double angleDeg = 0.0;
    bool corrected = false;
};

const char* frameEventName(FrameEventKind k);

class FrameCorrector {
public:
    void setParams(const FrameParams& p) { m_params = p; }
    [[nodiscard]] const FrameParams& params() const { return m_params; }
    void reset();

    // One tick of the target universe's lighthouse body devices (connected ones, tracking or not).
    // correct = false: report only; G stays where it is and every device gets G.
    void tick(double t, const std::vector<FrameSample>& devices, bool correct, std::vector<FrameEvent>* events);

    [[nodiscard]] Eigen::Isometry3d correction(uint8_t index) const; // stable = correction * raw
    [[nodiscard]] const Eigen::Isometry3d& global() const { return m_G; } // C_trust = C_applied * global^-1
    [[nodiscard]] Eigen::Isometry3d pin(uint8_t index) const; // G^-1 * Q: the driver applies C_applied * pin
    [[nodiscard]] bool known() const { return m_haveRef; }
    [[nodiscard]] uint32_t referenceMoves() const { return m_referenceMoves; }
    [[nodiscard]] uint32_t deviceMoves() const { return m_deviceMoves; }
    [[nodiscard]] uint32_t switchesAbsorbed() const { return m_switchesAbsorbed; }
    [[nodiscard]] uint32_t switchesShown() const { return m_switchesShown; }

private:
    struct Device {
        bool seen = false;
        bool hasQ = false;
        Eigen::Isometry3d Q = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d W = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        double lastTracking = -1e9;
        double lastSampleT = -1e9;
        bool lastSampleTracking = false;
    };

    [[nodiscard]] bool same(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) const;
    [[nodiscard]] bool sameFamily(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) const;
    [[nodiscard]] Eigen::Isometry3d target(const Device& d) const;

    FrameParams m_params;
    Device m_dev[64];
    bool m_haveRef = false;
    bool m_correcting = false;
    Eigen::Isometry3d m_Sr = Eigen::Isometry3d::Identity(); // the reference family's frame in the stable world
    Eigen::Isometry3d m_Wref = Eigen::Isometry3d::Identity(); // the head tracker's version of it
    Eigen::Isometry3d m_G = Eigen::Isometry3d::Identity();
    double m_lastT = -1.0;
    uint32_t m_referenceMoves = 0;
    uint32_t m_deviceMoves = 0;
    uint32_t m_switchesAbsorbed = 0;
    uint32_t m_switchesShown = 0;
};

} // namespace spacecal::trust
