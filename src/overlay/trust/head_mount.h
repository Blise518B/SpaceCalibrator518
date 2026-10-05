#pragma once

// The head tracker's place on the headset, remembered across sessions (fork, docs/DESIGN.md "Head mount").
//
// X = H^-1 * C * T: the head tracker's pose in the headset's frame (H the headset in the reference
// world, C the applied calibration, T the head tracker's raw lighthouse pose). The tracker is strapped
// to the headset, so X stays put from one session to the next while the calibration does not: SteamVR
// re-solves its base stations at every start (2026-10-04 20:50: the head tracker's station moved 21 cm
// in the first two minutes) and the Quest may re-center. X is learned only while the head tracker is
// trusted, sits within learn_max_residual of where the calibration expects it and the calibration fits
// the solver's samples well; it follows slowly (learn_time_constant), so a short bad stretch hardly
// moves it. It is used
//   * at startup: once headset and head tracker track and hold still, the stored calibration is shifted
//     so that the head tracker lands at X
//   * as a prior for the solver: a solve that puts the head tracker more than solve_max_pos away from X
//     is rejected, unless it puts it closer than the calibration in use (after SteamVR moved its base
//     stations, the first solve often gets only part of the way back), and until solve_disagree_n
//     solves in a row agree with each other on another place (the tracker was re-mounted): then that
//     solve is accepted and X is learned anew.
//   * by the automatic position fix (HeadMountFix below): when the head tracker sits steadily away from
//     X while headset and tracker move slowly, the calibration's translation slides back at 1.5 cm/s.
// Only X's position is used. Six recorded sessions (2026-09-30 to 2026-10-04) put the tracker at right
// -1.0 to -1.5 / up 10.4 to 10.8 / back -2.9 to -3.4 cm, but its rotation against the headset differed
// by 2.5 to 7.3 deg from session to session (1 to 1.7 deg within one): the strap sits a little
// differently every time. use_rotation turns the rotation on for rigid mounts.
//
// Pure logic (Eigen only); the TrustManager feeds it and keeps it in head_mount.json, the tests drive it
// directly.

#include <Eigen/Geometry>
#include <deque>
#include <string>

namespace spacecal::trust {

struct HeadMountParams {
    double learn_max_residual = 0.02; // m: the head tracker within this of where the calibration expects it
    double learn_max_rms = 0.01; // m: and the calibration's RMS on the solver's samples below this
    double learn_time_constant = 300.0; // s: once learned, the estimate follows this slowly
    double confident_after = 120.0; // s of learning before the estimate is used
    bool use_rotation = false; // also place and judge by X's rotation (only for a tracker fixed rigidly to the headset)
    double rotation_steady_deg = 2.0; // ... and then only while samples scatter less than this around it
    double startup_still_s = 1.5; // s both tracking and still before the startup placement
    double startup_still_speed = 0.10; // m/s: headset and head tracker slower than this count as still
    double startup_min_change = 0.015; // m at the head (and 0.5 deg): below this the stored calibration is kept
    double startup_max_change = 0.8; // m at the head: beyond this (or startup_max_deg) something else is wrong, keep the stored one
    double startup_max_deg = 25.0;
    double solve_max_pos = 0.05; // m: a solve that puts the head tracker further from X than this is rejected ...
    double solve_max_deg = 3.0; // ... or turns it more than this against the headset
    int solve_disagree_n = 5; // ... unless this many solves in a row agree with each other on another place
};

// what head_mount.json keeps
struct HeadMountState {
    std::string target_serial; // the head tracker
    std::string reference_serial; // the headset
    double x = 0.0, y = 0.0, z = 0.0; // m, in the headset's frame
    double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
    double learned_seconds = 0.0;
    double rotation_scatter_deg = 180.0;
};

enum class SolveVerdict : uint8_t {
    NO_MODEL = 0, // not confident yet: the solver decides alone
    AGREES = 1,
    DISAGREES = 2, // rejected
    REMOUNTED = 3, // solve_disagree_n solves in a row put the tracker at the same other place: accepted, X re-learned
    IMPROVES = 4, // further than solve_max_pos from X, but closer than the calibration in use: accepted
};

class HeadMount {
public:
    void setParams(const HeadMountParams& p) { m_params = p; }
    [[nodiscard]] const HeadMountParams& params() const { return m_params; }

    void reset();
    // takes a stored state when it belongs to this tracker on this headset; false otherwise (then reset)
    bool restore(const HeadMountState& s, const std::string& target, const std::string& reference);
    [[nodiscard]] HeadMountState state(const std::string& target, const std::string& reference) const;

    // one sample of X while the head tracker is healthy (the caller checks the conditions above)
    void learn(const Eigen::Isometry3d& X, double dt);

    [[nodiscard]] bool confident() const { return m_valid && m_learned >= m_params.confident_after; }
    [[nodiscard]] bool rotationSteady() const { return confident() && m_rotScatter <= m_params.rotation_steady_deg; }
    [[nodiscard]] bool usesRotation() const { return m_params.use_rotation && rotationSteady(); }
    [[nodiscard]] const Eigen::Isometry3d& pose() const { return m_X; }
    [[nodiscard]] double learnedSeconds() const { return m_learned; }
    [[nodiscard]] double rotationScatterDeg() const { return m_rotScatter; }

    // the calibration that puts the head tracker at its remembered place: `current` shifted so that the
    // head tracker lands there, or H * X * T^-1 when the rotation is used (usesRotation())
    [[nodiscard]] Eigen::Isometry3d placeCalibration(const Eigen::Isometry3d& current, const Eigen::Isometry3d& H, const Eigen::Isometry3d& T) const;
    // how far a calibration puts the head tracker from its remembered place: m at the head, deg against the headset
    void disagreement(const Eigen::Isometry3d& C, const Eigen::Isometry3d& H, const Eigen::Isometry3d& T, double& pos, double& deg) const;
    // the solver prior (see above); `current` is the calibration in use (nullptr: no improvement check)
    SolveVerdict judgeSolve(const Eigen::Isometry3d& C, const Eigen::Isometry3d& H, const Eigen::Isometry3d& T, double* pos = nullptr, double* deg = nullptr,
        const Eigen::Isometry3d* current = nullptr);

private:
    HeadMountParams m_params;
    bool m_valid = false;
    Eigen::Isometry3d m_X = Eigen::Isometry3d::Identity();
    double m_learned = 0.0;
    double m_rotScatter = 180.0; // deg, smoothed angle of the samples against the estimate
    int m_disagree = 0;
    Eigen::Isometry3d m_disagreeX = Eigen::Isometry3d::Identity();
};

// The automatic position fix (docs/DESIGN.md "Head mount fix"). The gap g = H * X - C * T (reference
// world, position only) is the calibration's error at the head, as long as the head tracker itself is
// right. When it has been steady above `on` for `hold` seconds while headset and head tracker move
// slowly, the calibration's translation slides by it at `rate` until it is below `off`. A tracker that
// glitches must not take the body along (simulated over the recordings of 2026-09-29 to 2026-10-04: a
// plain version moved the calibration 192 cm in total while the head tracker had just jumped on its own,
// 2026-10-04 08:26 up to 9 cm at a time while it slid back), hence:
//   * after the gap steps by `jump` within one tick without the calibration changing (the head tracker or
//     the headset jumped), nothing is fixed for `quarantine` seconds
//   * the gap must not be trending (a tracker sliding back after a glitch moves 0.4 to 1 cm/s)
//   * when the gap without the fix falls below `snap`, the cause went away and the fix is undone at once
//   * gaps beyond `max_gap` are left to the solver
// The trust state is not used: on the night of 2026-10-03 the guard had the head tracker as untrusted
// for half of the time it sat off, while the calibration was what was wrong. A calibration that someone
// else applies (a solve, the trigger hold, a frame correction) replaces the fix. Pure logic; the
// TrustManager feeds it every trust tick and applies the shifts.
struct HeadMountFixParams {
    double on = 0.025; // m: start when the steady gap is above this
    double off = 0.005; // m: stop when it is below this
    double hold = 5.0; // s of steady gap while moving slowly before a fix starts
    double steady = 0.01; // m: rms scatter of the gap over `hold`
    double trend = 0.0025; // m/s: the gap's slope over `hold` must stay below this
    double rate = 0.015; // m/s: the calibration slides at this speed
    double max_gap = 0.5; // m: larger gaps are left to the solver
    double max_speed = 0.20; // m/s: headset and head tracker slower than this count as moving slowly
    double max_turn_dps = 30.0; // deg/s: the headset turning slower than this
    double jump = 0.03; // m: a step of the gap within one tick that no calibration change explains ...
    double jump_speed = 0.5; // ... while the headset moves slower than this (m/s) ...
    double jump_turn_dps = 90.0; // ... and turns slower than this (fast movement alone moves the gap by the systems' latency)
    double jump_any = 0.08; // m: a step this large counts at any speed
    double quarantine = 20.0; // s after such a jump before anything is fixed
    double snap = 0.01; // m: when the gap without the fix is below this ...
    double snap_min = 0.02; // m: ... a fix of at least this much is undone at once
    double max_tick = 0.15; // s: a longer pause between ticks starts the jump check afresh
};

class HeadMountFix {
public:
    struct Input {
        double t = 0.0;
        Eigen::Vector3d gap = Eigen::Vector3d::Zero(); // H * X - C * T with the calibration in use (m)
        Eigen::Vector3d gapBefore = Eigen::Vector3d::Zero(); // the same with the calibration of the previous tick (only movement changes it)
        bool calibrationChanged = false; // someone else changed the calibration since the last tick
        double headsetSpeed = 0.0; // m/s
        double headsetTurn = 0.0; // deg/s
        double trackerSpeed = 0.0; // m/s
        bool allowed = true; // enabled, X confident, a calibration applied, nothing holding it
    };
    struct Output {
        Eigen::Vector3d shift = Eigen::Vector3d::Zero(); // add this to the calibration's translation (reference world)
        bool started = false; // size: the gap
        bool finished = false; // size: what this fix moved
        bool interrupted = false; // a calibration from elsewhere replaced a running fix; size: what it had moved
        bool snapped = false; // size: what was undone
        bool jumped = false; // size: the step
        double size = 0.0;
        Eigen::Vector3d gap = Eigen::Vector3d::Zero(); // started: the gap that is being fixed
    };

    void setParams(const HeadMountFixParams& p) { m_params = p; }
    [[nodiscard]] const HeadMountFixParams& params() const { return m_params; }
    void reset();
    Output update(const Input& in);

    [[nodiscard]] bool active() const { return m_active; }
    [[nodiscard]] double remaining() const { return m_remaining; } // m, while active
    [[nodiscard]] const Eigen::Vector3d& offset() const { return m_offset; } // what the fix has shifted since the calibration last changed elsewhere
    [[nodiscard]] bool quarantined(double t) const { return t < m_quarantineUntil; }

private:
    struct Entry {
        double t;
        Eigen::Vector3d raw; // the gap without the fix
        double dt;
    };
    HeadMountFixParams m_params;
    std::deque<Entry> m_window;
    Eigen::Vector3d m_offset = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_prevGap = Eigen::Vector3d::Zero();
    double m_lastT = -1.0;
    double m_quarantineUntil = -1e18;
    bool m_active = false;
    double m_moved = 0.0;
    double m_remaining = 0.0;
};

} // namespace spacecal::trust
