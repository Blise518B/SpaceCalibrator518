#pragma once

// Per-device plausibility (fork, docs/DESIGN.md section 5). Pure functions over pose samples,
// no OpenVR or logging dependency, so the same code runs in the overlay, in tests and in the
// replay tool.

#include <Eigen/Geometry>
#include <cstdint>

namespace spacecal::trust {

constexpr double k_PI = 3.14159265358979323846; // MSVC has no M_PI without _USE_MATH_DEFINES

// A device pose in the world frame of its own tracking universe (WorldFromDriver applied).
struct PoseSample {
    double t = 0.0; // seconds, any monotonic clock shared by all devices of a tick
    bool tracking = false; // connected && poseIsValid && result == Running_OK
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d v = Eigen::Vector3d::Zero(); // velocity reported by the driver [m/s]
    Eigen::Vector3d w = Eigen::Vector3d::Zero(); // angular velocity reported by the driver [rad/s]; only its magnitude is used (drivers differ in the frame)
};

struct PlausibilityParams {
    // Jumps are judged by the part of a step the device's own reported velocity does not explain
    // (2026-09-30 session: hands reached 7.5 m/s and wrists ~1,000 deg/s in real motion, while
    // glitches disagreed with the reported velocity by 5 to 137 m/s). A device that reports no
    // velocity at all (exactly zero) falls back to the fixed bounds v_max / rot_max_dps.
    double v_unexplained = 1.0; // m/s, allowance for motion the reported velocity does not explain
    double rot_unexplained_dps = 200.0; // deg/s, the same for rotation
    double v_body_max = 10.0; // m/s, reported speeds above this are not believed (capped)
    double w_body_max_dps = 3000.0; // deg/s, the same for angular speed
    double v_max = 4.0; // m/s, fallback position bound for devices without reported velocity
    double j_tol = 0.03; // m, jump tolerance
    double v_err_max = 1.0; // m/s, reported vs observed velocity (NOISY only)
    double a_max = 60.0; // m/s^2 (NOISY only)
    double rot_max_dps = 400.0; // deg/s, fallback rotation bound for devices without reported angular velocity
    double rot_tol_deg = 10.0; // deg
    double t_stale = 0.25; // s without a new pose
    double dt_max = 0.5; // s; a gap longer than this resets the history instead of flagging
};

enum class Flag : uint8_t {
    NO_HISTORY = 0, // first sample after (re)start or a long gap; nothing to compare with
    OK = 1,
    JUMP = 2, // position or rotation step
    NOISY = 3, // velocity or acceleration inconsistent, no step
    STALE = 4, // identical pose for longer than t_stale
    NOT_TRACKING = 5,
};

struct PlausibilityResult {
    Flag flag = Flag::NO_HISTORY;
    bool fresh = false; // this sample is a new, distinct pose (the history advanced)
    double dt = 0.0;
    double d = 0.0; // position step [m]
    double unexplained = 0.0; // |step - reported velocity * dt| [m] (= d without reported velocity)
    double jumpLimit = 0.0; // the bound `unexplained` was judged against [m] (Live tab)
    Eigen::Vector3d unexplainedVec = Eigen::Vector3d::Zero();
    double vErr = 0.0; // |observed - reported velocity| [m/s]
    double accel = 0.0; // |dv/dt| observed [m/s^2]
    double rotDeg = 0.0; // rotation step [deg]
    double rotUnexplained = 0.0; // rotation step minus reported angular speed * dt [deg]
    Eigen::Vector3d delta = Eigen::Vector3d::Zero(); // p(t) - p(t-1)
    Eigen::Vector3d prevP = Eigen::Vector3d::Zero(); // p(t-1), for pre-jump bookkeeping
    Eigen::Vector3d prevV = Eigen::Vector3d::Zero(); // observed velocity before this sample
    Eigen::Vector3d explainedP = Eigen::Vector3d::Zero(); // prevP + reported velocity * dt: where the device should be without a jump
    double prevT = 0.0;
};

// Per-device memory between ticks.
struct DeviceHistory {
    bool hasPrev = false;
    PoseSample prev;
    bool hasPrevVel = false;
    Eigen::Vector3d prevObservedV = Eigen::Vector3d::Zero();
    double staleSince = -1.0;
    void reset() { *this = DeviceHistory {}; }
};

// Evaluates one new sample against the device's history and updates the history.
PlausibilityResult evaluate(DeviceHistory& history, const PoseSample& sample, const PlausibilityParams& params);

inline const char* flagName(Flag f)
{
    switch (f) {
    case Flag::NO_HISTORY: return "no_history";
    case Flag::OK: return "ok";
    case Flag::JUMP: return "jump";
    case Flag::NOISY: return "noisy";
    case Flag::STALE: return "stale";
    case Flag::NOT_TRACKING: return "not_tracking";
    default: return "?";
    }
}

} // namespace spacecal::trust
