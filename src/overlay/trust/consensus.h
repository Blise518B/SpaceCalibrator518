#pragma once

// Cross-device consensus and per-device trust state machine (fork, docs/DESIGN.md section 6).
// Pure logic over one tick of device samples; the TrustManager feeds it from the overlay, the
// tests and the replay tool feed it from recordings.

#include "plausibility.h"
#include "trust_types.h"

#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

namespace spacecal::trust {

struct ConsensusParams {
    PlausibilityParams plausibility;
    double t_confirm = 0.5; // s in SUSPECT before UNTRUSTED
    double r_ok = 0.03; // m, healthy head-tracker residual
    double t_recover = 3.0; // s of r < r_ok before RECOVERING
    double d_agree = 0.05; // m, recovery solve must agree with the frozen calibration
    double a_agree_deg = 2.0; // deg
    double r_same_glitch = 0.05; // m, residual within this of r_step = still the same glitch (logging only)
    double jump_fit_residual = 0.03; // m, rigid fit residual for a playspace jump
    double r_high = 0.10; // m, trusted head tracker with residual above this ...
    double t_residual_high = 0.5; // ... for this long becomes SUSPECT (catches glitches without a visible step)
    int n_jurors = 2; // other OK devices needed to call a single-device glitch
    double t_device_lost = 2.0; // s not tracking before the device state resets
    double c_l_smoothing = 0.002; // per-tick blend factor for the estimated HMD-local offset (fixed mode) and the frozen world calibration (relative mode); ~25 s time constant at 20 Hz, never while the residual is above r_ok
    double back_on_path_fraction = 0.2; // SUSPECT returns to TRUSTED when within max(j_tol, fraction * step) of the pre-jump path
    double ref_jump_min = 0.10; // m: an HMD step counts as a reference discontinuity (re-center, tracking snap) only
                                // from this size on; smaller ones are frame-timing noise of its 72-90 Hz poses
                                // and would needlessly clear the solver's samples
    double t_shift_window = 0.15; // s: a step waits up to this long for the rest of the target universe before it is judged.
                                  // A playspace shift reaches the devices over a few frames (a device without a fresh pose
                                  // this frame steps in the next one), so the whole universe is judged together.
};

enum class Universe : uint8_t {
    REFERENCE = 0, // the HMD's tracking system
    TARGET = 1, // the lighthouse universe being calibrated
    OTHER = 2, // anything else: ignored
};

struct DeviceInput {
    uint8_t index = 255;
    Universe universe = Universe::OTHER;
    bool bodyWorn = true; // HMD, controllers, generic trackers; base stations are not
    PoseSample sample; // raw world pose in the device's own universe
};

struct CalibrationInput {
    bool valid = false; // a calibration is currently applied
    bool relativeMode = false;
    uint8_t referenceIndex = 255; // HMD
    uint8_t targetIndex = 255; // head tracker
    bool C_world_valid = false;
    Eigen::Isometry3d C_world = Eigen::Isometry3d::Identity(); // target universe -> reference universe (fixed mode)
    bool C_L_valid = false;
    Eigen::Isometry3d C_L = Eigen::Isometry3d::Identity(); // head tracker in the HMD frame (relative mode)
    // feedback from a recovery solve the manager ran since the last tick
    bool solveAttempted = false;
    bool solveValid = false;
    bool solveAgrees = false;
};

struct TickInput {
    double t = 0.0;
    std::vector<DeviceInput> devices;
    CalibrationInput calib;
};

struct Transition {
    uint8_t device = 255;
    State from = State::TRUSTED;
    State to = State::TRUSTED;
    Reason reason = Reason::NONE;
    double t = 0.0;
    float residual = 0.0f;
    float numbers[3] = { 0.0f, 0.0f, 0.0f }; // the values that triggered it (step, dt, jurors ...)
};

struct TickOutput {
    // Transition numbers for jumps: [displacement not explained by the reported velocity (m), dt of the
    // first step (s), jurors]
    std::vector<Transition> transitions;
    Event event = Event::NONE;
    Eigen::Isometry3d eventDelta = Eigen::Isometry3d::Identity(); // PLAYSPACE_JUMP: T_new = delta * T_old in target space
    double eventResidual = 0.0;
    bool holdTarget = false; // head tracker not TRUSTED, or its step is still being judged: driver holds, solver does not apply
    bool targetRecovering = false; // manager should run a solve and report agreement next tick
    bool targetResidualValid = false;
    double targetResidual = 0.0;
    Eigen::Vector3d targetResidualVec = Eigen::Vector3d::Zero(); // calibrated minus expected head tracker position (reference world)
    bool referenceDiscontinuity = false;
};

struct DeviceStatus {
    bool known = false;
    State state = State::TRUSTED;
    Reason reason = Reason::NONE;
    double since = 0.0;
    Flag lastFlag = Flag::NO_HISTORY;
    double lastResidual = 0.0;
    bool tracking = false;
    bool fresh = false; // the last tick brought a new pose
    double unexplained = 0.0; // its step not explained by the reported velocity [m]
    double jumpLimit = 0.0; // the guard's bound for that step [m]
};

class Consensus {
public:
    void setParams(const ConsensusParams& params) { m_params = params; }
    const ConsensusParams& params() const { return m_params; }

    void tick(const TickInput& in, TickOutput& out);

    // manual override (trigger hold / UI): every device TRUSTED, frozen calibration re-baselined
    void forceTrustAll(double t, std::vector<Transition>* transitions = nullptr);
    void reset();

    DeviceStatus status(uint8_t device) const;
    bool frozenCalibrationValid() const { return m_CfrozenValid; }
    const Eigen::Isometry3d& frozenCalibration() const { return m_Cfrozen; }
    bool localOffsetValid() const { return m_CLValid; }
    const Eigen::Isometry3d& localOffset() const { return m_CL; }
    // times the learned estimate was re-anchored to a calibration change (see updateCalibrationEstimates)
    uint64_t anchorCount() const { return m_anchors; }

private:
    struct DeviceState {
        bool known = false;
        Universe universe = Universe::OTHER;
        bool bodyWorn = true;
        DeviceHistory history;
        State state = State::TRUSTED;
        Reason reason = Reason::NONE;
        double since = 0.0;
        Flag lastFlag = Flag::NO_HISTORY;
        PlausibilityResult lastResult;
        PoseSample lastSample;
        double lastTrackingT = -1.0;
        double lastFreshT = -1.0; // last tick with a new, distinct pose
        // shift window (t_shift_window): every tracking target device sums what its velocity does not
        // explain from the window's first frame on; a device whose own step opened or joined the
        // window is `pending` and gets judged when the window closes
        bool winHasFrom = false;
        bool winFreshAfterOpen = false; // a fresh pose in a later frame than the one that opened the window
        Eigen::Vector3d winFrom = Eigen::Vector3d::Zero(); // where the device would have been at its first fresh pose in the window
        Eigen::Vector3d winU = Eigen::Vector3d::Zero(); // displacement not explained by its velocity since
        bool pending = false;
        PlausibilityResult pendingResult; // its first step in the window, for the SUSPECT bookkeeping
        float pendingNumbers[3] = { 0.0f, 0.0f, 0.0f };
        // jump bookkeeping
        Eigen::Vector3d preJumpP = Eigen::Vector3d::Zero();
        Eigen::Vector3d preJumpV = Eigen::Vector3d::Zero();
        double preJumpT = 0.0;
        Eigen::Vector3d stepDelta = Eigen::Vector3d::Zero();
        double lastJumpT = -1.0;
        double lastOddT = -1e9; // last tick the distance witness singled this device out
        // recovery bookkeeping
        double rStep = 0.0;
        double residualOkSince = -1.0;
        double residualHighSince = -1.0;
        double quietSince = -1.0;
        double lastResidual = 0.0;
        // distance witness
        bool calPValid = false;
        Eigen::Vector3d calP = Eigen::Vector3d::Zero(); // position in reference space last tick
        Eigen::Vector3d calV = Eigen::Vector3d::Zero(); // reported velocity in reference space last tick
    };

    DeviceState& slot(uint8_t index);
    void transition(DeviceState& d, uint8_t index, State to, Reason reason, double t, double residual, const float numbers[3], TickOutput& out);
    // judges the pending steps; returns true for a playspace jump. `judged` gets every device whose
    // step was judged individually this tick.
    bool resolveShift(const TickInput& in, const std::vector<uint8_t>& tracking, bool residualValid, double residual, std::vector<uint8_t>& judged, TickOutput& out);
    void judgeJump(const TickInput& in, uint8_t index, Reason reasonIfTrusted, const PlausibilityResult& step, const Eigen::Vector3d& stepDelta, const float numbers[3],
        bool residualValid, double residual, TickOutput& out);
    void clearPending();
    void updateCalibrationEstimates(const TickInput& in, const DeviceInput* ref, const DeviceInput* tgt, bool residualValid, double residual);
    bool computeResidual(const DeviceInput* ref, const DeviceInput* tgt, double& r, Eigen::Vector3d* vec = nullptr) const;
    int countJurors(const TickInput& in, uint8_t except) const;
    void distanceWitness(const TickInput& in, std::vector<uint8_t>& oddOnesOut);

private:
    ConsensusParams m_params;
    DeviceState m_devices[64];
    bool m_CfrozenValid = false;
    Eigen::Isometry3d m_Cfrozen = Eigen::Isometry3d::Identity();
    bool m_CLValid = false;
    Eigen::Isometry3d m_CL = Eigen::Isometry3d::Identity();
    double m_lastT = -1.0;
    double m_shiftStartT = -1.0; // frame that opened the current shift window, -1 = none
    double m_refJumpT = -1.0; // last REFERENCE_DISCONTINUITY
    Eigen::Isometry3d m_CworldAtRefJump = Eigen::Isometry3d::Identity();
    bool m_anchorPending = false; // the applied calibration moved the head tracker's expected pose; re-anchor once both are tracked
    uint64_t m_anchors = 0;
};

// Rigid fit b_i ~= delta * a_i (Kabsch). With two points only the translation is fitted.
// Returns the RMS residual, or a negative number when fewer than two points were given.
double fitRigidDelta(const std::vector<Eigen::Vector3d>& a, const std::vector<Eigen::Vector3d>& b, Eigen::Isometry3d& delta);

} // namespace spacecal::trust
