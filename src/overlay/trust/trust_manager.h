#pragma once

// Overlay glue for the trust layer (fork, docs/DESIGN.md sections 6 and 7): feeds the pure
// Consensus from the shared pose buffer and the active calibration, logs and records the
// transitions, tells the driver to hold, gates the solver, and answers the UI.

#include "consensus.h"
#include "trust_types.h"
#include "frame_corrector.h"
#include "head_mount.h"
#include "universe_watch.h"

#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace spacecal {
class TrackingSystemCalibration;
}

namespace spacecal::trust {

struct TrustDeviceView {
    uint8_t index = 255;
    std::string name; // "HMD", model + serial
    Universe universe = Universe::OTHER;
    DeviceStatus status;
};

class TrustManager {
public:
    void init();
    void reloadParams(); // after guard.json changed

    // once per overlay frame, after the poses were polled and before the calibrations tick
    void tick(double currentTime);

    // manual override: everything trusted, solver forced (trigger hold / UI button)
    void forceTrustAll(double currentTime, const char* who);
    // trigger hold, before forceTrustAll (which forgets the learned offset): the calibration is shifted,
    // its rotation kept, so that the head tracker sits at its learned place on the headset, from the
    // current poses. False when there is nothing to anchor to (no learned offset yet, relative mode, a
    // device not tracking) or the shift would be implausibly large (over 0.5 m).
    bool reanchorCalibration(TrackingSystemCalibration* calibration, double currentTime);

    // ---- solver gating (called by TrackingSystemCalibration) ---------------------------------
    // true while the head tracker of `calibration` is SUSPECT or UNTRUSTED: no samples, no solves
    bool shouldRejectSamples(const TrackingSystemCalibration& calibration) const;
    // true while the head tracker is RECOVERING: solve, but apply only when it agrees
    bool isRecovering(const TrackingSystemCalibration& calibration) const;
    // true while the driver must hold the relative correction for the target universe
    bool shouldHold(const TrackingSystemCalibration& calibration) const;
    bool recoverySolveAgrees(const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation, bool valuesAreLocal) const;
    [[nodiscard]] bool hasResidualContext() const { return m_consensus.frozenCalibrationValid() && m_consensus.localOffsetValid(); }
    void reportRecoverySolve(bool valid, bool agrees);
    // the solver applied a continuous solve on its own (observe-only: nobody asked for a recovery solve)
    void noteSolveApplied();
    // the head tracker's remembered place on the headset as a prior for a solve that passed every other
    // check (head_mount.h): DISAGREES = reject it
    SolveVerdict judgeSolveWithHeadMount(const TrackingSystemCalibration& calibration, const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation);

    // ---- UI ----------------------------------------------------------------------------------
    [[nodiscard]] bool enabled() const { return m_enabled; }
    [[nodiscard]] bool holdActive() const { return m_holdActive; }
    [[nodiscard]] bool residualValid() const { return m_residualValid; }
    [[nodiscard]] double residual() const { return m_residual; }
    [[nodiscard]] const std::string& lastEventText() const { return m_lastEventText; }
    [[nodiscard]] uint32_t transitionsThisSession() const { return m_transitionsThisSession; }
    [[nodiscard]] std::vector<TrustDeviceView> deviceViews() const;
    [[nodiscard]] const Consensus& consensus() const { return m_consensus; }
    [[nodiscard]] const UniverseWatch& universe() const { return m_universe; }
    [[nodiscard]] uint32_t universeShiftsThisSession() const { return m_perDeviceFrames ? m_frames.referenceMoves() : m_universe.shifts(); }
    [[nodiscard]] const FrameCorrector& frames() const { return m_frames; }
    [[nodiscard]] const HeadMount& headMount() const { return m_headMount; }
    [[nodiscard]] bool headMountEnabled() const { return m_hmEnabled; }
    [[nodiscard]] const HeadMountFix& headMountFix() const { return m_hmFix; }
    [[nodiscard]] bool headMountFixEnabled() const { return m_hmEnabled && m_hmFixEnabled; }
    // what the driver adds on top of the calibration for this device: SteamVR moved the base station
    // it is tracked from (frame_corrector.h). Identity unless per-device frames correct
    [[nodiscard]] Eigen::Isometry3d devicePin(uint8_t index) const;
    [[nodiscard]] static TrustManager* getInstance() { return s_instance; }

private:
    bool buildInput(double currentTime, TickInput& in, TrackingSystemCalibration** outCalibration);
    void handleTransitions(const TickOutput& out, TrackingSystemCalibration* calibration);
    void handleEvent(const TickOutput& out, TrackingSystemCalibration* calibration);
    void handleUniverseShift(const UniverseShift& shift, TrackingSystemCalibration* calibration);
    void handleFrameEvents(const std::vector<FrameEvent>& events, TrackingSystemCalibration* calibration);
    void refreshPins(TrackingSystemCalibration* calibration);
    void updateHeadMount(const TickInput& in, TrackingSystemCalibration* calibration, double currentTime);
    void runHeadMountFix(TrackingSystemCalibration* calibration, double currentTime, const PoseSample& headset, const PoseSample& tracker,
        const Eigen::Isometry3d& H, const Eigen::Isometry3d& T, const Eigen::Isometry3d& C, bool calibrationValid);
    [[nodiscard]] Eigen::Isometry3d trustGlobal() const { return m_perDeviceFrames ? m_frames.global() : m_universe.global(); }
    void feedLive(const TickInput& in, const TickOutput& out, const TrackingSystemCalibration* calibration);
    const std::string& liveLabel(uint8_t index, bool isTarget);

private:
    Consensus m_consensus;
    UniverseWatch m_universe; // SteamVR re-solving its base stations: the trust layer works in a frame without the shift
    bool m_fixUniverseShifts = true;
    FrameCorrector m_frames; // the same per device and base station (trust.per_device_frames)
    bool m_perDeviceFrames = true;
    bool m_framesCorrecting = false;
    std::array<Eigen::Isometry3d, 64> m_sentPins; // what the driver has, see refreshPins()
    // the head tracker's remembered place on the headset (head_mount.h, head_mount.json)
    HeadMount m_headMount;
    bool m_hmEnabled = true;
    bool m_hmStartup = true;
    bool m_hmPrior = true;
    bool m_hmLoaded = false;
    std::string m_hmTarget;
    std::string m_hmReference;
    bool m_hmDirty = false;
    double m_hmLastLearnT = -1.0;
    double m_hmLastSaveT = -1.0;
    bool m_hmStartupDone = false;
    double m_hmStillSince = -1.0;
    bool m_hmPosesValid = false;
    Eigen::Isometry3d m_hmH = Eigen::Isometry3d::Identity(); // headset, last trust tick
    Eigen::Isometry3d m_hmT = Eigen::Isometry3d::Identity(); // head tracker in the trust layer's frame, last trust tick
    Eigen::Isometry3d m_hmC = Eigen::Isometry3d::Identity(); // the calibration in use, trust layer's frame, last trust tick
    // the automatic position fix (head_mount.h HeadMountFix)
    HeadMountFix m_hmFix;
    bool m_hmFixEnabled = true;
    bool m_fixHaveLast = false;
    Eigen::Isometry3d m_fixLastC = Eigen::Isometry3d::Identity(); // the calibration the last gap was measured with
    Eigen::Quaterniond m_fixAppliedRot = Eigen::Quaterniond::Identity(); // the applied calibration as the fix left it
    Eigen::Vector3d m_fixAppliedTrans = Eigen::Vector3d::Zero();
    Eigen::Isometry3d m_fixPrevH = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d m_fixPrevT = Eigen::Isometry3d::Identity();
    double m_fixPrevTime = -1.0;
    bool m_fixOpen = false; // a fix is running: its black-box record is written when it ends
    Eigen::Quaterniond m_fixStartRot = Eigen::Quaterniond::Identity();
    Eigen::Vector3d m_fixStartTrans = Eigen::Vector3d::Zero();
    bool m_enabled = true;
    bool m_holdEnabled = true;
    bool m_autoMarker = true;
    bool m_applyPlayspaceJumpFix = true;
    double m_tickInterval = 0.05;
    double m_lastTick = -1.0;
    double m_currentTime = 0.0;
    bool m_holdActive = false;
    bool m_residualValid = false;
    double m_residual = 0.0;
    std::string m_lastEventText;
    uint32_t m_transitionsThisSession = 0;
    // recovery solve feedback for the next tick
    bool m_solveAttempted = false;
    bool m_solveValid = false;
    bool m_solveAgrees = false;
    uint8_t m_lastTargetIndex = 255;
    std::string m_lastTargetSerial;
    size_t m_guardedCalibration = 0; // index into CalibrationManager's calibrations
    std::string m_targetSystem;
    std::string m_referenceSystem;
    std::array<std::string, 64> m_liveLabels {}; // Live tab device names, rebuilt when the serial changes
    std::array<std::string, 64> m_liveLabelKeys {};
    static TrustManager* s_instance;
};

} // namespace spacecal::trust
