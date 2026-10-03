#include "trust_manager.h"
#include "trust_params.h"

#include "calibration.h"
#include "guard/calibration_record.h"
#include "guard/event_marker.h"
#include "guard/guard_config.h"
#include "guard/live_stats.h"
#include "log.h"
#include "recorder/recorder_host.h"
#include "vr_core.h"

#include <cmath>
#include <fmt/format.h>

namespace spacecal::trust {

TrustManager* TrustManager::s_instance = nullptr;

namespace {
    PoseSample sampleFromDriverPose(const vr::DriverPose_t& pose, bool connected, double t)
    {
        PoseSample s;
        s.t = t;
        s.tracking = connected && pose.deviceIsConnected && pose.poseIsValid && pose.result == vr::TrackingResult_Running_OK;
        const Eigen::Quaterniond wfdRot(pose.qWorldFromDriverRotation.w, pose.qWorldFromDriverRotation.x, pose.qWorldFromDriverRotation.y, pose.qWorldFromDriverRotation.z);
        const Eigen::Vector3d wfdTrans(pose.vecWorldFromDriverTranslation[0], pose.vecWorldFromDriverTranslation[1], pose.vecWorldFromDriverTranslation[2]);
        const Eigen::Vector3d p(pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]);
        const Eigen::Quaterniond q(pose.qRotation.w, pose.qRotation.x, pose.qRotation.y, pose.qRotation.z);
        const Eigen::Vector3d v(pose.vecVelocity[0], pose.vecVelocity[1], pose.vecVelocity[2]);
        const Eigen::Vector3d w(pose.vecAngularVelocity[0], pose.vecAngularVelocity[1], pose.vecAngularVelocity[2]);
        s.p = wfdTrans + wfdRot * p;
        s.q = (wfdRot * q).normalized();
        s.v = wfdRot * v;
        s.w = w;
        return s;
    }

    bool isBodyWorn(vr::TrackedDeviceClass c)
    {
        return c == vr::TrackedDeviceClass_HMD || c == vr::TrackedDeviceClass_Controller || c == vr::TrackedDeviceClass_GenericTracker;
    }

    Eigen::Isometry3d isoFromCalibration(const Eigen::Quaterniond& rot, const Eigen::Vector3d& trans)
    {
        Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
        iso.linear() = rot.normalized().toRotationMatrix();
        iso.translation() = trans;
        return iso;
    }

}

void TrustManager::init()
{
    s_instance = this;
    m_consensus.reset();
    m_frames.reset();
    for (Eigen::Isometry3d& p : m_sentPins)
        p = Eigen::Isometry3d::Identity();
    reloadParams();
}

void TrustManager::reloadParams()
{
    const guard::GuardConfig& cfg = guard::GuardConfigManager::getInstance()->get();
    m_enabled = cfg.trust.enabled;
    m_holdEnabled = cfg.trust.hold_enabled;
    m_autoMarker = cfg.trust.auto_marker;
    m_applyPlayspaceJumpFix = cfg.trust.apply_playspace_jump_fix;
    m_fixUniverseShifts = cfg.trust.fix_universe_shifts;
    m_perDeviceFrames = cfg.trust.per_device_frames;
    FrameParams fp = m_frames.params();
    fp.slide_mps = cfg.trust.frames_slide_cm_s / 100.0;
    fp.slide_dps = cfg.trust.frames_slide_deg_s;
    fp.absorb_switches = cfg.trust.frames_absorb_switches;
    m_frames.setParams(fp);
    m_tickInterval = cfg.trust.tick_hz > 0 ? 1.0 / cfg.trust.tick_hz : 0.0; // 0 = every frame
    m_consensus.setParams(paramsFromConfig(cfg.trust));
}

bool TrustManager::buildInput(double currentTime, TickInput& in, TrackingSystemCalibration** outCalibration)
{
    auto* manager = CalibrationManager::getInstance();
    auto* vrState = VRState::getInstance();
    if (!manager || !vrState)
        return false;

    // the calibration we guard: the first active continuous one, else the first one
    TrackingSystemCalibration* calibration = nullptr;
    for (size_t i = 0; i < manager->getCalibrationCount(); i++) {
        TrackingSystemCalibration& c = manager->getCalibration(i);
        if (c.isContinuousCalibration() && c.devicesAreValid()) {
            calibration = &c;
            break;
        }
    }
    if (!calibration && manager->getCalibrationCount() > 0)
        calibration = &manager->getCalibration(0);
    if (!calibration)
        return false;
    *outCalibration = calibration;
    for (size_t i = 0; i < manager->getCalibrationCount(); i++)
        if (&manager->getCalibration(i) == calibration)
            m_guardedCalibration = i;

    m_referenceSystem = calibration->referenceDevice.trackingSystem;
    m_targetSystem = calibration->targetDevice.trackingSystem;

    in.t = currentTime;
    in.devices.clear();
    for (uint8_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        const VRDevice_t dev = vrState->getVrDevice(i);
        if (!dev.bIsConnected || !isBodyWorn(dev.eDeviceClass))
            continue;
        DeviceInput d;
        d.index = i;
        d.bodyWorn = true;
        if (dev.szTrackingSystemId == m_referenceSystem)
            d.universe = Universe::REFERENCE;
        else if (dev.szTrackingSystemId == m_targetSystem)
            d.universe = Universe::TARGET;
        else
            d.universe = Universe::OTHER;
        d.sample = sampleFromDriverPose(manager->m_poses[i], true, currentTime);
        in.devices.push_back(d);
    }

    const bool relativeMode = calibration->isContinuousCalibration() && calibration->isRelativeCalibration;
    // relative mode follows the head tracker every frame anyway; upstream's own fix would correct twice
    const bool correctShift = m_fixUniverseShifts && !relativeMode && calibration->isActive && calibration->isValidCalibration() && !calibration->autoFixPlayspaceJumps;
    if (m_perDeviceFrames) {
        // SteamVR moving base stations, per device (frame_corrector.h): a device keeps its place when
        // SteamVR moves the station it is tracked from; the head tracker's station is the calibration fix
        std::vector<FrameSample> frames;
        for (const DeviceInput& d : in.devices) {
            if (d.universe != Universe::TARGET)
                continue;
            const vr::DriverPose_t& pose = manager->m_poses[d.index];
            const Eigen::Quaterniond wq(pose.qWorldFromDriverRotation.w, pose.qWorldFromDriverRotation.x, pose.qWorldFromDriverRotation.y, pose.qWorldFromDriverRotation.z);
            if (std::abs(wq.norm() - 1.0) > 0.01)
                continue; // no transform reported yet
            const Eigen::Quaterniond pq(pose.qRotation.w, pose.qRotation.x, pose.qRotation.y, pose.qRotation.z);
            if (std::abs(pq.norm() - 1.0) > 0.01)
                continue;
            FrameSample f;
            f.index = d.index;
            f.tracking = d.sample.tracking;
            f.reference = calibration->targetDevice.deviceId == d.index;
            f.wfd = isoFromCalibration(wq, Eigen::Vector3d(pose.vecWorldFromDriverTranslation[0], pose.vecWorldFromDriverTranslation[1], pose.vecWorldFromDriverTranslation[2]));
            f.pose = isoFromCalibration(pq, Eigen::Vector3d(pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]));
            frames.push_back(f);
        }
        std::vector<FrameEvent> events;
        m_frames.tick(currentTime, frames, correctShift, &events);
        m_framesCorrecting = correctShift;
        handleFrameEvents(events, calibration);
    }
    // SteamVR universe shifts (universe_watch.h): the target universe's shared WorldFromDriver
    // changes when SteamVR re-solves its base stations. Correct the calibration by exactly that
    // change and hand the trust layer the poses in a frame where the shift never happened.
    std::vector<UniverseDevice> universe;
    for (const DeviceInput& d : in.devices) {
        if (m_perDeviceFrames)
            break;
        if (d.universe != Universe::TARGET)
            continue;
        const vr::DriverPose_t& pose = manager->m_poses[d.index];
        const Eigen::Quaterniond wq(pose.qWorldFromDriverRotation.w, pose.qWorldFromDriverRotation.x, pose.qWorldFromDriverRotation.y, pose.qWorldFromDriverRotation.z);
        if (std::abs(wq.norm() - 1.0) > 0.01)
            continue; // no transform reported yet
        UniverseDevice u;
        u.index = d.index;
        u.wfd = isoFromCalibration(wq, Eigen::Vector3d(pose.vecWorldFromDriverTranslation[0], pose.vecWorldFromDriverTranslation[1], pose.vecWorldFromDriverTranslation[2]));
        u.driverPos = Eigen::Vector3d(pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]);
        universe.push_back(u);
    }
    UniverseShift shift;
    if (!m_perDeviceFrames && m_universe.tick(currentTime, universe, correctShift, &shift))
        handleUniverseShift(shift, calibration);
    for (DeviceInput& d : in.devices) {
        if (d.universe != Universe::TARGET)
            continue;
        const Eigen::Isometry3d a = m_perDeviceFrames ? m_frames.correction(d.index) : m_universe.correction(d.index);
        d.sample.p = a * d.sample.p;
        d.sample.q = Eigen::Quaterniond(a.linear() * d.sample.q.toRotationMatrix()).normalized();
        d.sample.v = a.linear() * d.sample.v;
    }

    CalibrationInput& c = in.calib;
    c.referenceIndex = static_cast<uint8_t>(calibration->referenceDevice.deviceId < 64 ? calibration->referenceDevice.deviceId : 255);
    c.targetIndex = static_cast<uint8_t>(calibration->targetDevice.deviceId < 64 ? calibration->targetDevice.deviceId : 255);
    c.valid = calibration->isActive && calibration->isValidCalibration();
    c.relativeMode = calibration->isContinuousCalibration() && calibration->isRelativeCalibration;
    if (c.relativeMode) {
        c.C_L_valid = c.valid;
        c.C_L = isoFromCalibration(calibration->calibratedRotation, calibration->calibratedTranslation);
    } else {
        c.C_world_valid = c.valid;
        // in the trust layer's frame: C_world * stable == applied calibration * raw
        c.C_world = isoFromCalibration(calibration->calibratedRotation, calibration->calibratedTranslation) * (m_perDeviceFrames ? m_frames.global() : m_universe.global()).inverse();
    }
    c.solveAttempted = m_solveAttempted;
    c.solveValid = m_solveValid;
    c.solveAgrees = m_solveAgrees;
    m_solveAttempted = false;
    return true;
}

void TrustManager::tick(double currentTime)
{
    m_currentTime = currentTime;
    if (!m_enabled)
        return;
    // runs every frame (cheap): every calibration tick in this frame then sees a fresh verdict.
    // Repeated polls of an unchanged pose are recognised as such by the plausibility history.
    if (m_lastTick >= 0.0 && (currentTime - m_lastTick) < m_tickInterval)
        return;
    m_lastTick = currentTime;

    TickInput in;
    TrackingSystemCalibration* calibration = nullptr;
    if (!buildInput(currentTime, in, &calibration))
        return;

    // the head tracker's device index is unassigned while it is off (assignTarget re-resolves it
    // by serial): keep the history through that; start over only for a different tracker
    if (in.calib.targetIndex >= 64) {
        return;
    }
    if (in.calib.targetIndex != m_lastTargetIndex) {
        const std::string serial = calibration->targetDevice.deviceSerialNumber;
        if (m_lastTargetIndex != 255 && serial != m_lastTargetSerial) {
            m_consensus.reset();
        }
        m_lastTargetIndex = in.calib.targetIndex;
        m_lastTargetSerial = serial;
    }

    TickOutput out;
    m_consensus.tick(in, out);

    m_residualValid = out.targetResidualValid;
    m_residual = out.targetResidual;
    const bool holdBefore = m_holdActive;
    m_holdActive = m_holdEnabled && out.holdTarget;

    handleTransitions(out, calibration);
    handleEvent(out, calibration);
    feedLive(in, out, calibration);
    refreshPins(calibration);

    // the driver learns about hold changes through the transform flags (see apply())
    if (holdBefore != m_holdActive && calibration) {
        calibration->apply();
    }
}

void TrustManager::handleTransitions(const TickOutput& out, TrackingSystemCalibration* calibration)
{
    if (out.transitions.empty())
        return;
    auto* manager = CalibrationManager::getInstance();
    for (const Transition& tr : out.transitions) {
        m_transitionsThisSession++;
        const VRDevice_t dev = VRState::getInstance()->getVrDevice(tr.device);
        const std::string text = fmt::format("[trust] device {} ({} {}) {} -> {} ({}) residual={:.3f} m numbers=[{:.3f}, {:.3f}, {:.3f}]",
            tr.device, dev.szModel, dev.szSerial, stateName(tr.from), stateName(tr.to), reasonName(tr.reason), tr.residual, tr.numbers[0], tr.numbers[1], tr.numbers[2]);
        if (tr.to == State::SUSPECT || tr.to == State::UNTRUSTED) {
            LOG_CALIB_WARN("{}", text);
        } else {
            LOG_CALIB_INFO("{}", text);
        }
        m_lastEventText = text;

        // into the black box, through the driver
        if (auto* host = recorder::RecorderHost::getInstance()) {
            const float numbers[3] = { static_cast<float>(tr.numbers[0]), static_cast<float>(tr.numbers[1]), static_cast<float>(tr.numbers[2]) };
            host->blackBox().recordTrust(tr.device, static_cast<uint8_t>(tr.from), static_cast<uint8_t>(tr.to), static_cast<uint16_t>(tr.reason),
                static_cast<float>(tr.residual), numbers, blackbox::monoNow());
        }

        // automatic marker so the recording around the event is kept: only for new events
        // (from TRUSTED), never for the SUSPECT->UNTRUSTED confirmation or recovery loops
        if (m_autoMarker && tr.from == State::TRUSTED && (tr.to == State::SUSPECT || tr.to == State::UNTRUSTED) && guard::EventMarker::getInstance()) {
            guard::EventMarker::getInstance()->mark(blackbox::MarkerSource::AUTO, text, static_cast<uint32_t>(tr.reason));
        }
    }
    (void)calibration;
}

void TrustManager::handleEvent(const TickOutput& out, TrackingSystemCalibration* calibration)
{
    if (out.event == Event::NONE || !calibration)
        return;
    if (out.event == Event::PLAYSPACE_JUMP) {
        const Eigen::Vector3d dt = out.eventDelta.translation();
        const Eigen::Quaterniond dq(out.eventDelta.linear());
        const double angle = 2.0 * std::acos(std::min(1.0, std::abs(dq.w()))) * 180.0 / k_PI;
        m_lastEventText = fmt::format("[trust] playspace jump: every {} device moved together by {:.3f} m / {:.2f} deg (fit residual {:.3f} m)", m_targetSystem, dt.norm(), angle, out.eventResidual);
        LOG_CALIB_WARN("{}", m_lastEventText);
        if (guard::EventMarker::getInstance() && m_autoMarker)
            guard::EventMarker::getInstance()->mark(blackbox::MarkerSource::AUTO, m_lastEventText, 1000);
        const bool relative = calibration->isContinuousCalibration() && calibration->isRelativeCalibration;
        const bool fix = m_applyPlayspaceJumpFix && !relative && calibration->isValidCalibration() && !calibration->autoFixPlayspaceJumps;
        if (auto* live = guard::LiveStats::getInstance())
            live->pushEvent({ m_currentTime, guard::LiveEventKind::PLAYSPACE_SHIFT, static_cast<float>(dt.norm() * 100.0), fix ? "corrected" : "" });
        // upstream's own WorldFromDriver jump fix would correct the same event a second time
        if (fix) {
            // C_new = C_old * delta^-1 keeps every calibrated pose where it was
            guard::CalibrationAttempt jumpFix(*calibration, false, static_cast<uint8_t>(blackbox::CalibrationTrigger::PLAYSPACE_JUMP));
            Eigen::Isometry3d c = isoFromCalibration(calibration->calibratedRotation, calibration->calibratedTranslation);
            c = c * out.eventDelta.inverse();
            calibration->calibratedRotation = Eigen::Quaterniond(c.linear()).normalized();
            calibration->calibratedTranslation = c.translation();
            calibration->invalidateMetrics();
            calibration->clearSamples();
            calibration->apply();
            jumpFix.corrected();
            LOG_CALIB_INFO("[trust] playspace jump correction applied");
        }
    } else if (out.event == Event::REFERENCE_DISCONTINUITY) {
        m_lastEventText = fmt::format("[trust] reference discontinuity: the {} device jumped alone (re-center or tracking loss); solver history cleared", m_referenceSystem);
        if (auto* live = guard::LiveStats::getInstance())
            live->pushEvent({ m_currentTime, guard::LiveEventKind::REFERENCE_JUMP, 0.0f, "" });
        LOG_CALIB_INFO("{}", m_lastEventText);
        const bool relative = calibration->isContinuousCalibration() && calibration->isRelativeCalibration;
        if (!relative) {
            calibration->invalidateMetrics();
            calibration->clearSamples();
        }
    }
}

void TrustManager::handleUniverseShift(const UniverseShift& shift, TrackingSystemCalibration* calibration)
{
    m_lastEventText = fmt::format("[trust] SteamVR moved the {} universe: every device shifted {:.3f} m / {:.2f} deg ({} devices){}", m_targetSystem,
        shift.moveAtDevices, shift.angleDeg, shift.devices, shift.corrected ? ", calibration corrected" : ", not corrected");
    LOG_CALIB_WARN("{}", m_lastEventText);
    if (guard::EventMarker::getInstance() && m_autoMarker)
        guard::EventMarker::getInstance()->mark(blackbox::MarkerSource::AUTO, m_lastEventText, 1001);
    if (auto* live = guard::LiveStats::getInstance())
        live->pushEvent({ m_currentTime, guard::LiveEventKind::PLAYSPACE_SHIFT, static_cast<float>(shift.moveAtDevices * 100.0), shift.corrected ? "SteamVR universe, corrected" : "SteamVR universe" });
    if (!shift.corrected || !calibration)
        return;
    // raw_new = delta * raw_old for every unmoved device: C_new = C_old * delta^-1 keeps them where they were
    guard::CalibrationAttempt fix(*calibration, false, static_cast<uint8_t>(blackbox::CalibrationTrigger::WORLD_FROM_DRIVER_JUMP));
    Eigen::Isometry3d c = isoFromCalibration(calibration->calibratedRotation, calibration->calibratedTranslation);
    c = c * shift.delta.inverse();
    calibration->calibratedRotation = Eigen::Quaterniond(c.linear()).normalized();
    calibration->calibratedTranslation = c.translation();
    calibration->invalidateMetrics();
    calibration->clearSamples(); // the solver's samples straddle the shift
    calibration->apply();
    fix.corrected();
}

Eigen::Isometry3d TrustManager::devicePin(uint8_t index) const
{
    if (!m_enabled || !m_perDeviceFrames || !m_framesCorrecting)
        return Eigen::Isometry3d::Identity();
    return m_frames.pin(index);
}

void TrustManager::handleFrameEvents(const std::vector<FrameEvent>& events, TrackingSystemCalibration* calibration)
{
    for (const FrameEvent& e : events) {
        const VRDevice_t dev = VRState::getInstance()->getVrDevice(e.index);
        switch (e.kind) {
        case FrameEventKind::REFERENCE_MOVED: {
            m_lastEventText = fmt::format("[trust] SteamVR moved the head tracker's base station on its map: {:.3f} m / {:.2f} deg at {} {}{}", e.move, e.angleDeg,
                dev.szModel, dev.szSerial, e.corrected ? ", calibration corrected" : ", not corrected");
            LOG_CALIB_WARN("{}", m_lastEventText);
            if (auto* live = guard::LiveStats::getInstance())
                live->pushEvent({ m_currentTime, guard::LiveEventKind::PLAYSPACE_SHIFT, static_cast<float>(e.move * 100.0), e.corrected ? "SteamVR moved a base station, corrected" : "SteamVR moved a base station" });
            if (!e.corrected || !calibration)
                break;
            // raw_new = delta * raw_old on the head tracker's station: C_new = C_old * delta^-1 keeps them where they were
            guard::CalibrationAttempt fix(*calibration, false, static_cast<uint8_t>(blackbox::CalibrationTrigger::WORLD_FROM_DRIVER_JUMP));
            Eigen::Isometry3d c = isoFromCalibration(calibration->calibratedRotation, calibration->calibratedTranslation);
            c = c * e.delta.inverse();
            calibration->calibratedRotation = Eigen::Quaterniond(c.linear()).normalized();
            calibration->calibratedTranslation = c.translation();
            calibration->invalidateMetrics();
            calibration->clearSamples(); // the solver's samples straddle the move
            calibration->apply();
            fix.corrected();
            for (uint8_t i = 0; i < 64; i++)
                m_sentPins[i] = devicePin(i);
            break;
        }
        case FrameEventKind::DEVICE_MOVED:
            LOG_CALIB_INFO("[trust] SteamVR moved the base station under {} {} on its map: {:.3f} m / {:.2f} deg there, {}", dev.szModel, dev.szSerial, e.move, e.angleDeg,
                e.corrected ? "held, sliding back to SteamVR's map" : "not corrected");
            break;
        case FrameEventKind::SWITCH_ABSORBED:
            LOG_CALIB_INFO("[trust] {} {} changed base stations; SteamVR's maps of the two differ by {:.3f} m / {:.2f} deg there: held, sliding back", dev.szModel, dev.szSerial, e.move, e.angleDeg);
            break;
        case FrameEventKind::SWITCH_SHOWN:
            LOG_CALIB_INFO("[trust] {} {} changed base stations with a {:.3f} m / {:.2f} deg step: shown as it is", dev.szModel, dev.szSerial, e.move, e.angleDeg);
            break;
        case FrameEventKind::REFERENCE_SWITCHED:
            LOG_CALIB_INFO("[trust] the head tracker {} {} is tracked from another base station now; its station is the reference", dev.szModel, dev.szSerial);
            break;
        }
    }
}

void TrustManager::refreshPins(TrackingSystemCalibration* calibration)
{
    // pins change when SteamVR moves a station under a device and while held corrections slide
    // back (up to 1.5 cm/s): resend the transforms when one moved by more than 0.2 mm / 0.005 deg
    if (!calibration || !m_perDeviceFrames)
        return;
    bool changed = false;
    for (uint8_t i = 0; i < 64 && !changed; i++) {
        const Eigen::Isometry3d p = devicePin(i);
        const Eigen::Isometry3d d = p * m_sentPins[i].inverse();
        const double angle = Eigen::AngleAxisd(d.linear()).angle() * 180.0 / k_PI;
        changed = d.translation().norm() > 0.0002 || angle > 0.005;
    }
    if (!changed)
        return;
    calibration->applyQuiet();
    for (uint8_t i = 0; i < 64; i++)
        m_sentPins[i] = devicePin(i);
}

void TrustManager::forceTrustAll(double currentTime, const char* who)
{
    std::vector<Transition> transitions;
    m_consensus.forceTrustAll(currentTime, &transitions);
    TickOutput fake;
    fake.transitions = transitions;
    auto* manager = CalibrationManager::getInstance();
    TrackingSystemCalibration* calibration = nullptr;
    if (manager && manager->getCalibrationCount() > m_guardedCalibration)
        calibration = &manager->getCalibration(m_guardedCalibration);
    handleTransitions(fake, calibration);
    LOG_CALIB_NOTICE("[trust] manual override by {}: all devices trusted", who ? who : "?");
    const bool holdBefore = m_holdActive;
    m_holdActive = false;
    if (holdBefore && calibration)
        calibration->apply();
}

bool TrustManager::shouldRejectSamples(const TrackingSystemCalibration& calibration) const
{
    if (!m_enabled || !m_holdEnabled)
        return false; // observe-only: the solver runs exactly like upstream
    if (!calibration.isValidCalibration())
        return false; // nothing to protect yet: the first calibration must be allowed to happen
    const uint8_t target = static_cast<uint8_t>(calibration.targetDevice.deviceId < 64 ? calibration.targetDevice.deviceId : 255);
    if (target >= 64)
        return false;
    const State s = m_consensus.status(target).state;
    return s == State::SUSPECT || s == State::UNTRUSTED;
}

bool TrustManager::isRecovering(const TrackingSystemCalibration& calibration) const
{
    if (!m_enabled || !m_holdEnabled)
        return false;
    const uint8_t target = static_cast<uint8_t>(calibration.targetDevice.deviceId < 64 ? calibration.targetDevice.deviceId : 255);
    return target < 64 && m_consensus.status(target).state == State::RECOVERING;
}

bool TrustManager::shouldHold(const TrackingSystemCalibration& calibration) const
{
    if (!m_enabled || !m_holdEnabled)
        return false;
    const uint8_t target = static_cast<uint8_t>(calibration.targetDevice.deviceId < 64 ? calibration.targetDevice.deviceId : 255);
    return target < 64 && m_consensus.status(target).state != State::TRUSTED;
}

bool TrustManager::recoverySolveAgrees(const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation, bool valuesAreLocal) const
{
    const ConsensusParams& p = m_consensus.params();
    Eigen::Isometry3d proposed = isoFromCalibration(rotation, translation);
    if (!valuesAreLocal)
        proposed = proposed * (m_perDeviceFrames ? m_frames.global() : m_universe.global()).inverse(); // into the trust layer's frame
    Eigen::Isometry3d frozen;
    if (valuesAreLocal) {
        if (!m_consensus.localOffsetValid())
            return true; // nothing to compare against: accept
        frozen = m_consensus.localOffset();
    } else {
        if (!m_consensus.frozenCalibrationValid())
            return true;
        frozen = m_consensus.frozenCalibration();
    }
    // compare where it matters: at the head tracker's current position, not at the universe origin
    // (a small yaw difference is metres away from the origin but centimetres at the tracker)
    Eigen::Vector3d probe = Eigen::Vector3d::Zero();
    auto* manager = CalibrationManager::getInstance();
    if (manager && m_lastTargetIndex < 64) {
        const vr::DriverPose_t& tp = manager->m_poses[m_lastTargetIndex];
        probe = Eigen::Vector3d(tp.vecPosition[0], tp.vecPosition[1], tp.vecPosition[2]);
    }
    const double dPos = (proposed * probe - frozen * probe).norm();
    const Eigen::Quaterniond qa(proposed.linear());
    const Eigen::Quaterniond qb(frozen.linear());
    const double dAng = 2.0 * std::acos(std::min(1.0, std::abs(qa.dot(qb)))) * 180.0 / k_PI;
    const bool agrees = dPos <= p.d_agree && dAng <= p.a_agree_deg;
    LOG_CALIB_INFO("[trust] recovery solve {} the frozen calibration (dpos={:.3f} m, dang={:.2f} deg)", agrees ? "agrees with" : "disagrees with", dPos, dAng);
    return agrees;
}

void TrustManager::reportRecoverySolve(bool valid, bool agrees)
{
    m_solveAttempted = true;
    m_solveValid = valid;
    m_solveAgrees = agrees;
}

const std::string& TrustManager::liveLabel(uint8_t index, bool isTarget)
{
    const VRDevice_t dev = VRState::getInstance()->getVrDevice(index);
    const std::string key = fmt::format("{}|{}|{}", dev.szSerial, isTarget ? 1 : 0, static_cast<int>(dev.eControllerRole));
    if (m_liveLabelKeys[index] == key && !m_liveLabels[index].empty())
        return m_liveLabels[index];
    m_liveLabelKeys[index] = key;
    std::string label;
    if (dev.eDeviceClass == vr::TrackedDeviceClass_HMD) {
        label = "Headset";
    } else if (isTarget) {
        label = "Head tracker";
    } else if (dev.eDeviceClass == vr::TrackedDeviceClass_Controller) {
        const std::string& m = dev.szModel;
        const std::string kind = (m.find("Knuckles") != std::string::npos || m.find("Index") != std::string::npos) ? "Index"
            : (m.find("Quest") != std::string::npos || m.find("Oculus") != std::string::npos || m.find("Touch") != std::string::npos) ? "Quest"
                                                                                                                                       : "Controller";
        const char* side = dev.eControllerRole == vr::TrackedControllerRole_LeftHand ? " left" : (dev.eControllerRole == vr::TrackedControllerRole_RightHand ? " right" : "");
        label = kind + side;
    } else {
        std::string serial = dev.szSerial;
        const size_t dash = serial.find('-');
        if (dash != std::string::npos)
            serial = serial.substr(dash + 1);
        label = "Tracker " + serial;
    }
    m_liveLabels[index] = label;
    return m_liveLabels[index];
}

void TrustManager::feedLive(const TickInput& in, const TickOutput& out, const TrackingSystemCalibration* calibration)
{
    auto* live = guard::LiveStats::getInstance();
    if (!live || !VRState::getInstance())
        return;
    const double t = in.t;
    const uint8_t target = in.calib.targetIndex;
    const uint8_t reference = in.calib.referenceIndex;

    guard::LiveHeadSample head;
    head.t = t;
    head.valid = out.targetResidualValid;
    head.errorCm = static_cast<float>(out.targetResidual * 100.0);
    head.state = static_cast<uint8_t>(m_consensus.status(target).state);
    head.hold = m_holdActive;
    for (const DeviceInput& d : in.devices) {
        if (d.index == reference && head.valid) {
            const Eigen::Vector3d e = guard::headFrame(d.sample.q.toRotationMatrix(), out.targetResidualVec) * 100.0;
            head.forwardCm = static_cast<float>(e.x());
            head.sideCm = static_cast<float>(e.y());
            head.upCm = static_cast<float>(e.z());
        }
    }
    live->pushHead(head);

    for (const DeviceInput& d : in.devices) {
        if (d.universe == Universe::OTHER || d.index >= 64)
            continue;
        const DeviceStatus st = m_consensus.status(d.index);
        if (!st.known)
            continue;
        const VRDevice_t dev = VRState::getInstance()->getVrDevice(d.index);
        guard::LiveDeviceInfo info;
        info.known = true;
        info.isTarget = d.index == target;
        info.isReference = d.index == reference;
        info.isTracker = dev.eDeviceClass == vr::TrackedDeviceClass_GenericTracker && !info.isTarget;
        info.label = liveLabel(d.index, info.isTarget);
        guard::LiveDeviceSample s;
        s.t = t;
        s.unexplainedCm = static_cast<float>(st.unexplained * 100.0);
        s.limitCm = static_cast<float>(st.jumpLimit * 100.0);
        s.heightM = static_cast<float>(d.sample.p.y());
        s.state = static_cast<uint8_t>(st.state);
        s.tracking = st.tracking;
        live->pushDevice(d.index, info, s);
    }

    if (calibration && calibration->isValidCalibration()) {
        guard::LiveCalibrationSample c;
        c.t = t;
        c.xCm = static_cast<float>(calibration->calibratedTranslation.x() * 100.0);
        c.yCm = static_cast<float>(calibration->calibratedTranslation.y() * 100.0);
        c.zCm = static_cast<float>(calibration->calibratedTranslation.z() * 100.0);
        c.yawDeg = static_cast<float>(guard::yawDegrees(calibration->calibratedRotation));
        live->pushCalibration(c);
    }
}

std::vector<TrustDeviceView> TrustManager::deviceViews() const
{
    std::vector<TrustDeviceView> views;
    auto* vrState = VRState::getInstance();
    if (!vrState)
        return views;
    for (uint8_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        const DeviceStatus st = m_consensus.status(i);
        if (!st.known)
            continue;
        const VRDevice_t dev = vrState->getVrDevice(i);
        TrustDeviceView v;
        v.index = i;
        v.name = dev.eDeviceClass == vr::TrackedDeviceClass_HMD ? "HMD" : fmt::format("{} {}", dev.szModel, dev.szSerial);
        v.universe = dev.szTrackingSystemId == m_referenceSystem ? Universe::REFERENCE : (dev.szTrackingSystemId == m_targetSystem ? Universe::TARGET : Universe::OTHER);
        v.status = st;
        views.push_back(v);
    }
    return views;
}

} // namespace spacecal::trust
