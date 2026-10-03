#include "trigger_hold.h"

#include "calibration.h"
#include "event_marker.h"
#include "guard_config.h"
#include "log.h"
#include "trust/trust_manager.h"
#include "util.h"
#include "vr_core.h"

#include <filesystem>
#include <openvr.h>

namespace spacecal::guard {

TriggerHold* TriggerHold::s_instance = nullptr;

void TriggerHold::init()
{
    s_instance = this;
    m_status = {};
    m_bothDownSince = -1.0;
    m_armed = true;

    if (!vr::VRInput() || !vr::VRSystem())
        return;

    // the manifest lives next to the other assets; SteamVR wants an absolute path
    std::error_code ec;
    const std::filesystem::path manifest = std::filesystem::absolute(util::getSpaceCalibratorInstallDir() / "assets" / "input" / "actions.json", ec);
    if (ec || !std::filesystem::exists(manifest, ec)) {
        LOG_WARN("[triggers] action manifest not found at {}, using legacy controller input", manifest.string());
        m_status.usingLegacyInput = true;
        return;
    }
    vr::EVRInputError err = vr::VRInput()->SetActionManifestPath(manifest.string().c_str());
    if (err != vr::VRInputError_None) {
        LOG_WARN("[triggers] SetActionManifestPath failed ({}), using legacy controller input", static_cast<int>(err));
        m_status.usingLegacyInput = true;
        return;
    }
    bool ok = vr::VRInput()->GetActionSetHandle("/actions/spacecal", &m_actionSet) == vr::VRInputError_None;
    ok = ok && vr::VRInput()->GetActionHandle("/actions/spacecal/in/hold_recalibrate_left", &m_actionLeft) == vr::VRInputError_None;
    ok = ok && vr::VRInput()->GetActionHandle("/actions/spacecal/in/hold_recalibrate_right", &m_actionRight) == vr::VRInputError_None;
    vr::VRInput()->GetActionHandle("/actions/spacecal/out/haptic_left", &m_hapticLeft);
    vr::VRInput()->GetActionHandle("/actions/spacecal/out/haptic_right", &m_hapticRight);
    if (!ok) {
        LOG_WARN("[triggers] action handles unavailable, using legacy controller input");
        m_status.usingLegacyInput = true;
        return;
    }
    m_status.inputReady = true;
    LOG_INFO("[triggers] action manifest registered: {}", manifest.string());
}

bool TriggerHold::readTriggersLegacy(bool& left, bool& right)
{
    left = right = false;
    if (!vr::VRSystem())
        return false;
    bool any = false;
    const vr::ETrackedControllerRole roles[2] = { vr::TrackedControllerRole_LeftHand, vr::TrackedControllerRole_RightHand };
    for (int hand = 0; hand < 2; hand++) {
        const vr::TrackedDeviceIndex_t idx = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(roles[hand]);
        if (idx == vr::k_unTrackedDeviceIndexInvalid)
            continue;
        vr::VRControllerState_t state = {};
        if (!vr::VRSystem()->GetControllerState(idx, &state, sizeof(state)))
            continue;
        any = true;
        // trigger: analog axis 1 (Vive, Index, Touch) or the SteamVR_Trigger button mask
        const bool pressed = state.rAxis[1].x > 0.75f || (state.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)) != 0;
        (hand == 0 ? left : right) = pressed;
    }
    return any;
}

bool TriggerHold::readTriggers(bool& left, bool& right)
{
    if (!m_status.inputReady || !vr::VRInput())
        return readTriggersLegacy(left, right);

    vr::VRActiveActionSet_t active = {};
    active.ulActionSet = m_actionSet;
    active.ulRestrictedToDevice = vr::k_ulInvalidInputValueHandle;
    active.nPriority = 0;
    if (vr::VRInput()->UpdateActionState(&active, sizeof(active), 1) != vr::VRInputError_None)
        return readTriggersLegacy(left, right);

    vr::InputDigitalActionData_t l = {}, r = {};
    const bool okL = vr::VRInput()->GetDigitalActionData(m_actionLeft, &l, sizeof(l), vr::k_ulInvalidInputValueHandle) == vr::VRInputError_None;
    const bool okR = vr::VRInput()->GetDigitalActionData(m_actionRight, &r, sizeof(r), vr::k_ulInvalidInputValueHandle) == vr::VRInputError_None;
    left = okL && l.bActive && l.bState;
    right = okR && r.bActive && r.bState;
    if (!(okL && l.bActive) && !(okR && r.bActive)) {
        // bindings not active for this controller type: fall back
        return readTriggersLegacy(left, right);
    }
    return true;
}

void TriggerHold::pulse(float durationSec, float amplitude)
{
    if (!m_status.inputReady || !vr::VRInput())
        return;
    if (m_hapticLeft)
        vr::VRInput()->TriggerHapticVibrationAction(m_hapticLeft, 0.0f, durationSec, 150.0f, amplitude, vr::k_ulInvalidInputValueHandle);
    if (m_hapticRight)
        vr::VRInput()->TriggerHapticVibrationAction(m_hapticRight, 0.0f, durationSec, 150.0f, amplitude, vr::k_ulInvalidInputValueHandle);
}

void TriggerHold::notifyForcedSolveApplied(double currentTime)
{
    if (m_status.lastFireTime >= 0.0 && currentTime - m_status.lastFireTime < 30.0) {
        LOG_INFO("[triggers] forced solve applied {:.1f} s after the hold", currentTime - m_status.lastFireTime);
        if (GuardConfigManager::getInstance()->get().triggers.haptics) {
            pulse(0.08f, 0.8f);
            m_secondPulseAt = currentTime + 0.2;
        }
    }
}

void TriggerHold::tick(double currentTime)
{
    const GuardConfig& cfg = GuardConfigManager::getInstance()->get();
    if (m_secondPulseAt >= 0.0 && currentTime >= m_secondPulseAt) {
        m_secondPulseAt = -1.0;
        pulse(0.08f, 0.8f);
    }
    bool left = false, right = false;
    readTriggers(left, right);
    // both controllers must be tracking: a hold with a sleeping controller is not a deliberate one
    if (vr::VRSystem()) {
        vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
        vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0f, poses, vr::k_unMaxTrackedDeviceCount);
        const vr::TrackedDeviceIndex_t li = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_LeftHand);
        const vr::TrackedDeviceIndex_t ri = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand);
        auto ok = [&](vr::TrackedDeviceIndex_t i) { return i < vr::k_unMaxTrackedDeviceCount && poses[i].bDeviceIsConnected && poses[i].bPoseIsValid && poses[i].eTrackingResult == vr::TrackingResult_Running_OK; };
        left = left && ok(li);
        right = right && ok(ri);
    }
    m_status.leftDown = left;
    m_status.rightDown = right;
    m_status.bothDown = left && right;

    if (!m_status.bothDown) {
        m_bothDownSince = -1.0;
        m_status.heldFor = 0.0;
        m_armed = true;
        return;
    }
    if (m_bothDownSince < 0.0)
        m_bothDownSince = currentTime;
    m_status.heldFor = currentTime - m_bothDownSince;

    if (cfg.triggers.override_enabled && m_armed && m_status.heldFor >= cfg.triggers.hold_seconds) {
        m_armed = false; // once per press
        fire(currentTime);
    }
}

void TriggerHold::fire(double currentTime)
{
    const GuardConfig& cfg = GuardConfigManager::getInstance()->get();
    m_status.lastFireTime = currentTime;
    m_status.firesThisSession++;
    LOG_NOTICE("[triggers] both triggers held for {:.1f} s: manual recalibration", cfg.triggers.hold_seconds);

    if (EventMarker::getInstance())
        EventMarker::getInstance()->mark(blackbox::MarkerSource::TRIGGERS, "trigger hold: manual recalibration");

    if (trust::TrustManager::getInstance())
        trust::TrustManager::getInstance()->forceTrustAll(currentTime, "trigger hold");

    auto* manager = CalibrationManager::getInstance();
    if (manager) {
        for (size_t i = 0; i < manager->getCalibrationCount(); i++) {
            TrackingSystemCalibration& c = manager->getCalibration(i);
            if (!c.isContinuousCalibration())
                continue;
            c.clearSamples();
            c.invalidateMetrics();
            c.forkForceTrigger = static_cast<uint8_t>(blackbox::CalibrationTrigger::TRIGGER_HOLD);
            c.forceNextCalibration();
        }
    }
    if (cfg.triggers.haptics)
        pulse(0.15f, 0.8f);
}

} // namespace spacecal::guard
