#pragma once

// Trigger-hold recalibration (fork, docs/DESIGN.md section 8).
//   - manual override: both triggers held for `hold_seconds` -> marker, the calibration shifted at
//     once (rotation kept) to the head tracker's learned place on the headset (no motion needed),
//     every device trusted, a fresh calibration follows when a solve passes the usual checks (never
//     forced: a forced solve from a still head threw the calibration far off). The way out of any
//     trust state.
//   - optional 1.5.1 behaviour: continuous updates are applied only while both triggers are held
// Input comes from IVRInput with the action manifest in assets/input (works for any controller),
// with the legacy GetControllerState as fallback when the manifest could not be registered.

#include <cstdint>
#include <string>

namespace spacecal::guard {

struct TriggerStatus {
    bool inputReady = false; // IVRInput manifest registered
    bool usingLegacyInput = false;
    bool leftDown = false;
    bool rightDown = false;
    bool bothDown = false;
    double heldFor = 0.0; // seconds both have been held, 0 when not
    double lastFireTime = -1.0;
    uint32_t firesThisSession = 0;
};

class TriggerHold {
public:
    void init();
    void tick(double currentTime);
    // true while both triggers are down (for the apply-only-while-held option)
    [[nodiscard]] bool bothHeld() const { return m_status.bothDown; }
    [[nodiscard]] const TriggerStatus& status() const { return m_status; }
    // short haptic pulse on both hands (no-op without IVRInput)
    void pulse(float durationSec, float amplitude);
    // called by the solver when a forced solve was applied after a trigger hold
    void notifyForcedSolveApplied(double currentTime);
    [[nodiscard]] static TriggerHold* getInstance() { return s_instance; }

private:
    bool readTriggers(bool& left, bool& right);
    bool readTriggersLegacy(bool& left, bool& right);
    void fire(double currentTime);

private:
    TriggerStatus m_status;
    double m_bothDownSince = -1.0;
    bool m_armed = true; // re-armed only after both triggers were released
    double m_secondPulseAt = -1.0;
    uint64_t m_actionSet = 0;
    uint64_t m_actionLeft = 0;
    uint64_t m_actionRight = 0;
    uint64_t m_hapticLeft = 0;
    uint64_t m_hapticRight = 0;
    static TriggerHold* s_instance;
};

} // namespace spacecal::guard
