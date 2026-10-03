#pragma once

// The overlay side of the thin-driver split (docs/DESIGN.md section 13): attaches to the
// driver's pose ring, runs the black-box recorder on it and follows SteamVR restarts. Everything
// that records lives here, so changing it only needs an overlay restart, never a SteamVR one.
//
// An overlay restart continues the same session: the recorder adopts the session's live chunks
// and resumes the ring where the previous overlay stopped, so the recording has no gap as long
// as the overlay is back within the ring's span (about three minutes).
//
// Recorded devices: those of the guarded calibration's two tracking systems (the HMD's, e.g.
// oculus with the Quest controllers, and the target's, e.g. lighthouse with every tracker, Index
// controller and base station). Devices of other systems (Standable's virtual trackers, Virtual
// Desktop hand tracking, ...) are left out unless recorder.record_other_devices is set. Only
// devices whose system is known are left out; anything unknown is recorded.

#include "black_box.h"
#include "transform_carry.h"
#include "pose_ring.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <string>

namespace spacecal::recorder {

struct RecorderStatus {
    bool recording = false;
    uint32_t sessionId = 0; // the driver's session (one per SteamVR start)
    std::string waitingFor; // why nothing is recorded right now
    uint64_t lostRecords = 0; // records the ring overwrote before this overlay read them
    uint32_t recordedDevices = 0;
    std::string leftOut; // "standable (8), VirtualDesktop (2)", empty when nothing is left out
};

class RecorderHost {
public:
    RecorderHost() = default;
    ~RecorderHost();
    RecorderHost(const RecorderHost&) = delete;
    RecorderHost& operator=(const RecorderHost&) = delete;

    void init();
    // stops the recorder (flushes to disk); the live chunks stay for the next overlay run
    void shutdown();
    // overlay main loop: attach, follow SteamVR restarts, device table, device info
    void tick(double currentTime);
    // guard.json recorder section -> the running recorder
    void applyConfig();

    [[nodiscard]] blackbox::BlackBox& blackBox() { return m_blackBox; }
    [[nodiscard]] bool isRecording() const { return m_blackBox.isRunning(); }
    [[nodiscard]] const RecorderStatus& status() const { return m_status; }
    [[nodiscard]] static RecorderHost* getInstance() { return s_instance; }

private:
    struct DeviceEntry {
        int deviceClass = 0; // vr::ETrackedDeviceClass, 0 = no device in this slot
        int role = -1;
        std::string system, model, serial, registeredType, controllerType;
    };

    bool tryStart();
    void refreshDevices(double currentTime);
    void recordDeviceInfo();
    [[nodiscard]] blackbox::Params paramsFromConfig() const;

    blackbox::BlackBox m_blackBox;
    blackbox::PoseRingReader m_ring;
    RecorderStatus m_status;
    std::atomic<uint64_t> m_lost { 0 }; // grows on the recorder's writer thread
    std::atomic<uint64_t> m_excludeMask { 0 }; // device slots whose ring records are not written
    std::array<DeviceEntry, 64> m_devices {};
    blackbox::TransformCarry m_carry; // writer thread only
    std::atomic<bool> m_resetCarry { true }; // set per recording: device slots belong to one SteamVR session
    uint64_t m_notedMask = ~0ull; // exclusion set last written into the recording
    double m_nextDeviceScan = 0.0;
    double m_nextAttempt = 0.0;
    int m_lastIpcConnectCount = -1;
    static RecorderHost* s_instance;
};

} // namespace spacecal::recorder