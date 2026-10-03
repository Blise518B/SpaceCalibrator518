#pragma once

// Live statistics for the Live tab (fork, docs/DESIGN.md section 13): the last five minutes of what
// the guard and the solver see. Fed every trust tick (trust/trust_manager.cpp), every solve attempt
// (guard/calibration_record.cpp), every playspace event and every marker. Pure data: no SteamVR, no
// ImGui, one clock (the overlay's frame time). Tested in tests/live_test.cpp.

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace spacecal::guard {

// the head tracker against the headset, every trust tick
struct LiveHeadSample {
    double t = 0.0;
    bool valid = false; // a residual exists (calibration, both devices tracking)
    float errorCm = 0.0f; // distance between where the head tracker is and where headset + offset expect it
    float forwardCm = 0.0f; // the same error along the head's facing direction (horizontal)
    float sideCm = 0.0f; // ... to its right
    float upCm = 0.0f; // ... up
    uint8_t state = 0; // trust::State of the head tracker
    bool hold = false; // the guard holds the calibration
};

// one body device, every trust tick
struct LiveDeviceSample {
    double t = 0.0;
    float unexplainedCm = 0.0f; // this tick's step that its own reported velocity does not explain (0 without a new pose)
    float limitCm = 0.0f; // the guard's jump limit for that step
    float heightM = 0.0f; // height in its own tracking space (for the hip / foot guess)
    uint8_t state = 0; // trust::State
    bool tracking = false;
};

struct LiveDeviceInfo {
    bool known = false;
    std::string label; // "Head tracker", "Index left", "Tracker 1A2B3C4D"
    bool isTarget = false; // the head tracker
    bool isReference = false; // the headset
    bool isTracker = false; // a generic tracker (hip / foot guess applies)
};

// the applied calibration, at most k_CAL_RATE_HZ per second
struct LiveCalibrationSample {
    double t = 0.0;
    float xCm = 0.0f, yCm = 0.0f, zCm = 0.0f;
    float yawDeg = 0.0f;
};

// one solve attempt (or playspace correction) of the solver
struct LiveSolve {
    double t = 0.0;
    uint8_t trigger = 0; // blackbox::CalibrationTrigger
    uint8_t outcome = 0; // blackbox::CalibrationOutcome
    std::string errorKey; // translation key of the rejection reason ("" when applied)
    std::string errorText; // English rejection reason (fallback)
    float rmsMm = std::numeric_limits<float>::quiet_NaN(); // the new solve's error on its samples
    float currentRmsMm = std::numeric_limits<float>::quiet_NaN(); // the active calibration's error on the same samples
    float axisVariance = 0.0f; // how well the samples pin down the rotation (higher is better)
    float xCm = 0.0f, yCm = 0.0f, zCm = 0.0f, yawDeg = 0.0f; // the proposed calibration
    float changeCm = 0.0f, changeDeg = 0.0f; // proposed against the active one
};

enum class LiveEventKind : uint8_t {
    PLAYSPACE_SHIFT = 0, // every lighthouse device moved together
    REFERENCE_JUMP = 1, // the headset jumped alone (re-center)
    MARKER = 2,
};

struct LiveEvent {
    double t = 0.0;
    LiveEventKind kind = LiveEventKind::MARKER;
    float valueCm = 0.0f; // shift size
    std::string text; // marker source, "corrected"
};

class LiveStats {
public:
    static constexpr double k_KEEP_SECONDS = 310.0; // the longest window (5 min) plus a margin
    static constexpr double k_CAL_RATE_HZ = 10.0;

    void init();
    void clear();

    void pushHead(const LiveHeadSample& s);
    void pushDevice(uint8_t index, const LiveDeviceInfo& info, const LiveDeviceSample& s);
    void pushCalibration(const LiveCalibrationSample& s);
    void pushSolve(LiveSolve s); // stamped with the latest time when s.t <= 0
    void pushEvent(LiveEvent e); // stamped with the latest time when e.t <= 0

    [[nodiscard]] double now() const { return m_now; } // latest time pushed (-1 = nothing yet)
    [[nodiscard]] const std::deque<LiveHeadSample>& head() const { return m_head; }
    [[nodiscard]] const std::deque<LiveDeviceSample>& device(uint8_t index) const { return m_devices[index < 64 ? index : 63]; }
    [[nodiscard]] const LiveDeviceInfo& deviceInfo(uint8_t index) const { return m_deviceInfo[index < 64 ? index : 63]; }
    [[nodiscard]] const std::deque<LiveCalibrationSample>& calibration() const { return m_calibration; }
    [[nodiscard]] const std::deque<LiveSolve>& solves() const { return m_solves; }
    [[nodiscard]] const std::deque<LiveEvent>& events() const { return m_events; }
    [[nodiscard]] uint32_t playspaceShiftsTotal() const { return m_shiftsTotal; }
    [[nodiscard]] static LiveStats* getInstance() { return s_instance; }

private:
    void advance(double t);

    double m_now = -1.0;
    std::deque<LiveHeadSample> m_head;
    std::array<std::deque<LiveDeviceSample>, 64> m_devices {};
    std::array<LiveDeviceInfo, 64> m_deviceInfo {};
    std::deque<LiveCalibrationSample> m_calibration;
    std::deque<LiveSolve> m_solves;
    std::deque<LiveEvent> m_events;
    uint32_t m_shiftsTotal = 0;
    static LiveStats* s_instance;
};

// The error vector (reference world: calibrated head tracker minus expected) in the head's horizontal
// frame: x = forward along where the headset faces, y = to its right, z = world up. Looking straight
// up or down falls back to the headset's own up axis for the heading.
Eigen::Vector3d headFrame(const Eigen::Matrix3d& hmdRotation, const Eigen::Vector3d& errorWorld);

// Heading of a rotation around world up (OpenVR: y up, -z forward), degrees.
double yawDegrees(const Eigen::Quaterniond& q);

// For plotting many points: keeps each bucket's lowest and highest point (in x order), so spikes
// survive. NaN marks a gap and is kept as a gap. xs must be ascending.
void decimateMinMax(const std::vector<double>& xs, const std::vector<double>& ys, size_t maxPoints, std::vector<double>& outX, std::vector<double>& outY);

} // namespace spacecal::guard
