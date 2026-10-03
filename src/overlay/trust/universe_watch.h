#pragma once

// SteamVR universe shifts (fork, docs/DESIGN.md section 13, "Universe shifts").
//
// Every lighthouse device reports its pose in driver space together with a WorldFromDriver (WFD)
// transform; for body-worn devices that is the frame of the base station the universe is built
// on, and all of them share it. When SteamVR re-solves its base stations it changes that shared
// transform: every lighthouse device moves by the same rigid delta in the raw world while nothing
// physically moved, and seen from a headset of another tracking system (Quest) the whole body
// jumps (2026-10-02 02:43: 5.7 cm / 1.25 deg, 04:14: 18 cm / 0.84 deg).
//
// The watch recognises that from the transforms themselves: two or more body-worn devices of the
// universe switch to the same new transform, and the change is a refinement (centimetres, a few
// degrees), not another base station's frame (metres away). It then keeps a stable frame in which
// the shift never happened: the trust layer works on correction(index) * raw, sees no jump, and
// the overlay corrects the calibration by exactly the delta (C_new = C_old * delta^-1). A single
// device switching to another base station's frame (a tracker that lost sight of the origin
// station) is not a shift and stays visible as its own jump.
//
// Pure logic (Eigen only): the overlay feeds it from the driver's pose buffer, the replay tool and
// the tests from recordings.

#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

namespace spacecal::trust {

struct UniverseWatchParams {
    double same_pos = 0.001; // m: two transforms are the same frame below this ...
    double same_deg = 0.02; // ... and this
    double min_shift = 0.002; // m at the devices: smaller changes are float noise, not a shift
    double max_shift = 0.5; // m at the devices: larger is another base station's frame, not a re-solve
    double max_shift_deg = 5.0;
    double window = 0.25; // s a lone device in a new frame waits for a second one before it counts as alone
    int min_devices = 2;
};

struct UniverseDevice {
    uint8_t index = 255;
    Eigen::Isometry3d wfd = Eigen::Isometry3d::Identity(); // driver -> raw world, as reported with the pose
    Eigen::Vector3d driverPos = Eigen::Vector3d::Zero(); // position in driver space
};

struct UniverseShift {
    double t = 0.0;
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity(); // raw_new = delta * raw_old for an unmoved device
    double moveAtDevices = 0.0; // m, how far it moved the devices (mean over the devices that switched)
    double angleDeg = 0.0;
    int devices = 0; // devices that switched together
    bool corrected = false; // the caller corrects the calibration; the stable frame keeps the old one
};

class UniverseWatch {
public:
    void setParams(const UniverseWatchParams& p) { m_params = p; }
    const UniverseWatchParams& params() const { return m_params; }
    void reset();

    // One tick of the target universe's body-worn devices (connected ones, tracking or not).
    // `correct`: the caller corrects the calibration for a shift (fixed mode, valid calibration);
    // otherwise the shift is reported but the stable frame follows the raw world (no correction).
    // Returns true when a shift was confirmed in this tick (details in *out).
    bool tick(double t, const std::vector<UniverseDevice>& devices, bool correct, UniverseShift* out);

    // stable = correction(index) * raw for the device's current pose
    Eigen::Isometry3d correction(uint8_t index) const;
    // the shared correction of the universe frame: C_trust = C_applied * global().inverse()
    const Eigen::Isometry3d& global() const { return m_global; }
    bool known() const { return m_haveFrame; }
    uint32_t shifts() const { return m_shifts; }

private:
    bool same(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) const;
    static double angleDeg(const Eigen::Isometry3d& a);

    struct Frame {
        Eigen::Isometry3d wfd = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d global = Eigen::Isometry3d::Identity(); // the stable-frame correction that goes with it
    };
    struct Candidate {
        bool active = false;
        Eigen::Isometry3d wfd = Eigen::Isometry3d::Identity();
        double since = 0.0;
    };

    UniverseWatchParams m_params;
    bool m_haveFrame = false;
    Eigen::Isometry3d m_frame = Eigen::Isometry3d::Identity(); // the universe's current WFD
    Eigen::Isometry3d m_global = Eigen::Isometry3d::Identity();
    std::vector<Frame> m_previous; // recent frames, for devices whose WFD lags behind a shift
    Candidate m_candidate; // a new frame seen on one device so far
    Eigen::Isometry3d m_correction[64];
    uint32_t m_shifts = 0;
    double m_lastInFrameT = -1.0; // last tick a device was in the current frame
};

} // namespace spacecal::trust
