#pragma once

// Record encoders shared by the driver (the pose ring producer) and the overlay's recorder, and
// the clock every record uses. No OpenVR dependency: the POSE encoder that needs DriverPose_t
// lives in src/driver/pose_record.h.

#include "blackbox_format.h"
#include <chrono>

namespace spacecal::blackbox {

// Steady-clock seconds, the time base of every record. std::chrono::steady_clock is the
// performance counter on Windows, so the driver (in vrserver) and the overlay agree on it.
inline double monoNow()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// system-clock seconds since the unix epoch (UTC)
inline double unixNow()
{
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

inline Record makeWorldFromDriverRecord(uint8_t device, const double trans[3], const double quatWxyz[4], double t)
{
    Record r;
    r.t = t;
    r.type = static_cast<uint8_t>(RecordType::WORLD_FROM_DRIVER);
    r.device = device;
    for (int i = 0; i < 3; i++)
        r.v[i] = static_cast<float>(trans[i]);
    for (int i = 0; i < 4; i++)
        r.v[3 + i] = static_cast<float>(quatWxyz[i]);
    return r;
}

inline Record makeAppliedRecord(uint8_t device, const double trans[3], const double quatWxyz[4], uint8_t deltaSize, double scale, double t)
{
    Record r;
    r.t = t;
    r.type = static_cast<uint8_t>(RecordType::APPLIED);
    r.device = device;
    r.a = deltaSize;
    r.f0 = static_cast<float>(scale);
    for (int i = 0; i < 3; i++)
        r.v[i] = static_cast<float>(trans[i]);
    for (int i = 0; i < 4; i++)
        r.v[3 + i] = static_cast<float>(quatWxyz[i]);
    return r;
}

// previous: 0 / 1, or 255 when the device is seen for the first time
inline Record makeDeviceStateRecord(uint8_t device, bool connected, uint8_t previous, uint16_t trackingResult, double t)
{
    Record r;
    r.t = t;
    r.type = static_cast<uint8_t>(RecordType::DEVICE_STATE);
    r.device = device;
    r.a = connected ? 1 : 0;
    r.b = previous;
    r.code = trackingResult;
    return r;
}

} // namespace spacecal::blackbox
