#pragma once

// POSE record from a raw DriverPose_t (driver side; the overlay never sees a DriverPose_t).

#include "blackbox_records.h"
#include <openvr_driver.h>

namespace spacecal::blackbox {

inline Record makePoseRecord(uint8_t device, const vr::DriverPose_t& pose, double t)
{
    Record r;
    r.t = t;
    r.type = static_cast<uint8_t>(RecordType::POSE);
    r.device = device;
    r.a = static_cast<uint8_t>(pose.result);
    r.code = static_cast<uint16_t>(pose.result); // format 2: `a` wraps for results above 255
    r.b = (pose.poseIsValid ? k_POSE_FLAG_VALID : 0) | (pose.deviceIsConnected ? k_POSE_FLAG_CONNECTED : 0)
        | (pose.willDriftInYaw ? k_POSE_FLAG_DRIFT_IN_YAW : 0) | (pose.shouldApplyHeadModel ? k_POSE_FLAG_HEAD_MODEL : 0);
    r.f0 = static_cast<float>(pose.poseTimeOffset);
    r.v[0] = static_cast<float>(pose.vecPosition[0]);
    r.v[1] = static_cast<float>(pose.vecPosition[1]);
    r.v[2] = static_cast<float>(pose.vecPosition[2]);
    r.v[3] = static_cast<float>(pose.qRotation.w);
    r.v[4] = static_cast<float>(pose.qRotation.x);
    r.v[5] = static_cast<float>(pose.qRotation.y);
    r.v[6] = static_cast<float>(pose.qRotation.z);
    r.v[7] = static_cast<float>(pose.vecVelocity[0]);
    r.v[8] = static_cast<float>(pose.vecVelocity[1]);
    r.v[9] = static_cast<float>(pose.vecVelocity[2]);
    r.v[10] = static_cast<float>(pose.vecAngularVelocity[0]);
    r.v[11] = static_cast<float>(pose.vecAngularVelocity[1]);
    r.v[12] = static_cast<float>(pose.vecAngularVelocity[2]);
    return r;
}

} // namespace spacecal::blackbox
