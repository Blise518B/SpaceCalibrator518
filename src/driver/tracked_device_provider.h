#pragma once

#include "pose_ring.h"
#include "ipc_server.h"
#include "latency_estimator.h"
#include <Eigen/Geometry>
#include <chrono>
#include <openvr_driver.h>

namespace spacecal {

enum class DeltaSize {
    TINY,
    SMALL,
    LARGE
};

struct Pose_t {
    Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
};

struct DeviceCalibration_t {
    Pose_t pose;
    DeltaSize eDeltaSize = DeltaSize::TINY;
    bool hasCalibration = false;
    std::chrono::steady_clock::time_point lastUpdateTime {};
};

class ServerTrackedDeviceProvider : public vr::IServerTrackedDeviceProvider {
public:
    vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override;
    void Cleanup() override;
    const char* const* GetInterfaceVersions() override;
    void RunFrame() override;
    bool ShouldBlockStandbyMode() override;
    void EnterStandby() override;
    void LeaveStandby() override;

    bool HandleDevicePoseUpdated(vr::TrackedDeviceIndex_t unWhichDevice, vr::DriverPose_t& newPose);
    void ResetCalibration();
    void RequestVirtualDesktopProps();
    void SetAlignmentSpeedParams(ipc::protocol::Command_SetAlignmentSpeedParams_t& params);
    void HandleQuirks(ipc::protocol::DeviceQuirks_t quirks, vr::DriverPose_t& pose);

    // fork: records published into the pose ring (docs/DESIGN.md section 13)
    struct RecordedTransform_t {
        bool valid = false;
        double trans[3] = {};
        double quat[4] = {}; // w x y z
    };

    [[nodiscard]] inline bool IsDeviceIndexValid(const vr::TrackedDeviceIndex_t index) const
    {
        return index < vr::k_unMaxTrackedDeviceCount && index != vr::k_unTrackedDeviceIndexInvalid && index != vr::k_unTrackedDeviceIndexOther;
    }

    [[nodiscard]] inline bool IsPoseValid(const vr::DriverPose_t pose) const
    {
        return pose.deviceIsConnected && pose.poseIsValid && pose.result == vr::ETrackingResult::TrackingResult_Running_OK;
    }

    [[nodiscard]] inline bool IsPoseValid(const vr::TrackedDevicePose_t pose) const
    {
        return pose.bDeviceIsConnected && pose.bPoseIsValid && pose.eTrackingResult == vr::ETrackingResult::TrackingResult_Running_OK;
    }

private:
    void applyCalibrationToPose(vr::DriverPose_t& pose, vr::HmdQuaternion_t rotation, vr::HmdVector3d_t pos, double scale, bool calibrateMotionVecs);
    double getTransformRate(DeltaSize delta) const;
    DeltaSize getTransformDeltaSize(DeltaSize priorDelta, const Eigen::Vector3d& deviceWorldPos, const Pose_t& current, const Pose_t& target) const;

private:
    ipc::Server m_ipcServer;
    ipc::protocol::Command_SetAlignmentSpeedParams_t m_alignmentParams = {};

    double m_fVsyncPredictionTime = 0.0;
    ipc::protocol::SharedData_HmdMetadata m_hmdMetaData = {};

    ipc::protocol::Command_SetDeviceTransform_t m_transforms[vr::k_unMaxTrackedDeviceCount] = {};
    DeviceCalibration_t m_cachedCalibrations[vr::k_unMaxTrackedDeviceCount] = {}; // cache of calibrations for relative calibration
    vr::DriverPose_t m_poses[vr::k_unMaxTrackedDeviceCount] = {}; // raw poses
    LatencyEstimator m_latencyEstimator;

    // fork: pose ring publisher
    void recordWorldFromDriverIfChanged(vr::TrackedDeviceIndex_t device, const vr::DriverPose_t& pose, double t);
    void recordAppliedIfChanged(vr::TrackedDeviceIndex_t device, const vr::HmdQuaternion_t& rot, const vr::HmdVector3d_t& pos, uint8_t deltaSize, double scale, double t);
    void recordConnectionIfChanged(vr::TrackedDeviceIndex_t device, const vr::DriverPose_t& pose, double t);
    blackbox::PoseRingWriter m_poseRing;
    std::atomic<uint8_t> m_lastConnected[vr::k_unMaxTrackedDeviceCount] {}; // 0 no, 1 yes, 255 not seen yet
    RecordedTransform_t m_lastRecordedWorldFromDriver[vr::k_unMaxTrackedDeviceCount] = {};
    RecordedTransform_t m_lastRecordedApplied[vr::k_unMaxTrackedDeviceCount] = {};
    friend class ipc::Server;
};
}