#pragma once

#include <cstdint>
#include <inttypes.h>

#if !defined(_OPENVR_API)
#if defined(IS_DRIVER)
#include <openvr_driver.h>
#elif defined(IS_OVERLAY)
#include <openvr.h>
#endif
#endif

#ifndef EIGEN_PI
#define EIGEN_PI 3.141592653589793238462643383279502884197169399375105820974944592307816406L
#endif

#if defined(_OPENVR_API) && defined(IS_OVERLAY)

namespace vr {
// We can't include openvr_driver.h as it will result in multiple definition of some structures.
// However, we need to share driver-specific structures with the client application, so duplicate them
// here.

struct DriverPose_t {
    /* Time offset of this pose, in seconds from the actual time of the pose,
     * relative to the time of the PoseUpdated() call made by the driver.
     */
    double poseTimeOffset;

    /* Generally, the pose maintained by a driver
     * is in an inertial coordinate system different
     * from the world system of x+ right, y+ up, z+ back.
     * Also, the driver is not usually tracking the "head" position,
     * but instead an internal IMU or another reference point in the HMD.
     * The following two transforms transform positions and orientations
     * to app world space from driver world space,
     * and to HMD head space from driver local body space.
     *
     * We maintain the driver pose state in its internal coordinate system,
     * so we can do the pose prediction math without having to
     * use angular acceleration.  A driver's angular acceleration is generally not measured,
     * and is instead calculated from successive samples of angular velocity.
     * This leads to a noisy angular acceleration values, which are also
     * lagged due to the filtering required to reduce noise to an acceptable level.
     */
    vr::HmdQuaternion_t qWorldFromDriverRotation;
    double vecWorldFromDriverTranslation[3];

    vr::HmdQuaternion_t qDriverFromHeadRotation;
    double vecDriverFromHeadTranslation[3];

    /* State of driver pose, in meters and radians. */
    /* Position of the driver tracking reference in driver world space
     * +[0] (x) is right
     * +[1] (y) is up
     * -[2] (z) is forward
     */
    double vecPosition[3];

    /* Velocity of the pose in meters/second */
    double vecVelocity[3];

    /* Acceleration of the pose in meters/second */
    double vecAcceleration[3];

    /* Orientation of the tracker, represented as a quaternion */
    vr::HmdQuaternion_t qRotation;

    /* Angular velocity of the pose in axis-angle
     * representation. The direction is the angle of
     * rotation and the magnitude is the angle around
     * that axis in radians/second. */
    double vecAngularVelocity[3];

    /* Angular acceleration of the pose in axis-angle
     * representation. The direction is the angle of
     * rotation and the magnitude is the angle around
     * that axis in radians/second^2. */
    double vecAngularAcceleration[3];

    ETrackingResult result;

    bool poseIsValid;
    bool willDriftInYaw;
    bool shouldApplyHeadModel;
    bool deviceIsConnected;
};

}

#endif

#define ENUM_FLAG_OPERATORS(T)                                                                                                                                             \
    inline T operator~(T a) { return static_cast<T>(~static_cast<std::underlying_type<T>::type>(a)); }                                                                     \
    inline T operator|(T a, T b) { return static_cast<T>(static_cast<std::underlying_type<T>::type>(a) | static_cast<std::underlying_type<T>::type>(b)); }                 \
    inline T operator&(T a, T b) { return static_cast<T>(static_cast<std::underlying_type<T>::type>(a) & static_cast<std::underlying_type<T>::type>(b)); }                 \
    inline T operator^(T a, T b) { return static_cast<T>(static_cast<std::underlying_type<T>::type>(a) ^ static_cast<std::underlying_type<T>::type>(b)); }                 \
    inline T& operator|=(T& a, T b) { return reinterpret_cast<T&>(reinterpret_cast<std::underlying_type<T>::type&>(a) |= static_cast<std::underlying_type<T>::type>(b)); } \
    inline T& operator&=(T& a, T b) { return reinterpret_cast<T&>(reinterpret_cast<std::underlying_type<T>::type&>(a) &= static_cast<std::underlying_type<T>::type>(b)); } \
    inline T& operator^=(T& a, T b) { return reinterpret_cast<T&>(reinterpret_cast<std::underlying_type<T>::type&>(a) ^= static_cast<std::underlying_type<T>::type>(b)); }
#define ENUM_HAS_FLAGS(val, flags) ((val & flags) == flags)

namespace ipc::protocol {
enum Version_t {
    IPC_PROTOCOL_VER_INVALID,
    IPC_PROTOCOL_VER_1 = 1,
    IPC_PROTOCOL_VER_2,
    IPC_PROTOCOL_VER_3,
    IPC_PROTOCOL_VER_4,
    IPC_PROTOCOL_VER_5,
    IPC_PROTOCOL_VER_6, // fork: black-box recorder + event markers (docs/DESIGN.md)
    IPC_PROTOCOL_VER_7, // fork: keep-all session archive, calibration records
    IPC_PROTOCOL_VER_8, // fork: thin driver; the recorder lives in the overlay and reads the pose ring (pose_ring.h)

    IPC_PROTOCOL_CURRENT = IPC_PROTOCOL_VER_8,
};

enum CommandType_t : uint8_t {
    IPC_COMMAND_UNKNOWN,
    IPC_COMMAND_HANDSHAKE,
    IPC_COMMAND_SET_ALIGNMENT_SPEED_PARAMS,
    IPC_COMMAND_RESET_CALIBRATION,
    IPC_COMMAND_REQUEST_VIRTUAL_DESKTOP_PROPS,
    // fork additions (keep appended so upstream ids stay stable). Retired in VER_8, when the
    // recorder moved into the overlay; the ids stay reserved.
    IPC_COMMAND_MARK_EVENT,
    IPC_COMMAND_SET_BLACKBOX_PARAMS,
    IPC_COMMAND_RECORD_TRUST,
    IPC_COMMAND_RECORD_CALIBRATION,
    IPC_COMMAND_COUNT,
};

// these quirks enable unique behaviour depending on what device is detected, to correct for poor vendor implementations
enum class DeviceQuirks_t : uint16_t {
    QUIRK_NONE = 0,
    QUIRK_SCALE = 1 << 0, // 1 unit in this playspace is not 1 meter // @TODO: re-evalute if this is even necessary? we batch scale into device transforms already so i think this is redundant
    QUIRK_RECOMPUTE_VELOCITY = 1 << 1, // The velocity of this device is not considered to be reliable. We shall thus recompute it entirely from first principles.
    QUIRK_RECOMPUTE_ANGULAR_VELOCITY = 1 << 2, // The angular velocity of this device is not considered to be reliable. We shall thus recompute it entirely from first principles.
    QUIRK_HAS_WORLDSPACE_ANGULAR_VELOCITY = 1 << 3, // The angular velocity of this device is given in world space rather than object space by the vendor. We shall transform it to local space to improve SteamVR prediction.
};
ENUM_FLAG_OPERATORS(DeviceQuirks_t);

enum VD_HmdModel {
    VD_HmdModel_None,
    VD_HmdModel_GearVR,
    VD_HmdModel_OculusGo,
    VD_HmdModel_OculusQuest,
    VD_HmdModel_OculusQuest2,
    VD_HmdModel_MetaQuestPro,
    VD_HmdModel_Pico4,
    VD_HmdModel_MetaQuest3,
    VD_HmdModel_MetaQuest3S,
    VD_HmdModel_Pico4Ultra,
    VD_HmdModel_SamsungMoohan,
    VD_HmdModel_PlayForDreamMR,
    VD_HmdModel_Count,
};

struct Command_Handshake_t {
    Version_t version = Version_t::IPC_PROTOCOL_CURRENT;
};

struct Command_SetDeviceTransform_t {
    // @TODO: does this make sense to perform per device or per tracking system? per system probably makes a lot more sense tbh also bc it lends itself to multiple calibrations for effectively free
    // (need to think about ux for it though lol)
private:
    uint8_t _boolFlags = 0;

public:
    DeviceQuirks_t quirks = DeviceQuirks_t::QUIRK_NONE;
    vr::TrackedDeviceIndex_t unTargetOpenVrDeviceId = vr::k_unTrackedDeviceIndexInvalid;
    vr::TrackedDeviceIndex_t unRelativeReferenceOpenvrDeviceId = vr::k_unTrackedDeviceIndexInvalid;
    vr::TrackedDeviceIndex_t unRelativeTargetOpenvrDeviceId = vr::k_unTrackedDeviceIndexInvalid;
    vr::HmdVector3d_t translation = { .v = { 0.0, 0.0, 0.0 } };
    vr::HmdQuaternion_t rotation = { .w = 1.0, .x = 0.0, .y = 0.0, .z = 0.0 };
    double scale = 1.0;

#define FLAG(name, bitfield)                                                                     \
    inline bool name() const                                                                     \
    {                                                                                            \
        return (_boolFlags & bitfield) == bitfield;                                              \
    }                                                                                            \
    inline void name(bool newBool)                                                               \
    {                                                                                            \
        _boolFlags = (_boolFlags & ~bitfield) | (static_cast<std::uint8_t>(newBool) * bitfield); \
    }

    FLAG(enabled, 0b00000001)
    FLAG(lerpCalibrations, 0b00000010)
    FLAG(hideContinuousTracker, 0b00000100)
    FLAG(relativeCoordSystem, 0b00001000) // the calibration values are interpreted as relative to the hmd if this is set
    FLAG(calibrateMotionVecs, 0b00010000) // the calibration will be applied to velocity, angular velocity, acceleration and angular acceleration as well for improved accuracy
    FLAG(holdRelative, 0b00100000) // fork: keep the cached relative correction instead of recomputing it from the (untrusted) head tracker
#undef FLAG
};

struct Command_SetAlignmentSpeedParams_t {
    /**
     * The threshold at which we adjust the alignment speed based on the position offset
     * between current and target calibrations. Generally, we increase the speed if we go
     * above small/large, and decrease it only once it's under tiny.
     *
     * These values are expressed as distance squared
     */
    double thr_trans_tiny = 0.1f / 1000.0; // mm
    double thr_trans_small = 1.0f / 1000.0; // mm
    double thr_trans_large = 20.0f / 1000.0; // mm

    /**
     * Similar thresholds for rotation offsets, in radians
     */
    double thr_rot_tiny = 0.1f * (EIGEN_PI / 180.0f);
    double thr_rot_small = 1.0f * (EIGEN_PI / 180.0f);
    double thr_rot_large = 5.0f * (EIGEN_PI / 180.0f);

    /**
     * The speed of alignment, expressed as a lerp/slerp factor. 1 will blend most of the way in <1 second.
     * (We actually do a lerp(s * delta_t) where s is the speed factor here)
     */
    double align_speed_tiny = 0.05f;
    double align_speed_small = 0.2f;
    double align_speed_large = 2.0f;
};

struct SharedData_HmdMetadata {
    bool isVirtualDesktopAvailable = false;
    bool VD_stageTrackingEnabled = false;
    VD_HmdModel VD_hmdModel = VD_HmdModel_None;
};

constexpr uint32_t k_unSharedMemoryPoseSize = vr::k_unMaxTrackedDeviceCount * sizeof(vr::DriverPose_t);
constexpr uint32_t k_unSharedMemoryDeviceTransformSize = vr::k_unMaxTrackedDeviceCount * sizeof(protocol::Command_SetDeviceTransform_t);
constexpr uint32_t k_unSharedMemoryElementSize = k_unSharedMemoryPoseSize + k_unSharedMemoryDeviceTransformSize + sizeof(SharedData_HmdMetadata);
constexpr const char* k_szIpcIdentifier = "SPACE_CALIBRATOR_NOVA";
}