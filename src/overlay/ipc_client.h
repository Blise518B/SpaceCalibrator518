#pragma once

// clang-format off
#include "protocol.h"
#include "ipc.h"
// clang-format on
#include <deque>
#include <vector>

namespace ipc {
class IpcClient {
public:
    bool Connect();
    void Shutdown();

    inline const bool IsConnected() const { return m_connected; }

    void SetDeviceTransform(protocol::Command_SetDeviceTransform_t deviceTransform);
    void SetDeviceTransforms(const protocol::Command_SetDeviceTransform_t* transforms, size_t count); // fork: one write for many
    void SetAlignmentSpeed(protocol::Command_SetAlignmentSpeedParams_t alignmentParams);
    void ResetCalibration();
    void RequestVirtualDesktopProps();
    void PollPoses();

    // fork: queued commands are sent one per PumpQueue() call once the driver has consumed the
    // previous one, because the IPC has a single command slot and back-to-back dispatches
    // overwrite each other. (No fork command uses it since VER_8; kept for future commands.)
    void PumpQueue(int maxPerCall = 4);
    [[nodiscard]] size_t QueuedCommands() const { return m_queue.size(); }
    [[nodiscard]] int GetConnectCount() const { return m_connectCount; }

private:
    bool m_connected = false;
    int m_connectCount = 0; // fork: bumped on every successful Connect()
    ::IpcHandle_t m_hIpc = k_hInvalidIpcHandle;
    ::IpcOperation_t m_poseDataOperation;
    ::IpcOperation_t m_deviceTransformOperation;
    ::IpcOperation_t m_hmdMetaOperation;

    ipc::protocol::Command_SetDeviceTransform_t m_transforms[vr::k_unMaxTrackedDeviceCount] = {};

    struct QueuedCommand {
        protocol::CommandType_t type;
        std::vector<uint8_t> payload;
    };
    std::deque<QueuedCommand> m_queue; // fork
    void Enqueue(protocol::CommandType_t type, const void* data, size_t size);

    static const IpcFunction_t m_funcs[];

    friend class VRState; // for m_poses
};
}