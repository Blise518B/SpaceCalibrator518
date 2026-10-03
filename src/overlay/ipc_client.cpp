#include "ipc_client.h"
#include "calibration.h"
#include "log.h"
#include "vr_core.h"
#include <chrono>
#include <thread>


namespace ipc {
const IpcFunction_t IpcClient::m_funcs[] = {
    {
        .commandType = protocol::IPC_COMMAND_HANDSHAKE,
        .szFunctionName = "SpaceCalibratorNova_Handshake",
    },
    {
        .commandType = protocol::IPC_COMMAND_SET_ALIGNMENT_SPEED_PARAMS,
        .szFunctionName = "SpaceCalibratorNova_SetAlignmentSpeedParams",
    },
    {
        .commandType = protocol::IPC_COMMAND_RESET_CALIBRATION,
        .szFunctionName = "SpaceCalibratorNova_ResetCalibration",
    },
    {
        .commandType = protocol::IPC_COMMAND_REQUEST_VIRTUAL_DESKTOP_PROPS,
        .szFunctionName = "SpaceCalibratorNova_RequestVirtualDesktopProps",
    },
};

bool IpcClient::Connect()
{
    m_hIpc = ipc_client_init({
        .szSharedMemoryName = protocol::k_szIpcIdentifier,
        .dwSharedBufferSizeBytes = protocol::k_unSharedMemoryElementSize,
        .aFunctions = m_funcs,
        .dwFunctionCount = IPC_COUNT_OF(m_funcs),
        .aOperations = nullptr,
        .dwOperationCount = 0,
        .userData = this,
    });

    if (m_hIpc == k_hInvalidIpcHandle) {
        LOG_IPC_ERROR("Failed to initialise IPC client!");
        m_connected = false;
        return false;
    }

    m_poseDataOperation = {
        .szIdentifier = "SpaceCalibratorNova_PoseSharedBuffer",
        .dwSharedMemoryOffset = 0,
    };
    m_deviceTransformOperation = {
        .szIdentifier = "SpaceCalibratorNova_DeviceTransformsSharedBuffer",
        .dwSharedMemoryOffset = protocol::k_unSharedMemoryPoseSize,
    };
    m_hmdMetaOperation = {
        .szIdentifier = "SpaceCalibratorNova_HmdMetaData",
        .dwSharedMemoryOffset = protocol::k_unSharedMemoryPoseSize + protocol::k_unSharedMemoryDeviceTransformSize,
    };

    if (!ipc_server_register_operation(m_hIpc, &m_poseDataOperation)) {
        LOG_IPC_ERROR("Failed to register pose data shared memory operation!");
    }

    if (!ipc_server_register_operation(m_hIpc, &m_deviceTransformOperation)) {
        LOG_IPC_ERROR("Failed to register device transform shared memory operation!");
    }

    if (!ipc_server_register_operation(m_hIpc, &m_hmdMetaOperation)) {
        LOG_IPC_ERROR("Failed to register hmd metadata shared memory operation!");
    }

    // send handshake to driver
    protocol::Command_Handshake_t handshakeArgs;
    ipc_client_dispatch_function(m_hIpc, protocol::IPC_COMMAND_HANDSHAKE, &handshakeArgs, sizeof(handshakeArgs));

    m_connected = m_hIpc != k_hInvalidIpcHandle;
    if (m_connected)
        m_connectCount++;

    LOG_IPC_INFO("IPC connection established: {}", m_connected);

    return m_connected;
}

void IpcClient::Shutdown()
{
    ipc_client_shutdown(&m_hIpc);
}

void IpcClient::SetDeviceTransform(protocol::Command_SetDeviceTransform_t deviceTransform)
{
    m_transforms[deviceTransform.unTargetOpenVrDeviceId] = deviceTransform;
    if (!ipc_server_write_shared_memory(m_hIpc, m_deviceTransformOperation, m_transforms, sizeof(m_transforms))) {
        LOG_IPC_ERROR("Tried updating device transform buffer, but operation is invalid!");
    }
}
void IpcClient::SetDeviceTransforms(const protocol::Command_SetDeviceTransform_t* transforms, size_t count)
{
    for (size_t i = 0; i < count; i++)
        if (transforms[i].unTargetOpenVrDeviceId < vr::k_unMaxTrackedDeviceCount)
            m_transforms[transforms[i].unTargetOpenVrDeviceId] = transforms[i];
    if (!ipc_server_write_shared_memory(m_hIpc, m_deviceTransformOperation, m_transforms, sizeof(m_transforms))) {
        LOG_IPC_ERROR("Tried updating device transform buffer, but operation is invalid!");
    }
}
void IpcClient::SetAlignmentSpeed(protocol::Command_SetAlignmentSpeedParams_t alignmentParams)
{
    ipc_client_dispatch_function(m_hIpc, protocol::IPC_COMMAND_SET_ALIGNMENT_SPEED_PARAMS, &alignmentParams, sizeof(alignmentParams));
}
void IpcClient::ResetCalibration()
{
    ipc_client_dispatch_function(m_hIpc, protocol::IPC_COMMAND_RESET_CALIBRATION, nullptr, 0);
}
void IpcClient::RequestVirtualDesktopProps()
{
    if (spacecal::VRState::getInstance()->isHmdVirtualDesktop()) {
        ipc_client_dispatch_function(m_hIpc, protocol::IPC_COMMAND_REQUEST_VIRTUAL_DESKTOP_PROPS, nullptr, 0);
    }
}
void IpcClient::Enqueue(protocol::CommandType_t type, const void* data, size_t size)
{
    if (m_queue.size() > 256) {
        LOG_IPC_WARN("IPC command queue full, dropping command {}", static_cast<int>(type));
        return;
    }
    QueuedCommand q;
    q.type = type;
    q.payload.assign(reinterpret_cast<const uint8_t*>(data), reinterpret_cast<const uint8_t*>(data) + size);
    m_queue.push_back(std::move(q));
}
void IpcClient::PumpQueue(int maxPerCall)
{
    if (!m_connected)
        return;
    for (int i = 0; i < maxPerCall && !m_queue.empty(); i++) {
        if (!ipc_client_is_idle(m_hIpc)) {
            // the driver has not consumed the previous command yet; try again next frame
            return;
        }
        QueuedCommand& q = m_queue.front();
        ipc_client_dispatch_function(m_hIpc, q.type, q.payload.data(), q.payload.size());
        m_queue.pop_front();
        // give the driver thread a moment before the next one (it wakes within microseconds; this
        // keeps two fork commands in one frame from racing)
        for (int spin = 0; spin < 200 && !ipc_client_is_idle(m_hIpc); spin++) {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    }
}
void IpcClient::PollPoses()
{
    ipc_client_read_shared_memory(m_hIpc, m_poseDataOperation, spacecal::CalibrationManager::getInstance()->m_poses, sizeof(spacecal::CalibrationManager::getInstance()->m_poses));
    ipc_client_read_shared_memory(m_hIpc, m_hmdMetaOperation, &spacecal::VRState::getInstance()->m_hmdMetadata, sizeof(spacecal::VRState::getInstance()->m_hmdMetadata));
}
}