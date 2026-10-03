#include "recorder_host.h"

#include "calibration.h"
#include "constants.h"
#include "guard/guard_config.h"
#include "log.h"
#include "protocol.h"
#include "util.h"
#include "vr_core.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <openvr.h>

namespace spacecal::recorder {

RecorderHost* RecorderHost::s_instance = nullptr;

namespace {
    constexpr size_t k_MAX_PULL = 200000; // records per writer iteration; a catch-up after a restart takes a few
    constexpr double k_ATTACH_RETRY_SECONDS = 2.0;
    constexpr double k_DEVICE_SCAN_SECONDS = 1.0;

    bool isDeviceRecord(uint8_t type)
    {
        using T = blackbox::RecordType;
        return type == static_cast<uint8_t>(T::POSE) || type == static_cast<uint8_t>(T::WORLD_FROM_DRIVER) || type == static_cast<uint8_t>(T::APPLIED)
            || type == static_cast<uint8_t>(T::DEVICE_STATE);
    }

    // the tracking systems of the calibration the trust layer guards (same choice as TrustManager)
    bool guardedSystems(std::string& reference, std::string& target)
    {
        auto* manager = CalibrationManager::getInstance();
        if (!manager || manager->getCalibrationCount() == 0)
            return false;
        TrackingSystemCalibration* calibration = nullptr;
        for (size_t i = 0; i < manager->getCalibrationCount(); i++) {
            TrackingSystemCalibration& c = manager->getCalibration(i);
            if (c.isContinuousCalibration() && c.devicesAreValid()) {
                calibration = &c;
                break;
            }
        }
        if (!calibration)
            calibration = &manager->getCalibration(0);
        reference = calibration->referenceDevice.trackingSystem;
        target = calibration->targetDevice.trackingSystem;
        return !reference.empty() && !target.empty();
    }
}

RecorderHost::~RecorderHost()
{
    shutdown();
    if (s_instance == this)
        s_instance = nullptr;
}

void RecorderHost::init()
{
    s_instance = this;
    blackbox::g_logFn = [](const char* msg) { LOG_INFO("{}", msg); };
}

void RecorderHost::shutdown()
{
    if (m_blackBox.isRunning())
        m_blackBox.stop(); // joins the writer: nothing uses the ring after this
    m_ring.detach();
    m_status.recording = false;
}

blackbox::Params RecorderHost::paramsFromConfig() const
{
    blackbox::Params p;
    const auto* manager = guard::GuardConfigManager::getInstance();
    if (!manager)
        return p;
    const auto& r = manager->get().recorder;
    p.enabled = r.enabled;
    p.liveWindowSeconds = r.live_window_seconds;
    p.preSeconds = r.pre_seconds;
    p.postSeconds = r.post_seconds;
    p.chunkSeconds = r.chunk_seconds;
    p.maxEventsPerSession = 50;
    p.keepAll = r.keep_all;
    p.archiveMaxMb = static_cast<uint32_t>(std::max(0.0, r.archive_max_gb) * 1024.0 + 0.5);
    return p;
}

void RecorderHost::applyConfig()
{
    if (m_blackBox.isRunning())
        m_blackBox.setParams(paramsFromConfig());
    m_nextDeviceScan = 0.0; // record_other_devices may have changed
}

bool RecorderHost::tryStart()
{
    std::string error;
    if (!m_ring.attached() && !m_ring.attach(blackbox::k_RING_DEFAULT_NAME, &error)) {
        if (error != m_status.waitingFor)
            LOG_INFO("Recorder waiting: {}", error);
        m_status.waitingFor = error;
        return false;
    }

    const uint64_t lost = m_ring.resume();
    const uint64_t backlog = m_ring.backlog();
    blackbox::StartOptions options;
    options.sessionId = m_ring.sessionId();
    options.sessionStartUnix = m_ring.driverStartUnix();
    options.pull = [this](std::vector<blackbox::Record>& out, bool newChunk, double chunkStart) {
        const size_t before = out.size();
        if (m_resetCarry.exchange(false))
            m_carry.reset();
        const uint64_t mask = m_excludeMask.load(std::memory_order_relaxed);
        if (newChunk)
            m_carry.beginChunk(mask); // each chunk starts with the transforms in effect (transform_carry.h)
        uint64_t lostNow = 0;
        m_ring.read(out, k_MAX_PULL, lostNow);
        if (lostNow > 0) {
            m_blackBox.noteDropped(lostNow);
            m_lost += lostNow;
        }
        // devices outside the calibration's tracking systems are left out (a filter, not a loss), and
        // WorldFromDriver is kept only when it changed (older drivers repeat it with every pose)
        size_t w = before;
        for (size_t i = before; i < out.size(); i++) {
            const blackbox::Record& r = out[i];
            if (r.device < 64 && isDeviceRecord(r.type) && ((mask >> r.device) & 1ull) != 0)
                continue;
            if (!m_carry.accept(r))
                continue;
            if (w != i)
                out[w] = r;
            w++;
        }
        out.resize(w);
        m_carry.endPull(out, before, chunkStart);
    };
    options.committed = [this] { m_ring.commit(); };

    const blackbox::Params params = paramsFromConfig();
    if (!m_blackBox.start(util::getSpaceCalibratorConfigDir() / "blackbox", SPACECAL_VERSION_STRING, params, options)) {
        m_status.waitingFor = "the recording folders could not be created";
        m_ring.detach();
        return false;
    }
    m_blackBox.setParams(params); // clamps, and puts the settings into the recording
    if (lost > 0) {
        m_blackBox.noteDropped(lost);
        m_lost += lost;
        m_blackBox.recordLifecycle(blackbox::LifecycleKind::RING_GAP, 0, static_cast<float>(lost), nullptr, 0, blackbox::monoNow());
    }
    m_status.recording = true;
    m_status.sessionId = options.sessionId;
    m_status.waitingFor.clear();
    m_lastIpcConnectCount = -1;
    m_notedMask = ~0ull; // the new recording gets its own filter note
    m_resetCarry = true;
    m_nextDeviceScan = 0.0;
    LOG_INFO("Recorder on driver session {:08x} (driver {}): {} records to catch up, {} lost", options.sessionId, m_ring.driverVersion(), backlog, lost);
    return true;
}

void RecorderHost::tick(double currentTime)
{
    if (m_blackBox.isRunning() && m_ring.sessionChanged()) {
        LOG_INFO("Recorder: SteamVR restarted the driver, switching to its new session");
        m_blackBox.stop();
        m_ring.detach();
        m_status.recording = false;
        m_nextAttempt = 0.0;
        m_devices = {}; // device slots are reassigned by the new SteamVR session
    }
    // the filter must be known before the first pull, so it is refreshed before a start too
    refreshDevices(currentTime);
    if (!m_blackBox.isRunning()) {
        m_status.recording = false;
        if (currentTime < m_nextAttempt)
            return;
        m_nextAttempt = currentTime + k_ATTACH_RETRY_SECONDS;
        refreshDevices(-1.0);
        if (!tryStart())
            return;
    }
    m_status.lostRecords = m_lost.load();

    // every (re)established IPC connection is a boundary worth seeing in the recording
    if (auto* manager = CalibrationManager::getInstance()) {
        const int connectCount = manager->getIpcClient().GetConnectCount();
        if (manager->getIpcClient().IsConnected() && connectCount != m_lastIpcConnectCount) {
            m_lastIpcConnectCount = connectCount;
            m_blackBox.recordLifecycle(blackbox::LifecycleKind::OVERLAY_CONNECTED, static_cast<uint16_t>(ipc::protocol::IPC_PROTOCOL_CURRENT), 0.0f, nullptr, 0, blackbox::monoNow());
        }
    }

    if (m_blackBox.takeDeviceInfoRequest()) { // once per chunk
        recordDeviceInfo();
    }
}

void RecorderHost::refreshDevices(double currentTime)
{
    if (currentTime >= 0.0 && currentTime < m_nextDeviceScan)
        return;
    if (currentTime >= 0.0)
        m_nextDeviceScan = currentTime + k_DEVICE_SCAN_SECONDS;
    vr::IVRSystem* system = vr::VRSystem();
    if (!system)
        return;

    auto prop = [system](vr::TrackedDeviceIndex_t i, vr::ETrackedDeviceProperty p) {
        char buf[256] = {};
        vr::ETrackedPropertyError err = vr::TrackedProp_Success;
        system->GetStringTrackedDeviceProperty(i, p, buf, sizeof(buf), &err);
        return err == vr::TrackedProp_Success ? std::string(buf) : std::string();
    };
    bool changed = false;
    for (vr::TrackedDeviceIndex_t i = 0; i < 64; i++) {
        const int cls = static_cast<int>(system->GetTrackedDeviceClass(i));
        DeviceEntry& e = m_devices[i];
        if (cls == e.deviceClass && (cls == 0 || !e.system.empty()))
            continue; // known (properties of a slot do not change within a session)
        e = {};
        e.deviceClass = cls;
        if (cls != 0) {
            e.system = prop(i, vr::Prop_TrackingSystemName_String);
            e.model = prop(i, vr::Prop_ModelNumber_String);
            e.serial = prop(i, vr::Prop_SerialNumber_String);
            e.registeredType = prop(i, vr::Prop_RegisteredDeviceType_String);
            e.controllerType = prop(i, vr::Prop_ControllerType_String);
            vr::ETrackedPropertyError err = vr::TrackedProp_Success;
            e.role = system->GetInt32TrackedDeviceProperty(i, vr::Prop_ControllerRoleHint_Int32, &err);
            if (err != vr::TrackedProp_Success)
                e.role = -1;
        }
        changed = true;
    }
    if (changed)
        m_blackBox.requestDeviceInfo();

    std::string reference, target;
    const bool known = guardedSystems(reference, target);
    const auto* cfg = guard::GuardConfigManager::getInstance();
    const bool recordOther = cfg && cfg->get().recorder.record_other_devices;
    uint64_t mask = 0;
    uint32_t recorded = 0;
    std::map<std::string, int> leftOut;
    for (uint32_t i = 0; i < 64; i++) {
        const DeviceEntry& e = m_devices[i];
        if (e.deviceClass == 0)
            continue;
        if (known && !recordOther && !e.system.empty() && e.system != reference && e.system != target) {
            mask |= 1ull << i;
            leftOut[e.system]++;
        } else {
            recorded++;
        }
    }
    m_excludeMask.store(mask, std::memory_order_relaxed);
    m_status.recordedDevices = recorded;
    m_status.leftOut.clear();
    for (const auto& [system, n] : leftOut)
        m_status.leftOut += (m_status.leftOut.empty() ? "" : ", ") + system + " (" + std::to_string(n) + ")";

    // the recording says what it leaves out, whenever that changes
    if (m_blackBox.isRunning() && mask != m_notedMask) {
        m_notedMask = mask;
        std::string note = "device filter: keep " + (known ? reference + "," + target : std::string("all (no calibration yet)"));
        if (mask != 0) {
            note += "; left out";
            std::map<std::string, std::string> slots;
            for (uint32_t i = 0; i < 64; i++)
                if ((mask >> i) & 1ull)
                    slots[m_devices[i].system] += (slots[m_devices[i].system].empty() ? "" : " ") + std::to_string(i);
            for (const auto& [system, list] : slots)
                note += " " + system + "[" + list + "]";
        }
        m_blackBox.recordText(blackbox::RecordType::TEXT, 255, blackbox::TextKind::NOTE, note, blackbox::monoNow());
        LOG_INFO("Recorder {}", note);
    }
}

void RecorderHost::recordDeviceInfo()
{
    const double t = blackbox::monoNow();
    for (size_t i = 0; i < m_devices.size(); i++) {
        const DeviceEntry& e = m_devices[i];
        if (e.deviceClass == 0)
            continue;
        const bool recorded = ((m_excludeMask.load(std::memory_order_relaxed) >> i) & 1ull) == 0;
        const std::string info = "class=" + std::to_string(e.deviceClass) + ";role=" + std::to_string(e.role) + ";sys=" + e.system + ";model=" + e.model
            + ";serial=" + e.serial + ";type=" + e.registeredType + ";ctrl=" + e.controllerType + ";recorded=" + (recorded ? "1" : "0");
        m_blackBox.recordText(blackbox::RecordType::DEVICE, static_cast<uint8_t>(i), blackbox::TextKind::DEVICE_INFO, info, t);
    }
}

} // namespace spacecal::recorder