#include "event_marker.h"

#include "calibration.h"
#include "constants.h"
#include "guard_config.h"
#include "live_stats.h"
#include "log.h"
#include "recorder/recorder_host.h"
#include "platform.h"
#include "util.h"
#include "vr_core.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <fmt/format.h>

#if OS_WINDOWS
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif

namespace spacecal::guard {

EventMarker* EventMarker::s_instance = nullptr;

namespace {
    using notes::jsonEscape;

    std::string localClock(std::time_t when)
    {
        std::tm tm {};
#if OS_WINDOWS
        localtime_s(&tm, &when);
#else
        localtime_r(&when, &tm);
#endif
        return fmt::format("{:02}:{:02}:{:02}", tm.tm_hour, tm.tm_min, tm.tm_sec);
    }

    bool writeTextFile(const std::filesystem::path& path, const std::string& text)
    {
        FILE* f = nullptr;
#if OS_WINDOWS
        f = _wfopen(path.c_str(), L"wb");
#else
        f = std::fopen(path.c_str(), "wb");
#endif
        if (!f)
            return false;
        const bool ok = std::fwrite(text.data(), 1, text.size(), f) == text.size();
        std::fclose(f);
        return ok;
    }

    bool readTextFile(const std::filesystem::path& path, std::string& text)
    {
        FILE* f = nullptr;
#if OS_WINDOWS
        f = _wfopen(path.c_str(), L"rb");
#else
        f = std::fopen(path.c_str(), "rb");
#endif
        if (!f)
            return false;
        text.clear();
        char buf[4096];
        size_t n = 0;
        while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0)
            text.append(buf, n);
        std::fclose(f);
        return true;
    }

    const char* calibrationStateName(CalibrationState state)
    {
        switch (state) {
        case CalibrationState::NONE: return "none";
        case CalibrationState::START: return "start";
        case CalibrationState::SAMPLE: return "sample";
        case CalibrationState::EDITING: return "editing";
        case CalibrationState::CONTINUOUS: return "continuous";
        case CalibrationState::CONTINUOUS_IDLE: return "continuous_idle";
        case CalibrationState::AUTO_DETECT_DEVICES_STANDARD: return "auto_detect_standard";
        case CalibrationState::AUTO_DETECT_DEVICES_CONTINUOUS: return "auto_detect_continuous";
        default: return "unknown";
        }
    }
}

void EventMarker::init()
{
    s_instance = this;
    m_status = {};
    m_pendingLogCopies.clear();
    m_lastMarkTime = -1.0;
    m_manual.clear();
    m_editIndex = -1;
    m_noteFocusRequest = false;
    m_returnFocusTo = nullptr;
    m_raisedForIndex = -1;
}

std::filesystem::path EventMarker::eventsDir() const
{
    return util::getSpaceCalibratorConfigDir() / "blackbox" / "events";
}

std::filesystem::path EventMarker::sessionsDir() const
{
    return util::getSpaceCalibratorConfigDir() / "blackbox" / "sessions";
}

void EventMarker::pushParamsToRecorder()
{
    if (auto* host = recorder::RecorderHost::getInstance())
        host->applyConfig();
}

void EventMarker::tick(double currentTime)
{
    auto* manager = CalibrationManager::getInstance();
    if (!manager)
        return;
    m_currentTime = currentTime;

    pollHotkey(currentTime);

    // deferred log copies after the post window
    for (size_t i = 0; i < m_pendingLogCopies.size();) {
        if (currentTime >= m_pendingLogCopies[i].dueTime) {
            copyOverlayLog(m_pendingLogCopies[i].folder);
            m_pendingLogCopies.erase(m_pendingLogCopies.begin() + i);
        } else {
            i++;
        }
    }
}

void EventMarker::pollHotkey(double currentTime)
{
    const GuardConfig& cfg = GuardConfigManager::getInstance()->get();
    if (!cfg.hotkey.enabled || cfg.hotkey.vk <= 0) {
        m_hotkeyWasDown = false;
        m_status.hotkeyDown = false;
        return;
    }
#if OS_WINDOWS
    const bool down = (GetAsyncKeyState(cfg.hotkey.vk) & 0x8000) != 0;
    m_status.hotkeyDown = down;
    if (down && !m_hotkeyWasDown) {
        mark(blackbox::MarkerSource::HOTKEY);
    }
    m_hotkeyWasDown = down;
#else
    (void)currentTime;
#endif
}

std::string EventMarker::makeFolderName(blackbox::MarkerSource source) const
{
    const std::time_t now = std::time(nullptr);
    std::tm tm {};
#if OS_WINDOWS
    localtime_s(&tm, &now);
#else
    localtime_r(&now, &tm);
#endif
    return fmt::format("{:04}-{:02}-{:02}_{:02}-{:02}-{:02}_{}", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, blackbox::markerSourceName(source));
}

bool EventMarker::mark(blackbox::MarkerSource source, const std::string& note, uint32_t label)
{
    auto* manager = CalibrationManager::getInstance();
    const GuardConfig& cfg = GuardConfigManager::getInstance()->get();
    auto* host = recorder::RecorderHost::getInstance();
    if (!host || !host->isRecording()) {
        LOG_WARN("Event marker ignored: the recorder is not running ({})", host ? host->status().waitingFor : std::string("no recorder"));
        return false;
    }
    (void)manager;
    if (!cfg.recorder.enabled) {
        LOG_WARN("Event marker ignored: recorder disabled in guard.json");
        return false;
    }
    if (source == blackbox::MarkerSource::AUTO && m_status.autoMarkersThisSession >= cfg.recorder.max_auto_markers_per_session) {
        LOG_WARN("Auto marker ignored: cap of {} per session reached", cfg.recorder.max_auto_markers_per_session);
        return false;
    }

    const double now = m_currentTime;
    const bool merged = m_lastMarkTime >= 0.0 && (now - m_lastMarkTime) < cfg.recorder.merge_window_seconds && !m_status.lastFolder.empty();
    const std::string folder = merged ? m_status.lastFolder : makeFolderName(source);

    const double monoMark = blackbox::monoNow();
    if (!host->blackBox().markEventAt(source, label, folder, monoMark)) {
        LOG_WARN("Event marker refused by the recorder (disabled, or the per-session event cap is reached)");
        return false;
    }

    LOG_NOTICE("Event marker set ({}) -> {}{}", blackbox::markerSourceName(source), folder, merged ? " (merged with previous marker)" : "");

    if (auto* live = LiveStats::getInstance())
        live->pushEvent({ 0.0, LiveEventKind::MARKER, 0.0f, blackbox::markerSourceName(source) });

    // a marker the user set himself asks for a note (what happened); the guard's own markers do not
    const bool manual = source == blackbox::MarkerSource::OVERLAY_BUTTON || source == blackbox::MarkerSource::HOTKEY || source == blackbox::MarkerSource::TRIGGERS;
    if (manual) {
        ManualMarker m;
        m.folder = folder;
        m.source = blackbox::markerSourceName(source);
        const std::time_t nowWall = std::time(nullptr);
        m.clock = localClock(nowWall);
        m.unixMark = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        m.monoMark = monoMark;
        m_manual.push_back(m);
        m_editIndex = -1;
        // the trigger hold happens in VR with the dashboard usually closed: its prompt waits in the Guard tab
        m_noteFocusRequest = source != blackbox::MarkerSource::TRIGGERS;
        if (source == blackbox::MarkerSource::HOTKEY && cfg.hotkey.raise_for_note) {
            if (m_returnFocusTo == nullptr)
                m_returnFocusTo = platform::raiseOwnWindowForInput();
            m_raisedForIndex = static_cast<int>(m_manual.size()) - 1;
        }
    }

    writeOverlayJson(folder, source, note, label, merged);
    copyOverlayLog(folder);
    m_pendingLogCopies.push_back({ folder, now + cfg.recorder.post_seconds + 5.0 });

    m_lastMarkTime = now;
    m_status.lastFolder = folder;
    m_status.lastMarkTime = now;
    m_status.lastSource = blackbox::markerSourceName(source);
    m_status.markersThisSession++;
    if (source == blackbox::MarkerSource::AUTO)
        m_status.autoMarkersThisSession++;
    return true;
}

void EventMarker::writeOverlayJson(const std::string& folder, blackbox::MarkerSource source, const std::string& note, uint32_t label, bool merged) const
{
    std::error_code ec;
    const std::filesystem::path dir = eventsDir() / folder;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
        LOG_ERROR("Cannot create event folder {}: {}", dir.string(), ec.message());
        return;
    }

    const double unixNow = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    auto* manager = CalibrationManager::getInstance();
    auto* vrState = VRState::getInstance();
    const GuardConfig& cfg = GuardConfigManager::getInstance()->get();

    std::string json;
    json += "{\n";
    json += fmt::format("  \"format\": \"spacecal-blackbox-event-overlay\",\n  \"version\": \"{}\",\n", SPACECAL_VERSION_STRING);
    json += fmt::format("  \"source\": \"{}\",\n  \"label\": {},\n  \"merged_into_previous\": {},\n", blackbox::markerSourceName(source), label, merged ? "true" : "false");
    json += fmt::format("  \"t_mark_unix\": {:.3f},\n  \"t_mark_overlay\": {:.6f},\n", unixNow, m_currentTime);
    json += fmt::format("  \"note\": \"{}\",\n", jsonEscape(note));
    json += notes::overlayJsonLine(noteEntries(folder)) + "\n"; // the user's notes (marker_notes.h), replaced in place when he saves one
    json += fmt::format("  \"recorder\": {{ \"pre_seconds\": {}, \"post_seconds\": {}, \"live_window_seconds\": {}, \"chunk_seconds\": {} }},\n",
        cfg.recorder.pre_seconds, cfg.recorder.post_seconds, cfg.recorder.live_window_seconds, cfg.recorder.chunk_seconds);

    // devices
    json += "  \"devices\": [\n";
    bool first = true;
    if (vrState) {
        for (size_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            const VRDevice_t dev = vrState->getVrDevice(i);
            if (!dev.bIsConnected)
                continue;
            json += fmt::format("    {}{{ \"index\": {}, \"class\": {}, \"role\": {}, \"tracking_system\": \"{}\", \"model\": \"{}\", \"serial\": \"{}\" }}\n",
                first ? "" : ",", i, static_cast<int>(dev.eDeviceClass), static_cast<int>(dev.eControllerRole), jsonEscape(dev.szTrackingSystemId), jsonEscape(dev.szModel), jsonEscape(dev.szSerial));
            first = false;
        }
    }
    json += "  ],\n";

    // calibrations
    json += "  \"calibrations\": [\n";
    if (manager) {
        for (size_t i = 0; i < manager->getCalibrationCount(); i++) {
            const TrackingSystemCalibration& cal = manager->getCalibration(i);
            const Eigen::Vector3d euler = cal.calibratedRotation.toRotationMatrix().canonicalEulerAngles(2, 1, 0) * (180.0 / EIGEN_PI);
            json += fmt::format("    {}{{\n", i ? "," : "");
            json += fmt::format("      \"active\": {}, \"state\": \"{}\", \"continuous\": {}, \"relative\": {}, \"valid\": {},\n",
                cal.isActive ? "true" : "false", calibrationStateName(cal.state), cal.isContinuousCalibration() ? "true" : "false",
                cal.isRelativeCalibration ? "true" : "false", cal.isValidCalibration() ? "true" : "false");
            json += fmt::format("      \"error\": \"{}\",\n", getCalibrationErrorMapping(cal.calibrationError).szLogString);
            json += fmt::format("      \"reference\": {{ \"index\": {}, \"tracking_system\": \"{}\", \"model\": \"{}\", \"serial\": \"{}\" }},\n",
                cal.referenceDevice.deviceId, jsonEscape(cal.referenceDevice.trackingSystem), jsonEscape(cal.referenceDevice.deviceModel), jsonEscape(cal.referenceDevice.deviceSerialNumber));
            json += fmt::format("      \"target\": {{ \"index\": {}, \"tracking_system\": \"{}\", \"model\": \"{}\", \"serial\": \"{}\" }},\n",
                cal.targetDevice.deviceId, jsonEscape(cal.targetDevice.trackingSystem), jsonEscape(cal.targetDevice.deviceModel), jsonEscape(cal.targetDevice.deviceSerialNumber));
            json += fmt::format("      \"translation_m\": [{:.6f}, {:.6f}, {:.6f}],\n", cal.calibratedTranslation.x(), cal.calibratedTranslation.y(), cal.calibratedTranslation.z());
            json += fmt::format("      \"rotation_quat_wxyz\": [{:.8f}, {:.8f}, {:.8f}, {:.8f}],\n", cal.calibratedRotation.w(), cal.calibratedRotation.x(), cal.calibratedRotation.y(), cal.calibratedRotation.z());
            json += fmt::format("      \"rotation_euler_deg_yaw_pitch_roll\": [{:.4f}, {:.4f}, {:.4f}],\n", euler[0], euler[1], euler[2]);
            json += fmt::format("      \"scale\": {:.6f},\n", cal.calibratedScale);
            json += fmt::format("      \"last_rms_error_m\": {:.6f}, \"last_axis_variance\": {:.6f}, \"sample_count\": {}\n",
                cal.errorMetrics.rmsError.last(), cal.errorMetrics.axisIndependence.last(), cal.m_samples.size());
            json += "    }\n";
        }
    }
    json += "  ]\n";
    json += "}\n";

    const std::filesystem::path jsonPath = dir / "overlay.json";
    FILE* f = nullptr;
#if OS_WINDOWS
    f = _wfopen(jsonPath.c_str(), L"wb");
#else
    f = std::fopen(jsonPath.c_str(), "wb");
#endif
    if (!f) {
        LOG_ERROR("Cannot write {}", jsonPath.string());
        return;
    }
    std::fwrite(json.data(), 1, json.size(), f);
    std::fclose(f);
}

int EventMarker::notePromptIndex() const
{
    if (m_editIndex >= 0 && m_editIndex < static_cast<int>(m_manual.size()))
        return m_editIndex;
    for (size_t i = m_manual.size(); i-- > 0;) {
        if (!m_manual[i].noteSaved && !m_manual[i].dismissed)
            return static_cast<int>(i);
    }
    return -1;
}

bool EventMarker::takeNoteFocusRequest()
{
    const bool request = m_noteFocusRequest;
    m_noteFocusRequest = false;
    return request;
}

std::vector<notes::Entry> EventMarker::noteEntries(const std::string& folder) const
{
    std::vector<notes::Entry> entries;
    for (const ManualMarker& m : m_manual) {
        if (m.folder == folder)
            entries.push_back({ m.unixMark, m.clock, m.source, m.note });
    }
    return entries;
}

void EventMarker::writeNoteFiles(const std::string& folder) const
{
    std::error_code ec;
    const std::filesystem::path dir = eventsDir() / folder;
    std::filesystem::create_directories(dir, ec);
    const std::vector<notes::Entry> entries = noteEntries(folder);
    if (!writeTextFile(dir / "notes.txt", notes::notesFile(folder, entries)))
        LOG_WARN("Could not write the marker notes into {}", dir.string());
    std::string json;
    if (readTextFile(dir / "overlay.json", json) && notes::replaceOverlayJsonLine(json, notes::overlayJsonLine(entries))) {
        if (!writeTextFile(dir / "overlay.json", json))
            LOG_WARN("Could not update the notes in {}", (dir / "overlay.json").string());
    }
}

bool EventMarker::saveNote(size_t index, const std::string& text)
{
    if (index >= m_manual.size())
        return false;
    ManualMarker& m = m_manual[index];
    const std::string note = notes::sanitize(text);
    const bool changed = note != m.note;
    m.note = note;
    m.noteSaved = !note.empty();
    m.dismissed = note.empty();
    if (m_editIndex == static_cast<int>(index))
        m_editIndex = -1;
    writeNoteFiles(m.folder);
    if (changed) {
        // into the recording too: the keep-all archive and the analysis tools see it next to the data
        auto* host = recorder::RecorderHost::getInstance();
        if (host && host->isRecording())
            host->blackBox().recordText(blackbox::RecordType::TEXT, 255, blackbox::TextKind::MARKER_NOTE, notes::recordPayload(m.monoMark, note), blackbox::monoNow());
        LOG_NOTICE("Marker note {} ({}, {}): {}", m.clock, m.source, m.folder, note.empty() ? std::string("(removed)") : note);
    }
    giveFocusBack(index);
    return true;
}

void EventMarker::dismissNote(size_t index)
{
    if (index >= m_manual.size())
        return;
    if (!m_manual[index].noteSaved)
        m_manual[index].dismissed = true;
    if (m_editIndex == static_cast<int>(index))
        m_editIndex = -1;
    giveFocusBack(index);
}

void EventMarker::editNote(size_t index)
{
    if (index >= m_manual.size())
        return;
    m_editIndex = static_cast<int>(index);
    m_noteFocusRequest = true;
}

void EventMarker::giveFocusBack(size_t index)
{
    if (m_raisedForIndex != static_cast<int>(index))
        return;
    if (m_returnFocusTo)
        platform::restoreForegroundWindow(m_returnFocusTo);
    m_returnFocusTo = nullptr;
    m_raisedForIndex = -1;
}

void EventMarker::copyOverlayLog(const std::string& folder) const
{
    std::error_code ec;
    const std::filesystem::path src = util::getSpaceCalibratorLogsDir() / "log_overlay_latest.log";
    const std::filesystem::path dir = eventsDir() / folder;
    std::filesystem::create_directories(dir, ec);
    if (!std::filesystem::exists(src, ec))
        return;
    std::filesystem::copy_file(src, dir / "log_overlay.log", std::filesystem::copy_options::overwrite_existing, ec);
    if (ec) {
        LOG_WARN("Could not copy overlay log into {}: {}", dir.string(), ec.message());
    }
}

} // namespace spacecal::guard
