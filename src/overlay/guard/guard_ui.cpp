#include "guard_ui.h"

#include "IconsMaterialSymbols.h"
#include "calibration.h"
#include "event_marker.h"
#include "guard_config.h"
#include "imgui.h"
#include "imgui_extensions.h"
#include "localisation.h"
#include "platform.h"
#include "recorder/recorder_host.h"
#include "trigger_hold.h"
#include "trust/trust_manager.h"
#include "vr_core.h"

#include <algorithm>
#include <filesystem>
#include <fmt/format.h>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>
#endif

namespace spacecal::guard {

namespace {
    // size of the keep-all archive (written by the driver), rescanned every 30 s
    struct ArchiveInfo {
        double scannedAt = -1e9;
        size_t sessions = 0;
        uint64_t bytesOnDisk = 0;
    };

    uint64_t fileSizeOnDisk(const std::filesystem::path& p)
    {
#ifdef _WIN32
        DWORD hi = 0;
        const DWORD lo = GetCompressedFileSizeW(p.c_str(), &hi);
        if (!(lo == INVALID_FILE_SIZE && GetLastError() != NO_ERROR))
            return (static_cast<uint64_t>(hi) << 32) | lo;
#endif
        std::error_code ec;
        const auto n = std::filesystem::file_size(p, ec);
        return ec ? 0 : static_cast<uint64_t>(n);
    }

    const ArchiveInfo& archiveInfo(double currentTime)
    {
        static ArchiveInfo info;
        if (currentTime - info.scannedAt < 30.0)
            return info;
        info = {};
        info.scannedAt = currentTime;
        std::error_code ec;
        const std::filesystem::path dir = EventMarker::getInstance()->sessionsDir();
        if (!std::filesystem::exists(dir, ec))
            return info;
        for (const auto& session : std::filesystem::directory_iterator(dir, ec)) {
            std::error_code ec2;
            if (!session.is_directory(ec2))
                continue;
            info.sessions++;
            for (const auto& f : std::filesystem::directory_iterator(session.path(), ec2)) {
                std::error_code ec3;
                if (f.is_regular_file(ec3))
                    info.bytesOnDisk += fileSizeOnDisk(f.path());
            }
        }
        return info;
    }

    // hotkey choices offered in the UI (virtual-key codes)
    constexpr int k_HOTKEY_CHOICES[] = { 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x13, 0x91, 0x2D, 0x24, 0x23 };

    void saveAndPush()
    {
        GuardConfigManager::getInstance()->save();
        if (EventMarker::getInstance())
            EventMarker::getInstance()->pushParamsToRecorder();
        if (trust::TrustManager::getInstance())
            trust::TrustManager::getInstance()->reloadParams();
    }

    ImVec4 stateColor(trust::State s)
    {
        switch (s) {
        case trust::State::TRUSTED: return ImVec4(0.35f, 0.6f, 0.4f, 1.0f);
        case trust::State::SUSPECT: return ImVec4(0.8f, 0.6f, 0.2f, 1.0f);
        case trust::State::UNTRUSTED: return ImVec4(0.75f, 0.3f, 0.3f, 1.0f);
        case trust::State::RECOVERING: return ImVec4(0.3f, 0.5f, 0.8f, 1.0f);
        default: return ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
        }
    }

    void drawTrustCard(double currentTime, GuardConfig& cfg)
    {
        trust::TrustManager* tm = trust::TrustManager::getInstance();
        if (!tm)
            return;
        ImGui::BeginCard("guard_trust");
        {
            ImGui::TextHeading("%s", LOCALE_GET("guard_trust_title").c_str());
            ImGui::TextWrappedDisabled(LOCALE_GET("guard_trust_desc").c_str());
            if (ImGui::CheckboxWithDescription(LOCALE_GET("guard_trust_enabled").c_str(), &cfg.trust.enabled, LOCALE_GET("guard_trust_enabled_desc").c_str())) {
                saveAndPush();
            }
            if (ImGui::CheckboxWithDescription(LOCALE_GET("guard_trust_hold").c_str(), &cfg.trust.hold_enabled, LOCALE_GET("guard_trust_hold_desc").c_str())) {
                saveAndPush();
            }
            if (ImGui::CheckboxWithDescription(LOCALE_GET("guard_trust_auto_marker").c_str(), &cfg.trust.auto_marker, LOCALE_GET("guard_trust_auto_marker_desc").c_str())) {
                saveAndPush();
            }
            if (ImGui::CheckboxWithDescription(LOCALE_GET("guard_keep_stored_on_start").c_str(), &cfg.startup.keep_stored_calibration, LOCALE_GET("guard_keep_stored_on_start_desc").c_str())) {
                saveAndPush();
            }
            if (VRState* vrState = VRState::getInstance(); vrState && vrState->isSteamVrAvailable()) {
                bool launch = vrState->launchesWithSteamVr();
                if (ImGui::CheckboxWithDescription(LOCALE_GET("guard_launch_with_steamvr").c_str(), &launch, LOCALE_GET("guard_launch_with_steamvr_desc").c_str())) {
                    vrState->setLaunchWithSteamVr(launch);
                }
            }

            ImGui::Spacing();
            if (tm->holdActive()) {
                ImGui::PillText(LOCALE_GET("guard_trust_holding").c_str(), ImVec4(0.75f, 0.3f, 0.3f, 1.0f));
            } else {
                ImGui::PillText(LOCALE_GET("guard_trust_normal").c_str(), ImVec4(0.35f, 0.6f, 0.4f, 1.0f));
            }
            ImGui::SameLine();
            if (tm->residualValid()) {
                const double residualCm = tm->residual() * 100.0;
                ImGui::TextDisabled("%s", LOCALE_FORMAT("guard_trust_residual", residualCm).c_str());
            } else {
                ImGui::TextDisabled("%s", LOCALE_GET("guard_trust_residual_none").c_str());
            }
            ImGui::SameLine();
            if (ImGui::IconButton(ICON_MS_GPP_GOOD, LOCALE_GET("guard_trust_force").c_str())) {
                tm->forceTrustAll(currentTime, "overlay button");
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("%s", LOCALE_GET("guard_trust_force_desc").c_str());
            }

            const std::vector<trust::TrustDeviceView> views = tm->deviceViews();
            if (!views.empty() && ImGui::BeginTable("guard_trust_table", 6, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH)) {
                ImGui::TableSetupColumn(LOCALE_GET("guard_trust_col_device").c_str());
                ImGui::TableSetupColumn(LOCALE_GET("guard_trust_col_universe").c_str());
                ImGui::TableSetupColumn(LOCALE_GET("guard_trust_col_state").c_str());
                ImGui::TableSetupColumn(LOCALE_GET("guard_trust_col_reason").c_str());
                ImGui::TableSetupColumn(LOCALE_GET("guard_trust_col_since").c_str());
                ImGui::TableSetupColumn(LOCALE_GET("guard_trust_col_flag").c_str());
                ImGui::TableHeadersRow();
                for (const trust::TrustDeviceView& v : views) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::Text("[%u] %s", v.index, v.name.c_str());
                    ImGui::TableNextColumn();
                    ImGui::TextDisabled("%s", v.universe == trust::Universe::REFERENCE ? LOCALE_GET("guard_trust_universe_reference").c_str() : (v.universe == trust::Universe::TARGET ? LOCALE_GET("guard_trust_universe_target").c_str() : "-"));
                    ImGui::TableNextColumn();
                    ImGui::TextColored(stateColor(v.status.state), "%s", trust::stateName(v.status.state));
                    ImGui::TableNextColumn();
                    ImGui::TextDisabled("%s", trust::reasonName(v.status.reason));
                    ImGui::TableNextColumn();
                    ImGui::TextDisabled("%.0f s", std::max(0.0, currentTime - v.status.since));
                    ImGui::TableNextColumn();
                    ImGui::TextDisabled("%s", v.status.tracking ? trust::flagName(v.status.lastFlag) : "off");
                }
                ImGui::EndTable();
            }
            if (!tm->lastEventText().empty()) {
                ImGui::TextWrappedDisabled(tm->lastEventText().c_str());
            }
            const unsigned int transitions = tm->transitionsThisSession();
            ImGui::TextWrappedDisabled(LOCALE_FORMAT("guard_trust_transitions", transitions).c_str());
        }
        ImGui::EndCard();
    }

    // The note for a marker the user set (event_marker.h): one line, Enter or Save keeps it, Later or
    // Esc puts it off (it stays in the list below). In the SteamVR dashboard the field opens the
    // SteamVR keyboard (window.cpp).
    void drawMarkerNotePrompt()
    {
        EventMarker* em = EventMarker::getInstance();
        static int s_bufFor = -1;
        static char s_buf[notes::k_MAX_NOTE_BYTES + 1] = {};
        const int idx = em->notePromptIndex();
        if (idx < 0) {
            s_bufFor = -1;
            return;
        }
        const ManualMarker& m = em->manualMarkers()[static_cast<size_t>(idx)];
        if (s_bufFor != idx) {
            std::snprintf(s_buf, sizeof(s_buf), "%s", m.note.c_str());
            s_bufFor = idx;
        }
        ImGui::Spacing();
        ImGui::TextWrapped("%s", LOCALE_FORMAT("guard_note_prompt", m.clock, m.source).c_str());
        if (em->takeNoteFocusRequest())
            ImGui::SetKeyboardFocusHere();
        const std::string saveText = LOCALE_GET("guard_note_save");
        const std::string laterText = LOCALE_GET("guard_note_later");
        const ImGuiStyle& style = ImGui::GetStyle();
        const float buttons = ImGui::CalcTextSize(saveText.c_str()).x + ImGui::CalcTextSize(laterText.c_str()).x + style.FramePadding.x * 4.0f + style.ItemSpacing.x * 2.0f;
        ImGui::SetNextItemWidth(std::max(200.0f, ImGui::GetContentRegionAvail().x - buttons));
        const bool enter = ImGui::InputTextWithHint("##marker_note", LOCALE_GET("guard_note_hint").c_str(), s_buf, sizeof(s_buf), ImGuiInputTextFlags_EnterReturnsTrue);
        const bool escaped = ImGui::IsItemDeactivated() && ImGui::IsKeyPressed(ImGuiKey_Escape, false);
        ImGui::SameLine();
        const bool save = ImGui::Button(saveText.c_str());
        ImGui::SameLine();
        const bool later = ImGui::Button(laterText.c_str());
        if (enter || save) {
            em->saveNote(static_cast<size_t>(idx), s_buf);
            s_bufFor = -1;
        } else if (later || escaped) {
            em->dismissNote(static_cast<size_t>(idx));
            s_bufFor = -1;
        }
    }

    void drawMarkerNotesList()
    {
        EventMarker* em = EventMarker::getInstance();
        const std::vector<ManualMarker>& list = em->manualMarkers();
        if (list.empty())
            return;
        const unsigned int count = static_cast<unsigned int>(list.size());
        if (ImGui::TreeNode("guard_notes_list", "%s", LOCALE_FORMAT("guard_notes_list", count).c_str())) {
            for (size_t i = list.size(); i-- > 0;) {
                const ManualMarker& m = list[i];
                ImGui::PushID(static_cast<int>(i));
                if (ImGui::SmallButton(LOCALE_GET("guard_note_edit").c_str()))
                    em->editNote(i);
                ImGui::SameLine();
                const std::string note = m.note.empty() ? LOCALE_GET("guard_note_none") : m.note;
                ImGui::TextWrapped("%s  %s  %s", m.clock.c_str(), m.source.c_str(), note.c_str());
                ImGui::PopID();
            }
            ImGui::TreePop();
        }
    }

    std::string lastMarkerText(double currentTime)
    {
        const MarkerStatus& st = EventMarker::getInstance()->status();
        if (st.lastMarkTime < 0.0)
            return LOCALE_GET("guard_last_marker_none");
        const int ago = static_cast<int>(currentTime - st.lastMarkTime);
        const std::string folder = st.lastFolder;
        const std::string source = st.lastSource;
        return LOCALE_FORMAT("guard_last_marker", folder, ago, source);
    }
}

void draw_mark_event_row()
{
    if (!EventMarker::getInstance() || !GuardConfigManager::getInstance())
        return;
    const bool recording = recorder::RecorderHost::getInstance() && recorder::RecorderHost::getInstance()->isRecording();
    const bool enabled = GuardConfigManager::getInstance()->get().recorder.enabled;

    ImGui::BeginDisabled(!recording || !enabled);
    if (ImGui::IconButtonPrimary(ICON_MS_FLAG, LOCALE_GET("guard_mark_event").c_str())) {
        EventMarker::getInstance()->mark(blackbox::MarkerSource::OVERLAY_BUTTON);
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("%s", LOCALE_GET("guard_mark_event_desc").c_str());
    }
    ImGui::SameLine();
    const MarkerStatus& st = EventMarker::getInstance()->status();
    if (!enabled) {
        ImGui::TextDisabled("%s", LOCALE_GET("guard_recorder_disabled_short").c_str());
    } else if (!recording) {
        ImGui::TextDisabled("%s", LOCALE_GET("guard_ipc_offline").c_str());
    } else if (st.lastMarkTime < 0.0) {
        ImGui::TextDisabled("%s", LOCALE_GET("guard_last_marker_none").c_str());
    } else {
        ImGui::TextDisabled("%s", st.lastFolder.c_str());
    }
    drawMarkerNotePrompt(); // fork: what happened (event_marker.h)
}

void page_guard(double currentTime)
{
    ImGui::TextTitle("%s", LOCALE_GET("guard_title").c_str());
    ImGui::TextWrappedDisabled(LOCALE_GET("guard_description").c_str());
    ImGui::Separator();

    if (!EventMarker::getInstance() || !GuardConfigManager::getInstance())
        return;
    GuardConfig& cfg = GuardConfigManager::getInstance()->get();
    const MarkerStatus& st = EventMarker::getInstance()->status();
    const bool ipcOk = CalibrationManager::getInstance() && CalibrationManager::getInstance()->getIpcClient().IsConnected();

    // ---- event recorder ---------------------------------------------------------------------
    ImGui::BeginCard("guard_recorder");
    {
        ImGui::TextHeading("%s", LOCALE_GET("guard_recorder_title").c_str());

        if (ImGui::CheckboxWithDescription(LOCALE_GET("guard_recorder_enabled").c_str(), &cfg.recorder.enabled, LOCALE_GET("guard_recorder_enabled_desc").c_str())) {
            saveAndPush();
        }
        if (ImGui::CheckboxWithDescription(LOCALE_GET("guard_keep_all").c_str(), &cfg.recorder.keep_all, LOCALE_GET("guard_keep_all_desc").c_str())) {
            saveAndPush();
        }
        if (ImGui::CheckboxWithDescription(LOCALE_GET("guard_record_other").c_str(), &cfg.recorder.record_other_devices, LOCALE_GET("guard_record_other_desc").c_str())) {
            saveAndPush();
        }
        if (cfg.recorder.keep_all) {
            const ArchiveInfo& ai = archiveInfo(currentTime);
            const size_t sessions = ai.sessions;
            const double gb = static_cast<double>(ai.bytesOnDisk) / (1024.0 * 1024.0 * 1024.0);
            const std::string sizeText = fmt::format("{:.2f}", gb);
            if (cfg.recorder.archive_max_gb > 0.0) {
                const std::string capText = fmt::format("{:.0f}", cfg.recorder.archive_max_gb);
                ImGui::TextWrappedDisabled(LOCALE_FORMAT("guard_archive_summary_cap", sessions, sizeText, capText).c_str());
                if (gb >= cfg.recorder.archive_max_gb)
                    ImGui::TextWrapped("%s", LOCALE_GET("guard_archive_cap_reached").c_str());
            } else {
                ImGui::TextWrappedDisabled(LOCALE_FORMAT("guard_archive_summary", sessions, sizeText).c_str());
            }
        }

        ImGui::Spacing();
        draw_mark_event_row();
        ImGui::TextWrappedDisabled(lastMarkerText(currentTime).c_str());
        const unsigned int markers = st.markersThisSession;
        const unsigned int autoMarkers = st.autoMarkersThisSession;
        const unsigned int autoCap = cfg.recorder.max_auto_markers_per_session;
        ImGui::TextWrappedDisabled(LOCALE_FORMAT("guard_markers_session", markers, autoMarkers, autoCap).c_str());
        drawMarkerNotesList();
        if (const auto* host = recorder::RecorderHost::getInstance()) {
            const recorder::RecorderStatus& rs = host->status();
            if (host->isRecording()) {
                const std::string session = fmt::format("{:08x}", rs.sessionId);
                const unsigned long long lost = rs.lostRecords;
                ImGui::TextWrappedDisabled(LOCALE_FORMAT("guard_recorder_status", session, lost).c_str());
                const unsigned int devices = rs.recordedDevices;
                const std::string leftOut = rs.leftOut.empty() ? std::string("-") : rs.leftOut;
                ImGui::TextWrappedDisabled(LOCALE_FORMAT("guard_recorder_devices", devices, leftOut).c_str());
            } else {
                const std::string why = rs.waitingFor.empty() ? std::string("starting") : rs.waitingFor;
                ImGui::TextWrappedDisabled(LOCALE_FORMAT("guard_recorder_waiting", why).c_str());
            }
        }

        ImGui::Spacing();
        if (ImGui::IconButton(ICON_MS_FOLDER_OPEN, LOCALE_GET("guard_open_events_folder").c_str())) {
            std::error_code ec;
            std::filesystem::create_directories(EventMarker::getInstance()->eventsDir(), ec);
            platform::launchDirInFileBrowser(EventMarker::getInstance()->eventsDir());
        }
        ImGui::SameLine();
        if (ImGui::IconButton(ICON_MS_FOLDER_OPEN, LOCALE_GET("guard_open_sessions_folder").c_str())) {
            std::error_code ec;
            std::filesystem::create_directories(EventMarker::getInstance()->sessionsDir(), ec);
            platform::launchDirInFileBrowser(EventMarker::getInstance()->sessionsDir());
        }
    }
    ImGui::EndCard();

    // ---- trust ---------------------------------------------------------------------------------
    drawTrustCard(currentTime, cfg);

    // ---- triggers -----------------------------------------------------------------------------
    ImGui::BeginCard("guard_triggers");
    {
        ImGui::TextHeading("%s", LOCALE_GET("guard_triggers_title").c_str());
        ImGui::TextWrappedDisabled(LOCALE_GET("guard_triggers_desc").c_str());
        if (ImGui::CheckboxWithDescription(LOCALE_GET("guard_triggers_override").c_str(), &cfg.triggers.override_enabled, LOCALE_GET("guard_triggers_override_desc").c_str())) {
            saveAndPush();
        }
        if (ImGui::CheckboxWithDescription(LOCALE_GET("guard_triggers_apply_only").c_str(), &cfg.triggers.apply_only_while_held, LOCALE_GET("guard_triggers_apply_only_desc").c_str())) {
            saveAndPush();
        }
        if (ImGui::CheckboxWithDescription(LOCALE_GET("guard_triggers_haptics").c_str(), &cfg.triggers.haptics, LOCALE_GET("guard_triggers_haptics_desc").c_str())) {
            saveAndPush();
        }
        float holdSeconds = static_cast<float>(cfg.triggers.hold_seconds);
        ImGui::SetNextItemWidth(220.0f);
        if (ImGui::SliderFloat(LOCALE_GET("guard_triggers_hold_seconds").c_str(), &holdSeconds, 0.5f, 5.0f, "%.1f s")) {
            cfg.triggers.hold_seconds = holdSeconds;
        }
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            saveAndPush();
        }
        if (TriggerHold::getInstance()) {
            const TriggerStatus& ts = TriggerHold::getInstance()->status();
            ImGui::TextDisabled("%s", ts.inputReady ? LOCALE_GET("guard_triggers_input_actions").c_str() : LOCALE_GET("guard_triggers_input_legacy").c_str());
            ImGui::Text("L %s   R %s", ts.leftDown ? ICON_MS_RADIO_BUTTON_CHECKED : ICON_MS_RADIO_BUTTON_UNCHECKED, ts.rightDown ? ICON_MS_RADIO_BUTTON_CHECKED : ICON_MS_RADIO_BUTTON_UNCHECKED);
            ImGui::SameLine();
            const float fraction = cfg.triggers.hold_seconds > 0.0 ? static_cast<float>(std::min(1.0, ts.heldFor / cfg.triggers.hold_seconds)) : 0.0f;
            ImGui::ProgressBar(fraction, ImVec2(160.0f, 0.0f), ts.bothDown ? nullptr : "");
            const unsigned int fires = ts.firesThisSession;
            ImGui::TextDisabled("%s", LOCALE_FORMAT("guard_triggers_fires", fires).c_str());
        }
    }
    ImGui::EndCard();

    // ---- hotkey ------------------------------------------------------------------------------
    ImGui::BeginCard("guard_hotkey");
    {
        ImGui::TextHeading("%s", LOCALE_GET("guard_hotkey_title").c_str());
        if (ImGui::CheckboxWithDescription(LOCALE_GET("guard_hotkey_enabled").c_str(), &cfg.hotkey.enabled, LOCALE_GET("guard_hotkey_enabled_desc").c_str())) {
            saveAndPush();
        }
        ImGui::BeginDisabled(!cfg.hotkey.enabled);
        ImGui::TextUnformatted(LOCALE_GET("guard_hotkey_key").c_str());
        ImGui::SameLine();
        const std::string current = vkName(cfg.hotkey.vk);
        ImGui::SetNextItemWidth(160.0f);
        if (ImGui::BeginCombo("##guard_hotkey_vk", current.c_str())) {
            for (int vk : k_HOTKEY_CHOICES) {
                const bool selected = vk == cfg.hotkey.vk;
                if (ImGui::Selectable(vkName(vk).c_str(), selected)) {
                    cfg.hotkey.vk = vk;
                    saveAndPush();
                }
                if (selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (st.hotkeyDown) {
            ImGui::PillText(LOCALE_GET("guard_hotkey_pressed").c_str(), ImVec4(0.35f, 0.6f, 0.4f, 1.0f));
        } else {
            ImGui::TextDisabled("%s", LOCALE_GET("guard_hotkey_hint").c_str());
        }
        if (ImGui::CheckboxWithDescription(LOCALE_GET("guard_hotkey_raise").c_str(), &cfg.hotkey.raise_for_note, LOCALE_GET("guard_hotkey_raise_desc").c_str())) {
            saveAndPush();
        }
        ImGui::EndDisabled();
    }
    ImGui::EndCard();

    // ---- window sizes (advanced) --------------------------------------------------------------
    ImGui::BeginCard("guard_window");
    {
        ImGui::TextHeading("%s", LOCALE_GET("guard_window_title").c_str());
        ImGui::TextWrappedDisabled(LOCALE_GET("guard_window_desc").c_str());
        int pre = static_cast<int>(cfg.recorder.pre_seconds);
        int post = static_cast<int>(cfg.recorder.post_seconds);
        int live = static_cast<int>(cfg.recorder.live_window_seconds);
        bool changed = false;
        bool commit = false;
        ImGui::SetNextItemWidth(220.0f);
        changed |= ImGui::SliderInt(LOCALE_GET("guard_pre_seconds").c_str(), &pre, 30, 600, "%d s");
        commit |= ImGui::IsItemDeactivatedAfterEdit();
        ImGui::SetNextItemWidth(220.0f);
        changed |= ImGui::SliderInt(LOCALE_GET("guard_post_seconds").c_str(), &post, 30, 600, "%d s");
        commit |= ImGui::IsItemDeactivatedAfterEdit();
        ImGui::SetNextItemWidth(220.0f);
        changed |= ImGui::SliderInt(LOCALE_GET("guard_live_window").c_str(), &live, 120, 1800, "%d s");
        commit |= ImGui::IsItemDeactivatedAfterEdit();
        if (changed) {
            cfg.recorder.pre_seconds = static_cast<uint32_t>(pre);
            cfg.recorder.post_seconds = static_cast<uint32_t>(post);
            cfg.recorder.live_window_seconds = static_cast<uint32_t>(std::max(live, pre));
        }
        if (commit) {
            saveAndPush(); // only once the slider is released
        }
        const unsigned long long estimateMb = (cfg.recorder.live_window_seconds * 10ull * 250ull * 80ull) / (1024ull * 1024ull);
        ImGui::TextWrappedDisabled(LOCALE_FORMAT("guard_window_estimate", estimateMb).c_str());
    }
    ImGui::EndCard();
}

} // namespace spacecal::guard
