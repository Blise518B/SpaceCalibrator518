#pragma once

// Fork settings (docs/DESIGN.md). Kept in their own file, %APPDATA%/space-calibrator/guard.json,
// so that upstream's versioned config.json and its migration code stay untouched. Every field
// has a default; a missing key in the file simply keeps the default.

#include <cstdint>
#include <string>

namespace spacecal::guard {

struct GuardConfig {
    int version = 1;

    struct Recorder {
        bool enabled = true;
        uint32_t live_window_seconds = 600;
        uint32_t pre_seconds = 300;
        uint32_t post_seconds = 300;
        uint32_t chunk_seconds = 60;
        uint32_t max_auto_markers_per_session = 20;
        uint32_t merge_window_seconds = 600; // markers closer than this share one event folder
        bool keep_all = false; // keep every chunk: chunks leaving the live window move to blackbox/sessions (NTFS-compressed) instead of being deleted
        double archive_max_gb = 0.0; // stop archiving above this size on disk, 0 = no cap; the archive itself is never pruned
        bool record_other_devices = false; // also record devices outside the calibration's two tracking systems (Standable, Virtual Desktop hands, ...)
        bool glitch_collection = true; // at overlay start, condense finished sessions to the minutes around their glitches (tools/glitch_collection.py, guard/housekeeping.h)
    } recorder;

    struct Startup {
        // With a valid stored calibration, the first continuous solve after the overlay starts goes
        // through the normal checks and trust gating instead of being forced. Off = upstream: the
        // first solve is applied even when it fails a check (it moved the calibration 13 cm on
        // 2026-09-30 00:31 with too little translation variance).
        bool keep_stored_calibration = true;
    } startup;

    struct Hotkey {
        bool enabled = true;
        int vk = 0x78; // VK_F9; Windows virtual-key code
        bool raise_for_note = true; // the hotkey brings the window forward with the cursor in the marker note field
    } hotkey;

    // docs/DESIGN.md sections 5, 6, 9
    struct Trust {
        bool enabled = true;
        bool hold_enabled = false; // freeze the calibration / driver correction while the head tracker is in doubt.
                                   // Off = observe-only: states, logs, markers and TRUST records, but the solver and driver
                                   // behave exactly like upstream. Switch on once a few recorded sessions show no false SUSPECTs.
        bool auto_marker = true; // set a black-box marker on every SUSPECT / UNTRUSTED transition
        bool apply_playspace_jump_fix = true; // correct the calibration when every lighthouse device jumps together
        bool fix_universe_shifts = true; // correct the calibration by exactly the change when SteamVR re-solves its base stations (trust/universe_watch.h)
        bool per_device_frames = true; // ... per device and base station: also devices on other stations (trust/frame_corrector.h); off = one shared frame (universe_watch.h)
        double frames_slide_cm_s = 1.5; // cm/s at the device: held corrections slide back to SteamVR's map (0 = never)
        double frames_slide_deg_s = 0.5; // deg/s, the same for rotation
        bool frames_absorb_switches = true; // hold a device through a change of base station where SteamVR's maps of the two disagree
        // the head tracker's place on the headset, learned while all is well and kept in head_mount.json (trust/head_mount.h)
        bool head_mount = true; // learn and remember it
        bool head_mount_startup = true; // at every overlay start, set the calibration from it once headset and head tracker hold still
        bool head_mount_prior = true; // reject solves that would put the head tracker elsewhere on the headset
        double head_mount_solve_max_cm = 5.0; // with head_mount_fix on, the prior uses head_mount_fix_min_cm when that is smaller
        double head_mount_solve_max_deg = 3.0;
        bool head_mount_fix = true; // slide the calibration back when the head tracker sits steadily away from its place (HeadMountFix)
        double head_mount_fix_min_cm = 2.5; // ... by more than this
        double head_mount_fix_rate_cm_s = 1.5; // ... at this speed
        double tick_hz = 0.0; // 0 = every overlay frame (recommended: verdicts precede every solver tick)
        // plausibility: steps are judged by what the device's own reported velocity does not explain
        double v_unexplained = 1.0; // m/s allowance for unexplained motion
        double rot_unexplained_dps = 200.0; // deg/s, the same for rotation
        double v_body_max = 10.0; // m/s, reported speeds above this are capped
        double w_body_max_dps = 3000.0; // deg/s, reported angular speeds above this are capped
        double v_max = 4.0; // fallback bound for devices that report no velocity
        double j_tol = 0.03;
        double v_err_max = 1.0;
        double a_max = 60.0;
        double rot_max_dps = 400.0; // fallback bound for devices that report no angular velocity
        double rot_tol_deg = 10.0;
        double t_stale = 0.25;
        double dt_max = 0.5; // gap longer than this restarts a device's history instead of judging it
        // consensus / recovery
        double t_confirm = 0.5;
        double r_ok = 0.03;
        double t_recover = 3.0;
        double d_agree = 0.05;
        double a_agree_deg = 2.0;
        double jump_fit_residual = 0.03;
        double r_high = 0.10;
        double t_residual_high = 0.5;
        int n_jurors = 2;
        double t_device_lost = 2.0;
        double c_l_smoothing = 0.002;
        double back_on_path_fraction = 0.2;
        double ref_jump_min = 0.10; // m, smallest HMD step that counts as a reference discontinuity
        double t_shift_window = 0.15; // s a step waits for the rest of the universe (playspace shifts arrive over a few frames)
    } trust;

    // docs/DESIGN.md section 8
    struct Triggers {
        bool override_enabled = true; // both triggers held -> marker, trust all, force the next solve
        double hold_seconds = 1.5;
        bool apply_only_while_held = false; // 1.5.1 behaviour: continuous updates apply only while both triggers are down
        bool haptics = true;
    } triggers;
};

class GuardConfigManager {
public:
    // loads guard.json (creating it with defaults when absent)
    bool init();
    bool load();
    bool save() const;

    [[nodiscard]] GuardConfig& get() { return m_config; }
    [[nodiscard]] const GuardConfig& get() const { return m_config; }
    [[nodiscard]] const std::string& path() const { return m_path; }
    [[nodiscard]] static GuardConfigManager* getInstance() { return s_instance; }

private:
    std::string m_path;
    GuardConfig m_config;
    static GuardConfigManager* s_instance;
};

// Human-readable name of a Windows virtual-key code for the UI ("F9", "F12", "Pause", ...).
std::string vkName(int vk);

} // namespace spacecal::guard
