#pragma once

// Replay core (fork): loads an event's overlay.json and DEVICE table and feeds the recording
// through the same Consensus the overlay runs. Shared by spacecal-replay and the tests.

#include "blackbox_reader.h"
#include "guard/guard_config.h"
#include "trust/consensus.h"
#include "trust/frame_corrector.h"
#include "trust/universe_watch.h"

#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace spacecal::replay {

struct OverlayDeviceRef {
    int64_t index = -1; // overlay.json writes 4294967295 for an unassigned device
    std::string tracking_system;
    std::string model;
    std::string serial;
};
struct OverlayCalibration {
    bool active = false;
    std::string state;
    bool continuous = false;
    bool relative = false;
    bool valid = false;
    OverlayDeviceRef reference;
    OverlayDeviceRef target;
    std::vector<double> translation_m;
    std::vector<double> rotation_quat_wxyz;
};
struct OverlayJson {
    std::string source;
    double t_mark_unix = 0.0;
    std::vector<OverlayCalibration> calibrations;
};

struct DeviceMeta {
    int deviceClass = 0;
    std::string sys, model, serial;
};

struct ReplayResult {
    std::vector<trust::Transition> transitions;
    std::vector<std::pair<double, trust::Event>> events;
    std::vector<double> eventShift; // per event: translation of the PLAYSPACE_JUMP delta [m], 0 otherwise
    size_t ticks = 0;
    double firstT = 0.0, lastT = 0.0;
    uint64_t anchors = 0; // head-tracker estimate re-anchored to a calibration change
    std::vector<trust::UniverseShift> universeShifts; // SteamVR re-solved its base stations (universe_watch.h)
    std::vector<trust::FrameEvent> frameEvents; // the same per device and base station (frame_corrector.h)
};

// SteamVR moving base stations: per device (frame_corrector.h, the overlay's default) or one shared frame
struct ReplayFrames {
    bool perDevice = true;
    trust::FrameParams params;
};

bool applyOverride(guard::GuardConfig::Trust& t, const std::string& key, double value);
bool loadOverlayJson(const std::filesystem::path& file, OverlayJson& out, std::string* error = nullptr);
std::map<uint8_t, DeviceMeta> deviceMetaFromRecording(const blackbox::Recording& rec);
// Calibration in fixed mode:
//   RECORDED  follow every APPLIED record of the target, i.e. exactly what the live run applied
//   OWN       start from the first APPLIED record (else overlay.json), then only the replay's own
//             playspace-jump corrections (for short synthetic recordings; long ones drift because the
//             live solver kept refining the calibration)
//   CORRECTED follow the APPLIED records and add the replay's own playspace-jump corrections that the
//             live run did not make, until a recorded calibration shows the live solver caught up
enum class CalibrationMode : uint8_t { RECORDED = 0, OWN = 1, CORRECTED = 2 };
ReplayResult replay(const blackbox::Recording& rec, const OverlayCalibration& cal, const std::map<uint8_t, DeviceMeta>& meta, const trust::ConsensusParams& params, double hz, bool relative, const std::filesystem::path& csvPath, CalibrationMode calMode = CalibrationMode::RECORDED, const ReplayFrames& frames = ReplayFrames {});

} // namespace spacecal::replay
