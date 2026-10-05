#pragma once

// Black-box recorder file format (fork addition, see docs/DESIGN.md section 4).
//
// A ".scb" chunk file is one ChunkHeader followed by N fixed-size Records. Chunks are
// appended while open and may be copied while still growing, so readers must ignore a
// truncated trailing record. Everything is little-endian and packed; the Python reader in
// tools/blackbox.py mirrors these layouts byte for byte and tests/blackbox_test.cpp checks
// the sizes at compile time.
//
// This header has no OpenVR or logging dependency on purpose so that tools can include it.

#include <cstdint>
#include <cstring>

namespace spacecal::blackbox {

constexpr uint32_t k_MAGIC = 0x42424353; // "SCBB" as little-endian u32
constexpr uint16_t k_FORMAT_VERSION = 2; // 2: CALIBRATION, DEVICE_STATE, LIFECYCLE records; POSE carries the full tracking result in `code`
constexpr uint16_t k_RECORD_SIZE = 80;
constexpr uint16_t k_HEADER_SIZE = 64;
constexpr size_t k_TEXT_PAYLOAD_SIZE = 64; // bytes of a DEVICE/TEXT record that carry characters
constexpr const char* k_CHUNK_EXTENSION = ".scb";

enum class RecordType : uint8_t {
    NONE = 0,
    POSE = 1, // raw DriverPose_t as delivered by the vendor driver
    WORLD_FROM_DRIVER = 2, // the device's WorldFromDriver transform (written when it changes)
    APPLIED = 3, // the correction the spacecal driver applied to this device (written when it changes)
    TRUST = 4, // trust state transition of a device (section 6)
    MARKER = 5, // an event marker (F-key, overlay button, triggers, auto)
    DEVICE = 6, // device info text: "class=..;sys=..;model=..;serial=.." possibly spanning several records
    TEXT = 7, // free text (marker folder name, notes), several records per string
    SESSION = 8, // written once per chunk: recorder statistics
    CALIBRATION = 9, // one calibration attempt in the overlay (solve, rejection, skip or direct correction)
    DEVICE_STATE = 10, // a device connected or disconnected
    LIFECYCLE = 11, // recorder start/stop, overlay connected, recorder settings, archive state
};

enum class MarkerSource : uint8_t {
    UNKNOWN = 0,
    HOTKEY = 1,
    OVERLAY_BUTTON = 2,
    TRIGGERS = 3,
    AUTO = 4,
    SYNTHETIC = 5,
    SHUTDOWN = 6,
};

enum class TextKind : uint8_t {
    DEVICE_INFO = 0, // payload: key=value;... for one device
    MARKER_FOLDER = 1, // payload: event folder name the marker belongs to
    NOTE = 2,
    MARKER_NOTE = 3, // payload: "<t of the marker> <the user's note>" (guard/marker_notes.h), written when he
                     // saves it, possibly minutes after the marker; the latest one for a marker wins
};

// CALIBRATION: what started the attempt (Record::a)
enum class CalibrationTrigger : uint8_t {
    UNKNOWN = 0,
    STANDARD = 1, // a one-shot calibration finished sampling
    CONTINUOUS = 2, // periodic continuous solve (the sample buffer filled up)
    STARTUP = 3, // first continuous solve after the overlay started (forced)
    TRIGGER_HOLD = 4, // both triggers held (forced)
    MANUAL = 5, // a settings change in the UI forced the next solve
    PLAYSPACE_JUMP = 6, // trust layer: every lighthouse device moved together, the fitted delta was applied
    WORLD_FROM_DRIVER_JUMP = 7, // upstream auto-fix after a WorldFromDriver change
    HEAD_MOUNT_FIX = 8, // trust layer: the head tracker sat steadily away from its place on the headset, the calibration slid back (CORRECTED)
};

// CALIBRATION: what came of it (low nibble of Record::b)
enum class CalibrationOutcome : uint8_t {
    REJECTED = 0, // solved, failed a check (code = the overlay's CalibrationError); the active calibration stays
    APPLIED = 1, // solved and passed every check
    FORCED = 2, // applied although a check failed, because the solve was forced
    SKIPPED = 3, // nothing solved: the target device is not trusted
    CORRECTED = 4, // nothing solved: the active calibration was shifted by a measured jump
};
constexpr uint8_t k_CALIB_OUTCOME_MASK = 0x0F;
constexpr uint8_t k_CALIB_FLAG_CONTINUOUS = 1 << 4; // Record::b
constexpr uint8_t k_CALIB_FLAG_RELATIVE = 1 << 5; // Record::b

// LIFECYCLE: what happened (Record::a)
enum class LifecycleKind : uint8_t {
    NONE = 0,
    RECORDER_START = 1, // code = format version, f0 = 1 when this recorder continues its session's chunks (overlay restart)
    RECORDER_STOP = 2,
    OVERLAY_CONNECTED = 3, // code = IPC protocol version of the overlay
    PARAMS = 4, // v = enabled, live window [s], pre [s], post [s], chunk [s], keep-all, archive cap [MB]
    ARCHIVE_CAP_REACHED = 5, // f0 = archive size [MB]; expired chunks are deleted again from here on
    LEFTOVERS = 6, // chunks of an earlier session found at start: code = count, v[0] = 1 archived / 0 deleted
    RING_GAP = 7, // f0 = records the driver's pose ring overwrote before the recorder read them (overlay closed too long)
};

// POSE flag bits (Record::b)
constexpr uint8_t k_POSE_FLAG_VALID = 1 << 0;
constexpr uint8_t k_POSE_FLAG_CONNECTED = 1 << 1;
constexpr uint8_t k_POSE_FLAG_DRIFT_IN_YAW = 1 << 2;
constexpr uint8_t k_POSE_FLAG_HEAD_MODEL = 1 << 3;

#pragma pack(push, 1)
struct ChunkHeader {
    uint32_t magic = k_MAGIC;
    uint16_t formatVersion = k_FORMAT_VERSION;
    uint16_t recordSize = k_RECORD_SIZE;
    uint16_t headerSize = k_HEADER_SIZE;
    uint16_t reserved0 = 0;
    uint32_t chunkIndex = 0; // running index within the recorder session
    double monoStart = 0.0; // steady-clock seconds when this chunk was opened
    double unixStart = 0.0; // system-clock seconds (UTC) at the same instant; unix = unixStart + (t - monoStart)
    uint32_t sessionId = 0; // random per driver start, ties chunks of one SteamVR session together
    uint32_t chunkSeconds = 60; // nominal chunk length
    char version[24] = {}; // spacecal version string, NUL padded
};
static_assert(sizeof(ChunkHeader) == k_HEADER_SIZE, "ChunkHeader must be 64 bytes");

// One record. Field meaning depends on `type`:
//
//   POSE:              device = index, a = vr::ETrackingResult, b = flags, f0 = poseTimeOffset,
//                      v = pos[3], quat[w,x,y,z], vel[3], angVel[3]
//   WORLD_FROM_DRIVER: device = index, v = trans[3], quat[w,x,y,z]
//   APPLIED:           device = index, a = blend delta size (0 tiny, 1 small, 2 large), f0 = scale,
//                      v = trans[3], quat[w,x,y,z]
//   TRUST:             device = index, a = old state, b = new state, code = reason, f0 = residual [m],
//                      v[0..2] = the numbers that triggered the transition
//   MARKER:            a = MarkerSource, code = label id
//   DEVICE / TEXT:     device = index (255 if n/a), a = TextKind, b = sequence number, code = total
//                      sequence count, text = up to 64 chars, NUL padded but not necessarily
//                      NUL terminated (see Record::text())
//   SESSION:           v[0] = records dropped since last SESSION record, v[1] = ring high-water mark,
//                      v[2] = flush interval [s]
//   (format 2)
//   Written by the driver into the pose ring (pose_ring.h): POSE, WORLD_FROM_DRIVER, APPLIED,
//   DEVICE_STATE. Everything else is written by the overlay's recorder.
//   POSE:              code = vr::ETrackingResult in full (`a` wraps above 255, e.g. Fallback_RotationOnly)
//   WORLD_FROM_DRIVER, APPLIED: b = k_RECORD_CARRIED when the overlay's recorder repeated the transform
//                      in effect at the start of a chunk (every chunk carries each device's last one,
//                      so it reads on its own); such a record is not a change
//   CALIBRATION:       device = target index, reserved = reference index, a = CalibrationTrigger,
//                      b = CalibrationOutcome | k_CALIB_FLAG_*, code = the overlay's CalibrationError,
//                      f0 = RMS of this solve [m] (NaN when nothing was solved),
//                      v[0] = RMS of the active calibration on the same samples [m] (-1 = none),
//                      v[1] = samples used, v[2] = translation change vs the active calibration [m],
//                      v[3] = rotation change vs the active calibration [deg], v[4..6] = solved translation [m],
//                      v[7..10] = solved rotation w,x,y,z, v[11] = axis variance, v[12] = previous axis variance
//   DEVICE_STATE:      device = index, a = 1 connected / 0 disconnected, b = previous (255 = first seen),
//                      code = vr::ETrackingResult
//   LIFECYCLE:         a = LifecycleKind, code / f0 / v as documented on the enum
// WORLD_FROM_DRIVER / APPLIED: `b` of a record repeated at the start of a chunk (see above)
constexpr uint8_t k_RECORD_CARRIED = 1;

struct Record {
    double t = 0.0; // steady-clock seconds at record time
    uint8_t type = 0;
    uint8_t device = 255;
    uint8_t a = 0;
    uint8_t b = 0;
    uint16_t code = 0;
    uint16_t reserved = 0;
    float f0 = 0.0f;
    float v[13] = {};
    uint8_t pad[8] = {};

    // The text payload of DEVICE/TEXT records overlays f0 + v + pad (64 bytes).
    char* text() { return reinterpret_cast<char*>(&f0); }
    const char* text() const { return reinterpret_cast<const char*>(&f0); }
};
static_assert(sizeof(Record) == k_RECORD_SIZE, "Record must be 80 bytes");
static_assert(sizeof(float) + sizeof(float) * 13 + 8 == k_TEXT_PAYLOAD_SIZE, "text payload spans f0, v and pad");
static_assert(k_RECORD_SIZE - 16 == k_TEXT_PAYLOAD_SIZE, "text payload starts at byte 16");
#pragma pack(pop)

inline const char* recordTypeName(RecordType t)
{
    switch (t) {
    case RecordType::POSE: return "POSE";
    case RecordType::WORLD_FROM_DRIVER: return "WORLD_FROM_DRIVER";
    case RecordType::APPLIED: return "APPLIED";
    case RecordType::TRUST: return "TRUST";
    case RecordType::MARKER: return "MARKER";
    case RecordType::DEVICE: return "DEVICE";
    case RecordType::TEXT: return "TEXT";
    case RecordType::SESSION: return "SESSION";
    case RecordType::CALIBRATION: return "CALIBRATION";
    case RecordType::DEVICE_STATE: return "DEVICE_STATE";
    case RecordType::LIFECYCLE: return "LIFECYCLE";
    default: return "NONE";
    }
}

inline const char* markerSourceName(MarkerSource s)
{
    switch (s) {
    case MarkerSource::HOTKEY: return "hotkey";
    case MarkerSource::OVERLAY_BUTTON: return "overlay";
    case MarkerSource::TRIGGERS: return "triggers";
    case MarkerSource::AUTO: return "auto";
    case MarkerSource::SYNTHETIC: return "synthetic";
    case MarkerSource::SHUTDOWN: return "shutdown";
    default: return "unknown";
    }
}

inline const char* calibrationTriggerName(CalibrationTrigger t)
{
    switch (t) {
    case CalibrationTrigger::STANDARD: return "standard";
    case CalibrationTrigger::CONTINUOUS: return "continuous";
    case CalibrationTrigger::STARTUP: return "startup";
    case CalibrationTrigger::TRIGGER_HOLD: return "trigger_hold";
    case CalibrationTrigger::MANUAL: return "manual";
    case CalibrationTrigger::PLAYSPACE_JUMP: return "playspace_jump";
    case CalibrationTrigger::WORLD_FROM_DRIVER_JUMP: return "world_from_driver_jump";
    case CalibrationTrigger::HEAD_MOUNT_FIX: return "head_mount_fix";
    default: return "unknown";
    }
}

inline const char* calibrationOutcomeName(CalibrationOutcome o)
{
    switch (o) {
    case CalibrationOutcome::REJECTED: return "rejected";
    case CalibrationOutcome::APPLIED: return "applied";
    case CalibrationOutcome::FORCED: return "forced";
    case CalibrationOutcome::SKIPPED: return "skipped";
    case CalibrationOutcome::CORRECTED: return "corrected";
    default: return "unknown";
    }
}

inline const char* lifecycleKindName(LifecycleKind k)
{
    switch (k) {
    case LifecycleKind::RECORDER_START: return "recorder_start";
    case LifecycleKind::RECORDER_STOP: return "recorder_stop";
    case LifecycleKind::OVERLAY_CONNECTED: return "overlay_connected";
    case LifecycleKind::PARAMS: return "params";
    case LifecycleKind::ARCHIVE_CAP_REACHED: return "archive_cap_reached";
    case LifecycleKind::LEFTOVERS: return "leftovers";
    case LifecycleKind::RING_GAP: return "ring_gap";
    default: return "none";
    }
}

} // namespace spacecal::blackbox
