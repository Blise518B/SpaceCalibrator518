#pragma once

// Black-box pose recorder (fork addition, docs/DESIGN.md section 4).
//
// Lives in the overlay since the thin-driver split (docs/DESIGN.md section 13): the driver only
// publishes raw records into the shared-memory pose ring (pose_ring.h), and this recorder pulls
// them (StartOptions::pull) together with the overlay's own records (trust, calibration,
// markers, device info) pushed into a mutex-protected in-process ring. One writer thread
// flushes both to rolling chunk files every 250 ms and promotes the chunks around an event
// marker into a saved event folder. An overlay restart continues the same session: the new
// recorder adopts the session's live chunks and the ring cursor resumes without a gap.
//
// Layout on disk (root = %APPDATA%/space-calibrator/blackbox):
//   live/<sessionId>_<chunkIndex>.scb        rolling window, oldest chunks deleted (or archived, see keepAll)
//   events/<folder>/<chunk files>.scb        promoted copies, plus driver.json and the driver log
//   sessions/<local start>_<sessionId>/      keepAll: every chunk of a SteamVR session, NTFS-compressed,
//                                            never deleted by the recorder
//
// Nothing is deleted or archived before the overlay has sent the recorder settings (or
// k_POLICY_WAIT_SECONDS after start when it never does), so a keepAll session loses nothing to
// the default settings the driver starts with.
//
// The class has no OpenVR dependency so tests can drive it directly.

#include "blackbox_format.h"
#include "blackbox_records.h"
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace spacecal::blackbox {

struct Params {
    bool enabled = true;
    uint32_t liveWindowSeconds = 600;
    uint32_t preSeconds = 300;
    uint32_t postSeconds = 300;
    uint32_t chunkSeconds = 60;
    uint32_t maxEventsPerSession = 50;
    bool keepAll = false; // archive expired chunks into sessions/ instead of deleting them
    uint32_t archiveMaxMb = 0; // stop archiving above this size on disk (0 = no cap); the archive is never pruned
};

struct Stats {
    bool running = false;
    bool enabled = false;
    uint64_t recordsWritten = 0;
    uint64_t recordsDropped = 0;
    uint32_t chunksOnDisk = 0;
    uint64_t bytesOnDisk = 0;
    uint32_t pendingEvents = 0;
    uint32_t eventsSaved = 0;
    double lastMarkerMono = 0.0;
    bool keepAll = false;
    uint32_t archivedChunks = 0; // this session
    uint64_t archiveBytes = 0; // whole archive on disk (after compression)
    bool archiveCapReached = false;
};

// Optional log sink (the overlay points this at its logger; tests leave it null).
extern void (*g_logFn)(const char* message);

struct StartOptions {
    double flushIntervalSec = 0.25;
    bool waitForPolicy = false; // the params are only defaults until setParams() arrives
    uint32_t sessionId = 0; // 0 = a new random session; otherwise continue this session's live chunks
    double sessionStartUnix = 0.0; // names the archive folder; 0 = now
    // writer thread: appends records from outside (the driver's pose ring) before every flush;
    // newChunk is true on the first pull into a chunk opened at chunkStart
    std::function<void(std::vector<Record>&, bool newChunk, double chunkStart)> pull;
    // writer thread: called once everything pulled so far is on disk (the ring cursor may advance)
    std::function<void()> committed;
};

class BlackBox {
public:
    BlackBox() = default;
    ~BlackBox();
    BlackBox(const BlackBox&) = delete;
    BlackBox& operator=(const BlackBox&) = delete;

    // Creates rootDir/live and rootDir/events and starts the writer thread. flushIntervalSec is
    // exposed for tests; the driver uses the default. waitForPolicy: the params are only defaults,
    // so nothing is deleted or archived until setParams() arrives (the driver); false = the params
    // passed here are final (tests).
    bool start(const std::filesystem::path& rootDir, const char* versionString, const Params& params = {}, double flushIntervalSec = 0.25, bool waitForPolicy = false);
    bool start(const std::filesystem::path& rootDir, const char* versionString, const Params& params, const StartOptions& options);
    // Flushes, closes the open chunk, promotes it to every pending event and joins the thread.
    void stop();
    [[nodiscard]] bool isRunning() const { return m_running.load(std::memory_order_acquire); }

    void setParams(const Params& params);
    Params getParams() const;
    Stats getStats() const;
    const std::filesystem::path& rootDir() const { return m_root; }

    // ---- hot path, any thread ------------------------------------------------------------
    void recordWorldFromDriver(uint8_t device, const double trans[3], const double quatWxyz[4], double t);
    void recordApplied(uint8_t device, const double trans[3], const double quatWxyz[4], uint8_t deltaSize, double scale, double t);
    void recordTrust(uint8_t device, uint8_t oldState, uint8_t newState, uint16_t reason, float residual, const float triggers[3], double t);
    // Splits text into as many DEVICE/TEXT records as needed (each carries 64 chars).
    void recordText(RecordType type, uint8_t device, TextKind kind, const std::string& text, double t);
    void recordCalibration(uint8_t targetDevice, uint8_t referenceDevice, uint8_t trigger, uint8_t outcome, uint16_t error, float rms, const float values[13], double t);
    void recordLifecycle(LifecycleKind kind, uint16_t code, float f0, const float* values, int count, double t);
    void push(const Record& record);
    // records lost outside (pose ring overruns): counted like ring drops, reported in SESSION records
    void noteDropped(uint64_t count);

    // ---- control, any thread ------------------------------------------------------------
    // Sets a marker at the current time. `folder` names the event folder (no path separators);
    // a marker for a folder that is already pending extends that event's window instead of
    // creating a new one. Returns false when the recorder is stopped, disabled or the session
    // event cap is reached.
    bool markEvent(MarkerSource source, uint32_t label, const std::string& folder);
    // Same, but with an explicit marker time (tests, and markers relayed with their own time).
    bool markEventAt(MarkerSource source, uint32_t label, const std::string& folder, double tMark);

    // True once per chunk: the driver should record DEVICE info for every connected device.
    bool takeDeviceInfoRequest();
    void requestDeviceInfo() { m_deviceInfoRequested = true; }
    [[nodiscard]] const std::filesystem::path& sessionsDir() const { return m_sessionsDir; }
    [[nodiscard]] const std::filesystem::path& sessionArchiveDir() const { return m_sessionArchiveDir; }

    // Called by the writer at chunk boundaries; exposed so tests can force a rotation.
    void forceRotate();

private:
    struct ChunkInfo {
        uint32_t index = 0;
        std::filesystem::path path;
        double monoStart = 0.0;
        double monoEnd = 0.0; // set when closed
        bool open = false;
        uint64_t bytes = 0;
    };

    struct PendingEvent {
        std::string folder;
        MarkerSource source = MarkerSource::UNKNOWN;
        uint32_t label = 0;
        double tMark = 0.0;
        double unixMark = 0.0;
        double tStart = 0.0;
        double tEnd = 0.0;
        double lastOpenChunkCopy = 0.0;
        std::vector<uint32_t> copiedChunks;
        std::vector<std::string> chunkFiles;
        bool folderCreated = false;
    };

    void writerMain();
    void flushRing(std::vector<Record>& scratch);
    void writeRecords(const std::vector<Record>& records);
    bool openChunk(double t);
    void closeChunk(double t);
    void applyRetention(double t);
    void processEvents(double t, bool shuttingDown);
    bool copyChunkToEvent(const ChunkInfo& chunk, PendingEvent& ev, bool overwrite);
    void finalizeEvent(PendingEvent& ev, bool incomplete);
    std::filesystem::path eventDir(const PendingEvent& ev) const;
    bool chunkOverlaps(const ChunkInfo& chunk, const PendingEvent& ev) const;
    // keepAll archive (writer thread only)
    bool policyKnown(double t) const;
    bool archiveCapReached(const Params& params, double t);
    bool archiveFile(const std::filesystem::path& file, const std::filesystem::path& destDir);
    void resolveLeftovers(double t, const Params& params);
    void compressOneQueued();
    void scanArchive();
    // external records (pose ring)
    void pullExternal(std::vector<Record>& scratch);
    void commitExternal();

private:
    std::filesystem::path m_root;
    std::filesystem::path m_liveDir;
    std::filesystem::path m_eventsDir;
    std::filesystem::path m_sessionsDir;
    std::filesystem::path m_sessionArchiveDir; // sessions/<local start>_<sessionId>
    std::string m_version;
    StartOptions m_options;
    uint32_t m_sessionId = 0;
    double m_flushInterval = 0.25;

    // ring (producers + writer)
    mutable std::mutex m_ringMutex;
    std::vector<Record> m_ring;
    size_t m_ringHead = 0; // next write
    size_t m_ringCount = 0;
    uint64_t m_droppedSinceSession = 0;
    size_t m_ringHighWater = 0;

    // params + events (control + writer)
    mutable std::mutex m_ctrlMutex;
    Params m_params;
    std::vector<PendingEvent> m_pending;
    std::map<std::string, PendingEvent> m_finalized; // folder -> what was saved, for merged markers
    uint32_t m_eventsCreated = 0;
    std::atomic<bool> m_deviceInfoRequested { false };

    // writer-only state
    std::vector<ChunkInfo> m_chunks; // live chunks, oldest first (the last one may be open)
    FILE* m_file = nullptr;
    uint32_t m_nextChunkIndex = 0;
    bool m_newChunk = false; // writer thread: no pull has gone into the open chunk yet
    std::atomic<bool> m_rotateRequested { false };
    double m_lastOpenFailLog = -1e9;
    std::vector<std::filesystem::path> m_leftovers; // chunks of an earlier session found in live/ at start
    std::deque<std::filesystem::path> m_toCompress; // archived files waiting for NTFS compression
    double m_startMono = 0.0;
    double m_lastArchiveFailLog = -1e9;
    bool m_waitForPolicy = false;

    // thread
    std::thread m_thread;
    std::atomic<bool> m_running { false };
    std::atomic<bool> m_stopRequested { false };
    std::condition_variable m_cv;
    std::mutex m_cvMutex;

    // stats
    std::atomic<uint64_t> m_recordsWritten { 0 };
    std::atomic<uint64_t> m_recordsDropped { 0 };
    std::atomic<uint32_t> m_eventsSaved { 0 };
    std::atomic<uint32_t> m_chunksOnDisk { 0 };
    std::atomic<uint64_t> m_bytesOnDisk { 0 };
    std::atomic<double> m_lastMarkerMono { 0.0 };
    std::atomic<bool> m_paramsReceived { false };
    std::atomic<uint32_t> m_archivedChunks { 0 };
    std::atomic<uint64_t> m_archiveBytes { 0 };
    std::atomic<bool> m_archiveCapReached { false };
};

} // namespace spacecal::blackbox
