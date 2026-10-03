#include "black_box.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstring>
#include <ctime>
#include <limits>
#include <random>
#include <system_error>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winioctl.h>
#endif

namespace spacecal::blackbox {

// Optional log sink; the driver points this at its logger, tests and tools leave it empty.
void (*g_logFn)(const char* message) = nullptr;

namespace {
    constexpr size_t k_RING_CAPACITY = 65536; // 5.2 MB; > 6 s at 10 000 records/s, covers a slow chunk copy
    constexpr double k_OPEN_CHUNK_COPY_INTERVAL = 10.0; // s between re-copies of a growing chunk into an event
    constexpr int k_MAX_RECORDS_PER_TEXT = 8; // 512 chars is plenty for device info
    constexpr double k_POLICY_WAIT_SECONDS = 1800.0; // no settings from the overlay: fall back to the defaults after 30 min
    constexpr double k_ARCHIVE_FAIL_LOG_INTERVAL = 30.0;

    std::string localStamp(double unixSeconds)
    {
        const std::time_t tt = static_cast<std::time_t>(unixSeconds);
        std::tm tmv {};
#if defined(_WIN32)
        localtime_s(&tmv, &tt);
#else
        localtime_r(&tt, &tmv);
#endif
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tmv);
        return buf;
    }

    std::string hex8(uint32_t v)
    {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%08x", v);
        return buf;
    }

    // bytes the file really occupies (NTFS compression aware)
    uint64_t sizeOnDisk(const std::filesystem::path& p)
    {
#if defined(_WIN32)
        DWORD hi = 0;
        const DWORD lo = GetCompressedFileSizeW(p.c_str(), &hi);
        if (!(lo == INVALID_FILE_SIZE && GetLastError() != NO_ERROR))
            return (static_cast<uint64_t>(hi) << 32) | lo;
#endif
        std::error_code ec;
        const auto n = std::filesystem::file_size(p, ec);
        return ec ? 0 : static_cast<uint64_t>(n);
    }

    bool isCompressed(const std::filesystem::path& p)
    {
#if defined(_WIN32)
        const DWORD attrs = GetFileAttributesW(p.c_str());
        return attrs != INVALID_FILE_ATTRIBUTES && (attrs & FILE_ATTRIBUTE_COMPRESSED) != 0;
#else
        (void)p;
        return true; // no transparent compression elsewhere: nothing to do
#endif
    }

    // NTFS (LZNT1) compression in place, about 1.5x on pose data and transparent to every reader;
    // a no-op on file systems without it
    bool compressInPlace(const std::filesystem::path& p, bool isDirectory)
    {
#if defined(_WIN32)
        HANDLE h = CreateFileW(p.c_str(), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_EXISTING,
            isDirectory ? FILE_FLAG_BACKUP_SEMANTICS : 0, nullptr);
        if (h == INVALID_HANDLE_VALUE)
            return false;
        USHORT format = COMPRESSION_FORMAT_DEFAULT;
        DWORD returned = 0;
        const BOOL ok = DeviceIoControl(h, FSCTL_SET_COMPRESSION, &format, sizeof(format), nullptr, 0, &returned, nullptr);
        CloseHandle(h);
        return ok != FALSE;
#else
        (void)p;
        (void)isDirectory;
        return false;
#endif
    }

    // header and the time of the last complete record (monoStart when there is none)
    bool readChunkSpan(const std::filesystem::path& p, ChunkHeader& header, double& lastT)
    {
#if defined(_WIN32)
        FILE* f = _wfopen(p.c_str(), L"rb");
#else
        FILE* f = std::fopen(p.c_str(), "rb");
#endif
        if (!f)
            return false;
        bool ok = std::fread(&header, sizeof(header), 1, f) == 1 && header.magic == k_MAGIC && header.recordSize == k_RECORD_SIZE;
        lastT = header.monoStart;
        if (ok) {
            std::error_code ec;
            const uint64_t size = static_cast<uint64_t>(std::filesystem::file_size(p, ec));
            const uint64_t n = size > header.headerSize ? (size - header.headerSize) / k_RECORD_SIZE : 0;
            if (n > 0) {
                Record r;
                if (std::fseek(f, static_cast<long>(header.headerSize + (n - 1) * k_RECORD_SIZE), SEEK_SET) == 0 && std::fread(&r, sizeof(r), 1, f) == 1)
                    lastT = r.t;
            }
        }
        std::fclose(f);
        return ok;
    }

    // unix start and session id from a chunk header; false when the file is not a readable chunk
    bool readChunkIdentity(const std::filesystem::path& p, double& unixStart, uint32_t& sessionId)
    {
#if defined(_WIN32)
        FILE* f = _wfopen(p.c_str(), L"rb");
#else
        FILE* f = std::fopen(p.c_str(), "rb");
#endif
        if (!f)
            return false;
        ChunkHeader h;
        const size_t n = std::fread(&h, sizeof(h), 1, f);
        std::fclose(f);
        if (n != 1 || h.magic != k_MAGIC)
            return false;
        unixStart = h.unixStart;
        sessionId = h.sessionId;
        return true;
    }

    void logMessage(const char* fmt, ...)
    {
        if (!g_logFn)
            return;
        char buf[1024];
        va_list args;
        va_start(args, fmt);
        std::vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        g_logFn(buf);
    }

    std::string jsonEscape(const std::string& s)
    {
        std::string out;
        out.reserve(s.size() + 8);
        for (char c : s) {
            switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char b[8];
                    std::snprintf(b, sizeof(b), "\\u%04x", c);
                    out += b;
                } else {
                    out += c;
                }
            }
        }
        return out;
    }

    bool isSafeFolderName(const std::string& name)
    {
        if (name.empty() || name.size() > 63 || name == "." || name == "..")
            return false;
        for (char c : name) {
            const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
            if (!ok)
                return false;
        }
        return true;
    }
}

BlackBox::~BlackBox()
{
    // Cleanup() stops explicitly. If we get here still running (DLL unload without Cleanup), do
    // not join: the writer may already be gone and joining under the loader lock hangs vrserver.
    if (m_running.load()) {
        m_stopRequested = true;
        m_cv.notify_all();
        if (m_thread.joinable())
            m_thread.detach();
        m_running = false;
    }
}

bool BlackBox::start(const std::filesystem::path& rootDir, const char* versionString, const Params& params, double flushIntervalSec, bool waitForPolicy)
{
    StartOptions options;
    options.flushIntervalSec = flushIntervalSec;
    options.waitForPolicy = waitForPolicy;
    return start(rootDir, versionString, params, options);
}

bool BlackBox::start(const std::filesystem::path& rootDir, const char* versionString, const Params& params, const StartOptions& options)
{
    if (m_running.load())
        return true;

    m_root = rootDir;
    m_liveDir = rootDir / "live";
    m_eventsDir = rootDir / "events";
    m_sessionsDir = rootDir / "sessions";
    m_version = versionString ? versionString : "";
    m_options = options;
    m_flushInterval = options.flushIntervalSec > 0.01 ? options.flushIntervalSec : 0.25;
    {
        std::lock_guard<std::mutex> lock(m_ctrlMutex);
        m_params = params;
        m_pending.clear();
        m_eventsCreated = 0;
    }

    std::error_code ec;
    std::filesystem::create_directories(m_liveDir, ec);
    std::filesystem::create_directories(m_eventsDir, ec);
    if (ec) {
        logMessage("BlackBox: cannot create %s (%s)", m_root.string().c_str(), ec.message().c_str());
        return false;
    }

    if (options.sessionId != 0) {
        m_sessionId = options.sessionId;
    } else {
        std::random_device rd;
        m_sessionId = static_cast<uint32_t>(rd()) ^ static_cast<uint32_t>(std::time(nullptr));
        if (m_sessionId == 0)
            m_sessionId = 1;
    }
    const double sessionStartUnix = options.sessionStartUnix > 0.0 ? options.sessionStartUnix : unixNow();
    m_sessionArchiveDir = m_sessionsDir / (localStamp(sessionStartUnix) + "_" + hex8(m_sessionId));
    m_waitForPolicy = options.waitForPolicy;
    m_paramsReceived = !options.waitForPolicy;
    m_startMono = monoNow();
    m_toCompress.clear();
    m_archivedChunks = 0;
    m_archiveBytes = 0;
    m_archiveCapReached = false;
    m_lastArchiveFailLog = -1e9;

    {
        std::lock_guard<std::mutex> lock(m_ringMutex);
        m_ring.assign(k_RING_CAPACITY, Record {});
        m_ringHead = 0;
        m_ringCount = 0;
        m_ringHighWater = 0;
        m_droppedSinceSession = 0;
    }

    m_chunks.clear();
    m_file = nullptr;
    m_nextChunkIndex = 0;
    m_rotateRequested = false;
    m_recordsWritten = 0;
    m_recordsDropped = 0;
    m_eventsSaved = 0;
    m_chunksOnDisk = 0;
    m_bytesOnDisk = 0;
    m_lastMarkerMono = 0.0;
    m_deviceInfoRequested = true;

    // live chunks: this session's own (an earlier overlay run recorded them) are continued, the
    // rest belong to earlier sessions (SteamVR was restarted, or crashed): archived with keepAll,
    // deleted otherwise, once the settings are known (resolveLeftovers, writer thread)
    m_leftovers.clear();
    for (auto& entry : std::filesystem::directory_iterator(m_liveDir, ec)) {
        std::error_code ec2;
        if (!entry.is_regular_file(ec2) || entry.path().extension() != k_CHUNK_EXTENSION)
            continue;
        ChunkHeader h;
        double lastT = 0.0;
        if (options.sessionId != 0 && readChunkSpan(entry.path(), h, lastT) && h.sessionId == m_sessionId) {
            ChunkInfo c;
            c.index = h.chunkIndex;
            c.path = entry.path();
            c.monoStart = h.monoStart;
            c.monoEnd = std::max(lastT, h.monoStart);
            c.open = false;
            c.bytes = static_cast<uint64_t>(std::filesystem::file_size(entry.path(), ec2));
            m_chunks.push_back(c);
            m_bytesOnDisk += c.bytes;
        } else {
            m_leftovers.push_back(entry.path());
        }
    }
    std::sort(m_chunks.begin(), m_chunks.end(), [](const ChunkInfo& a, const ChunkInfo& b) { return a.index < b.index; });
    if (!m_chunks.empty())
        m_nextChunkIndex = m_chunks.back().index + 1;
    m_chunksOnDisk = static_cast<uint32_t>(m_chunks.size());
    const size_t adopted = m_chunks.size();

    m_stopRequested = false;
    m_running = true;
    m_thread = std::thread([this] { writerMain(); });
    if (adopted > 0)
        logMessage("BlackBox: recording to %s (session %08x, continuing %zu chunks of this session)", m_root.string().c_str(), m_sessionId, adopted);
    else
        logMessage("BlackBox: recording to %s (session %08x)", m_root.string().c_str(), m_sessionId);
    recordLifecycle(LifecycleKind::RECORDER_START, k_FORMAT_VERSION, adopted > 0 ? 1.0f : 0.0f, nullptr, 0, monoNow());
    return true;
}

void BlackBox::stop()
{
    if (!m_running.load())
        return;
    recordLifecycle(LifecycleKind::RECORDER_STOP, 0, 0.0f, nullptr, 0, monoNow());
    m_stopRequested = true;
    m_cv.notify_all();
    if (m_thread.joinable())
        m_thread.join();
    m_running = false;
    logMessage("BlackBox: stopped (%llu records written, %llu dropped)", (unsigned long long)m_recordsWritten.load(), (unsigned long long)m_recordsDropped.load());
}

void BlackBox::setParams(const Params& params)
{
    Params applied;
    {
        std::lock_guard<std::mutex> lock(m_ctrlMutex);
        m_params = params;
        if (m_params.chunkSeconds < 5)
            m_params.chunkSeconds = 5;
        if (m_params.liveWindowSeconds < m_params.chunkSeconds * 2)
            m_params.liveWindowSeconds = m_params.chunkSeconds * 2;
        if (m_params.preSeconds > m_params.liveWindowSeconds)
            m_params.preSeconds = m_params.liveWindowSeconds;
        applied = m_params;
    }
    m_paramsReceived = true;
    const float v[7] = { applied.enabled ? 1.0f : 0.0f, static_cast<float>(applied.liveWindowSeconds), static_cast<float>(applied.preSeconds),
        static_cast<float>(applied.postSeconds), static_cast<float>(applied.chunkSeconds), applied.keepAll ? 1.0f : 0.0f,
        static_cast<float>(applied.archiveMaxMb) };
    recordLifecycle(LifecycleKind::PARAMS, 0, 0.0f, v, 7, monoNow());
    m_cv.notify_all();
}

Params BlackBox::getParams() const
{
    std::lock_guard<std::mutex> lock(m_ctrlMutex);
    return m_params;
}

Stats BlackBox::getStats() const
{
    Stats s;
    s.running = m_running.load();
    s.recordsWritten = m_recordsWritten.load();
    s.recordsDropped = m_recordsDropped.load();
    s.chunksOnDisk = m_chunksOnDisk.load();
    s.bytesOnDisk = m_bytesOnDisk.load();
    s.eventsSaved = m_eventsSaved.load();
    s.lastMarkerMono = m_lastMarkerMono.load();
    {
        std::lock_guard<std::mutex> lock(m_ctrlMutex);
        s.enabled = m_params.enabled;
        s.pendingEvents = static_cast<uint32_t>(m_pending.size());
        s.keepAll = m_params.keepAll;
    }
    s.archivedChunks = m_archivedChunks.load();
    s.archiveBytes = m_archiveBytes.load();
    s.archiveCapReached = m_archiveCapReached.load();
    return s;
}

// ---- hot path ----------------------------------------------------------------------------

void BlackBox::push(const Record& record)
{
    if (!m_running.load(std::memory_order_acquire))
        return;
    std::lock_guard<std::mutex> lock(m_ringMutex);
    if (m_ringCount >= m_ring.size()) {
        m_droppedSinceSession++;
        m_recordsDropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    m_ring[m_ringHead] = record;
    m_ringHead = (m_ringHead + 1) % m_ring.size();
    m_ringCount++;
    if (m_ringCount > m_ringHighWater)
        m_ringHighWater = m_ringCount;
}

void BlackBox::recordWorldFromDriver(uint8_t device, const double trans[3], const double quatWxyz[4], double t)
{
    push(makeWorldFromDriverRecord(device, trans, quatWxyz, t));
}

void BlackBox::recordApplied(uint8_t device, const double trans[3], const double quatWxyz[4], uint8_t deltaSize, double scale, double t)
{
    push(makeAppliedRecord(device, trans, quatWxyz, deltaSize, scale, t));
}

void BlackBox::recordTrust(uint8_t device, uint8_t oldState, uint8_t newState, uint16_t reason, float residual, const float triggers[3], double t)
{
    Record r;
    r.t = t;
    r.type = static_cast<uint8_t>(RecordType::TRUST);
    r.device = device;
    r.a = oldState;
    r.b = newState;
    r.code = reason;
    r.f0 = residual;
    if (triggers) {
        r.v[0] = triggers[0];
        r.v[1] = triggers[1];
        r.v[2] = triggers[2];
    }
    push(r);
}

void BlackBox::recordText(RecordType type, uint8_t device, TextKind kind, const std::string& text, double t)
{
    const size_t total = std::min<size_t>(k_MAX_RECORDS_PER_TEXT, (text.size() + k_TEXT_PAYLOAD_SIZE - 1) / k_TEXT_PAYLOAD_SIZE);
    if (total == 0) {
        Record r;
        r.t = t;
        r.type = static_cast<uint8_t>(type);
        r.device = device;
        r.a = static_cast<uint8_t>(kind);
        r.b = 0;
        r.code = 1;
        push(r);
        return;
    }
    for (size_t seq = 0; seq < total; seq++) {
        Record r;
        r.t = t;
        r.type = static_cast<uint8_t>(type);
        r.device = device;
        r.a = static_cast<uint8_t>(kind);
        r.b = static_cast<uint8_t>(seq);
        r.code = static_cast<uint16_t>(total);
        const size_t offset = seq * k_TEXT_PAYLOAD_SIZE;
        const size_t n = std::min(k_TEXT_PAYLOAD_SIZE, text.size() - offset);
        std::memcpy(r.text(), text.data() + offset, n);
        push(r);
    }
}

void BlackBox::noteDropped(uint64_t count)
{
    if (count == 0)
        return;
    std::lock_guard<std::mutex> lock(m_ringMutex);
    m_droppedSinceSession += count;
    m_recordsDropped.fetch_add(count, std::memory_order_relaxed);
}

void BlackBox::pullExternal(std::vector<Record>& scratch)
{
    if (!m_options.pull)
        return;
    const size_t before = scratch.size();
    const bool newChunk = m_newChunk;
    m_newChunk = false;
    m_options.pull(scratch, newChunk, m_chunks.empty() ? monoNow() : m_chunks.back().monoStart);
    if (scratch.size() == before)
        return;
    for (size_t i = before; i < scratch.size(); i++) {
        if (scratch[i].type == static_cast<uint8_t>(RecordType::DEVICE_STATE) && scratch[i].a == 1) {
            m_deviceInfoRequested = true; // model and serial of a device that just connected
            break;
        }
    }
    // records from the ring and from the overlay interleave in time; keep chunks ordered
    std::stable_sort(scratch.begin(), scratch.end(), [](const Record& a, const Record& b) { return a.t < b.t; });
}

void BlackBox::commitExternal()
{
    if (m_options.committed)
        m_options.committed();
}

void BlackBox::recordCalibration(uint8_t targetDevice, uint8_t referenceDevice, uint8_t trigger, uint8_t outcome, uint16_t error, float rms, const float values[13], double t)
{
    Record r;
    r.t = t;
    r.type = static_cast<uint8_t>(RecordType::CALIBRATION);
    r.device = targetDevice;
    r.reserved = referenceDevice;
    r.a = trigger;
    r.b = outcome;
    r.code = error;
    r.f0 = rms;
    if (values)
        std::memcpy(r.v, values, sizeof(r.v));
    push(r);
}

void BlackBox::recordLifecycle(LifecycleKind kind, uint16_t code, float f0, const float* values, int count, double t)
{
    Record r;
    r.t = t;
    r.type = static_cast<uint8_t>(RecordType::LIFECYCLE);
    r.a = static_cast<uint8_t>(kind);
    r.code = code;
    r.f0 = f0;
    for (int i = 0; values && i < count && i < 13; i++)
        r.v[i] = values[i];
    push(r);
}

// ---- control -----------------------------------------------------------------------------

bool BlackBox::markEvent(MarkerSource source, uint32_t label, const std::string& folder)
{
    return markEventAt(source, label, folder, monoNow());
}

bool BlackBox::markEventAt(MarkerSource source, uint32_t label, const std::string& folder, double tMark)
{
    if (!m_running.load())
        return false;
    if (!isSafeFolderName(folder)) {
        logMessage("BlackBox: refusing marker with unsafe folder name '%s'", folder.c_str());
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_ctrlMutex);
        if (!m_params.enabled)
            return false;

        const double unixMark = unixNow() - (monoNow() - tMark);
        auto it = std::find_if(m_pending.begin(), m_pending.end(), [&](const PendingEvent& e) { return e.folder == folder; });
        if (it != m_pending.end()) {
            it->tEnd = std::max(it->tEnd, tMark + m_params.postSeconds);
            it->tStart = std::min(it->tStart, tMark - m_params.preSeconds);
            logMessage("BlackBox: marker (%s) extends event %s", markerSourceName(source), folder.c_str());
        } else {
            if (m_eventsCreated >= m_params.maxEventsPerSession) {
                logMessage("BlackBox: event cap (%u) reached, marker ignored", m_params.maxEventsPerSession);
                return false;
            }
            PendingEvent ev;
            ev.folder = folder;
            ev.source = source;
            ev.label = label;
            ev.tMark = tMark;
            ev.unixMark = unixMark;
            ev.tStart = tMark - m_params.preSeconds;
            ev.tEnd = tMark + m_params.postSeconds;
            auto prev = m_finalized.find(folder);
            if (prev != m_finalized.end()) {
                // the overlay merged this marker into an event that was already finalised: keep its files listed
                ev.chunkFiles = prev->second.chunkFiles;
                ev.copiedChunks = prev->second.copiedChunks;
                ev.tMark = prev->second.tMark;
                ev.unixMark = prev->second.unixMark;
                ev.tStart = std::min(ev.tStart, prev->second.tStart);
                ev.folderCreated = true;
            }
            m_pending.push_back(std::move(ev));
            m_eventsCreated++;
            logMessage("BlackBox: marker (%s) -> new event %s", markerSourceName(source), folder.c_str());
        }
    }

    Record r;
    r.t = tMark;
    r.type = static_cast<uint8_t>(RecordType::MARKER);
    r.a = static_cast<uint8_t>(source);
    r.code = static_cast<uint16_t>(label & 0xFFFF);
    push(r);
    recordText(RecordType::TEXT, 255, TextKind::MARKER_FOLDER, folder, tMark);
    m_lastMarkerMono = tMark;
    m_cv.notify_all();
    return true;
}

bool BlackBox::takeDeviceInfoRequest()
{
    return m_deviceInfoRequested.exchange(false);
}

void BlackBox::forceRotate()
{
    m_rotateRequested = true;
    m_cv.notify_all();
}

// ---- writer thread -----------------------------------------------------------------------

void BlackBox::writerMain()
{
    std::vector<Record> scratch;
    scratch.reserve(k_RING_CAPACITY);

    openChunk(monoNow());
    scanArchive(); // size of the archive so far, and files a previous shutdown left uncompressed

    while (!m_stopRequested.load()) {
        {
            std::unique_lock<std::mutex> lock(m_cvMutex);
            m_cv.wait_for(lock, std::chrono::duration<double>(m_flushInterval));
        }
        const double t = monoNow();

        Params params = getParams();
        if (!params.enabled) {
            // keep draining the ring so producers never block on a full buffer (discarded on purpose,
            // not counted as drops); finish pending events
            flushRing(scratch);
            pullExternal(scratch); // the ring is drained and dropped too, so re-enabling starts fresh
            scratch.clear();
            commitExternal();
            if (m_file)
                closeChunk(t);
            processEvents(t, false);
            if (!m_leftovers.empty() && policyKnown(t))
                resolveLeftovers(t, params);
            compressOneQueued();
            continue;
        }

        if (m_file == nullptr || m_rotateRequested.load() || (!m_chunks.empty() && m_chunks.back().open && (t - m_chunks.back().monoStart) >= params.chunkSeconds)) {
            m_rotateRequested = false;
            closeChunk(t);
            if (!openChunk(t)) {
                // disk problem: drop this batch and retry next iteration (logged once per 10 s)
                flushRing(scratch);
                m_recordsDropped.fetch_add(scratch.size(), std::memory_order_relaxed);
                scratch.clear();
                continue;
            }
            applyRetention(t);
            m_deviceInfoRequested = true;
        }

        flushRing(scratch);
        pullExternal(scratch);
        writeRecords(scratch);
        commitExternal();
        scratch.clear();

        processEvents(t, false);
        if (!m_leftovers.empty() && policyKnown(t))
            resolveLeftovers(t, params);
        compressOneQueued();
    }

    const double t = monoNow();
    flushRing(scratch);
    pullExternal(scratch);
    writeRecords(scratch);
    commitExternal();
    scratch.clear();
    closeChunk(t);
    processEvents(t, true);

    // the rest of the session stays in live/: the next recorder of this session adopts it, a
    // recorder of a later session archives (keepAll) or deletes it as leftovers

}

void BlackBox::flushRing(std::vector<Record>& scratch)
{
    std::lock_guard<std::mutex> lock(m_ringMutex);
    const size_t n = m_ringCount;
    size_t idx = (m_ringHead + m_ring.size() - n) % m_ring.size();
    for (size_t i = 0; i < n; i++) {
        scratch.push_back(m_ring[idx]);
        idx = (idx + 1) % m_ring.size();
    }
    m_ringCount = 0;
}

void BlackBox::writeRecords(const std::vector<Record>& records)
{
    if (!m_file || records.empty())
        return;
    const size_t written = std::fwrite(records.data(), sizeof(Record), records.size(), m_file);
    std::fflush(m_file);
    m_recordsWritten.fetch_add(written, std::memory_order_relaxed);
    if (!m_chunks.empty()) {
        m_chunks.back().bytes += written * sizeof(Record);
        m_bytesOnDisk.fetch_add(written * sizeof(Record), std::memory_order_relaxed);
    }
}

bool BlackBox::openChunk(double t)
{
    if (m_file)
        return true;

    ChunkInfo chunk;
    chunk.index = m_nextChunkIndex++;
    chunk.monoStart = t;
    chunk.open = true;
    char name[64];
    std::snprintf(name, sizeof(name), "%08x_%06u%s", m_sessionId, chunk.index, k_CHUNK_EXTENSION);
    chunk.path = m_liveDir / name;

#if defined(_WIN32)
    m_file = _wfopen(chunk.path.c_str(), L"wb");
#else
    m_file = std::fopen(chunk.path.c_str(), "wb");
#endif
    if (!m_file) {
        if (t - m_lastOpenFailLog > 10.0) {
            logMessage("BlackBox: cannot open chunk %s", chunk.path.string().c_str());
            m_lastOpenFailLog = t;
        }
        m_nextChunkIndex--; // reuse the index next time
        return false;
    }

    ChunkHeader header;
    header.chunkIndex = chunk.index;
    header.monoStart = t;
    header.unixStart = unixNow() - (monoNow() - t);
    header.sessionId = m_sessionId;
    header.chunkSeconds = getParams().chunkSeconds;
    std::strncpy(header.version, m_version.c_str(), sizeof(header.version) - 1);
    std::fwrite(&header, sizeof(header), 1, m_file);
    std::fflush(m_file);
    chunk.bytes = sizeof(header);
    m_bytesOnDisk.fetch_add(sizeof(header), std::memory_order_relaxed);

    // session statistics as the first record of every chunk
    Record s;
    s.t = t;
    s.type = static_cast<uint8_t>(RecordType::SESSION);
    {
        std::lock_guard<std::mutex> lock(m_ringMutex);
        s.v[0] = static_cast<float>(m_droppedSinceSession);
        s.v[1] = static_cast<float>(m_ringHighWater);
        m_droppedSinceSession = 0;
        m_ringHighWater = 0;
    }
    s.v[2] = static_cast<float>(m_flushInterval);
    std::fwrite(&s, sizeof(s), 1, m_file);
    chunk.bytes += sizeof(s);
    m_bytesOnDisk.fetch_add(sizeof(s), std::memory_order_relaxed);

    m_chunks.push_back(chunk);
    m_newChunk = true;
    m_chunksOnDisk = static_cast<uint32_t>(m_chunks.size());
    return true;
}

void BlackBox::closeChunk(double t)
{
    if (!m_file)
        return;
    std::fflush(m_file);
    std::fclose(m_file);
    m_file = nullptr;
    if (!m_chunks.empty() && m_chunks.back().open) {
        m_chunks.back().open = false;
        m_chunks.back().monoEnd = t;
    }
}

void BlackBox::applyRetention(double t)
{
    const Params params = getParams();
    std::vector<PendingEvent> pending;
    {
        std::lock_guard<std::mutex> lock(m_ctrlMutex);
        pending = m_pending;
    }

    std::vector<ChunkInfo> keep;
    keep.reserve(m_chunks.size());
    const bool policy = policyKnown(t); // until the overlay has spoken, nothing leaves live/
    for (const ChunkInfo& chunk : m_chunks) {
        bool expired = policy && !chunk.open && (t - chunk.monoEnd) > params.liveWindowSeconds;
        if (expired) {
            // never delete a chunk a pending event still needs and has not copied yet
            for (const PendingEvent& ev : pending) {
                const bool copied = std::find(ev.copiedChunks.begin(), ev.copiedChunks.end(), chunk.index) != ev.copiedChunks.end();
                if (chunkOverlaps(chunk, ev) && !copied) {
                    expired = false;
                    break;
                }
            }
        }
        if (expired) {
            if (params.keepAll && !archiveCapReached(params, t)) {
                if (!archiveFile(chunk.path, m_sessionArchiveDir)) {
                    keep.push_back(chunk); // retried at the next rotation
                    continue;
                }
                m_archivedChunks++;
            } else {
                std::error_code ec;
                std::filesystem::remove(chunk.path, ec);
            }
            m_bytesOnDisk.fetch_sub(std::min<uint64_t>(chunk.bytes, m_bytesOnDisk.load()), std::memory_order_relaxed);
        } else {
            keep.push_back(chunk);
        }
    }
    m_chunks.swap(keep);
    m_chunksOnDisk = static_cast<uint32_t>(m_chunks.size());
}

bool BlackBox::chunkOverlaps(const ChunkInfo& chunk, const PendingEvent& ev) const
{
    const double end = chunk.open ? std::numeric_limits<double>::infinity() : chunk.monoEnd;
    return end >= ev.tStart && chunk.monoStart <= ev.tEnd;
}

std::filesystem::path BlackBox::eventDir(const PendingEvent& ev) const
{
    return m_eventsDir / ev.folder;
}

bool BlackBox::copyChunkToEvent(const ChunkInfo& chunk, PendingEvent& ev, bool overwrite)
{
    std::error_code ec;
    if (!ev.folderCreated) {
        std::filesystem::create_directories(eventDir(ev), ec);
        if (ec) {
            logMessage("BlackBox: cannot create event folder %s (%s)", eventDir(ev).string().c_str(), ec.message().c_str());
            return false;
        }
        ev.folderCreated = true;
    }
    const std::filesystem::path dest = eventDir(ev) / chunk.path.filename();
    if (overwrite) {
        std::filesystem::copy_file(chunk.path, dest, std::filesystem::copy_options::overwrite_existing, ec);
    } else {
        std::filesystem::copy_file(chunk.path, dest, std::filesystem::copy_options::skip_existing, ec);
    }
    if (ec) {
        logMessage("BlackBox: copy of %s failed (%s)", chunk.path.string().c_str(), ec.message().c_str());
        return false;
    }
    const std::string fname = chunk.path.filename().string();
    if (std::find(ev.chunkFiles.begin(), ev.chunkFiles.end(), fname) == ev.chunkFiles.end())
        ev.chunkFiles.push_back(fname);
    return true;
}

void BlackBox::processEvents(double t, bool shuttingDown)
{
    std::vector<PendingEvent> pending;
    {
        std::lock_guard<std::mutex> lock(m_ctrlMutex);
        pending = m_pending;
    }
    if (pending.empty())
        return;

    std::vector<std::string> finished;
    // at most one file copy per writer iteration (unless shutting down), so a 12 MB copy on a
    // slow disk never stalls the ring for more than one chunk's worth
    bool copyBudget = true;
    for (PendingEvent& ev : pending) {
        bool allClosedCopied = true;
        for (const ChunkInfo& chunk : m_chunks) {
            if (!chunkOverlaps(chunk, ev))
                continue;
            const bool copied = std::find(ev.copiedChunks.begin(), ev.copiedChunks.end(), chunk.index) != ev.copiedChunks.end();
            if (!chunk.open) {
                if (!copied) {
                    if (copyBudget || shuttingDown) {
                        if (copyChunkToEvent(chunk, ev, true))
                            ev.copiedChunks.push_back(chunk.index);
                        copyBudget = false;
                    } else {
                        allClosedCopied = false;
                    }
                }
            } else if ((shuttingDown || (copyBudget && (t - ev.lastOpenChunkCopy) >= k_OPEN_CHUNK_COPY_INTERVAL))) {
                // growing chunk: refresh the copy now and then so a crash loses at most 10 s
                if (copyChunkToEvent(chunk, ev, true))
                    ev.lastOpenChunkCopy = t;
                copyBudget = false;
            }
        }

        // done when the window has passed and every chunk in it is closed and copied
        bool windowClosed = t > ev.tEnd && allClosedCopied;
        if (windowClosed) {
            for (const ChunkInfo& chunk : m_chunks) {
                if (chunk.open && chunkOverlaps(chunk, ev)) {
                    windowClosed = false;
                    break;
                }
            }
        }
        if (windowClosed || shuttingDown) {
            finalizeEvent(ev, shuttingDown && !windowClosed);
            finished.push_back(ev.folder);
        }
    }

    // write back copy bookkeeping and drop finished events. An event that received another
    // marker while we were copying (its live tEnd grew) stays pending and is finalised again
    // later; the driver.json written above is simply overwritten then.
    std::lock_guard<std::mutex> lock(m_ctrlMutex);
    for (PendingEvent& src : pending) {
        auto it = std::find_if(m_pending.begin(), m_pending.end(), [&](const PendingEvent& e) { return e.folder == src.folder; });
        if (it == m_pending.end())
            continue;
        it->copiedChunks = src.copiedChunks;
        it->chunkFiles = src.chunkFiles;
        it->folderCreated = src.folderCreated;
        it->lastOpenChunkCopy = src.lastOpenChunkCopy;
        const bool isFinished = std::find(finished.begin(), finished.end(), src.folder) != finished.end();
        if (isFinished && it->tEnd > src.tEnd && !shuttingDown) {
            finished.erase(std::remove(finished.begin(), finished.end(), src.folder), finished.end());
            m_eventsSaved.fetch_sub(1, std::memory_order_relaxed);
        }
    }
    m_pending.erase(std::remove_if(m_pending.begin(), m_pending.end(), [&](const PendingEvent& e) {
        return std::find(finished.begin(), finished.end(), e.folder) != finished.end();
    }),
        m_pending.end());
}

void BlackBox::finalizeEvent(PendingEvent& ev, bool incomplete)
{
    std::error_code ec;
    std::filesystem::create_directories(eventDir(ev), ec);
    ev.folderCreated = true;

    std::string json;
    json += "{\n";
    json += "  \"format\": \"spacecal-blackbox-event\",\n";
    json += "  \"version\": \"" + jsonEscape(m_version) + "\",\n";
    char buf[256];
    std::snprintf(buf, sizeof(buf), "  \"session_id\": \"%08x\",\n", m_sessionId);
    json += buf;
    json += "  \"source\": \"" + std::string(markerSourceName(ev.source)) + "\",\n";
    std::snprintf(buf, sizeof(buf), "  \"label\": %u,\n", ev.label);
    json += buf;
    std::snprintf(buf, sizeof(buf), "  \"t_mark_mono\": %.6f,\n  \"t_mark_unix\": %.3f,\n  \"t_start_mono\": %.6f,\n  \"t_end_mono\": %.6f,\n", ev.tMark, ev.unixMark, ev.tStart, ev.tEnd);
    json += buf;
    json += std::string("  \"incomplete\": ") + (incomplete ? "true" : "false") + ",\n";
    json += "  \"chunks\": [";
    for (size_t i = 0; i < ev.chunkFiles.size(); i++) {
        json += (i ? ", " : "") + std::string("\"") + jsonEscape(ev.chunkFiles[i]) + "\"";
    }
    json += "]\n";
    json += "}\n";

    const std::filesystem::path jsonPath = eventDir(ev) / "driver.json";
#if defined(_WIN32)
    FILE* f = _wfopen(jsonPath.c_str(), L"wb");
#else
    FILE* f = std::fopen(jsonPath.c_str(), "wb");
#endif
    if (f) {
        std::fwrite(json.data(), 1, json.size(), f);
        std::fclose(f);
    }

    // the driver's text log lives next to the black box: <config>/logs/log_driver_latest.log
    const std::filesystem::path driverLog = m_root.parent_path() / "logs" / "log_driver_latest.log";
    if (std::filesystem::exists(driverLog, ec)) {
        std::filesystem::copy_file(driverLog, eventDir(ev) / "log_driver.log", std::filesystem::copy_options::overwrite_existing, ec);
    }

    m_eventsSaved.fetch_add(1, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(m_ctrlMutex);
        m_finalized[ev.folder] = ev;
    }
    logMessage("BlackBox: event %s saved (%zu chunks%s)", ev.folder.c_str(), ev.chunkFiles.size(), incomplete ? ", incomplete" : "");
}

// ---- keepAll archive (writer thread) -----------------------------------------------------

bool BlackBox::policyKnown(double t) const
{
    return m_paramsReceived.load() || (t - m_startMono) > k_POLICY_WAIT_SECONDS;
}

bool BlackBox::archiveCapReached(const Params& params, double t)
{
    const bool reached = params.archiveMaxMb > 0 && m_archiveBytes.load() >= static_cast<uint64_t>(params.archiveMaxMb) * 1024ull * 1024ull;
    if (reached && !m_archiveCapReached.load()) {
        const float mb = static_cast<float>(static_cast<double>(m_archiveBytes.load()) / (1024.0 * 1024.0));
        logMessage("BlackBox: archive cap reached (%.0f MB of %u MB), expired chunks are deleted again", mb, params.archiveMaxMb);
        recordLifecycle(LifecycleKind::ARCHIVE_CAP_REACHED, 0, mb, nullptr, 0, t);
    }
    m_archiveCapReached = reached;
    return reached;
}

bool BlackBox::archiveFile(const std::filesystem::path& file, const std::filesystem::path& destDir)
{
    std::error_code ec;
    const double now = monoNow();
    if (!std::filesystem::exists(destDir, ec)) {
        std::filesystem::create_directories(destDir, ec);
        if (ec) {
            if (now - m_lastArchiveFailLog > k_ARCHIVE_FAIL_LOG_INTERVAL) {
                logMessage("BlackBox: cannot create archive folder %s (%s)", destDir.string().c_str(), ec.message().c_str());
                m_lastArchiveFailLog = now;
            }
            return false;
        }
        compressInPlace(destDir, true); // files created in it later are compressed by NTFS itself
    }
    const std::filesystem::path dest = destDir / file.filename();
    // fork: a chunk that is no longer in live/ (moved by another recorder, 2026-09-30) has nothing
    // left to archive. Drop it instead of retrying every minute, and never touch the archive's copy.
    if (!std::filesystem::exists(file, ec)) {
        const bool inArchive = std::filesystem::exists(dest, ec);
        logMessage("BlackBox: %s is no longer in live, nothing to archive (%s)", file.filename().string().c_str(),
            inArchive ? "the archive has it" : "not in the archive either");
        return true;
    }
    std::filesystem::rename(file, dest, ec);
    if (ec) {
        // another volume, or someone has the file open without delete sharing: copy to a temporary
        // name, rename, then remove the original. A failed copy only ever deletes its own temporary
        // file; it used to delete `dest`, which destroyed chunks another recorder had archived.
        std::error_code ec2;
        std::filesystem::path part = dest;
        part += ".part";
        std::filesystem::copy_file(file, part, std::filesystem::copy_options::overwrite_existing, ec2);
        if (!ec2)
            std::filesystem::rename(part, dest, ec2);
        if (!ec2)
            std::filesystem::remove(file, ec2);
        if (ec2) {
            std::error_code ec3;
            std::filesystem::remove(part, ec3);
            if (now - m_lastArchiveFailLog > k_ARCHIVE_FAIL_LOG_INTERVAL) {
                logMessage("BlackBox: cannot archive %s (%s), keeping it in live", file.string().c_str(), ec2.message().c_str());
                m_lastArchiveFailLog = now;
            }
            return false;
        }
    }
    m_archiveBytes.fetch_add(sizeOnDisk(dest), std::memory_order_relaxed);
    m_toCompress.push_back(dest);
    return true;
}

void BlackBox::resolveLeftovers(double t, const Params& params)
{
    const bool archive = params.keepAll && !archiveCapReached(params, t);
    std::error_code ec;

    // group by session: every chunk of one earlier session goes into that session's folder, which
    // may already exist when the session archived part of itself before it died
    struct Group {
        double firstUnix = 0.0;
        std::vector<std::filesystem::path> files;
    };
    std::map<uint32_t, Group> groups;
    size_t unreadable = 0;
    for (const auto& file : m_leftovers) {
        if (!std::filesystem::exists(file, ec))
            continue;
        double unixStart = 0.0;
        uint32_t sid = 0;
        if (!readChunkIdentity(file, unixStart, sid)) {
            std::filesystem::remove(file, ec); // empty or torn file from the crash: nothing to keep
            unreadable++;
            continue;
        }
        Group& g = groups[sid];
        if (g.files.empty() || unixStart < g.firstUnix)
            g.firstUnix = unixStart;
        g.files.push_back(file);
    }

    size_t handled = 0;
    for (auto& [sid, g] : groups) {
        std::filesystem::path dest;
        if (archive) {
            const std::string suffix = "_" + hex8(sid);
            if (std::filesystem::exists(m_sessionsDir, ec)) {
                for (auto& entry : std::filesystem::directory_iterator(m_sessionsDir, ec)) {
                    const std::string name = entry.path().filename().string();
                    std::error_code ec2;
                    if (entry.is_directory(ec2) && name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
                        dest = entry.path();
                        break;
                    }
                }
            }
            if (dest.empty())
                dest = m_sessionsDir / (localStamp(g.firstUnix) + suffix);
        }
        for (const auto& file : g.files) {
            if (archive) {
                if (archiveFile(file, dest))
                    handled++;
                // a file that could not be archived stays in live/ and is looked at again next start
            } else {
                std::filesystem::remove(file, ec);
                handled++;
            }
        }
    }

    if (handled > 0 || unreadable > 0) {
        logMessage("BlackBox: %zu chunk(s) of an earlier session %s, %zu unreadable removed", handled, archive ? "archived" : "deleted", unreadable);
        const float v[1] = { archive ? 1.0f : 0.0f };
        recordLifecycle(LifecycleKind::LEFTOVERS, static_cast<uint16_t>(std::min<size_t>(handled, 65535)), 0.0f, v, 1, t);
    }
    m_leftovers.clear();
}

void BlackBox::compressOneQueued()
{
    if (m_toCompress.empty())
        return;
    const std::filesystem::path p = m_toCompress.front();
    m_toCompress.pop_front();
    std::error_code ec;
    if (!std::filesystem::exists(p, ec) || isCompressed(p))
        return;
    const uint64_t before = sizeOnDisk(p);
    if (compressInPlace(p, false)) {
        const uint64_t after = sizeOnDisk(p);
        if (after < before)
            m_archiveBytes.fetch_sub(std::min<uint64_t>(before - after, m_archiveBytes.load()), std::memory_order_relaxed);
    }
}

void BlackBox::scanArchive()
{
    std::error_code ec;
    if (!std::filesystem::exists(m_sessionsDir, ec))
        return;
    uint64_t total = 0;
    for (auto it = std::filesystem::recursive_directory_iterator(m_sessionsDir, ec); !ec && it != std::filesystem::recursive_directory_iterator(); it.increment(ec)) {
        std::error_code ec2;
        if (!it->is_regular_file(ec2))
            continue;
        total += sizeOnDisk(it->path());
        if (it->path().extension() == k_CHUNK_EXTENSION && !isCompressed(it->path()))
            m_toCompress.push_back(it->path());
    }
    m_archiveBytes.fetch_add(total, std::memory_order_relaxed);
}

} // namespace spacecal::blackbox
