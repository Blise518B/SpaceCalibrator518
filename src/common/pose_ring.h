#pragma once

// Pose ring (fork, docs/DESIGN.md section 13): the one data path from the thin driver to the
// overlay. The driver copies every raw pose, WorldFromDriver change, applied-correction change
// and connect/disconnect as a black-box Record into a ring in shared memory; the overlay's
// recorder reads it at its own pace. The driver never waits for, and never depends on, the
// reader: it just overwrites the oldest slot. The reader keeps its cursor in the ring header,
// so a restarted overlay resumes where the previous one stopped without a gap (as long as the
// restart takes less than the ring's span, about three minutes at 2,600 records/s).
//
// Slots are published seqlock-style: a slot's `seq` equals the record's global index once the
// record is complete, so a reader detects both torn reads and records overwritten before it got
// to them. Producers inside the driver are serialised by an in-process mutex.
//
// Contract: bump k_RING_LAYOUT_VERSION whenever RingHeader, RingSlot or Record change. The
// overlay refuses to attach to another layout (and says so), and tools/install.ps1 treats a
// change as a driver interface change that needs a SteamVR restart.

#include "blackbox_format.h"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace spacecal::blackbox {

constexpr uint32_t k_RING_MAGIC = 0x52504353; // "SCPR"
constexpr uint16_t k_RING_LAYOUT_VERSION = 1;
constexpr uint32_t k_RING_DEFAULT_CAPACITY = 1u << 19; // 524,288 records: 46 MB, ~3.3 min at 2,600 records/s
constexpr const wchar_t* k_RING_DEFAULT_NAME = L"Local\\SpaceCalibrator518_PoseRing";

struct alignas(64) RingHeader {
    uint32_t magic = 0;
    uint16_t layoutVersion = 0;
    uint16_t slotSize = 0;
    uint32_t capacity = 0; // slots, a power of two
    std::atomic<uint32_t> sessionId { 0 }; // random per driver start; written last when (re)initialising
    double driverStartUnix = 0.0;
    double driverStartMono = 0.0;
    char driverVersion[24] = {};
    uint8_t pad0[8] = {};
    alignas(64) std::atomic<uint64_t> writeIndex { 0 }; // records published so far this session
    alignas(64) std::atomic<uint64_t> readerCursor { 0 }; // next record the recorder has not written to disk yet
    std::atomic<uint64_t> readerHeartbeatUnixMs { 0 };
    uint8_t pad2[112] = {};
};
static_assert(sizeof(RingHeader) == 256, "RingHeader must stay 256 bytes");
static_assert(std::atomic<uint64_t>::is_always_lock_free && std::atomic<uint32_t>::is_always_lock_free, "shared-memory atomics must be lock free");

struct RingSlot {
    std::atomic<uint64_t> seq { ~0ull }; // == global index of `record` once it is complete
    Record record;
};
static_assert(sizeof(RingSlot) == 88, "RingSlot must stay 88 bytes");

// Driver side. create() opens (or re-initialises) the named mapping; write() is safe from any
// thread and never blocks on the reader.
class PoseRingWriter {
public:
    PoseRingWriter() = default;
    ~PoseRingWriter();
    PoseRingWriter(const PoseRingWriter&) = delete;
    PoseRingWriter& operator=(const PoseRingWriter&) = delete;

    bool create(const std::wstring& name, uint32_t capacity, uint32_t sessionId, double startUnix, double startMono, const char* version, std::string* error = nullptr);
    void close();
    [[nodiscard]] bool isOpen() const { return m_header != nullptr; }
    [[nodiscard]] uint32_t capacity() const { return m_header ? m_header->capacity : 0; }
    void write(const Record& record);

private:
    void* m_mapping = nullptr;
    RingHeader* m_header = nullptr;
    RingSlot* m_slots = nullptr;
    uint64_t m_mask = 0;
    std::mutex m_mutex;
};

// Overlay side. Not thread-safe: one thread reads (the recorder's writer thread); other threads
// may only call sessionChanged().
class PoseRingReader {
public:
    PoseRingReader() = default;
    ~PoseRingReader();
    PoseRingReader(const PoseRingReader&) = delete;
    PoseRingReader& operator=(const PoseRingReader&) = delete;

    // Opens an existing ring. Fails when the driver is not running or its layout differs
    // (the reason goes to *error).
    bool attach(const std::wstring& name, std::string* error = nullptr);
    void detach();
    [[nodiscard]] bool attached() const { return m_header != nullptr; }

    [[nodiscard]] uint32_t sessionId() const { return m_sessionId; }
    [[nodiscard]] double driverStartUnix() const { return m_header ? m_header->driverStartUnix : 0.0; }
    [[nodiscard]] std::string driverVersion() const;
    [[nodiscard]] uint32_t capacity() const { return m_header ? m_header->capacity : 0; }
    // the driver restarted the ring under us (SteamVR was restarted)
    [[nodiscard]] bool sessionChanged() const;

    // Positions the cursor where the previous reader of this session stopped (gapless), or at the
    // oldest record still in the ring. Returns how many records were lost in between.
    uint64_t resume();
    // Appends up to maxRecords published records; `lost` grows by records that were overwritten
    // (or torn) before they could be read.
    size_t read(std::vector<Record>& out, size_t maxRecords, uint64_t& lost);
    // Stores the cursor in the header: call once the records read so far are safely on disk.
    void commit();
    [[nodiscard]] uint64_t cursor() const { return m_cursor; }
    [[nodiscard]] uint64_t backlog() const;

private:
    void* m_mapping = nullptr;
    RingHeader* m_header = nullptr;
    RingSlot* m_slots = nullptr;
    uint64_t m_mask = 0;
    uint32_t m_sessionId = 0;
    uint64_t m_cursor = 0;
};

} // namespace spacecal::blackbox
