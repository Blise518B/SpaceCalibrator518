#include "pose_ring.h"

#include <algorithm>
#include <cstring>
#include <chrono>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace spacecal::blackbox {

namespace {
    uint64_t mappingBytes(uint32_t capacity)
    {
        return sizeof(RingHeader) + static_cast<uint64_t>(capacity) * sizeof(RingSlot);
    }

    bool isPowerOfTwo(uint32_t v)
    {
        return v != 0 && (v & (v - 1)) == 0;
    }

    void setError(std::string* error, const std::string& text)
    {
        if (error)
            *error = text;
    }

    uint64_t unixMsNow()
    {
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    }
}

// ---- writer (driver) ---------------------------------------------------------------------

PoseRingWriter::~PoseRingWriter()
{
    close();
}

bool PoseRingWriter::create(const std::wstring& name, uint32_t capacity, uint32_t sessionId, double startUnix, double startMono, const char* version, std::string* error)
{
    close();
    if (!isPowerOfTwo(capacity)) {
        setError(error, "capacity must be a power of two");
        return false;
    }
#if defined(_WIN32)
    const uint64_t bytes = mappingBytes(capacity);
    HANDLE mapping = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, static_cast<DWORD>(bytes >> 32), static_cast<DWORD>(bytes & 0xFFFFFFFFu), name.c_str());
    if (!mapping) {
        setError(error, "CreateFileMappingW failed (" + std::to_string(GetLastError()) + ")");
        return false;
    }
    const bool existed = GetLastError() == ERROR_ALREADY_EXISTS;
    void* view = MapViewOfFile(mapping, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    if (!view) {
        setError(error, "MapViewOfFile failed (" + std::to_string(GetLastError()) + ")");
        CloseHandle(mapping);
        return false;
    }
    RingHeader* header = static_cast<RingHeader*>(view);
    if (existed) {
        // an overlay still holds the ring of the previous SteamVR session open: reuse it when it
        // has our layout, otherwise give up (the mapping's size is fixed by whoever created it)
        MEMORY_BASIC_INFORMATION info {};
        VirtualQuery(view, &info, sizeof(info));
        if (header->magic != k_RING_MAGIC || header->layoutVersion != k_RING_LAYOUT_VERSION || header->slotSize != sizeof(RingSlot)
            || header->capacity != capacity || info.RegionSize < bytes) {
            setError(error, "an incompatible pose ring is still open (an old overlay?)");
            UnmapViewOfFile(view);
            CloseHandle(mapping);
            return false;
        }
    }
    m_mapping = mapping;
#else
    (void)name;
    (void)sessionId;
    (void)startUnix;
    (void)startMono;
    (void)version;
    setError(error, "the pose ring is only implemented on Windows");
    return false;
#endif
#if defined(_WIN32)
    m_header = header;
    m_slots = reinterpret_cast<RingSlot*>(reinterpret_cast<uint8_t*>(view) + sizeof(RingHeader));
    m_mask = capacity - 1;

    // (re)initialise: the session id goes last, so a reader that sees it also sees the rest
    m_header->sessionId.store(0, std::memory_order_release);
    for (uint64_t i = 0; i < capacity; i++)
        m_slots[i].seq.store(~0ull, std::memory_order_relaxed);
    m_header->magic = k_RING_MAGIC;
    m_header->layoutVersion = k_RING_LAYOUT_VERSION;
    m_header->slotSize = static_cast<uint16_t>(sizeof(RingSlot));
    m_header->capacity = capacity;
    m_header->driverStartUnix = startUnix;
    m_header->driverStartMono = startMono;
    std::memset(m_header->driverVersion, 0, sizeof(m_header->driverVersion));
    if (version)
        std::strncpy(m_header->driverVersion, version, sizeof(m_header->driverVersion) - 1);
    m_header->writeIndex.store(0, std::memory_order_relaxed);
    m_header->readerCursor.store(0, std::memory_order_relaxed);
    m_header->readerHeartbeatUnixMs.store(0, std::memory_order_relaxed);
    m_header->sessionId.store(sessionId ? sessionId : 1u, std::memory_order_release);
    return true;
#endif
}

void PoseRingWriter::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
#if defined(_WIN32)
    if (m_header)
        UnmapViewOfFile(m_header);
    if (m_mapping)
        CloseHandle(static_cast<HANDLE>(m_mapping));
#endif
    m_header = nullptr;
    m_slots = nullptr;
    m_mapping = nullptr;
    m_mask = 0;
}

void PoseRingWriter::write(const Record& record)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_header)
        return;
    const uint64_t index = m_header->writeIndex.load(std::memory_order_relaxed);
    RingSlot& slot = m_slots[index & m_mask];
    slot.seq.store(~0ull, std::memory_order_relaxed); // mark as being written
    std::atomic_thread_fence(std::memory_order_release);
    std::memcpy(&slot.record, &record, sizeof(Record));
    slot.seq.store(index, std::memory_order_release);
    m_header->writeIndex.store(index + 1, std::memory_order_release);
}

// ---- reader (overlay) --------------------------------------------------------------------

PoseRingReader::~PoseRingReader()
{
    detach();
}

bool PoseRingReader::attach(const std::wstring& name, std::string* error)
{
    detach();
#if defined(_WIN32)
    HANDLE mapping = OpenFileMappingW(FILE_MAP_READ | FILE_MAP_WRITE, FALSE, name.c_str());
    if (!mapping) {
        setError(error, "no pose ring (the SpaceCalibrator518 driver is not running)");
        return false;
    }
    void* view = MapViewOfFile(mapping, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, 0);
    if (!view) {
        setError(error, "MapViewOfFile failed (" + std::to_string(GetLastError()) + ")");
        CloseHandle(mapping);
        return false;
    }
    RingHeader* header = static_cast<RingHeader*>(view);
    const uint32_t session = header->sessionId.load(std::memory_order_acquire);
    MEMORY_BASIC_INFORMATION info {};
    VirtualQuery(view, &info, sizeof(info));
    if (header->magic != k_RING_MAGIC || header->layoutVersion != k_RING_LAYOUT_VERSION || header->slotSize != sizeof(RingSlot)
        || !isPowerOfTwo(header->capacity) || info.RegionSize < mappingBytes(header->capacity)) {
        setError(error, "the driver's pose ring has another layout (driver and overlay are from different builds)");
        UnmapViewOfFile(view);
        CloseHandle(mapping);
        return false;
    }
    if (session == 0) {
        setError(error, "the pose ring is being initialised");
        UnmapViewOfFile(view);
        CloseHandle(mapping);
        return false;
    }
    m_mapping = mapping;
    m_header = header;
    m_slots = reinterpret_cast<RingSlot*>(reinterpret_cast<uint8_t*>(view) + sizeof(RingHeader));
    m_mask = header->capacity - 1;
    m_sessionId = session;
    m_cursor = 0;
    return true;
#else
    (void)name;
    setError(error, "the pose ring is only implemented on Windows");
    return false;
#endif
}

void PoseRingReader::detach()
{
#if defined(_WIN32)
    if (m_header)
        UnmapViewOfFile(m_header);
    if (m_mapping)
        CloseHandle(static_cast<HANDLE>(m_mapping));
#endif
    m_header = nullptr;
    m_slots = nullptr;
    m_mapping = nullptr;
    m_mask = 0;
    m_sessionId = 0;
    m_cursor = 0;
}

std::string PoseRingReader::driverVersion() const
{
    if (!m_header)
        return {};
    return std::string(m_header->driverVersion, strnlen(m_header->driverVersion, sizeof(m_header->driverVersion)));
}

bool PoseRingReader::sessionChanged() const
{
    return m_header && m_header->sessionId.load(std::memory_order_acquire) != m_sessionId;
}

uint64_t PoseRingReader::resume()
{
    if (!m_header)
        return 0;
    const uint64_t w = m_header->writeIndex.load(std::memory_order_acquire);
    const uint64_t oldest = w > m_header->capacity ? w - m_header->capacity : 0;
    uint64_t c = m_header->readerCursor.load(std::memory_order_acquire);
    uint64_t lost = 0;
    if (c > w)
        c = w;
    if (c < oldest) {
        lost = oldest - c;
        c = oldest;
    }
    m_cursor = c;
    return lost;
}

size_t PoseRingReader::read(std::vector<Record>& out, size_t maxRecords, uint64_t& lost)
{
    if (!m_header)
        return 0;
    const uint64_t w = m_header->writeIndex.load(std::memory_order_acquire);
    const uint64_t capacity = m_header->capacity;
    if (w > capacity && m_cursor < w - capacity) {
        lost += (w - capacity) - m_cursor;
        m_cursor = w - capacity;
    }
    const uint64_t available = w > m_cursor ? w - m_cursor : 0;
    const uint64_t n = std::min<uint64_t>(available, maxRecords);
    size_t got = 0;
    Record r;
    for (uint64_t i = m_cursor; i < m_cursor + n; i++) {
        const RingSlot& slot = m_slots[i & m_mask];
        if (slot.seq.load(std::memory_order_acquire) != i) {
            lost++;
            continue;
        }
        std::memcpy(&r, &slot.record, sizeof(Record));
        std::atomic_thread_fence(std::memory_order_acquire);
        if (slot.seq.load(std::memory_order_relaxed) != i) {
            lost++; // overwritten while we copied it
            continue;
        }
        out.push_back(r);
        got++;
    }
    m_cursor += n;
    return got;
}

void PoseRingReader::commit()
{
    if (!m_header)
        return;
    m_header->readerCursor.store(m_cursor, std::memory_order_release);
    m_header->readerHeartbeatUnixMs.store(unixMsNow(), std::memory_order_relaxed);
}

uint64_t PoseRingReader::backlog() const
{
    if (!m_header)
        return 0;
    const uint64_t w = m_header->writeIndex.load(std::memory_order_acquire);
    return w > m_cursor ? w - m_cursor : 0;
}

} // namespace spacecal::blackbox
