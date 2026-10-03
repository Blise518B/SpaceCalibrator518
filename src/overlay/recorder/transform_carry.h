#pragma once

// Every chunk of a recording must be readable on its own: a replay or an analysis may start at any
// chunk. The driver writes a device's WorldFromDriver and applied correction only when they change,
// so a chunk recorded while nothing changed would hold poses without the transforms that place them.
// TransformCarry remembers the last of each per device and puts them at the start of every new
// chunk, marked k_RECORD_CARRIED (a repeat, not a change). It also drops WorldFromDriver records that
// did not change (older drivers repeated it with every pose). Writer thread only.

#include "blackbox_format.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace spacecal::blackbox {

class TransformCarry {
public:
    static constexpr size_t k_SLOTS = 64;

    // forget everything (a new SteamVR session reassigns the device slots)
    void reset()
    {
        m_worldFromDriver = {};
        m_applied = {};
        m_pending.clear();
    }

    // Before the first pull into a new chunk: takes the transforms in effect so far. Devices in
    // excludeMask are left out, like their other records.
    void beginChunk(uint64_t excludeMask)
    {
        m_pending.clear();
        take(m_worldFromDriver, excludeMask);
        take(m_applied, excludeMask);
    }

    // For every pulled record, in order. false = drop it (a WorldFromDriver that did not change).
    bool accept(const Record& r)
    {
        if (r.device >= k_SLOTS)
            return true;
        if (r.type == static_cast<uint8_t>(RecordType::WORLD_FROM_DRIVER)) {
            Slot& last = m_worldFromDriver[r.device];
            if (last.valid && !changed(last.record.v, r.v))
                return false;
            last.valid = true;
            last.record = r;
        } else if (r.type == static_cast<uint8_t>(RecordType::APPLIED)) {
            m_applied[r.device].valid = true;
            m_applied[r.device].record = r;
        }
        return true;
    }

    // After the pull: appends what beginChunk took, timed just ahead of the chunk start and of every
    // pulled record (the chunk is sorted by time, so they end up in front of the poses they place).
    void endPull(std::vector<Record>& out, size_t pulledFrom, double chunkStart)
    {
        if (m_pending.empty())
            return;
        double t = chunkStart;
        for (size_t i = pulledFrom; i < out.size(); i++)
            t = std::min(t, out[i].t);
        t -= 1e-6;
        for (Record& r : m_pending) {
            r.t = t;
            out.push_back(r);
        }
        m_pending.clear();
    }

    // same thresholds as the driver's change test (0.5 mm, 0.05 deg), with normalised quaternions
    static bool changed(const float a[7], const float b[7])
    {
        const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
        if (std::sqrt(dx * dx + dy * dy + dz * dz) > 0.0005)
            return true;
        const double dot = std::abs(double(a[3]) * b[3] + double(a[4]) * b[4] + double(a[5]) * b[5] + double(a[6]) * b[6]);
        const double na = double(a[3]) * a[3] + double(a[4]) * a[4] + double(a[5]) * a[5] + double(a[6]) * a[6];
        const double nb = double(b[3]) * b[3] + double(b[4]) * b[4] + double(b[5]) * b[5] + double(b[6]) * b[6];
        const double norms = std::sqrt(na * nb);
        return norms <= 0.0 || dot / norms < 0.99999990;
    }

private:
    struct Slot {
        bool valid = false;
        Record record;
    };

    void take(const std::array<Slot, k_SLOTS>& slots, uint64_t excludeMask)
    {
        for (size_t d = 0; d < slots.size(); d++) {
            if (!slots[d].valid || ((excludeMask >> d) & 1ull) != 0)
                continue;
            Record r = slots[d].record;
            r.b = k_RECORD_CARRIED;
            m_pending.push_back(r);
        }
    }

    std::array<Slot, k_SLOTS> m_worldFromDriver {};
    std::array<Slot, k_SLOTS> m_applied {};
    std::vector<Record> m_pending;
};

} // namespace spacecal::blackbox
