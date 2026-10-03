#pragma once

// Header-only reader for black-box chunk files (fork). Used by the tests and the replay tool;
// the overlay and driver never read recordings. Mirrors tools/blackbox.py.

#include "blackbox_format.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace spacecal::blackbox {

struct ParsedChunk {
    std::filesystem::path path;
    ChunkHeader header {};
    std::vector<Record> records;
    bool truncatedTail = false;
};

inline bool readChunk(const std::filesystem::path& path, ParsedChunk& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    out.path = path;
    f.read(reinterpret_cast<char*>(&out.header), sizeof(ChunkHeader));
    if (!f || out.header.magic != k_MAGIC || out.header.recordSize != k_RECORD_SIZE || out.header.headerSize != k_HEADER_SIZE)
        return false;
    f.seekg(out.header.headerSize, std::ios::beg);
    Record r;
    while (f.read(reinterpret_cast<char*>(&r), sizeof(Record))) {
        out.records.push_back(r);
    }
    out.truncatedTail = f.gcount() != 0;
    return true;
}

// Text of one DEVICE/TEXT record (NUL padded, not necessarily terminated).
inline std::string recordText(const Record& r)
{
    const char* p = r.text();
    size_t n = 0;
    while (n < k_TEXT_PAYLOAD_SIZE && p[n] != '\0')
        n++;
    return std::string(p, n);
}

struct AssembledText {
    double t = 0.0;
    uint8_t device = 255;
    RecordType type = RecordType::TEXT;
    TextKind kind = TextKind::NOTE;
    std::string text;
};

struct Recording {
    std::vector<ParsedChunk> chunks;
    std::vector<Record> records; // all records of all chunks, sorted by t (stable)
    std::vector<AssembledText> texts;
    std::map<uint8_t, std::map<std::string, std::string>> devices; // index -> key/value from DEVICE info
};

// Loads every .scb file of a folder, sorts the records and assembles multi-record texts.
inline bool loadRecording(const std::filesystem::path& folder, Recording& out)
{
    std::vector<std::filesystem::path> files;
    std::error_code ec;
    for (const auto& e : std::filesystem::directory_iterator(folder, ec)) {
        if (e.is_regular_file(ec) && e.path().extension() == k_CHUNK_EXTENSION)
            files.push_back(e.path());
    }
    std::sort(files.begin(), files.end());
    for (const auto& p : files) {
        ParsedChunk c;
        if (readChunk(p, c))
            out.chunks.push_back(std::move(c));
    }
    if (out.chunks.empty())
        return false;
    for (const ParsedChunk& c : out.chunks)
        out.records.insert(out.records.end(), c.records.begin(), c.records.end());
    std::stable_sort(out.records.begin(), out.records.end(), [](const Record& a, const Record& b) { return a.t < b.t; });

    // texts: records of one string share t, device, type and kind; b is the sequence number
    struct Key {
        double t;
        uint8_t device, type, kind;
        bool operator<(const Key& o) const
        {
            if (t != o.t)
                return t < o.t;
            if (device != o.device)
                return device < o.device;
            if (type != o.type)
                return type < o.type;
            return kind < o.kind;
        }
    };
    std::map<Key, std::map<uint8_t, std::string>> parts;
    for (const Record& r : out.records) {
        if (r.type != static_cast<uint8_t>(RecordType::DEVICE) && r.type != static_cast<uint8_t>(RecordType::TEXT))
            continue;
        parts[{ r.t, r.device, r.type, r.a }][r.b] = recordText(r);
    }
    for (auto& [key, seq] : parts) {
        AssembledText a;
        a.t = key.t;
        a.device = key.device;
        a.type = static_cast<RecordType>(key.type);
        a.kind = static_cast<TextKind>(key.kind);
        for (auto& [i, s] : seq)
            a.text += s;
        out.texts.push_back(a);
        if (a.type == RecordType::DEVICE && a.kind == TextKind::DEVICE_INFO) {
            std::map<std::string, std::string> kv;
            size_t pos = 0;
            while (pos < a.text.size()) {
                size_t semi = a.text.find(';', pos);
                if (semi == std::string::npos)
                    semi = a.text.size();
                const std::string part = a.text.substr(pos, semi - pos);
                const size_t eq = part.find('=');
                if (eq != std::string::npos)
                    kv[part.substr(0, eq)] = part.substr(eq + 1);
                pos = semi + 1;
            }
            out.devices[a.device] = kv;
        }
    }
    return true;
}

} // namespace spacecal::blackbox
