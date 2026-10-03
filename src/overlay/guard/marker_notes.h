#pragma once

// Marker notes (fork, docs/DESIGN.md section 13): the text the user adds to a marker he set, to say
// what happened. Pure helpers for the three places a note goes; tested in tests/blackbox_test.cpp.
//   - notes.txt in the event folder (one line per marker of that folder),
//   - the "user_notes" line of the event's overlay.json,
//   - a TEXT record (TextKind::MARKER_NOTE) in the recording, so the keep-all archive and the
//     analysis tools have it next to the data.

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace spacecal::guard::notes {

// fits one TEXT message (8 records of 64 bytes) together with the marker time in front
constexpr size_t k_MAX_NOTE_BYTES = 400;

struct Entry {
    double unixMark = 0.0; // wall clock of the marker
    std::string clock; // "HH:MM:SS" local time of the marker
    std::string source; // "overlay", "hotkey", "triggers"
    std::string note;
};

inline std::string jsonEscape(const std::string& s)
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
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(c)));
                out += buf;
            } else {
                out += c;
            }
        }
    }
    return out;
}

// One line of plain text: line breaks and tabs become spaces, other control characters are dropped,
// runs of spaces collapse, the ends are trimmed, and the text is cut at k_MAX_NOTE_BYTES without
// splitting a UTF-8 character.
inline std::string sanitize(const std::string& in)
{
    std::string out;
    out.reserve(in.size());
    for (char c : in) {
        const unsigned char u = static_cast<unsigned char>(c);
        if (c == '\n' || c == '\r' || c == '\t')
            c = ' ';
        else if (u < 0x20 || u == 0x7F)
            continue;
        if (c == ' ' && (out.empty() || out.back() == ' '))
            continue;
        out += c;
    }
    while (!out.empty() && out.back() == ' ')
        out.pop_back();
    if (out.size() > k_MAX_NOTE_BYTES) {
        size_t cut = k_MAX_NOTE_BYTES;
        while (cut > 0 && (static_cast<unsigned char>(out[cut]) & 0xC0) == 0x80)
            cut--; // out[cut] is a continuation byte: the character started earlier
        out.resize(cut);
        while (!out.empty() && out.back() == ' ')
            out.pop_back();
    }
    return out;
}

// notes.txt: readable without any tool; entries without a note are left out
inline std::string notesFile(const std::string& folder, const std::vector<Entry>& entries)
{
    std::string s = "# Notes for event " + folder + " (Space Calibrator 518; local time, marker source, note)\n";
    for (const Entry& e : entries) {
        if (e.note.empty())
            continue;
        s += e.clock + "  " + e.source + "  " + e.note + "\n";
    }
    return s;
}

// the "user_notes" line of overlay.json, one line so it can be replaced in place later
inline std::string overlayJsonLine(const std::vector<Entry>& entries)
{
    std::string s = "  \"user_notes\": [";
    bool first = true;
    for (const Entry& e : entries) {
        if (e.note.empty())
            continue;
        char t[32];
        std::snprintf(t, sizeof(t), "%.3f", e.unixMark);
        s += std::string(first ? "" : ", ") + "{ \"t_mark_unix\": " + t + ", \"time\": \"" + jsonEscape(e.clock) + "\", \"source\": \""
            + jsonEscape(e.source) + "\", \"note\": \"" + jsonEscape(e.note) + "\" }";
        first = false;
    }
    s += "],";
    return s;
}

// Puts `line` (from overlayJsonLine) into an overlay.json text: replaces its "user_notes" line, or
// inserts it after the "note" line of files written before notes existed. false if neither is there.
inline bool replaceOverlayJsonLine(std::string& json, const std::string& line)
{
    auto lineEnd = [&json](size_t from) {
        const size_t e = json.find('\n', from);
        return e == std::string::npos ? json.size() : e;
    };
    const size_t at = json.find("\n  \"user_notes\": ");
    if (at != std::string::npos) {
        const size_t start = at + 1;
        size_t end = lineEnd(start);
        if (end > start && json[end - 1] == '\r')
            end--;
        json.replace(start, end - start, line);
        return true;
    }
    const size_t note = json.find("\n  \"note\": ");
    if (note != std::string::npos) {
        const size_t end = lineEnd(note + 1);
        if (end == json.size()) {
            json += "\n" + line;
        } else {
            const bool crlf = json[end - 1] == '\r';
            json.insert(end + 1, line + (crlf ? "\r\n" : "\n"));
        }
        return true;
    }
    return false;
}

// TEXT record payload: the marker's steady-clock time, a space, the note. Written when the note is
// saved (possibly minutes after the marker); the latest one for a marker time wins.
inline std::string recordPayload(double monoMark, const std::string& note)
{
    char t[40];
    std::snprintf(t, sizeof(t), "%.6f ", monoMark);
    return t + note;
}

} // namespace spacecal::guard::notes
