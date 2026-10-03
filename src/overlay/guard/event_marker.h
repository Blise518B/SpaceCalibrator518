#pragma once

// Event markers (fork, docs/DESIGN.md sections 4 and 13):
//   - polls the global hotkey (Windows) every tick,
//   - names the event folder and tells the recorder to promote the recording window,
//   - writes overlay.json (calibration state, device table, config) into the event folder,
//   - copies the overlay text log into the folder at mark time and again after the post window.
// The recorder (recorder/recorder_host.h) owns the chunk files; this class never touches them.

#include "blackbox_format.h"
#include "marker_notes.h"
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace spacecal::guard {

// A marker the user set himself (button, hotkey, trigger hold), with the note he wrote for it.
struct ManualMarker {
    std::string folder;
    std::string source; // markerSourceName
    std::string clock; // local "HH:MM:SS"
    double unixMark = 0.0;
    double monoMark = 0.0; // recorder clock of the MARKER record (blackbox::monoNow)
    std::string note;
    bool noteSaved = false;
    bool dismissed = false; // "later": no prompt, the note can still be added from the list
};

struct MarkerStatus {
    std::string lastFolder;
    double lastMarkTime = -1.0; // overlay time (glfwGetTime) of the last marker, -1 = none
    std::string lastSource;
    uint32_t markersThisSession = 0;
    uint32_t autoMarkersThisSession = 0;
    bool hotkeyDown = false;
};

class EventMarker {
public:
    void init();
    // called once per overlay frame
    void tick(double currentTime);

    // Sets a marker. `note` is free text stored in overlay.json (empty is fine). Returns true if
    // the marker was accepted; false when the recorder is disabled, the IPC is down, or the
    // auto-marker cap is reached.
    bool mark(blackbox::MarkerSource source, const std::string& note = "", uint32_t label = 0);

    // Hands changed recorder settings to the running recorder.
    void pushParamsToRecorder();

    [[nodiscard]] const MarkerStatus& status() const { return m_status; }

    // ---- marker notes (marker_notes.h) ----------------------------------------------------
    [[nodiscard]] const std::vector<ManualMarker>& manualMarkers() const { return m_manual; }
    // the marker the UI asks a note for: the one picked for editing, else the newest without a
    // note that was not put off; -1 = none
    [[nodiscard]] int notePromptIndex() const;
    // true once after a marker from the overlay button or the hotkey: the UI puts the cursor in the field
    bool takeNoteFocusRequest();
    // writes notes.txt, overlay.json and a MARKER_NOTE record; an empty text removes the note
    bool saveNote(size_t index, const std::string& text);
    void dismissNote(size_t index);
    void editNote(size_t index);
    [[nodiscard]] std::filesystem::path eventsDir() const;
    [[nodiscard]] std::filesystem::path sessionsDir() const; // keep-all archive, one folder per SteamVR session
    [[nodiscard]] static EventMarker* getInstance() { return s_instance; }

private:
    struct PendingLogCopy {
        std::string folder;
        double dueTime;
    };

    std::string makeFolderName(blackbox::MarkerSource source) const;
    void writeOverlayJson(const std::string& folder, blackbox::MarkerSource source, const std::string& note, uint32_t label, bool merged) const;
    void copyOverlayLog(const std::string& folder) const;
    void pollHotkey(double currentTime);
    [[nodiscard]] std::vector<notes::Entry> noteEntries(const std::string& folder) const;
    void writeNoteFiles(const std::string& folder) const;
    void giveFocusBack(size_t index);

private:
    MarkerStatus m_status;
    std::vector<PendingLogCopy> m_pendingLogCopies;
    double m_lastMarkTime = -1.0;
    double m_currentTime = 0.0;
    bool m_hotkeyWasDown = false;
    std::vector<ManualMarker> m_manual; // this overlay run
    int m_editIndex = -1;
    bool m_noteFocusRequest = false;
    void* m_returnFocusTo = nullptr; // window that had the focus before the hotkey raised ours
    int m_raisedForIndex = -1;
    static EventMarker* s_instance;
};

} // namespace spacecal::guard
