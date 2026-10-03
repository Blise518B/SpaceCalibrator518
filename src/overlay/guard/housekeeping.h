#pragma once

// Glitch collection at overlay start (fork, docs/DESIGN.md section 13, "Glitch collection").
// Starts tools/glitch_collection.py in the background, below normal priority, a minute after the
// overlay started (the recorder archives the previous session's live chunks first): every finished
// session is condensed to the minutes around its glitches in blackbox/glitches, the rest of it is
// deleted, old overlay logs lose their per-frame noise. Only on a development install (dist\ next
// to tools\ and .venv\); the script never touches the session being recorded.
// guard.json: recorder.glitch_collection.

namespace spacecal::guard {

void startGlitchCollection();

} // namespace spacecal::guard
