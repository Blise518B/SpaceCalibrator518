#pragma once

// Fork UI: the "Guard" tab (recorder status and settings, later trust states) and the small
// "Mark event" control that also sits on the calibration page.

namespace spacecal::guard {

void page_guard(double currentTime);
// One-line control: [Mark event] button + last-marker text. Drawn on the calibration page.
void draw_mark_event_row();
// The "Live" tab: what the guard and the solver see right now (live_ui.cpp)
void page_live(double currentTime);
// development aid: `--live-demo` fills the Live tab with made-up data (live_demo.cpp)
extern bool g_liveDemo;
void live_demo_tick(double currentTime);
void live_demo_control(double currentTime); // reads live_demo.txt: scroll=<px>, mouse=<x>,<y>

} // namespace spacecal::guard
