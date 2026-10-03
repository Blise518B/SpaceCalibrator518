#pragma once

// Trust states shared by the overlay (decides) and the driver (holds). Fork, docs/DESIGN.md
// sections 6 and 7. Kept dependency-free so the black-box tools can mirror it.

#include <cstdint>

namespace spacecal::trust {

enum class State : uint8_t {
    TRUSTED = 0,
    SUSPECT = 1,
    UNTRUSTED = 2,
    RECOVERING = 3,
};

enum class Reason : uint16_t {
    NONE = 0,
    JUMP_SINGLE = 1, // one device jumped, the jurors did not
    JUMP_AMBIGUOUS = 2, // several but not all devices jumped, or their deltas disagree
    JUMP_UNVERIFIABLE = 3, // one device jumped, fewer than n_jurors other devices available
    BACK_ON_PATH = 4, // SUSPECT device returned to its pre-jump path within t_confirm
    CONFIRM_TIMEOUT = 5, // SUSPECT lasted longer than t_confirm
    RESIDUAL_OK = 6, // head-tracker residual under r_ok for t_recover
    JUMP_BACK = 7, // a second jump brought the residual under r_ok at once
    SOLVE_AGREES = 8, // recovery solve agreed with the frozen calibration
    SOLVE_DISAGREES = 9, // recovery solve failed or disagreed
    MANUAL = 10, // trigger hold or UI reset
    DEVICE_LOST = 11, // device stopped tracking or disconnected; state reset
    DISTANCE_ODD_ONE_OUT = 12, // pairwise distances stepped against every other device
    RESIDUAL_HIGH = 13, // trusted head tracker drifted far from the rigid offset without a visible jump
    QUIET = 14, // non-target device: no jump for t_recover
    JUMP_WHILE_RECOVERING = 15,
};

enum class Event : uint8_t {
    NONE = 0,
    PLAYSPACE_JUMP = 1, // every target-universe device jumped together; delta attached
    REFERENCE_DISCONTINUITY = 2, // the reference device (HMD) jumped alone
};

inline const char* stateName(State s)
{
    switch (s) {
    case State::TRUSTED: return "TRUSTED";
    case State::SUSPECT: return "SUSPECT";
    case State::UNTRUSTED: return "UNTRUSTED";
    case State::RECOVERING: return "RECOVERING";
    default: return "?";
    }
}

inline const char* reasonName(Reason r)
{
    switch (r) {
    case Reason::NONE: return "none";
    case Reason::JUMP_SINGLE: return "jump_single";
    case Reason::JUMP_AMBIGUOUS: return "jump_ambiguous";
    case Reason::JUMP_UNVERIFIABLE: return "jump_unverifiable";
    case Reason::BACK_ON_PATH: return "back_on_path";
    case Reason::CONFIRM_TIMEOUT: return "confirm_timeout";
    case Reason::RESIDUAL_OK: return "residual_ok";
    case Reason::JUMP_BACK: return "jump_back";
    case Reason::SOLVE_AGREES: return "solve_agrees";
    case Reason::SOLVE_DISAGREES: return "solve_disagrees";
    case Reason::MANUAL: return "manual";
    case Reason::DEVICE_LOST: return "device_lost";
    case Reason::DISTANCE_ODD_ONE_OUT: return "distance_odd_one_out";
    case Reason::RESIDUAL_HIGH: return "residual_high";
    case Reason::QUIET: return "quiet";
    case Reason::JUMP_WHILE_RECOVERING: return "jump_while_recovering";
    default: return "?";
    }
}

} // namespace spacecal::trust
