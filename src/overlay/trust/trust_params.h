#pragma once

// guard.json -> ConsensusParams mapping, shared by the overlay and the replay tool (fork).

#include "consensus.h"
#include "guard/guard_config.h"

namespace spacecal::trust {

inline ConsensusParams paramsFromConfig(const guard::GuardConfig::Trust& c)
{
    ConsensusParams p;
    p.plausibility.v_unexplained = c.v_unexplained;
    p.plausibility.rot_unexplained_dps = c.rot_unexplained_dps;
    p.plausibility.v_body_max = c.v_body_max;
    p.plausibility.w_body_max_dps = c.w_body_max_dps;
    p.plausibility.v_max = c.v_max;
    p.plausibility.j_tol = c.j_tol;
    p.plausibility.v_err_max = c.v_err_max;
    p.plausibility.a_max = c.a_max;
    p.plausibility.rot_max_dps = c.rot_max_dps;
    p.plausibility.rot_tol_deg = c.rot_tol_deg;
    p.plausibility.t_stale = c.t_stale;
    p.plausibility.dt_max = c.dt_max;
    p.back_on_path_fraction = c.back_on_path_fraction;
    p.t_shift_window = c.t_shift_window;
    p.ref_jump_min = c.ref_jump_min;
    p.t_confirm = c.t_confirm;
    p.r_ok = c.r_ok;
    p.t_recover = c.t_recover;
    p.d_agree = c.d_agree;
    p.a_agree_deg = c.a_agree_deg;
    p.jump_fit_residual = c.jump_fit_residual;
    p.r_high = c.r_high;
    p.t_residual_high = c.t_residual_high;
    p.n_jurors = c.n_jurors;
    p.t_device_lost = c.t_device_lost;
    p.c_l_smoothing = c.c_l_smoothing;
    return p;
}

} // namespace spacecal::trust
