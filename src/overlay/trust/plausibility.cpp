#include "plausibility.h"

#include <algorithm>
#include <cmath>

namespace spacecal::trust {

namespace {
    Eigen::Vector3d capped(const Eigen::Vector3d& v, double maxNorm)
    {
        const double n = v.norm();
        return n > maxNorm && n > 0.0 ? Eigen::Vector3d(v * (maxNorm / n)) : v;
    }
}

PlausibilityResult evaluate(DeviceHistory& h, const PoseSample& s, const PlausibilityParams& params)
{
    PlausibilityResult r;

    if (!s.tracking) {
        // keep the last good sample so a short dropout does not erase the history, but do not
        // compare against it later than dt_max
        r.flag = Flag::NOT_TRACKING;
        h.staleSince = -1.0;
        return r;
    }

    if (!h.hasPrev) {
        h.hasPrev = true;
        h.prev = s;
        h.hasPrevVel = false;
        h.staleSince = -1.0;
        r.flag = Flag::NO_HISTORY;
        r.fresh = true;
        return r;
    }

    const double dt = s.t - h.prev.t;
    r.dt = dt;
    r.prevP = h.prev.p;
    r.prevT = h.prev.t;
    r.prevV = h.hasPrevVel ? h.prevObservedV : h.prev.v;

    if (dt <= 0.0) {
        // same tick twice (overlay polled faster than the device updates): nothing new
        r.flag = Flag::OK;
        return r;
    }
    if (dt > params.dt_max) {
        // long gap (device was off or overlay paused): restart the history, no verdict
        h.prev = s;
        h.hasPrevVel = false;
        h.staleSince = -1.0;
        r.flag = Flag::NO_HISTORY;
        r.fresh = true;
        return r;
    }

    // stale: identical pose for longer than t_stale. The history keeps the last *distinct* sample
    // so that, after a short stall, dt spans the real interval instead of one tick (a normally
    // moving device would otherwise look like a jump on the first fresh pose).
    const bool identical = (s.p - h.prev.p).norm() == 0.0 && std::abs(s.q.dot(h.prev.q)) > 0.9999999;
    if (identical) {
        if (h.staleSince < 0.0)
            h.staleSince = h.prev.t;
        r.flag = (s.t - h.staleSince > params.t_stale) ? Flag::STALE : Flag::OK;
        return r;
    }
    h.staleSince = -1.0;
    r.fresh = true;

    r.delta = s.p - h.prev.p;
    r.d = r.delta.norm();
    const Eigen::Vector3d observedV = r.delta / dt;
    r.vErr = (observedV - s.v).norm();
    if (h.hasPrevVel) {
        r.accel = (observedV - h.prevObservedV).norm() / dt;
    }
    const double dot = std::min(1.0, std::abs(s.q.dot(h.prev.q)));
    r.rotDeg = 2.0 * std::acos(dot) * (180.0 / k_PI);

    // position: the part of the step the reported velocity does not explain (trapezoid over the
    // interval, so steady acceleration is explained too). Exactly zero velocity on both ends means
    // the driver does not report one: fall back to the fixed bound.
    const bool hasVelocity = h.prev.v.squaredNorm() > 0.0 || s.v.squaredNorm() > 0.0;
    bool posJump;
    if (hasVelocity) {
        const Eigen::Vector3d vBar = capped(0.5 * (h.prev.v + s.v), params.v_body_max);
        r.explainedP = h.prev.p + vBar * dt;
        r.unexplainedVec = s.p - r.explainedP;
        r.unexplained = r.unexplainedVec.norm();
        r.jumpLimit = params.v_unexplained * dt + params.j_tol;
        posJump = r.unexplained > r.jumpLimit;
    } else {
        r.explainedP = h.prev.p;
        r.unexplainedVec = r.delta;
        r.unexplained = r.d;
        r.jumpLimit = params.v_max * dt + params.j_tol;
        posJump = r.d > r.jumpLimit;
    }

    // rotation: the same with the reported angular speed (magnitude only, frame-free)
    const double wPrev = h.prev.w.norm() * (180.0 / k_PI);
    const double wNow = s.w.norm() * (180.0 / k_PI);
    bool rotJump;
    if (wPrev > 0.0 || wNow > 0.0) {
        const double wBar = std::min(0.5 * (wPrev + wNow), params.w_body_max_dps);
        r.rotUnexplained = std::max(0.0, r.rotDeg - wBar * dt);
        rotJump = r.rotUnexplained > params.rot_unexplained_dps * dt + params.rot_tol_deg;
    } else {
        r.rotUnexplained = r.rotDeg;
        rotJump = r.rotDeg > params.rot_max_dps * dt + params.rot_tol_deg;
    }

    const bool noisy = r.vErr > params.v_err_max || (h.hasPrevVel && r.accel > params.a_max);

    if (posJump || rotJump)
        r.flag = Flag::JUMP;
    else if (noisy)
        r.flag = Flag::NOISY;
    else
        r.flag = Flag::OK;

    h.prevObservedV = observedV;
    h.hasPrevVel = true;
    h.prev = s;
    return r;
}

} // namespace spacecal::trust
