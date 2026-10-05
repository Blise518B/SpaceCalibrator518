#include "head_mount.h"

#include <algorithm>
#include <cmath>

namespace spacecal::trust {

namespace {
    constexpr double k_RAD_TO_DEG = 57.29577951308232;
    constexpr double k_SCATTER_TIME_CONSTANT = 30.0; // s
    constexpr double k_IMPROVE_MARGIN = 0.005; // m: a solve must bring the head tracker this much closer to X to count as an improvement

    double angleBetween(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b)
    {
        const double c = std::clamp(((a.transpose() * b).trace() - 1.0) * 0.5, -1.0, 1.0);
        return std::acos(c) * k_RAD_TO_DEG;
    }

    bool finiteIso(const Eigen::Isometry3d& m)
    {
        return m.matrix().allFinite();
    }
}

void HeadMount::reset()
{
    m_valid = false;
    m_X = Eigen::Isometry3d::Identity();
    m_learned = 0.0;
    m_rotScatter = 180.0;
    m_disagree = 0;
    m_disagreeX = Eigen::Isometry3d::Identity();
}

bool HeadMount::restore(const HeadMountState& s, const std::string& target, const std::string& reference)
{
    reset();
    if (s.target_serial.empty() || s.target_serial != target || s.reference_serial != reference)
        return false;
    const Eigen::Quaterniond q(s.qw, s.qx, s.qy, s.qz);
    const Eigen::Vector3d t(s.x, s.y, s.z);
    if (!t.allFinite() || !q.coeffs().allFinite() || q.norm() < 0.5 || t.norm() > 1.0 || !std::isfinite(s.learned_seconds))
        return false;
    m_X.linear() = q.normalized().toRotationMatrix();
    m_X.translation() = t;
    m_learned = std::max(0.0, s.learned_seconds);
    m_rotScatter = std::isfinite(s.rotation_scatter_deg) ? s.rotation_scatter_deg : 180.0;
    m_valid = true;
    return true;
}

HeadMountState HeadMount::state(const std::string& target, const std::string& reference) const
{
    HeadMountState s;
    s.target_serial = target;
    s.reference_serial = reference;
    const Eigen::Quaterniond q(m_X.linear());
    s.x = m_X.translation().x();
    s.y = m_X.translation().y();
    s.z = m_X.translation().z();
    s.qw = q.w();
    s.qx = q.x();
    s.qy = q.y();
    s.qz = q.z();
    s.learned_seconds = m_valid ? m_learned : 0.0;
    s.rotation_scatter_deg = m_rotScatter;
    return s;
}

void HeadMount::learn(const Eigen::Isometry3d& X, double dt)
{
    if (!finiteIso(X) || X.translation().norm() > 1.0)
        return;
    dt = std::clamp(dt, 0.0, 1.0);
    if (dt <= 0.0)
        return;
    if (!m_valid) {
        m_X = X;
        m_valid = true;
        m_learned = dt;
        m_rotScatter = 0.0;
        return;
    }
    // a plain average until learn_time_constant worth of samples, then an exponential one: early
    // samples count fully, later ones move the estimate only slowly
    const double alpha = std::min(1.0, dt / std::max(dt, std::min(m_params.learn_time_constant, m_learned + dt)));
    const double angle = angleBetween(m_X.linear(), X.linear());
    const double beta = std::min(1.0, dt / k_SCATTER_TIME_CONSTANT);
    m_rotScatter += beta * (angle - m_rotScatter);

    const Eigen::Quaterniond qa(m_X.linear());
    const Eigen::Quaterniond qb(X.linear());
    m_X.linear() = qa.slerp(alpha, qb).normalized().toRotationMatrix();
    m_X.translation() += alpha * (X.translation() - m_X.translation());
    m_learned += dt;
}

Eigen::Isometry3d HeadMount::placeCalibration(const Eigen::Isometry3d& current, const Eigen::Isometry3d& H, const Eigen::Isometry3d& T) const
{
    if (usesRotation())
        return H * m_X * T.inverse();
    Eigen::Isometry3d out = current;
    out.translation() += (H * m_X).translation() - (current * T).translation();
    return out;
}

void HeadMount::disagreement(const Eigen::Isometry3d& C, const Eigen::Isometry3d& H, const Eigen::Isometry3d& T, double& pos, double& deg) const
{
    const Eigen::Isometry3d Xc = H.inverse() * C * T;
    pos = (Xc.translation() - m_X.translation()).norm(); // a rotation keeps lengths: the same at the head in the world
    deg = angleBetween(Xc.linear(), m_X.linear());
}

SolveVerdict HeadMount::judgeSolve(const Eigen::Isometry3d& C, const Eigen::Isometry3d& H, const Eigen::Isometry3d& T, double* posOut, double* degOut,
    const Eigen::Isometry3d* current)
{
    if (!confident())
        return SolveVerdict::NO_MODEL;
    double pos = 0.0, deg = 0.0;
    disagreement(C, H, T, pos, deg);
    const bool useRotation = usesRotation();
    if (posOut)
        *posOut = pos;
    if (degOut)
        *degOut = deg;
    if (pos <= m_params.solve_max_pos && (!useRotation || deg <= m_params.solve_max_deg)) {
        m_disagree = 0;
        return SolveVerdict::AGREES;
    }
    // still too far, but a step towards X from a calibration that is even further off: taken (2026-10-01
    // 01:12 and 2026-10-02 04:00, replayed: after SteamVR moved its base stations the first solve got
    // the head tracker from 15 to 5.5 cm; rejecting it kept the worse calibration until the next one)
    if (current) {
        double posCurrent = 0.0, degCurrent = 0.0;
        disagreement(*current, H, T, posCurrent, degCurrent);
        if (pos < posCurrent - k_IMPROVE_MARGIN)
            return SolveVerdict::IMPROVES;
    }
    // disagreeing solves that keep putting the tracker at the same other place: it was re-mounted
    const Eigen::Isometry3d Xc = H.inverse() * C * T;
    const bool same = m_disagree > 0 && (Xc.translation() - m_disagreeX.translation()).norm() <= 0.5 * m_params.solve_max_pos
        && (!useRotation || angleBetween(Xc.linear(), m_disagreeX.linear()) <= m_params.solve_max_deg);
    if (same) {
        m_disagree++;
    } else {
        m_disagree = 1;
        m_disagreeX = Xc;
    }
    if (m_disagree >= m_params.solve_disagree_n) {
        m_X = Xc;
        m_learned = 0.0; // not used again until learned anew
        m_rotScatter = 0.0;
        m_disagree = 0;
        return SolveVerdict::REMOUNTED;
    }
    return SolveVerdict::DISAGREES;
}

void HeadMountFix::reset()
{
    m_window.clear();
    m_offset.setZero();
    m_prevGap.setZero();
    m_lastT = -1.0;
    m_quarantineUntil = -1e18;
    m_active = false;
    m_moved = 0.0;
    m_remaining = 0.0;
}

HeadMountFix::Output HeadMountFix::update(const Input& in)
{
    Output out;
    const HeadMountFixParams& p = m_params;
    if (!in.gap.allFinite() || !in.gapBefore.allFinite())
        return out;
    const double sinceLast = m_lastT >= 0.0 ? in.t - m_lastT : -1.0;
    const double dt = sinceLast > 0.0 ? std::min(sinceLast, 0.1) : 0.0;

    if (in.calibrationChanged) {
        // a solve, the trigger hold or a frame correction replaced the calibration: the fix is gone with it
        if (m_active) {
            out.interrupted = true;
            out.size = m_moved;
        }
        m_offset.setZero();
        m_active = false;
        m_window.clear();
    }
    // the head tracker (or the headset) jumped on its own: a step of the gap that no calibration change explains
    if (sinceLast > 0.0 && sinceLast <= p.max_tick) {
        const double step = (in.gapBefore - m_prevGap).norm();
        const bool slowEnough = in.headsetSpeed < p.jump_speed && in.headsetTurn < p.jump_turn_dps;
        if ((step > p.jump && slowEnough) || step > p.jump_any) {
            if (in.t >= m_quarantineUntil) {
                out.jumped = true;
                out.size = step;
            }
            m_quarantineUntil = in.t + p.quarantine;
            m_window.clear();
            m_active = false;
        }
    }
    m_prevGap = in.gap;
    m_lastT = in.t;

    // the cause went away (the gap without the fix is back near zero): undo the fix at once
    const Eigen::Vector3d raw = in.gap + m_offset;
    if (in.allowed && m_offset.norm() > p.snap_min && raw.norm() < p.snap) {
        out.snapped = true;
        out.size = m_offset.norm();
        out.shift = -m_offset;
        m_offset.setZero();
        m_active = false;
        m_window.clear();
        return out;
    }

    const bool slow = in.headsetSpeed < p.max_speed && in.trackerSpeed < p.max_speed && in.headsetTurn < p.max_turn_dps;
    const bool gate = in.allowed && slow && in.t >= m_quarantineUntil;
    if (gate)
        m_window.push_back({ in.t, raw, dt });
    while (!m_window.empty() && m_window.front().t < in.t - p.hold)
        m_window.pop_front();
    if (!gate)
        return out;
    double covered = 0.0;
    for (const Entry& e : m_window)
        covered += e.dt;
    if (covered < 0.8 * p.hold || m_window.size() < 3)
        return out;

    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    double tMean = 0.0;
    for (const Entry& e : m_window) {
        mean += e.raw;
        tMean += e.t;
    }
    const double n = static_cast<double>(m_window.size());
    mean /= n;
    tMean /= n;
    double scatter = 0.0, tt = 0.0;
    Eigen::Vector3d ty = Eigen::Vector3d::Zero();
    for (const Entry& e : m_window) {
        scatter += (e.raw - mean).squaredNorm();
        tt += (e.t - tMean) * (e.t - tMean);
        ty += (e.t - tMean) * (e.raw - mean);
    }
    scatter = std::sqrt(scatter / n);
    Eigen::Vector3d slope = Eigen::Vector3d::Zero();
    if (tt > 1e-9)
        slope = ty / tt;
    const Eigen::Vector3d m = mean - m_offset; // what is still off with the fix so far
    const double mag = m.norm();
    m_remaining = mag;
    const bool steady = scatter < p.steady && slope.norm() < p.trend;

    if (!m_active && steady && mag > p.on && mag <= p.max_gap) {
        m_active = true;
        m_moved = 0.0;
        out.started = true;
        out.size = mag;
        out.gap = m;
    }
    if (!m_active)
        return out;
    if (mag < p.off) {
        m_active = false;
        out.finished = true;
        out.size = m_moved;
        return out;
    }
    const double step = std::min(mag, p.rate * dt);
    const Eigen::Vector3d s = m / mag * step;
    m_offset += s;
    m_moved += step;
    out.shift = s;
    return out;
}

} // namespace spacecal::trust
