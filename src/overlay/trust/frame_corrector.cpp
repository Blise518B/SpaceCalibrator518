#include "frame_corrector.h"

#include <algorithm>
#include <cmath>

namespace spacecal::trust {

namespace {
    constexpr double k_RAD_TO_DEG = 57.29577951308232;
    constexpr size_t k_REFERENCE_HISTORY = 6;

    double angleOf(const Eigen::Matrix3d& r)
    {
        const double c = std::clamp((r.trace() - 1.0) * 0.5, -1.0, 1.0);
        return std::acos(c) * k_RAD_TO_DEG;
    }

    double moveAt(const Eigen::Isometry3d& e, const Eigen::Vector3d& x)
    {
        return (e * x - x).norm();
    }

    // e taken f of the way from identity, around the point x: x moves on the straight line to e * x
    Eigen::Isometry3d partial(const Eigen::Isometry3d& e, double f, const Eigen::Vector3d& x)
    {
        const Eigen::Quaterniond q = Eigen::Quaterniond(e.linear()).normalized();
        const Eigen::Quaterniond qf = Eigen::Quaterniond::Identity().slerp(f, q);
        const Eigen::Vector3d xf = x + f * (e * x - x);
        Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
        out.linear() = qf.toRotationMatrix();
        out.translation() = xf - out.linear() * x;
        return out;
    }
}

const char* frameEventName(FrameEventKind k)
{
    switch (k) {
    case FrameEventKind::REFERENCE_MOVED:
        return "reference_moved";
    case FrameEventKind::DEVICE_MOVED:
        return "device_moved";
    case FrameEventKind::SWITCH_ABSORBED:
        return "switch_absorbed";
    case FrameEventKind::SWITCH_SHOWN:
        return "switch_shown";
    case FrameEventKind::REFERENCE_SWITCHED:
        return "reference_switched";
    }
    return "?";
}

void FrameCorrector::reset()
{
    const FrameParams p = m_params;
    for (Device& d : m_dev)
        d = Device {};
    m_params = p;
    m_haveRef = false;
    m_correcting = false;
    m_Sr = Eigen::Isometry3d::Identity();
    m_Wref = Eigen::Isometry3d::Identity();
    m_refHistory.clear();
    m_G = Eigen::Isometry3d::Identity();
    m_lastT = -1.0;
    m_referenceMoves = m_deviceMoves = m_switchesAbsorbed = m_switchesShown = 0;
}

bool FrameCorrector::same(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) const
{
    if ((a.translation() - b.translation()).norm() > m_params.same_pos)
        return false;
    return angleOf(a.linear().transpose() * b.linear()) <= m_params.same_deg;
}

bool FrameCorrector::sameFamily(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) const
{
    // measured at the frame's origin (near its station, a few metres from the devices); the rotation
    // alone already tells stations apart (tens of degrees)
    const Eigen::Isometry3d delta = a * b.inverse();
    return moveAt(delta, b.translation()) <= m_params.family_pos && angleOf(delta.linear()) <= m_params.family_deg;
}

bool FrameCorrector::knownReferenceVersion(const Eigen::Isometry3d& w) const
{
    for (const Eigen::Isometry3d& v : m_refHistory)
        if (same(w, v))
            return true;
    return false;
}

Eigen::Isometry3d FrameCorrector::target(const Device& d) const
{
    // SteamVR's map, pinned at the reference family: a device on that family (any version, it may
    // lag behind) keeps exactly its place relative to the pinned frame; any other device gets G
    if (m_haveRef && sameFamily(d.W, m_Wref))
        return m_Sr * d.W.inverse();
    return m_G;
}

void FrameCorrector::tick(double t, const std::vector<FrameSample>& devices, bool correct, std::vector<FrameEvent>* events)
{
    const double dt = m_lastT >= 0.0 ? std::max(0.0, t - m_lastT) : 0.0;
    m_lastT = t;
    if (correct && !m_correcting) {
        // (re)starting to correct: from SteamVR's map as it is now; G stays (the calibration contains it)
        for (Device& d : m_dev)
            d.hasQ = false;
        if (m_haveRef)
            m_Sr = m_G * m_Wref;
    }
    m_correcting = correct;

    auto emit = [&](FrameEventKind kind, uint8_t index, const Eigen::Isometry3d& delta, double move, double angle) {
        if (!events)
            return;
        FrameEvent e;
        e.kind = kind;
        e.t = t;
        e.index = index;
        e.delta = delta;
        e.move = move;
        e.angleDeg = angle;
        e.corrected = correct && kind != FrameEventKind::SWITCH_SHOWN;
        events->push_back(e);
    };

    // the reference family starts with the head tracker
    if (!m_haveRef) {
        for (const FrameSample& s : devices) {
            if (s.reference && s.tracking && s.index < 64) {
                m_haveRef = true;
                m_Wref = s.wfd;
                m_Sr = m_G * m_Wref;
                m_refHistory.assign(1, m_Wref);
                break;
            }
        }
    }

    for (const FrameSample& s : devices) {
        if (s.index >= 64)
            continue;
        Device& d = m_dev[s.index];
        const bool wasTracking = (t - d.lastTracking) <= m_params.lost_after;
        if (!d.seen) {
            d.seen = true;
            d.W = s.wfd;
            d.pose = s.pose;
        } else if (!same(s.wfd, d.W)) {
            const Eigen::Vector3d xOld = (d.W * d.pose).translation(); // where it was in the raw world
            const Eigen::Isometry3d delta = s.wfd * d.W.inverse(); // raw_new = delta * raw_old
            const double move = moveAt(delta, xOld);
            const double angle = angleOf(delta.linear());
            if (move <= m_params.family_pos && angle <= m_params.family_deg) {
                // SteamVR moved this device's station on its map: P did not change, W did
                if (d.hasQ)
                    d.Q = d.Q * d.W * s.wfd.inverse();
                const bool onReference = m_haveRef && sameFamily(s.wfd, m_Wref);
                if (!onReference && s.tracking) {
                    m_deviceMoves++;
                    emit(FrameEventKind::DEVICE_MOVED, s.index, delta, move, angle);
                }
            } else if (s.reference) {
                // the head tracker changed stations: its new station is the reference family now,
                // pinned where G puts it (its own jump stays visible to the trust layer)
                if (m_haveRef) {
                    m_Wref = s.wfd;
                    m_Sr = m_G * m_Wref;
                    m_refHistory.assign(1, m_Wref);
                    emit(FrameEventKind::REFERENCE_SWITCHED, s.index, delta, 0.0, 0.0);
                }
            } else {
                // another station's frame: absorb the disagreement of the two stations, if the
                // samples around the switch are close enough to tell it from motion
                const bool close = d.lastSampleTracking && s.tracking && (t - d.lastSampleT) <= m_params.sample_gap;
                if (close) {
                    const Eigen::Isometry3d before = d.W * d.pose;
                    const Eigen::Isometry3d after = s.wfd * s.pose;
                    const Eigen::Isometry3d J = before * after.inverse();
                    const double jMove = moveAt(J, after.translation());
                    const double jAngle = angleOf(J.linear());
                    if (m_params.absorb_switches && d.hasQ && jMove <= m_params.switch_pos && jAngle <= m_params.switch_deg) {
                        d.Q = d.Q * J;
                        m_switchesAbsorbed++;
                        emit(FrameEventKind::SWITCH_ABSORBED, s.index, J, jMove, jAngle);
                    } else {
                        m_switchesShown++;
                        emit(FrameEventKind::SWITCH_SHOWN, s.index, J, jMove, jAngle);
                    }
                }
            }
            d.W = s.wfd;
        }

        // the reference family moves with the first tracking device that shows a new version of it
        if (m_haveRef && s.tracking && !same(d.W, m_Wref) && sameFamily(d.W, m_Wref) && !knownReferenceVersion(d.W)) {
            const Eigen::Isometry3d delta = d.W * m_Wref.inverse();
            const Eigen::Vector3d x = (m_Wref * s.pose).translation();
            m_Wref = d.W;
            m_refHistory.push_back(m_Wref);
            if (m_refHistory.size() > k_REFERENCE_HISTORY)
                m_refHistory.erase(m_refHistory.begin());
            if (correct)
                m_G = m_Sr * m_Wref.inverse();
            else
                m_Sr = m_G * m_Wref;
            m_referenceMoves++;
            emit(FrameEventKind::REFERENCE_MOVED, s.index, delta, moveAt(delta, x), angleOf(delta.linear()));
        }

        if (s.tracking) {
            if (!d.hasQ || !wasTracking)
                d.Q = target(d); // new, or back from a loss: SteamVR's map
            d.hasQ = true;
            d.lastTracking = t;
        }
        if (s.reference && d.hasQ)
            d.Q = target(d); // the head tracker is the reference: exact, never slides
        d.pose = s.pose;
        d.lastSampleT = t;
        d.lastSampleTracking = s.tracking;
    }

    // corrections slide back to SteamVR's map, slowly enough to pass for nothing
    if (!correct || dt <= 0.0 || (m_params.slide_mps <= 0.0 && m_params.slide_dps <= 0.0))
        return;
    for (const FrameSample& s : devices) {
        if (s.index >= 64 || !s.tracking || s.reference)
            continue;
        Device& d = m_dev[s.index];
        if (!d.hasQ)
            continue;
        const Eigen::Isometry3d T = target(d);
        const Eigen::Isometry3d E = T * d.Q.inverse(); // stable -> where SteamVR's map has it
        const Eigen::Vector3d x = d.Q * (d.W * d.pose).translation();
        const double mag = moveAt(E, x);
        const double ang = angleOf(E.linear());
        if (mag < 1e-4 && ang < 0.01) {
            d.Q = T;
            continue;
        }
        double f = 1.0;
        if (m_params.slide_mps > 0.0 && mag > 1e-9)
            f = std::min(f, dt * m_params.slide_mps / mag);
        if (m_params.slide_dps > 0.0 && ang > 1e-9)
            f = std::min(f, dt * m_params.slide_dps / ang);
        d.Q = partial(E, f, x) * d.Q;
    }
}

Eigen::Isometry3d FrameCorrector::correction(uint8_t index) const
{
    if (!m_correcting || index >= 64)
        return m_G;
    const Device& d = m_dev[index];
    if (!d.seen)
        return m_G;
    return d.hasQ ? d.Q : target(d);
}

Eigen::Isometry3d FrameCorrector::pin(uint8_t index) const
{
    return m_G.inverse() * correction(index);
}

} // namespace spacecal::trust
