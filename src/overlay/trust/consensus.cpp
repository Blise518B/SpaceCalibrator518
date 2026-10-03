#include "consensus.h"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

namespace spacecal::trust {

namespace {
    Eigen::Isometry3d isoFrom(const PoseSample& s)
    {
        Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
        iso.linear() = s.q.normalized().toRotationMatrix();
        iso.translation() = s.p;
        return iso;
    }

    Eigen::Isometry3d blendIso(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double alpha)
    {
        Eigen::Quaterniond qa(a.linear());
        Eigen::Quaterniond qb(b.linear());
        Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
        out.linear() = qa.slerp(alpha, qb).toRotationMatrix();
        out.translation() = a.translation() + alpha * (b.translation() - a.translation());
        return out;
    }

    bool isoClose(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double dPos, double dAngleDeg)
    {
        if ((a.translation() - b.translation()).norm() > dPos)
            return false;
        Eigen::Quaterniond qa(a.linear());
        Eigen::Quaterniond qb(b.linear());
        const double dot = std::min(1.0, std::abs(qa.dot(qb)));
        return 2.0 * std::acos(dot) * (180.0 / k_PI) <= dAngleDeg;
    }

    Eigen::Vector3d capped(const Eigen::Vector3d& v, double maxNorm)
    {
        const double n = v.norm();
        return n > maxNorm && n > 0.0 ? Eigen::Vector3d(v * (maxNorm / n)) : v;
    }

    const DeviceInput* findDevice(const TickInput& in, uint8_t index)
    {
        if (index >= 64)
            return nullptr;
        for (const DeviceInput& d : in.devices)
            if (d.index == index)
                return &d;
        return nullptr;
    }
}

double fitRigidDelta(const std::vector<Eigen::Vector3d>& a, const std::vector<Eigen::Vector3d>& b, Eigen::Isometry3d& delta)
{
    delta = Eigen::Isometry3d::Identity();
    if (a.size() < 2 || a.size() != b.size())
        return -1.0;

    Eigen::Vector3d ca = Eigen::Vector3d::Zero(), cb = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < a.size(); i++) {
        ca += a[i];
        cb += b[i];
    }
    ca /= static_cast<double>(a.size());
    cb /= static_cast<double>(b.size());

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    if (a.size() >= 3) {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < a.size(); i++)
            H += (a[i] - ca) * (b[i] - cb).transpose();
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d d = Eigen::Matrix3d::Identity();
        if ((svd.matrixV() * svd.matrixU().transpose()).determinant() < 0)
            d(2, 2) = -1.0;
        R = svd.matrixV() * d * svd.matrixU().transpose();
    }
    delta.linear() = R;
    delta.translation() = cb - R * ca;

    double sum = 0.0;
    for (size_t i = 0; i < a.size(); i++)
        sum += (delta * a[i] - b[i]).squaredNorm();
    return std::sqrt(sum / static_cast<double>(a.size()));
}

Consensus::DeviceState& Consensus::slot(uint8_t index)
{
    return m_devices[index < 64 ? index : 63];
}

void Consensus::reset()
{
    for (auto& d : m_devices)
        d = DeviceState {};
    m_CfrozenValid = false;
    m_CLValid = false;
    m_lastT = -1.0;
    m_shiftStartT = -1.0;
    m_refJumpT = -1.0;
    m_anchorPending = false;
}

void Consensus::clearPending()
{
    for (auto& d : m_devices) {
        d.pending = false;
        d.winHasFrom = false;
        d.winFreshAfterOpen = false;
        d.winU = Eigen::Vector3d::Zero();
    }
    m_shiftStartT = -1.0;
}

DeviceStatus Consensus::status(uint8_t device) const
{
    DeviceStatus s;
    if (device >= 64)
        return s;
    const DeviceState& d = m_devices[device];
    s.known = d.known;
    s.state = d.state;
    s.reason = d.reason;
    s.since = d.since;
    s.lastFlag = d.lastFlag;
    s.lastResidual = d.lastResidual;
    s.tracking = d.lastSample.tracking;
    s.fresh = d.lastResult.fresh;
    s.unexplained = d.lastResult.fresh ? d.lastResult.unexplained : 0.0;
    s.jumpLimit = d.lastResult.jumpLimit;
    return s;
}

void Consensus::transition(DeviceState& d, uint8_t index, State to, Reason reason, double t, double residual, const float numbers[3], TickOutput& out)
{
    Transition tr;
    tr.device = index;
    tr.from = d.state;
    tr.to = to;
    tr.reason = reason;
    tr.t = t;
    tr.residual = static_cast<float>(residual);
    if (numbers) {
        tr.numbers[0] = numbers[0];
        tr.numbers[1] = numbers[1];
        tr.numbers[2] = numbers[2];
    }
    out.transitions.push_back(tr);
    d.state = to;
    d.reason = reason;
    d.since = t;
    if (to == State::TRUSTED || to == State::RECOVERING) {
        d.residualOkSince = -1.0;
        d.residualHighSince = -1.0;
        d.quietSince = -1.0;
    }
}

void Consensus::forceTrustAll(double t, std::vector<Transition>* transitions)
{
    TickOutput scratch;
    for (uint8_t i = 0; i < 64; i++) {
        DeviceState& d = m_devices[i];
        if (d.known && d.state != State::TRUSTED) {
            transition(d, i, State::TRUSTED, Reason::MANUAL, t, d.lastResidual, nullptr, scratch);
        }
        d.residualOkSince = -1.0;
        d.residualHighSince = -1.0;
        d.quietSince = -1.0;
    }
    // the next trusted tick re-baselines the frozen calibration and the local offset from scratch
    clearPending();
    m_CfrozenValid = false;
    m_CLValid = false;
    m_refJumpT = -1.0;
    if (transitions)
        transitions->insert(transitions->end(), scratch.transitions.begin(), scratch.transitions.end());
}

int Consensus::countJurors(const TickInput& in, uint8_t except) const
{
    int n = 0;
    for (const DeviceInput& dev : in.devices) {
        if (dev.index == except || dev.universe != Universe::TARGET || !dev.bodyWorn || dev.index >= 64)
            continue;
        const DeviceState& d = m_devices[dev.index];
        if (!dev.sample.tracking || d.state != State::TRUSTED)
            continue;
        if (d.lastFlag == Flag::OK || d.lastFlag == Flag::NOISY)
            n++;
    }
    return n;
}

bool Consensus::computeResidual(const DeviceInput* ref, const DeviceInput* tgt, double& r, Eigen::Vector3d* vec) const
{
    if (!ref || !tgt || !ref->sample.tracking || !tgt->sample.tracking || !m_CfrozenValid || !m_CLValid)
        return false;
    const Eigen::Isometry3d H = isoFrom(ref->sample);
    const Eigen::Isometry3d T = isoFrom(tgt->sample);
    const Eigen::Vector3d expected = (H * m_CL).translation();
    const Eigen::Vector3d calibrated = (m_Cfrozen * T).translation();
    r = (expected - calibrated).norm();
    if (vec)
        *vec = calibrated - expected;
    return true;
}

void Consensus::updateCalibrationEstimates(const TickInput& in, const DeviceInput* ref, const DeviceInput* tgt, bool residualValid, double residual)
{
    const CalibrationInput& c = in.calib;
    if (!c.valid)
        return;
    // The applied calibration is followed even while the head tracker is in doubt. In observe-only
    // mode the solver keeps applying new calibrations, and judging against a stale one kept the
    // residual high for good (2026-09-30: head tracker UNTRUSTED from 02:10 to 06:20 while the
    // solver had long corrected the calibration). With hold on, the solver does not apply while the
    // tracker is in doubt, so nothing changes here.
    //
    // A solve or correction that moves the head tracker's expected pose by more than r_ok at once
    // reads as a glitch against the estimate learned under the previous calibration, and since that
    // estimate is only learned while the tracker is trusted, the tracker never came back (replay of
    // 2026-09-30: a 10 cm startup solve, a 6 cm continuous solve and two playspace corrections of 12
    // and 20 cm each left it UNTRUSTED for the rest of the segment). The applied calibration is the
    // solver's word on the rigid offset, so the estimate is re-anchored to it. A glitch without a
    // calibration change (with hold on, the solver does not apply while the tracker is in doubt) is
    // still judged against the old one.
    const bool bothTracked = ref && tgt && ref->sample.tracking && tgt->sample.tracking;
    if (c.relativeMode && c.C_L_valid) {
        if (m_CLValid && m_CfrozenValid) {
            if (ref && ref->sample.tracking) {
                const Eigen::Isometry3d H = isoFrom(ref->sample);
                if (((H * c.C_L).translation() - (H * m_CL).translation()).norm() > m_params.r_ok)
                    m_anchorPending = true;
            } else if (!isoClose(c.C_L, m_CL, 1e-6, 1e-4)) {
                m_anchorPending = true; // cannot tell how far; re-anchor when both are tracked again
            }
        }
        m_CL = c.C_L;
        m_CLValid = true;
        if (m_anchorPending && bothTracked && m_CfrozenValid) {
            m_Cfrozen = isoFrom(ref->sample) * m_CL * isoFrom(tgt->sample).inverse();
            m_anchorPending = false;
            m_anchors++;
        }
    } else if (!c.relativeMode && c.C_world_valid) {
        if (m_CfrozenValid && m_CLValid && !isoClose(c.C_world, m_Cfrozen, 1e-6, 1e-4)) {
            if (ref && ref->sample.tracking) {
                const Eigen::Vector3d atHead = m_Cfrozen.inverse() * ref->sample.p; // near the head tracker
                if ((c.C_world * atHead - m_Cfrozen * atHead).norm() > m_params.r_ok)
                    m_anchorPending = true;
            } else {
                m_anchorPending = true;
            }
        }
        m_Cfrozen = c.C_world;
        m_CfrozenValid = true;
        if (m_anchorPending && bothTracked && m_CLValid) {
            m_CL = isoFrom(ref->sample).inverse() * m_Cfrozen * isoFrom(tgt->sample);
            m_anchorPending = false;
            m_anchors++;
        }
    }
    if (!bothTracked)
        return;
    if (tgt->index >= 64 || m_devices[tgt->index].state != State::TRUSTED || m_devices[tgt->index].pending)
        return; // the learned estimates are frozen while the head tracker is in doubt

    const Eigen::Isometry3d H = isoFrom(ref->sample);
    const Eigen::Isometry3d T = isoFrom(tgt->sample);

    if (c.relativeMode) {
        if (!c.C_L_valid)
            return;
        // The live world correction H * C_L * T^-1 is exactly what a glitch corrupts, so the
        // frozen calibration follows it only slowly and only while the residual is healthy.
        const Eigen::Isometry3d live = H * m_CL * T.inverse();
        if (!m_CfrozenValid) {
            m_Cfrozen = live;
            m_CfrozenValid = true;
        } else if (!residualValid || residual <= m_params.r_ok) {
            m_Cfrozen = blendIso(m_Cfrozen, live, m_params.c_l_smoothing);
        }
        return;
    }

    if (!c.C_world_valid)
        return;
    // fixed mode: the applied calibration is the truth (taken above); the local offset is estimated from it
    if (m_refJumpT >= 0.0) {
        // the HMD jumped (re-center): H is inconsistent with C_world until the solver applies a new
        // calibration. No residual is judged in the meantime (m_CL was invalidated).
        if (isoClose(c.C_world, m_CworldAtRefJump, 1e-6, 1e-4))
            return;
        m_refJumpT = -1.0;
        m_CLValid = false; // re-learn the offset from the new calibration
    }
    const Eigen::Isometry3d est = H.inverse() * c.C_world * T;
    if (!m_CLValid) {
        m_CL = est;
        m_CLValid = true;
    } else if (!residualValid || residual <= m_params.r_ok) {
        // learn slowly from consistent ticks only, so a ramping glitch is never absorbed into the offset
        m_CL = blendIso(m_CL, est, m_params.c_l_smoothing);
    }
}

void Consensus::distanceWitness(const TickInput& in, std::vector<uint8_t>& oddOnesOut)
{
    if (m_lastT < 0.0)
        return;
    const double dt = in.t - m_lastT;
    if (dt <= 0.0 || dt > m_params.plausibility.dt_max)
        return;
    const PlausibilityParams& pp = m_params.plausibility;
    // pairs whose both ends report velocity: the distance is compared with the one both velocities
    // predict (fast hands are explained); otherwise the fixed bound as before
    const double tolFixed = pp.v_max * dt + pp.j_tol;
    const double tolExplained = 2.0 * pp.v_unexplained * dt + pp.j_tol;

    struct Item {
        uint8_t index;
        Universe universe;
        Eigen::Vector3d prev;
        Eigen::Vector3d now;
        bool hasVelocity;
        Eigen::Vector3d predicted; // prev + mean velocity * dt
    };
    std::vector<Item> items;
    for (const DeviceInput& dev : in.devices) {
        if (dev.index >= 64 || !dev.bodyWorn || dev.universe == Universe::OTHER || !dev.sample.tracking)
            continue;
        const DeviceState& d = m_devices[dev.index];
        if (!d.calPValid)
            continue;
        Eigen::Vector3d now, nowV;
        if (dev.universe == Universe::TARGET) {
            if (!m_CfrozenValid)
                continue;
            now = m_Cfrozen * dev.sample.p;
            nowV = m_Cfrozen.linear() * dev.sample.v;
        } else {
            now = dev.sample.p;
            nowV = dev.sample.v;
        }
        const bool hasVelocity = d.calV.squaredNorm() > 0.0 || nowV.squaredNorm() > 0.0;
        const Eigen::Vector3d predicted = d.calP + capped(0.5 * (d.calV + nowV), pp.v_body_max) * dt;
        items.push_back({ dev.index, dev.universe, d.calP, now, hasVelocity, predicted });
    }
    const size_t n = items.size();
    if (n < 3)
        return;

    std::vector<std::vector<bool>> stepped(n, std::vector<bool>(n, false));
    for (size_t i = 0; i < n; i++) {
        for (size_t j = i + 1; j < n; j++) {
            const double dNow = (items[i].now - items[j].now).norm();
            const bool explained = items[i].hasVelocity && items[j].hasVelocity;
            const double dExpected = explained ? (items[i].predicted - items[j].predicted).norm() : (items[i].prev - items[j].prev).norm();
            if (std::abs(dNow - dExpected) > (explained ? tolExplained : tolFixed)) {
                stepped[i][j] = stepped[j][i] = true;
            }
        }
    }
    for (size_t i = 0; i < n; i++) {
        if (items[i].universe != Universe::TARGET)
            continue;
        int withOthers = 0;
        for (size_t j = 0; j < n; j++)
            if (j != i && stepped[i][j])
                withOthers++;
        if (withOthers < 2 || withOthers < static_cast<int>(n) - 1)
            continue; // must have stepped against everyone else
        bool othersQuiet = true;
        for (size_t j = 0; j < n && othersQuiet; j++)
            for (size_t k = j + 1; k < n; k++)
                if (j != i && k != i && stepped[j][k]) {
                    othersQuiet = false;
                    break;
                }
        if (othersQuiet)
            oddOnesOut.push_back(items[i].index);
    }
}

bool Consensus::resolveShift(const TickInput& in, const std::vector<uint8_t>& tracking, bool residualValid, double residual, std::vector<uint8_t>& judged, TickOutput& out)
{
    const double t = in.t;
    const double jTol = m_params.plausibility.j_tol;
    std::vector<uint8_t> displaced; // moved by more than j_tol that its velocity does not explain
    int quietWitnesses = 0;
    bool waiting = false;
    for (uint8_t idx : tracking) {
        const DeviceState& d = m_devices[idx];
        if (d.winHasFrom && d.winU.norm() > jTol) {
            displaced.push_back(idx);
        } else if (d.winFreshAfterOpen) {
            quietWitnesses++; // new poses after the step, and it stayed where its velocity says
        } else {
            waiting = true; // nothing new from it yet: it may still show the step
        }
    }
    const bool timedOut = t - m_shiftStartT >= m_params.t_shift_window;
    if (quietWitnesses == 0 && waiting && !timedOut)
        return false; // keep waiting; a pending head tracker already holds (see outputs)

    bool playspaceJump = false;
    int outlier = -1; // a device the shift's fit left out, judged on its own
    if (quietWitnesses == 0 && displaced.size() >= 2) {
        // the whole target universe moved: one rigid delta must explain every displacement
        auto fitWithout = [&](int skip, Eigen::Isometry3d& delta) {
            std::vector<Eigen::Vector3d> a, b;
            for (size_t i = 0; i < displaced.size(); i++) {
                if (static_cast<int>(i) == skip)
                    continue;
                const DeviceState& d = m_devices[displaced[i]];
                a.push_back(d.winFrom);
                b.push_back(d.winFrom + d.winU);
            }
            return fitRigidDelta(a, b, delta);
        };
        Eigen::Isometry3d delta;
        double fit = fitWithout(-1, delta);
        if (!(fit >= 0.0 && fit <= m_params.jump_fit_residual) && displaced.size() >= 3) {
            // one device off (2026-10-02 04:14:55: a tracker coming back from another base
            // station's frame while the universe moved) must not hide the shift of all others
            double best = -1.0;
            Eigen::Isometry3d bestDelta;
            for (size_t skip = 0; skip < displaced.size(); skip++) {
                Eigen::Isometry3d dlt;
                const double f = fitWithout(static_cast<int>(skip), dlt);
                if (f >= 0.0 && (best < 0.0 || f < best)) {
                    best = f;
                    bestDelta = dlt;
                    outlier = static_cast<int>(skip);
                }
            }
            if (best >= 0.0 && best <= m_params.jump_fit_residual) {
                fit = best;
                delta = bestDelta;
            } else {
                outlier = -1;
            }
        }
        if (fit >= 0.0 && fit <= m_params.jump_fit_residual) {
            playspaceJump = true;
            out.event = Event::PLAYSPACE_JUMP;
            out.eventDelta = delta;
            out.eventResidual = fit;
            if (m_CfrozenValid) {
                // C_new * (delta * T) == C_old * T  ->  C_new = C_old * delta^-1
                m_Cfrozen = m_Cfrozen * delta.inverse();
            }
            for (size_t i = 0; i < displaced.size(); i++)
                if (static_cast<int>(i) != outlier)
                    m_devices[displaced[i]].lastJumpT = -1.0;
        } else {
            outlier = -1;
        }
    }
    if (playspaceJump && outlier >= 0 && m_devices[displaced[outlier]].pending) {
        const uint8_t idx = displaced[outlier];
        DeviceState& d = m_devices[idx];
        const int jurors = countJurors(in, idx);
        const Reason reason = jurors >= m_params.n_jurors ? Reason::JUMP_SINGLE : Reason::JUMP_UNVERIFIABLE;
        const float numbers[3] = { static_cast<float>(d.winU.norm()), d.pendingNumbers[1], d.pendingNumbers[2] };
        judgeJump(in, idx, reason, d.pendingResult, d.winU, numbers, residualValid, residual, out);
        judged.push_back(idx);
    }
    if (!playspaceJump) {
        // judge the devices whose own step was flagged and that are still displaced (a spike that
        // came back within the window is forgotten)
        std::vector<uint8_t> toJudge;
        for (uint8_t idx : displaced)
            if (m_devices[idx].pending)
                toJudge.push_back(idx);
        for (uint8_t idx : toJudge) {
            DeviceState& d = m_devices[idx];
            Reason reason = Reason::JUMP_AMBIGUOUS;
            if (toJudge.size() == 1 && displaced.size() == 1) {
                const int jurors = countJurors(in, idx);
                reason = jurors >= m_params.n_jurors ? Reason::JUMP_SINGLE : Reason::JUMP_UNVERIFIABLE;
            }
            const float numbers[3] = { static_cast<float>(d.winU.norm()), d.pendingNumbers[1], d.pendingNumbers[2] };
            judgeJump(in, idx, reason, d.pendingResult, d.winU, numbers, residualValid, residual, out);
            judged.push_back(idx);
        }
    }
    clearPending();
    return playspaceJump;
}

void Consensus::judgeJump(const TickInput& in, uint8_t idx, Reason reasonIfTrusted, const PlausibilityResult& step, const Eigen::Vector3d& stepDelta, const float numbers[3],
    bool residualValid, double residual, TickOutput& out)
{
    const double t = in.t;
    const uint8_t targetIndex = in.calib.targetIndex;
    DeviceState& d = m_devices[idx];
    d.lastJumpT = t;
    switch (d.state) {
    case State::TRUSTED:
        d.preJumpP = step.prevP;
        d.preJumpV = step.prevV;
        d.preJumpT = step.prevT;
        d.stepDelta = stepDelta;
        transition(d, idx, State::SUSPECT, reasonIfTrusted, t, residualValid && idx == targetIndex ? residual : 0.0, numbers, out);
        break;
    case State::SUSPECT:
        // a second step while suspect: keep the original pre-jump path, nothing to do
        break;
    case State::UNTRUSTED:
        if (idx == targetIndex && residualValid && residual < m_params.r_ok)
            transition(d, idx, State::RECOVERING, Reason::JUMP_BACK, t, residual, numbers, out);
        break;
    case State::RECOVERING:
        transition(d, idx, State::UNTRUSTED, Reason::JUMP_WHILE_RECOVERING, t, residualValid ? residual : 0.0, numbers, out);
        break;
    }
}

void Consensus::tick(const TickInput& in, TickOutput& out)
{
    const double t = in.t;
    const DeviceInput* ref = findDevice(in, in.calib.referenceIndex);
    const DeviceInput* tgt = findDevice(in, in.calib.targetIndex);
    const uint8_t targetIndex = in.calib.targetIndex;

    // 1. plausibility for every body-worn device of both universes
    std::vector<uint8_t> present;
    for (const DeviceInput& dev : in.devices) {
        if (dev.index >= 64 || dev.universe == Universe::OTHER || !dev.bodyWorn)
            continue;
        DeviceState& d = m_devices[dev.index];
        d.known = true;
        d.universe = dev.universe;
        d.bodyWorn = dev.bodyWorn;
        d.lastResult = evaluate(d.history, dev.sample, m_params.plausibility);
        d.lastFlag = d.lastResult.flag;
        d.lastSample = dev.sample;
        if (dev.sample.tracking)
            d.lastTrackingT = t;
        if (d.lastResult.fresh)
            d.lastFreshT = t;
        present.push_back(dev.index);
    }

    // 2. devices that vanished or stopped tracking for a while start over
    for (uint8_t i = 0; i < 64; i++) {
        DeviceState& d = m_devices[i];
        if (!d.known)
            continue;
        const bool lost = d.lastTrackingT >= 0.0 && (t - d.lastTrackingT) > m_params.t_device_lost;
        if (lost) {
            // a device that comes back after a dropout is judged afresh, but a distrusted one stays
            // distrusted: the glitch offset may still be there (it becomes UNTRUSTED so the residual /
            // quiet rules decide, never TRUSTED by mere absence)
            if (d.state == State::SUSPECT || d.state == State::RECOVERING)
                transition(d, i, State::UNTRUSTED, Reason::DEVICE_LOST, t, d.lastResidual, nullptr, out);
            d.history.reset();
            d.calPValid = false;
            d.pending = false;
            d.lastTrackingT = -1.0;
            d.residualOkSince = -1.0;
            d.quietSince = -1.0;
        }
    }

    // 3. head-tracker residual against the frozen calibration (uses last tick's estimates)
    double residual = 0.0;
    Eigen::Vector3d residualVec = Eigen::Vector3d::Zero();
    const bool residualValid = computeResidual(ref, tgt, residual, &residualVec);
    if (residualValid && targetIndex < 64)
        m_devices[targetIndex].lastResidual = residual;
    out.targetResidualValid = residualValid;
    out.targetResidual = residual;
    out.targetResidualVec = residualVec;

    // 4. who stepped in the target universe, who is tracking with history. A step does not make a
    //    device SUSPECT at once: it waits (t_shift_window) until every other tracking device has shown
    //    whether it stepped too, so a playspace shift that reaches the devices over a few frames is
    //    recognised as one (2026-09-30 05:41:02: five devices stepped in one frame, the head tracker in
    //    the next, and all five were distrusted).
    std::vector<uint8_t> oddOnesOut;
    distanceWitness(in, oddOnesOut);

    std::vector<uint8_t> tracking, oddOnly;
    bool anyStep = false;
    for (uint8_t idx : present) {
        DeviceState& d = m_devices[idx];
        if (d.universe != Universe::TARGET || !d.lastSample.tracking)
            continue;
        if (d.lastFlag == Flag::NO_HISTORY || d.lastFlag == Flag::NOT_TRACKING)
            continue;
        tracking.push_back(idx);
        anyStep = anyStep || d.lastFlag == Flag::JUMP;
    }
    if (anyStep && m_shiftStartT < 0.0) {
        clearPending();
        m_shiftStartT = t;
    }
    for (uint8_t idx : tracking) {
        DeviceState& d = m_devices[idx];
        const bool odd = std::find(oddOnesOut.begin(), oddOnesOut.end(), idx) != oddOnesOut.end();
        if (odd)
            d.lastOddT = t;
        if (m_shiftStartT >= 0.0) {
            if (d.lastResult.fresh) {
                if (!d.winHasFrom) {
                    d.winFrom = d.lastResult.explainedP;
                    d.winHasFrom = true;
                }
                d.winU += d.lastResult.unexplainedVec; // a spike coming back shows up here too
                if (t > m_shiftStartT)
                    d.winFreshAfterOpen = true;
            }
            if (d.lastFlag == Flag::JUMP && !d.pending) {
                d.pending = true;
                d.pendingResult = d.lastResult;
                d.pendingNumbers[0] = static_cast<float>(d.lastResult.unexplained);
                d.pendingNumbers[1] = static_cast<float>(d.lastResult.dt);
                d.pendingNumbers[2] = static_cast<float>(countJurors(in, idx));
            }
        }
        if (odd && !d.pending)
            oddOnly.push_back(idx);
    }

    // 5. the reference jumped alone: not a glitch of ours; the solver re-baselines
    const bool refJumped = ref && ref->index < 64 && m_devices[ref->index].lastFlag == Flag::JUMP && m_devices[ref->index].lastResult.unexplained >= m_params.ref_jump_min;
    if (refJumped && m_shiftStartT < 0.0) {
        out.event = Event::REFERENCE_DISCONTINUITY;
        out.referenceDiscontinuity = true;
        for (uint8_t i = 0; i < 64; i++) {
            m_devices[i].residualHighSince = -1.0;
        }
        if (!in.calib.relativeMode) {
            // fixed mode: the stored calibration is stale until the solver re-solves; stop judging
            // residuals until C_world changes (see updateCalibrationEstimates)
            m_refJumpT = t;
            m_CworldAtRefJump = in.calib.C_world;
            m_CLValid = false;
        } else {
            // relative mode: the live correction follows the HMD; re-baseline the frozen value at once
            m_CfrozenValid = false;
        }
    }

    // 6. the pending steps: everyone in the target universe stepped together -> playspace moved,
    //    nobody is distrusted; otherwise each displaced device is judged on its own
    bool playspaceJump = false;
    std::vector<uint8_t> judged;
    if (m_shiftStartT >= 0.0)
        playspaceJump = resolveShift(in, tracking, residualValid, residual, judged, out);

    // 7. devices the distance witness singled out without a step of their own
    for (uint8_t idx : oddOnly) {
        DeviceState& d = m_devices[idx];
        const float numbers[3] = { static_cast<float>(d.lastResult.d), static_cast<float>(d.lastResult.dt), static_cast<float>(countJurors(in, idx)) };
        const Reason reason = oddOnly.size() == 1 ? Reason::DISTANCE_ODD_ONE_OUT : Reason::JUMP_AMBIGUOUS;
        judgeJump(in, idx, reason, d.lastResult, Eigen::Vector3d::Zero(), numbers, residualValid, residual, out);
        judged.push_back(idx);
    }

    // 8. timers and recoveries
    for (uint8_t idx : tracking) {
        DeviceState& d = m_devices[idx];
        const DeviceInput* dev = findDevice(in, idx);
        if (!dev)
            continue;
        const bool isTarget = idx == targetIndex;
        const bool jumpedNow = d.pending || d.lastFlag == Flag::JUMP || std::find(judged.begin(), judged.end(), idx) != judged.end();

        switch (d.state) {
        case State::SUSPECT: {
            if (d.reason == Reason::RESIDUAL_HIGH || d.reason == Reason::DISTANCE_ODD_ONE_OUT) {
                // no step to return to; judge after t_confirm by the residual (head tracker) or by
                // whether the device kept standing out from the others
                if (t - d.since >= m_params.t_confirm) {
                    bool fine;
                    if (isTarget && residualValid)
                        fine = residual < m_params.r_ok;
                    else
                        fine = (t - d.lastOddT) > m_params.t_confirm * 0.5 && d.lastFlag != Flag::JUMP;
                    if (fine)
                        transition(d, idx, State::TRUSTED, Reason::BACK_ON_PATH, t, residualValid ? residual : 0.0, nullptr, out);
                    else {
                        d.rStep = residualValid ? residual : 0.0;
                        transition(d, idx, State::UNTRUSTED, Reason::CONFIRM_TIMEOUT, t, residualValid ? residual : 0.0, nullptr, out);
                    }
                }
                break;
            }
            const Eigen::Vector3d predicted = d.preJumpP + d.preJumpV * (t - d.preJumpT);
            const double deviation = (dev->sample.p - predicted).norm();
            const double tol = std::max(m_params.plausibility.j_tol, m_params.back_on_path_fraction * d.stepDelta.norm());
            const float numbers[3] = { static_cast<float>(deviation), static_cast<float>(tol), static_cast<float>(t - d.since) };
            if (deviation < tol) {
                transition(d, idx, State::TRUSTED, Reason::BACK_ON_PATH, t, residualValid && isTarget ? residual : 0.0, numbers, out);
            } else if (t - d.since >= m_params.t_confirm) {
                d.rStep = isTarget && residualValid ? residual : d.stepDelta.norm();
                transition(d, idx, State::UNTRUSTED, Reason::CONFIRM_TIMEOUT, t, d.rStep, numbers, out);
            }
            break;
        }
        case State::UNTRUSTED: {
            if (isTarget && residualValid) {
                if (residual < m_params.r_ok) {
                    if (d.residualOkSince < 0.0)
                        d.residualOkSince = t;
                    if (t - d.residualOkSince >= m_params.t_recover) {
                        const float numbers[3] = { static_cast<float>(residual), static_cast<float>(t - d.residualOkSince), static_cast<float>(d.rStep) };
                        transition(d, idx, State::RECOVERING, Reason::RESIDUAL_OK, t, residual, numbers, out);
                    }
                } else {
                    d.residualOkSince = -1.0;
                }
            } else {
                // non-target devices, and the head tracker while no calibration exists yet: quiet for
                // t_recover (no jump, not odd against the others) is the only evidence available
                if (jumpedNow || d.lastFlag == Flag::JUMP) {
                    d.quietSince = -1.0;
                } else {
                    if (d.quietSince < 0.0)
                        d.quietSince = t;
                    if (t - d.quietSince >= m_params.t_recover) {
                        const float numbers[3] = { static_cast<float>(t - d.quietSince), 0.0f, 0.0f };
                        transition(d, idx, State::TRUSTED, Reason::QUIET, t, 0.0, numbers, out);
                    }
                }
            }
            break;
        }
        case State::RECOVERING: {
            if (isTarget && in.calib.solveAttempted) {
                if (in.calib.solveValid && in.calib.solveAgrees)
                    transition(d, idx, State::TRUSTED, Reason::SOLVE_AGREES, t, residualValid ? residual : 0.0, nullptr, out);
                else
                    transition(d, idx, State::UNTRUSTED, Reason::SOLVE_DISAGREES, t, residualValid ? residual : 0.0, nullptr, out);
            } else if (!isTarget) {
                transition(d, idx, State::TRUSTED, Reason::QUIET, t, 0.0, nullptr, out);
            }
            break;
        }
        case State::TRUSTED: {
            if (isTarget && residualValid && !jumpedNow && !playspaceJump && !out.referenceDiscontinuity) {
                const bool refSuppressed = m_refJumpT >= 0.0 && !in.calib.relativeMode;
                if (residual > m_params.r_high && !refSuppressed) {
                    if (d.residualHighSince < 0.0)
                        d.residualHighSince = t;
                    // no juror requirement here: the rigid offset itself is the evidence, and the
                    // HMD-jumped-alone case was excluded above
                    if (t - d.residualHighSince >= m_params.t_residual_high) {
                        const float numbers[3] = { static_cast<float>(residual), static_cast<float>(t - d.residualHighSince), static_cast<float>(countJurors(in, idx)) };
                        d.stepDelta = Eigen::Vector3d::Zero();
                        transition(d, idx, State::SUSPECT, Reason::RESIDUAL_HIGH, t, residual, numbers, out);
                    }
                } else {
                    d.residualHighSince = -1.0;
                }
            }
            break;
        }
        }
    }

    // 9. calibration estimates (only advance while the head tracker is trusted)
    updateCalibrationEstimates(in, ref, tgt, residualValid, residual);

    // 10. outputs
    if (targetIndex < 64 && m_devices[targetIndex].known) {
        const DeviceState& td = m_devices[targetIndex];
        // a head-tracker step still being judged already holds: the verdict may come a few frames later
        out.holdTarget = td.state != State::TRUSTED || (td.pending && td.winU.norm() > m_params.plausibility.j_tol);
        out.targetRecovering = m_devices[targetIndex].state == State::RECOVERING;
    }

    // 11. positions in reference space for the next tick's distance witness
    for (uint8_t idx : present) {
        DeviceState& d = m_devices[idx];
        if (!d.lastSample.tracking) {
            d.calPValid = false;
            continue;
        }
        if (d.universe == Universe::TARGET) {
            if (m_CfrozenValid) {
                d.calP = m_Cfrozen * d.lastSample.p;
                d.calV = m_Cfrozen.linear() * d.lastSample.v;
                d.calPValid = true;
            } else {
                d.calPValid = false;
            }
        } else {
            d.calP = d.lastSample.p;
            d.calV = d.lastSample.v;
            d.calPValid = true;
        }
    }
    m_lastT = t;
}

} // namespace spacecal::trust
