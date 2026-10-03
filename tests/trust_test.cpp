// Trust layer tests (fork, docs/DESIGN.md sections 5, 6, 10). A small synthetic body walks
// around in two tracking universes; glitches are injected into single devices and the
// consensus is checked for the transitions the design promises.

#include "consensus.h"
#include "plausibility.h"
#include "test_support.h"

#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <vector>

using namespace spacecal::trust;

namespace {

constexpr uint8_t HMD = 0, CTRL_L = 1, CTRL_R = 2, HEAD_TRK = 3, HIP = 4, FOOT_L = 5, FOOT_R = 6;
constexpr double TICK = 0.05; // 20 Hz
constexpr double VEL_EPS = 1e-4; // s, for the instantaneous (IMU-like) velocity

Eigen::Isometry3d makeIso(double yawDeg, const Eigen::Vector3d& t)
{
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = Eigen::AngleAxisd(yawDeg * k_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    iso.translation() = t;
    return iso;
}

// The body in reference-world coordinates at time t.
struct Body {
    Eigen::Isometry3d hmd, ctrlL, ctrlR, headTrk, hip, footL, footR;
};

struct World {
    Eigen::Isometry3d C_true = makeIso(30.0, Eigen::Vector3d(1.0, 0.0, -0.5)); // target -> reference
    Eigen::Isometry3d C_L_true = makeIso(2.0, Eigen::Vector3d(0.0, 0.08, -0.05)); // head tracker in HMD frame
    double headSpeedScale = 1.0;
    double kickT0 = -1.0; // left foot kicks 40 cm forward over 0.12 s from here (peak 5.2 m/s), real motion

    Body at(double t) const
    {
        Body b;
        const Eigen::Vector3d c(0.3 * std::sin(0.5 * t), 0.0, 0.3 * std::cos(0.3 * t));
        const double yaw = 20.0 * std::sin(0.4 * t) * headSpeedScale;
        b.hmd = makeIso(yaw, c + Eigen::Vector3d(0.02 * std::sin(1.3 * t), 1.7 + 0.01 * std::sin(2.0 * t), 0.0));
        b.headTrk = b.hmd * C_L_true;
        b.hip = makeIso(yaw * 0.3, c + Eigen::Vector3d(0.0, 1.0, 0.0));
        b.footL = makeIso(0.0, c + Eigen::Vector3d(-0.15, 0.05 + 0.05 * std::max(0.0, std::sin(2.0 * t)), 0.1 * std::sin(2.0 * t)));
        if (kickT0 >= 0.0 && t >= kickT0) {
            const double tau = std::min(t - kickT0, 0.12);
            b.footL.translation().z() += 0.2 * (1.0 - std::cos(k_PI * tau / 0.12));
        }
        b.footR = makeIso(0.0, c + Eigen::Vector3d(0.15, 0.05 + 0.05 * std::max(0.0, -std::sin(2.0 * t)), -0.1 * std::sin(2.0 * t)));
        b.ctrlL = makeIso(0.0, c + Eigen::Vector3d(-0.3, 1.1 + 0.1 * std::sin(3.0 * t), -0.3));
        b.ctrlR = makeIso(0.0, c + Eigen::Vector3d(0.3, 1.1 + 0.1 * std::cos(3.0 * t), -0.3));
        return b;
    }
};

// Injected disturbance in a device's own universe: offset(t) applied to the raw pose.
using Disturbance = std::function<Eigen::Vector3d(double t)>;

struct Sim {
    World world;
    bool relativeMode = false;
    std::map<uint8_t, Disturbance> disturbances;
    Consensus consensus;
    std::vector<Transition> allTransitions;
    std::vector<Event> events;
    Eigen::Isometry3d C_world; // what the "solver" currently applies (fixed mode)
    Eigen::Isometry3d C_L_applied; // what the "solver" currently applies (relative mode)
    bool solveFeedbackPending = false;
    bool solveAgrees = true;
    bool autoAnswerRecovery = true; // pretend the manager ran an agreeing solve
    Eigen::Vector3d hmdOffset = Eigen::Vector3d::Zero();
    std::map<uint8_t, std::pair<double, double>> dropouts; // device -> [from, to) not tracking
    std::map<uint8_t, std::pair<double, double>> frozen; // device -> [from, to) repeats its last pose (no fresh data)
    std::map<uint8_t, PoseSample> lastSamples;
    bool calibValid = true;

    Sim()
    {
        C_world = world.C_true;
        C_L_applied = world.C_L_true;
    }

    // worldPoseRBefore / After: the pose a moment (VEL_EPS) around t, for the instantaneous velocity
    PoseSample sample(double t, const Eigen::Isometry3d& worldPoseR, bool inTarget, uint8_t index, const Eigen::Isometry3d& worldPoseRBefore, const Eigen::Isometry3d& worldPoseRAfter)
    {
        auto fr = frozen.find(index);
        if (fr != frozen.end() && t >= fr->second.first && t < fr->second.second && lastSamples.count(index)) {
            PoseSample stale = lastSamples[index];
            stale.t = t;
            return stale;
        }
        PoseSample s;
        s.t = t;
        s.tracking = true;
        Eigen::Isometry3d raw = inTarget ? Eigen::Isometry3d(world.C_true.inverse() * worldPoseR) : worldPoseR;
        const Eigen::Isometry3d rawBefore = inTarget ? Eigen::Isometry3d(world.C_true.inverse() * worldPoseRBefore) : worldPoseRBefore;
        const Eigen::Isometry3d rawAfter = inTarget ? Eigen::Isometry3d(world.C_true.inverse() * worldPoseRAfter) : worldPoseRAfter;
        Eigen::Vector3d v = (rawAfter.translation() - rawBefore.translation()) / (2.0 * VEL_EPS); // the driver's IMU velocity follows real motion
        auto it = disturbances.find(index);
        if (it != disturbances.end()) {
            raw.translation() += it->second(t);
        }
        if (index == HMD)
            raw.translation() += hmdOffset;
        auto dr = dropouts.find(index);
        if (dr != dropouts.end() && t >= dr->second.first && t < dr->second.second)
            s.tracking = false;
        s.p = raw.translation();
        s.q = Eigen::Quaterniond(raw.linear());
        s.v = v;
        lastSamples[index] = s;
        return s;
    }

    TickOutput step(double t)
    {
        const Body b = world.at(t);
        const Body bb = world.at(t - VEL_EPS);
        const Body ba = world.at(t + VEL_EPS);
        TickInput in;
        in.t = t;
        in.devices.push_back({ HMD, Universe::REFERENCE, true, sample(t, b.hmd, false, HMD, bb.hmd, ba.hmd) });
        in.devices.push_back({ CTRL_L, Universe::REFERENCE, true, sample(t, b.ctrlL, false, CTRL_L, bb.ctrlL, ba.ctrlL) });
        in.devices.push_back({ CTRL_R, Universe::REFERENCE, true, sample(t, b.ctrlR, false, CTRL_R, bb.ctrlR, ba.ctrlR) });
        in.devices.push_back({ HEAD_TRK, Universe::TARGET, true, sample(t, b.headTrk, true, HEAD_TRK, bb.headTrk, ba.headTrk) });
        in.devices.push_back({ HIP, Universe::TARGET, true, sample(t, b.hip, true, HIP, bb.hip, ba.hip) });
        in.devices.push_back({ FOOT_L, Universe::TARGET, true, sample(t, b.footL, true, FOOT_L, bb.footL, ba.footL) });
        in.devices.push_back({ FOOT_R, Universe::TARGET, true, sample(t, b.footR, true, FOOT_R, bb.footR, ba.footR) });
        in.calib.valid = calibValid;
        in.calib.relativeMode = relativeMode;
        in.calib.referenceIndex = HMD;
        in.calib.targetIndex = HEAD_TRK;
        in.calib.C_world_valid = !relativeMode;
        in.calib.C_world = C_world;
        in.calib.C_L_valid = relativeMode;
        in.calib.C_L = C_L_applied;
        if (solveFeedbackPending) {
            in.calib.solveAttempted = true;
            in.calib.solveValid = true;
            in.calib.solveAgrees = solveAgrees;
            solveFeedbackPending = false;
        }
        TickOutput out;
        consensus.tick(in, out);
        for (const Transition& tr : out.transitions)
            allTransitions.push_back(tr);
        if (out.event != Event::NONE)
            events.push_back(out.event);
        if (out.event == Event::PLAYSPACE_JUMP && !relativeMode) {
            C_world = C_world * out.eventDelta.inverse(); // what the manager applies to the solver
        }
        if (out.targetRecovering && autoAnswerRecovery)
            solveFeedbackPending = true;
        return out;
    }

    void run(double from, double to, std::function<void(double, const TickOutput&)> onTick = nullptr)
    {
        for (double t = from; t < to + 1e-9; t += TICK) {
            TickOutput out = step(t);
            if (onTick)
                onTick(t, out);
        }
    }

    int countTransitions(uint8_t device, State to, Reason reason = Reason::NONE) const
    {
        int n = 0;
        for (const Transition& tr : allTransitions)
            if (tr.device == device && tr.to == to && (reason == Reason::NONE || tr.reason == reason))
                n++;
        return n;
    }

    void printTransitions() const
    {
        for (const Transition& tr : allTransitions)
            std::printf("    t=%.2f dev %d %s -> %s (%s) r=%.3f n=[%.3f %.3f %.3f]\n", tr.t, tr.device, stateName(tr.from), stateName(tr.to), reasonName(tr.reason), tr.residual, tr.numbers[0], tr.numbers[1], tr.numbers[2]);
    }
};

Disturbance stepOffset(double t0, double t1, const Eigen::Vector3d& offset)
{
    return [=](double t) -> Eigen::Vector3d { return (t >= t0 && t < t1) ? offset : Eigen::Vector3d(0.0, 0.0, 0.0); };
}

} // namespace

TEST(plausibility_flags)
{
    PlausibilityParams p;
    DeviceHistory h;
    PoseSample s;
    s.tracking = true;
    s.t = 0.0;
    s.p = Eigen::Vector3d(0, 1, 0);
    CHECK(evaluate(h, s, p).flag == Flag::NO_HISTORY);
    s.t = 0.05;
    s.p = Eigen::Vector3d(0.05, 1, 0);
    s.v = Eigen::Vector3d(1.0, 0, 0);
    CHECK(evaluate(h, s, p).flag == Flag::OK);
    // 30 cm step in one tick, velocity says otherwise -> JUMP
    s.t = 0.10;
    s.p = Eigen::Vector3d(0.40, 1, 0);
    PlausibilityResult r = evaluate(h, s, p);
    CHECK(r.flag == Flag::JUMP);
    CHECK_NEAR(r.d, 0.35, 1e-9);
    CHECK(r.vErr > 5.0);
    // not tracking
    s.tracking = false;
    CHECK(evaluate(h, s, p).flag == Flag::NOT_TRACKING);
    // long gap resets
    s.tracking = true;
    s.t = 5.0;
    CHECK(evaluate(h, s, p).flag == Flag::NO_HISTORY);
    // stale
    s.t = 5.05;
    CHECK(evaluate(h, s, p).flag == Flag::OK);
    s.t = 5.40;
    CHECK(evaluate(h, s, p).flag == Flag::STALE);
}

TEST(rigid_fit)
{
    Eigen::Isometry3d d = makeIso(5.0, Eigen::Vector3d(0.2, 0.0, 0.1));
    std::vector<Eigen::Vector3d> a = { { 0, 1.7, 0 }, { 0, 1.0, 0.1 }, { -0.15, 0.05, 0 }, { 0.15, 0.05, 0.2 } };
    std::vector<Eigen::Vector3d> b;
    for (auto& p : a)
        b.push_back(d * p);
    Eigen::Isometry3d fit;
    double res = fitRigidDelta(a, b, fit);
    CHECK_NEAR(res, 0.0, 1e-9);
    CHECK((fit.translation() - d.translation()).norm() < 1e-9);
    // two points: translation only
    std::vector<Eigen::Vector3d> a2 = { a[0], a[1] }, b2 = { a[0] + Eigen::Vector3d(0.3, 0, 0), a[1] + Eigen::Vector3d(0.3, 0, 0) };
    res = fitRigidDelta(a2, b2, fit);
    CHECK_NEAR(res, 0.0, 1e-9);
    CHECK_NEAR(fit.translation().x(), 0.3, 1e-9);
}

TEST(healthy_play_has_no_transitions)
{
    for (int mode = 0; mode < 2; mode++) {
        Sim sim;
        sim.relativeMode = mode == 1;
        sim.world.headSpeedScale = 3.0; // fast head turns
        sim.run(0.0, 60.0);
        CHECK_EQ(sim.allTransitions.size(), 0u);
        CHECK_EQ(sim.events.size(), 0u);
        CHECK(sim.consensus.frozenCalibrationValid());
        CHECK(sim.consensus.localOffsetValid());
        // the estimated local offset matches the truth in fixed mode
        if (!sim.relativeMode) {
            CHECK((sim.consensus.localOffset().translation() - sim.world.C_L_true.translation()).norm() < 0.01);
        }
    }
}

TEST(head_tracker_30cm_step_is_held_and_recovers)
{
    for (int mode = 0; mode < 2; mode++) {
        Sim sim;
        sim.relativeMode = mode == 1;
        sim.disturbances[HEAD_TRK] = stepOffset(10.0, 20.0, Eigen::Vector3d(0.3, 0.0, 0.0));
        double firstHoldT = -1.0, lastHoldT = -1.0;
        double maxResidualDuring = 0.0;
        sim.run(0.0, 30.0, [&](double t, const TickOutput& out) {
            if (out.holdTarget) {
                if (firstHoldT < 0.0)
                    firstHoldT = t;
                lastHoldT = t;
            }
            if (t > 11.0 && t < 19.0 && out.targetResidualValid)
                maxResidualDuring = std::max(maxResidualDuring, out.targetResidual);
        });
        std::printf("  mode=%s transitions:\n", sim.relativeMode ? "relative" : "fixed");
        sim.printTransitions();
        // detection at the step tick, confirmation after t_confirm
        CHECK_EQ(sim.countTransitions(HEAD_TRK, State::SUSPECT, Reason::JUMP_SINGLE), 1);
        CHECK_EQ(sim.countTransitions(HEAD_TRK, State::UNTRUSTED, Reason::CONFIRM_TIMEOUT), 1);
        CHECK_NEAR(firstHoldT, 10.0, TICK + 1e-6);
        // the residual during the glitch is the step size
        CHECK_NEAR(maxResidualDuring, 0.3, 0.03);
        // recovery right at the step back, then the solve confirms
        CHECK_EQ(sim.countTransitions(HEAD_TRK, State::RECOVERING, Reason::JUMP_BACK), 1);
        CHECK_EQ(sim.countTransitions(HEAD_TRK, State::TRUSTED, Reason::SOLVE_AGREES), 1);
        CHECK(lastHoldT >= 20.0 - 1e-6);
        CHECK(lastHoldT <= 20.0 + 3 * TICK + 1e-6);
        // nobody else was touched
        for (uint8_t d : { HMD, CTRL_L, CTRL_R, HIP, FOOT_L, FOOT_R })
            CHECK_EQ(sim.countTransitions(d, State::SUSPECT), 0);
        // nothing after recovery
        int late = 0;
        for (auto& tr : sim.allTransitions)
            if (tr.t > 20.5)
                late++;
        CHECK_EQ(late, 0);
    }
}

TEST(one_frame_spike_costs_one_tick)
{
    // the spike is held for the one tick it lasts; it came back before it was judged, so no state
    // ever changes (it used to cost a SUSPECT -> TRUSTED pair)
    Sim sim;
    sim.disturbances[HEAD_TRK] = stepOffset(10.0, 10.0 + TICK / 2, Eigen::Vector3d(0.3, 0.0, 0.0));
    int heldTicks = 0;
    double heldAt = -1.0;
    sim.run(0.0, 15.0, [&](double t, const TickOutput& out) {
        if (out.holdTarget) {
            heldTicks++;
            heldAt = t;
        }
    });
    sim.printTransitions();
    CHECK_EQ(sim.allTransitions.size(), 0u);
    CHECK_EQ(heldTicks, 1);
    CHECK_NEAR(heldAt, 10.0, 1e-6);
}

TEST(hip_tracker_step_does_not_hold_the_calibration)
{
    Sim sim;
    sim.disturbances[HIP] = stepOffset(10.0, 15.0, Eigen::Vector3d(0.0, 0.0, 0.4));
    bool everHeld = false;
    sim.run(0.0, 25.0, [&](double, const TickOutput& out) { everHeld = everHeld || out.holdTarget; });
    sim.printTransitions();
    CHECK(!everHeld);
    // the step out and the step back are both single-device jumps; a displaced but quiet hip
    // is trusted again after t_recover because its motion is still coherent with the body
    CHECK_EQ(sim.countTransitions(HIP, State::SUSPECT, Reason::JUMP_SINGLE), 2);
    CHECK_EQ(sim.countTransitions(HIP, State::UNTRUSTED, Reason::CONFIRM_TIMEOUT), 2);
    CHECK_EQ(sim.countTransitions(HIP, State::TRUSTED, Reason::QUIET), 2);
    CHECK_EQ(sim.countTransitions(HEAD_TRK, State::SUSPECT), 0);
    for (uint8_t d : { HMD, CTRL_L, CTRL_R, FOOT_L, FOOT_R })
        CHECK_EQ(sim.countTransitions(d, State::SUSPECT), 0);
    // quiet recovery t_recover after the confirmation
    double firstTrustedAt = 0.0;
    for (auto& tr : sim.allTransitions)
        if (tr.device == HIP && tr.to == State::TRUSTED) {
            firstTrustedAt = tr.t;
            break;
        }
    CHECK(firstTrustedAt > 10.5 + 3.0 - 1e-6);
    CHECK(firstTrustedAt < 10.5 + 3.0 + 3 * TICK);
}

TEST(playspace_jump_is_corrected_not_distrusted)
{
    Sim sim;
    const Eigen::Isometry3d delta = makeIso(5.0, Eigen::Vector3d(0.2, 0.0, 0.1));
    // every target device gets the same rigid delta in target space from t = 10
    const World w = sim.world;
    auto disturb = [&sim, delta, w](uint8_t idx, std::function<Eigen::Isometry3d(const Body&)> pick) {
        sim.disturbances[idx] = [=, &sim](double t) -> Eigen::Vector3d {
            if (t < 10.0)
                return Eigen::Vector3d(0.0, 0.0, 0.0);
            const Eigen::Isometry3d raw = w.C_true.inverse() * pick(w.at(t));
            return (delta * raw).translation() - raw.translation();
        };
    };
    disturb(HEAD_TRK, [](const Body& b) { return b.headTrk; });
    disturb(HIP, [](const Body& b) { return b.hip; });
    disturb(FOOT_L, [](const Body& b) { return b.footL; });
    disturb(FOOT_R, [](const Body& b) { return b.footR; });
    bool everHeld = false;
    double maxResidualAfter = 0.0;
    sim.run(0.0, 20.0, [&](double t, const TickOutput& out) {
        everHeld = everHeld || out.holdTarget;
        if (t > 10.5 && out.targetResidualValid)
            maxResidualAfter = std::max(maxResidualAfter, out.targetResidual);
    });
    sim.printTransitions();
    CHECK(!everHeld);
    CHECK_EQ(sim.events.size(), 1u);
    CHECK(sim.events.size() == 1 && sim.events[0] == Event::PLAYSPACE_JUMP);
    CHECK_EQ(sim.allTransitions.size(), 0u);
    // the corrected calibration maps the moved universe back onto the reference: the head
    // tracker residual stays healthy and the hip lands where the body really is
    CHECK(maxResidualAfter < sim.consensus.params().r_ok);
    const Body b = w.at(20.0);
    const Eigen::Isometry3d rawHip = w.C_true.inverse() * b.hip;
    const Eigen::Vector3d movedHip = (delta * rawHip).translation();
    CHECK((sim.C_world * movedHip - b.hip.translation()).norm() < 0.02);
}

TEST(hmd_recenter_is_a_reference_discontinuity)
{
    for (int mode = 0; mode < 2; mode++) {
        Sim sim;
        sim.relativeMode = mode == 1;
        sim.run(0.0, 10.0);
        sim.hmdOffset = Eigen::Vector3d(0.5, 0.0, 0.0); // Quest re-center, permanent
        sim.run(10.0 + TICK, 15.0);
        sim.printTransitions();
        CHECK(!sim.events.empty() && sim.events[0] == Event::REFERENCE_DISCONTINUITY);
        CHECK_EQ(sim.allTransitions.size(), 0u);
    }
}

TEST(ramp_glitch_caught_by_residual)
{
    for (int mode = 0; mode < 2; mode++) {
    // 30 cm over 0.5 s (0.6 m/s): no single tick looks like a jump, the rigid offset does
    Sim sim;
    sim.relativeMode = mode == 1;
    sim.disturbances[HEAD_TRK] = [](double t) -> Eigen::Vector3d {
        if (t < 10.0)
            return Eigen::Vector3d(0.0, 0.0, 0.0);
        if (t < 10.5)
            return Eigen::Vector3d(0.3 * (t - 10.0) / 0.5, 0.0, 0.0);
        if (t < 20.0)
            return Eigen::Vector3d(0.3, 0.0, 0.0);
        if (t < 20.5)
            return Eigen::Vector3d(0.3 * (1.0 - (t - 20.0) / 0.5), 0.0, 0.0);
        return Eigen::Vector3d(0.0, 0.0, 0.0);
    };
    double firstHold = -1.0;
    sim.run(0.0, 30.0, [&](double t, const TickOutput& out) {
        if (out.holdTarget && firstHold < 0.0)
            firstHold = t;
    });
    sim.printTransitions();
    CHECK_EQ(sim.countTransitions(HEAD_TRK, State::SUSPECT, Reason::RESIDUAL_HIGH), 1);
    CHECK_EQ(sim.countTransitions(HEAD_TRK, State::UNTRUSTED, Reason::CONFIRM_TIMEOUT), 1);
    CHECK(firstHold > 10.0 && firstHold < 11.5);
    CHECK_EQ(sim.countTransitions(HEAD_TRK, State::RECOVERING, Reason::RESIDUAL_OK), 1);
    CHECK_EQ(sim.countTransitions(HEAD_TRK, State::TRUSTED, Reason::SOLVE_AGREES), 1);
    }
}

TEST(stale_then_fresh_pose_is_not_a_jump)
{
    // a device moving at 1.5 m/s whose pose stalls for three ticks (dongle hiccup) must not look
    // like a jump when the next fresh pose arrives
    PlausibilityParams p;
    DeviceHistory h;
    PoseSample s;
    s.tracking = true;
    s.v = Eigen::Vector3d(1.5, 0, 0);
    for (int i = 0; i < 5; i++) {
        s.t = i * 0.05;
        s.p = Eigen::Vector3d(1.5 * s.t, 1, 0);
        evaluate(h, s, p);
    }
    // stall: same pose repeated
    for (int i = 5; i < 8; i++) {
        s.t = i * 0.05;
        CHECK(evaluate(h, s, p).flag == Flag::OK);
    }
    // fresh pose after 0.2 s: 30 cm further, consistent with 1.5 m/s over the real interval
    s.t = 8 * 0.05;
    s.p = Eigen::Vector3d(1.5 * s.t, 1, 0);
    PlausibilityResult r = evaluate(h, s, p);
    CHECK(r.flag == Flag::OK);
    CHECK_NEAR(r.dt, 0.2, 1e-9);
}

TEST(dropout_while_untrusted_does_not_restore_trust)
{
    for (int mode = 0; mode < 2; mode++) {
        Sim sim;
        sim.relativeMode = mode == 1;
        sim.disturbances[HEAD_TRK] = stepOffset(10.0, 30.0, Eigen::Vector3d(0.3, 0.0, 0.0)); // 20 s glitch
        sim.dropouts[HEAD_TRK] = { 12.0, 14.5 }; // tracker not tracking for 2.5 s in the middle
        bool heldAt20 = false;
        sim.run(0.0, 40.0, [&](double t, const TickOutput& out) {
            if (std::abs(t - 20.0) < 1e-6)
                heldAt20 = out.holdTarget;
        });
        std::printf("  mode=%s\n", sim.relativeMode ? "relative" : "fixed");
        sim.printTransitions();
        CHECK(heldAt20); // still held after the dropout, the offset is still there
        CHECK_EQ(sim.countTransitions(HEAD_TRK, State::TRUSTED, Reason::DEVICE_LOST), 0);
        // and it recovers when the glitch ends
        CHECK_EQ(sim.countTransitions(HEAD_TRK, State::RECOVERING, Reason::JUMP_BACK) + sim.countTransitions(HEAD_TRK, State::RECOVERING, Reason::RESIDUAL_OK), 1);
        CHECK_EQ(sim.countTransitions(HEAD_TRK, State::TRUSTED, Reason::SOLVE_AGREES), 1);
        CHECK(sim.consensus.status(HEAD_TRK).state == State::TRUSTED);
    }
}

TEST(no_calibration_yet_jump_recovers_by_quiet_rule)
{
    Sim sim;
    sim.calibValid = false; // first run: nothing calibrated, no residual possible
    sim.disturbances[HEAD_TRK] = stepOffset(10.0, 11.0, Eigen::Vector3d(0.3, 0.0, 0.0));
    sim.run(0.0, 20.0);
    sim.printTransitions();
    // the step out is a jump; the step back happens while UNTRUSTED and only restarts the quiet timer
    CHECK_EQ(sim.countTransitions(HEAD_TRK, State::SUSPECT), 1);
    CHECK_EQ(sim.countTransitions(HEAD_TRK, State::TRUSTED, Reason::QUIET), 1);
    CHECK(sim.consensus.status(HEAD_TRK).state == State::TRUSTED);
    double trustedAt = 0.0;
    for (auto& tr : sim.allTransitions)
        if (tr.to == State::TRUSTED)
            trustedAt = tr.t;
    CHECK(trustedAt > 11.0 + 3.0 - 1e-6 && trustedAt < 11.0 + 3.0 + 3 * TICK);
}

TEST(hmd_recenter_fixed_mode_waits_for_the_solver)
{
    // fixed mode, HMD re-centers, the user stands still for two minutes so no solve is applied:
    // nothing may be distrusted; when the solver finally applies, life goes on
    Sim sim;
    sim.run(0.0, 10.0);
    sim.hmdOffset = Eigen::Vector3d(0.5, 0.0, 0.0);
    sim.run(10.0 + TICK, 130.0);
    CHECK_EQ(sim.allTransitions.size(), 0u);
    // the solver catches up: C_world explains the new HMD origin
    sim.C_world = makeIso(0.0, Eigen::Vector3d(0.5, 0.0, 0.0)) * sim.world.C_true;
    sim.run(130.0 + TICK, 160.0);
    sim.printTransitions();
    CHECK_EQ(sim.allTransitions.size(), 0u);
    CHECK(sim.consensus.localOffsetValid());
}

TEST(residual_protects_with_a_single_juror)
{
    // only head tracker + hip: a jump is unverifiable, but the residual still catches the glitch
    Sim sim;
    sim.dropouts[FOOT_L] = { 0.0, 1e9 };
    sim.dropouts[FOOT_R] = { 0.0, 1e9 };
    sim.disturbances[HEAD_TRK] = [](double t) -> Eigen::Vector3d {
        if (t < 10.0 || t >= 20.0)
            return Eigen::Vector3d(0.0, 0.0, 0.0);
        return Eigen::Vector3d(0.3 * std::min(1.0, (t - 10.0) / 0.5), 0.0, 0.0);
    };
    sim.run(0.0, 30.0);
    sim.printTransitions();
    CHECK_EQ(sim.countTransitions(HEAD_TRK, State::SUSPECT, Reason::RESIDUAL_HIGH), 1);
    CHECK_EQ(sim.countTransitions(HEAD_TRK, State::UNTRUSTED, Reason::CONFIRM_TIMEOUT), 1);
}

TEST(manual_override_trusts_everything)
{
    Sim sim;
    sim.disturbances[HEAD_TRK] = stepOffset(10.0, 100.0, Eigen::Vector3d(0.3, 0.0, 0.0)); // permanent
    sim.run(0.0, 15.0);
    CHECK_EQ(sim.consensus.status(HEAD_TRK).state == State::UNTRUSTED, true);
    std::vector<Transition> manual;
    sim.consensus.forceTrustAll(15.0, &manual);
    CHECK_EQ(manual.size(), 1u);
    CHECK(manual[0].reason == Reason::MANUAL);
    // the "solver" now accepts the new geometry: C_world shifts to explain the offset
    sim.C_world = sim.world.C_true * makeIso(0.0, Eigen::Vector3d(-0.3, 0.0, 0.0));
    const size_t before = sim.allTransitions.size();
    sim.run(15.0 + TICK, 30.0);
    sim.printTransitions();
    CHECK_EQ(sim.allTransitions.size(), before); // stays trusted with the new geometry
}

TEST(fast_motion_explained_by_the_reported_velocity)
{
    PlausibilityParams p;
    DeviceHistory h;
    PoseSample s;
    s.tracking = true;
    s.v = Eigen::Vector3d(8.0, 0, 0); // a punch at 8 m/s; the driver's velocity agrees
    s.t = 0.0;
    s.p = Eigen::Vector3d(0, 1, 0);
    evaluate(h, s, p);
    s.t = 1.0 / 30.0;
    s.p = Eigen::Vector3d(8.0 / 30.0, 1, 0);
    PlausibilityResult r = evaluate(h, s, p);
    CHECK(r.d > p.v_max * r.dt + p.j_tol); // the old fixed bound would have called this a jump
    CHECK(r.flag != Flag::JUMP);
    CHECK(r.unexplained < 1e-9);
    // 15 cm teleport of a device that says it stands still: under the old bound (16 cm at 30 fps),
    // well over the unexplained allowance (6.3 cm)
    DeviceHistory h2;
    PoseSample q;
    q.tracking = true;
    q.v = Eigen::Vector3d(0.05, 0, 0);
    q.t = 0.0;
    q.p = Eigen::Vector3d(0, 1, 0);
    evaluate(h2, q, p);
    q.t = 1.0 / 30.0;
    q.p = Eigen::Vector3d(0.15, 1, 0);
    r = evaluate(h2, q, p);
    CHECK(r.d < p.v_max * r.dt + p.j_tol);
    CHECK(r.flag == Flag::JUMP);
    // a driver that reports no velocity at all keeps the fixed bound
    DeviceHistory h3;
    PoseSample z;
    z.tracking = true;
    z.t = 0.0;
    z.p = Eigen::Vector3d(0, 1, 0);
    evaluate(h3, z, p);
    z.t = 1.0 / 30.0;
    z.p = Eigen::Vector3d(0.15, 1, 0);
    CHECK(evaluate(h3, z, p).flag != Flag::JUMP);
}

TEST(wrist_flick_explained_by_the_reported_angular_speed)
{
    PlausibilityParams p;
    const double dt = 1.0 / 30.0;
    const double rate = 1200.0; // deg/s, a fast wrist turn
    const Eigen::Vector3d w(0.0, rate * k_PI / 180.0, 0.0);
    for (int withW = 0; withW < 2; withW++) {
        DeviceHistory h;
        PoseSample s;
        s.tracking = true;
        s.p = Eigen::Vector3d(0, 1, 0);
        s.w = withW ? w : Eigen::Vector3d::Zero();
        s.t = 0.0;
        evaluate(h, s, p);
        s.t = dt;
        s.p = Eigen::Vector3d(0.001, 1, 0);
        s.q = Eigen::Quaterniond(Eigen::AngleAxisd(rate * dt * k_PI / 180.0, Eigen::Vector3d::UnitY()));
        const PlausibilityResult r = evaluate(h, s, p);
        CHECK_NEAR(r.rotDeg, 40.0, 1e-6);
        // with the angular speed reported the turn is explained; without, the fixed bound flags it
        CHECK(withW ? r.flag != Flag::JUMP : r.flag == Flag::JUMP);
    }
}

TEST(fast_kick_is_not_a_jump)
{
    Sim sim;
    sim.world.kickT0 = 10.015; // the tick 10.05 -> 10.10 spans the fastest part of the kick
    const Body b0 = sim.world.at(10.05), b1 = sim.world.at(10.10);
    const double step = (b1.footL.translation() - b0.footL.translation()).norm();
    CHECK(step > 4.0 * TICK + 0.03); // over the old fixed bound
    sim.run(0.0, 15.0);
    sim.printTransitions();
    CHECK_EQ(sim.allTransitions.size(), 0u);
    CHECK_EQ(sim.events.size(), 0u);
}

TEST(playspace_shift_over_two_frames_is_one_jump)
{
    // 2026-09-30 05:41:02: the lighthouse universe moved 13 cm; five devices showed it in one frame,
    // the head tracker (no fresh pose that frame) in the next. One playspace jump, nobody distrusted.
    Sim sim;
    const Eigen::Isometry3d delta = makeIso(2.0, Eigen::Vector3d(0.10, 0.0, 0.08));
    const World w = sim.world;
    auto shift = [&sim, delta, w](uint8_t idx, double from, std::function<Eigen::Isometry3d(const Body&)> pick) {
        sim.disturbances[idx] = [=](double t) -> Eigen::Vector3d {
            if (t < from)
                return Eigen::Vector3d(0.0, 0.0, 0.0);
            const Eigen::Isometry3d raw = w.C_true.inverse() * pick(w.at(t));
            return (delta * raw).translation() - raw.translation();
        };
    };
    shift(HIP, 10.0, [](const Body& b) { return b.hip; });
    shift(FOOT_L, 10.0, [](const Body& b) { return b.footL; });
    shift(FOOT_R, 10.0, [](const Body& b) { return b.footR; });
    shift(HEAD_TRK, 10.0 + TICK, [](const Body& b) { return b.headTrk; });
    sim.frozen[HEAD_TRK] = { 10.0, 10.0 + TICK / 2 }; // no fresh head-tracker pose at 10.0
    double maxResidualAfter = 0.0;
    sim.run(0.0, 20.0, [&](double t, const TickOutput& out) {
        if (t > 10.5 && out.targetResidualValid)
            maxResidualAfter = std::max(maxResidualAfter, out.targetResidual);
    });
    sim.printTransitions();
    CHECK_EQ(sim.allTransitions.size(), 0u);
    CHECK_EQ(sim.events.size(), 1u);
    CHECK(sim.events.size() == 1 && sim.events[0] == Event::PLAYSPACE_JUMP);
    CHECK(maxResidualAfter < sim.consensus.params().r_ok);
}

TEST(head_tracker_recovers_when_the_applied_calibration_moves_on)
{
    // Observe-only: the universe drifted 20 cm without a visible step (the head tracker goes
    // UNTRUSTED on its residual), then the solver applied a calibration that explains it. The trust
    // layer follows the applied calibration and recovers; it used to keep the stale one for hours.
    Sim sim;
    const Eigen::Vector3d drift(0.2, 0.0, 0.0);
    auto ramp = [drift](double t) -> Eigen::Vector3d { return drift * std::clamp((t - 10.0) / 1.0, 0.0, 1.0); };
    for (uint8_t d : { HEAD_TRK, HIP, FOOT_L, FOOT_R })
        sim.disturbances[d] = ramp;
    sim.run(0.0, 15.0);
    CHECK(sim.consensus.status(HEAD_TRK).state == State::UNTRUSTED);
    // the solver's new calibration: raw target positions moved by +drift, so C_new = C * T(-drift)
    sim.C_world = sim.world.C_true * makeIso(0.0, -drift);
    sim.run(15.0 + TICK, 25.0);
    sim.printTransitions();
    CHECK_EQ(sim.countTransitions(HEAD_TRK, State::SUSPECT, Reason::RESIDUAL_HIGH), 1);
    CHECK_EQ(sim.countTransitions(HEAD_TRK, State::RECOVERING, Reason::RESIDUAL_OK), 1);
    CHECK_EQ(sim.countTransitions(HEAD_TRK, State::TRUSTED, Reason::SOLVE_AGREES), 1);
    CHECK(sim.consensus.status(HEAD_TRK).state == State::TRUSTED);
}

TEST(a_solve_that_moves_the_calibration_is_not_a_glitch)
{
    // The stored calibration is 12 cm off; at 20 s the solver applies the right one (2026-09-30:
    // startup and continuous solves of 6 to 10 cm, playspace corrections of 12 and 20 cm). The head
    // tracker did not move, so nothing may be distrusted, before or after.
    for (int mode = 0; mode < 2; mode++) {
        Sim sim;
        sim.relativeMode = mode == 1;
        sim.world.headSpeedScale = 0.25; // a stale calibration shows as a residual that swings with the head turn
        const Eigen::Vector3d err(0.12, 0.0, 0.0); // over r_high: the old estimate made this RESIDUAL_HIGH for good
        if (sim.relativeMode)
            sim.C_L_applied = sim.world.C_L_true * makeIso(0.0, err);
        else
            sim.C_world = sim.world.C_true * makeIso(0.0, err);
        sim.run(0.0, 20.0);
        const size_t before = sim.allTransitions.size();
        if (sim.relativeMode)
            sim.C_L_applied = sim.world.C_L_true;
        else
            sim.C_world = sim.world.C_true;
        double maxResidualAfter = 0.0;
        sim.run(20.0 + TICK, 40.0, [&](double t, const TickOutput& out) {
            if (t > 20.5 && out.targetResidualValid)
                maxResidualAfter = std::max(maxResidualAfter, out.targetResidual);
        });
        std::printf("  mode=%s max residual after %.3f\n", sim.relativeMode ? "relative" : "fixed", maxResidualAfter);
        sim.printTransitions();
        CHECK_EQ(before, 0u);
        CHECK_EQ(sim.allTransitions.size(), 0u);
        CHECK(sim.consensus.status(HEAD_TRK).state == State::TRUSTED);
        CHECK_EQ(sim.consensus.anchorCount(), 1u);
        CHECK(maxResidualAfter < sim.consensus.params().r_ok);
    }
}

int main()
{
    return spacecal::test::runAll();
}
