// The head tracker's place on the headset, remembered across sessions (src/overlay/trust/head_mount.h).

#include "head_mount.h"
#include "test_support.h"

#include <cmath>
#include <random>

using namespace spacecal::trust;

namespace {
constexpr double k_PI = 3.14159265358979323846;
constexpr double k_DT = 0.05;

Eigen::Isometry3d iso(double x, double y, double z, double yawDeg, double pitchDeg = 0.0, double rollDeg = 0.0)
{
    Eigen::Isometry3d i = Eigen::Isometry3d::Identity();
    i.linear() = (Eigen::AngleAxisd(yawDeg * k_PI / 180.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(pitchDeg * k_PI / 180.0, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(rollDeg * k_PI / 180.0, Eigen::Vector3d::UnitZ()))
                      .toRotationMatrix();
    i.translation() = Eigen::Vector3d(x, y, z);
    return i;
}

// the tracker on the headset as measured on 2026-10-03/04: 1 cm left, 10.7 cm up, 3.5 cm to the front
const Eigen::Isometry3d k_X = iso(-0.01, 0.107, -0.035, 12.0, -8.0, 3.0);
const Eigen::Isometry3d k_C = iso(-0.95, 0.30, 2.18, -37.0, 1.0, -0.5); // lighthouse -> Quest world

// headset somewhere in the room, looking around
Eigen::Isometry3d headAt(double t)
{
    return iso(0.4 + 0.3 * std::sin(0.3 * t), 1.5 + 0.1 * std::sin(0.7 * t), -1.0 + 0.2 * std::cos(0.2 * t), 40.0 * std::sin(0.25 * t), 15.0 * std::sin(0.4 * t), 5.0 * std::sin(0.5 * t));
}

// the head tracker's raw lighthouse pose for a headset pose: T = C^-1 * H * X, plus noise
Eigen::Isometry3d trackerFor(const Eigen::Isometry3d& H, std::mt19937& rng, double posNoise = 0.002, double degNoise = 0.3)
{
    std::normal_distribution<double> n(0.0, 1.0);
    const Eigen::Isometry3d noise = iso(posNoise * n(rng), posNoise * n(rng), posNoise * n(rng), degNoise * n(rng), degNoise * n(rng), degNoise * n(rng));
    return k_C.inverse() * H * k_X * noise;
}

HeadMount learned(double seconds)
{
    HeadMount hm;
    std::mt19937 rng(1);
    for (double t = 0.0; t < seconds; t += k_DT) {
        const Eigen::Isometry3d H = headAt(t);
        const Eigen::Isometry3d T = trackerFor(H, rng);
        hm.learn(H.inverse() * k_C * T, k_DT);
    }
    return hm;
}
}

TEST(learns_the_place_and_becomes_confident_after_two_minutes)
{
    HeadMount early = learned(60.0);
    CHECK(!early.confident());
    HeadMount hm = learned(200.0);
    CHECK(hm.confident());
    CHECK((hm.pose().translation() - k_X.translation()).norm() < 0.002);
    double pos = 0.0, deg = 0.0;
    hm.disagreement(k_C, headAt(3.0), k_C.inverse() * headAt(3.0) * k_X, pos, deg);
    CHECK(pos < 0.002);
    CHECK(deg < 0.5);
    CHECK(hm.rotationSteady()); // 0.3 deg of noise
}

TEST(startup_shifts_the_stored_calibration_onto_the_remembered_place)
{
    // the Quest re-centered since the last session: the stored calibration is 25 cm off; one pose and the
    // remembered place put the head tracker (and with it the body) back, the rotation stays the stored one
    HeadMount hm = learned(200.0);
    Eigen::Isometry3d stored = k_C;
    stored.translation() += Eigen::Vector3d(0.2, -0.1, 0.1);
    const Eigen::Isometry3d H = headAt(7.0);
    const Eigen::Isometry3d T = k_C.inverse() * H * k_X;
    const Eigen::Isometry3d C = hm.placeCalibration(stored, H, T);
    CHECK((C.translation() - k_C.translation()).norm() < 0.005);
    CHECK((C.linear() - stored.linear()).norm() < 1e-12);
}

TEST(a_rigid_mount_may_use_the_rotation_too)
{
    HeadMountParams p;
    p.use_rotation = true;
    HeadMount hm;
    hm.setParams(p);
    std::mt19937 rng(4);
    for (double t = 0.0; t < 200.0; t += k_DT) {
        const Eigen::Isometry3d H = headAt(t);
        hm.learn(H.inverse() * k_C * trackerFor(H, rng), k_DT);
    }
    CHECK(hm.usesRotation());
    const Eigen::Isometry3d stored = iso(0.2, -0.1, 0.15, 6.0) * k_C; // 6 deg off as well
    const Eigen::Isometry3d H = headAt(7.0);
    const Eigen::Isometry3d C = hm.placeCalibration(stored, H, k_C.inverse() * H * k_X);
    CHECK((C.translation() - k_C.translation()).norm() < 0.02);
    CHECK(Eigen::AngleAxisd(C.linear() * k_C.linear().transpose()).angle() * 180.0 / k_PI < 0.5);
}

TEST(an_unsteady_rotation_only_shifts)
{
    // samples whose rotation scatters (10 deg) leave the stored rotation alone and only shift it
    HeadMount hm;
    std::mt19937 rng(2);
    for (double t = 0.0; t < 200.0; t += k_DT) {
        const Eigen::Isometry3d H = headAt(t);
        hm.learn(H.inverse() * k_C * trackerFor(H, rng, 0.002, 10.0), k_DT);
    }
    CHECK(hm.confident());
    CHECK(!hm.rotationSteady());
    const Eigen::Isometry3d stored = iso(0.0, 0.12, 0.0, 3.0) * k_C;
    const Eigen::Isometry3d H = headAt(9.0);
    const Eigen::Isometry3d T = k_C.inverse() * H * k_X;
    const Eigen::Isometry3d C = hm.placeCalibration(stored, H, T);
    CHECK((C.linear() - stored.linear()).norm() < 1e-12);
    CHECK(((C * T).translation() - (H * hm.pose()).translation()).norm() < 1e-9);
}

TEST(the_prior_rejects_a_solve_that_moves_the_tracker_on_the_headset)
{
    HeadMount hm = learned(200.0);
    const Eigen::Isometry3d H = headAt(11.0);
    const Eigen::Isometry3d T = k_C.inverse() * H * k_X;
    CHECK(hm.judgeSolve(k_C, H, T) == SolveVerdict::AGREES);
    // a solve that follows the Quest's own drift keeps the tracker where it is on the headset
    const Eigen::Isometry3d drift = iso(0.0, -0.15, 0.0, 0.0);
    CHECK(hm.judgeSolve(drift * k_C, drift * H, T) == SolveVerdict::AGREES);
    // 15 cm down without the headset moving (lying still, no rotation to pin the height): rejected
    const Eigen::Isometry3d down = iso(0.0, -0.15, 0.0, 0.0) * k_C;
    CHECK(hm.judgeSolve(down, H, T) == SolveVerdict::DISAGREES);
    // turned by 8 deg around the room's origin, metres from the head: the tracker lands elsewhere, rejected
    CHECK(hm.judgeSolve(iso(0, 0, 0, 8.0) * k_C, H, T) == SolveVerdict::DISAGREES);
    CHECK(hm.judgeSolve(k_C, H, T) == SolveVerdict::AGREES);
}

TEST(a_solve_that_gets_part_of_the_way_back_is_accepted)
{
    // SteamVR moved its base stations: the calibration in use puts the head tracker 20 cm off. A solve that
    // brings it to 6 cm is still beyond the prior's 5 cm, but better than what is in use: taken
    HeadMount hm = learned(200.0);
    const Eigen::Isometry3d H = headAt(13.0);
    const Eigen::Isometry3d T = k_C.inverse() * H * k_X;
    const Eigen::Isometry3d inUse = iso(0.0, -0.20, 0.0, 0.0) * k_C;
    const Eigen::Isometry3d partWay = iso(0.0, -0.06, 0.0, 0.0) * k_C;
    CHECK(hm.judgeSolve(partWay, H, T, nullptr, nullptr, &inUse) == SolveVerdict::IMPROVES);
    CHECK(hm.judgeSolve(iso(0.0, -0.25, 0.0, 0.0) * k_C, H, T, nullptr, nullptr, &inUse) == SolveVerdict::DISAGREES);
    // from a calibration that is nearly right, the same solve is a step away: rejected
    const Eigen::Isometry3d nearlyRight = iso(0.0, -0.03, 0.0, 0.0) * k_C;
    CHECK(hm.judgeSolve(partWay, H, T, nullptr, nullptr, &nearlyRight) == SolveVerdict::DISAGREES);
    CHECK(hm.judgeSolve(partWay, H, T) == SolveVerdict::DISAGREES); // no calibration in use given: the plain prior
}

TEST(the_same_other_place_five_times_in_a_row_is_a_remount)
{
    HeadMount hm = learned(200.0);
    std::mt19937 rng(3);
    const Eigen::Isometry3d remounted = iso(0.0, 0.06, 0.03, 0.0) * k_X; // the strap moved: 6 cm up, 3 cm back
    SolveVerdict v = SolveVerdict::AGREES;
    for (int i = 0; i < 5; i++) {
        const Eigen::Isometry3d H = headAt(20.0 + 7.0 * i);
        const Eigen::Isometry3d T = k_C.inverse() * H * remounted;
        // the solver's answer for the new mount: the calibration that puts the tracker at the new place
        // is the true one; seen from the old place it reads as the tracker sitting elsewhere
        const Eigen::Isometry3d Ttrue = T;
        v = hm.judgeSolve(k_C, H, Ttrue);
        if (i < 4)
            CHECK(v == SolveVerdict::DISAGREES);
    }
    CHECK(v == SolveVerdict::REMOUNTED);
    CHECK(!hm.confident()); // learned anew before it is used again
    CHECK((hm.pose().translation() - remounted.translation()).norm() < 1e-9);
    (void)rng;
}

TEST(scattered_disagreements_never_count_as_a_remount)
{
    HeadMount hm = learned(200.0);
    const Eigen::Isometry3d H = headAt(5.0);
    const Eigen::Isometry3d T = k_C.inverse() * H * k_X;
    for (int i = 0; i < 20; i++) {
        const double s = (i % 2 == 0) ? 1.0 : -1.0;
        const Eigen::Isometry3d glitch = iso(0.10 * s, 0.08 * (i % 3), -0.07 * s, 0.0) * k_C;
        CHECK(hm.judgeSolve(glitch, H, T) == SolveVerdict::DISAGREES);
    }
    CHECK(hm.confident());
}

TEST(state_round_trip_and_other_devices)
{
    HeadMount hm = learned(200.0);
    const HeadMountState s = hm.state("LHR-HEAD", "HMD-1");
    HeadMount back;
    CHECK(back.restore(s, "LHR-HEAD", "HMD-1"));
    CHECK(back.confident());
    CHECK((back.pose().matrix() - hm.pose().matrix()).norm() < 1e-9);
    HeadMount other;
    CHECK(!other.restore(s, "LHR-OTHER", "HMD-1")); // another tracker on the head: start from scratch
    CHECK(!other.confident());
    HeadMount otherHmd;
    CHECK(!otherHmd.restore(s, "LHR-HEAD", "HMD-2"));
}

// ---- the automatic position fix (HeadMountFix) ------------------------------------------------------

namespace {
// The gap without the fix is `raw` (against the calibration last applied by someone else); the gap seen
// is raw - offset, offset being what the fix has shifted since.
struct FixRig {
    HeadMountFix fix;
    double t = 0.0;
    Eigen::Vector3d raw = Eigen::Vector3d::Zero();
    Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    Eigen::Vector3d gapSeenBefore = Eigen::Vector3d::Zero(); // last tick's calibration, this tick's poses
    Eigen::Vector3d offsetSeen = Eigen::Vector3d::Zero();
    double moved = 0.0;
    int started = 0, finished = 0, snapped = 0, jumped = 0, interrupted = 0;

    HeadMountFix::Output step(const HeadMountFix::Input& in)
    {
        const HeadMountFix::Output out = fix.update(in);
        offset += out.shift;
        moved += out.shift.norm();
        started += out.started;
        finished += out.finished;
        snapped += out.snapped;
        jumped += out.jumped;
        interrupted += out.interrupted;
        return out;
    }
    // one 20 Hz trust tick with the poses giving `newRaw`; returns the gap seen after the tick's shift
    Eigen::Vector3d tick(const Eigen::Vector3d& newRaw, double headsetSpeed = 0.0)
    {
        t += 0.05;
        HeadMountFix::Input in;
        in.t = t;
        in.gap = newRaw - offset;
        in.gapBefore = newRaw - offsetSeen;
        in.headsetSpeed = headsetSpeed;
        offsetSeen = offset;
        raw = newRaw;
        step(in);
        return raw - offset;
    }
    // someone else applies a calibration that puts the gap at `newRaw` (the poses did not move)
    void calibrationApplied(const Eigen::Vector3d& newRaw)
    {
        t += 0.05;
        HeadMountFix::Input in;
        in.t = t;
        in.gap = newRaw;
        in.gapBefore = raw - offsetSeen;
        in.calibrationChanged = true;
        offset.setZero();
        offsetSeen.setZero();
        raw = newRaw;
        step(in);
    }
    Eigen::Vector3d run(double seconds, const Eigen::Vector3d& newRaw, double headsetSpeed = 0.0)
    {
        Eigen::Vector3d seen = newRaw - offset;
        for (double s = 0.0; s < seconds - 1e-9; s += 0.05)
            seen = tick(newRaw, headsetSpeed);
        return seen;
    }
};
const Eigen::Vector3d k_DOWN8(0.0, -0.08, 0.01);
}

TEST(a_steady_offset_at_the_head_is_slid_back)
{
    // lying still, a solve put the calibration 8 cm off at the head (2026-10-04 06:00): the fix starts
    // after five steady seconds and slides at 1.5 cm/s
    FixRig r;
    r.run(1.0, Eigen::Vector3d::Zero());
    r.calibrationApplied(k_DOWN8);
    CHECK(r.jumped == 0); // a calibration change is not the head tracker jumping
    CHECK(r.run(3.0, k_DOWN8).norm() > 0.079); // steady for less than the hold: nothing yet
    CHECK(r.started == 0);
    r.run(3.0, k_DOWN8);
    CHECK(r.started == 1);
    const Eigen::Vector3d after = r.run(6.0, k_DOWN8);
    CHECK(after.norm() < 0.006);
    CHECK(r.finished == 1);
    CHECK(std::abs(r.moved - k_DOWN8.norm()) < 0.006);
}

TEST(a_head_tracker_that_jumps_and_slides_back_is_left_alone)
{
    // 2026-10-02 06:55: the head tracker jumped 4.4 cm on its own and slid back within ten seconds
    FixRig r;
    r.run(6.0, Eigen::Vector3d::Zero());
    for (int i = 0; i < 400; i++)
        r.tick(Eigen::Vector3d(0.0, 0.044 * std::exp(-0.05 * i / 3.0), 0.0));
    CHECK(r.jumped == 1);
    CHECK(r.moved < 1e-9);
}

TEST(an_offset_that_stays_after_a_jump_is_fixed_once_the_quarantine_is_over)
{
    // 2026-10-04 06:08: the gap jumped to 12 cm and stayed until the solver came 2.5 minutes later
    FixRig r;
    r.run(6.0, Eigen::Vector3d::Zero());
    const Eigen::Vector3d off(0.05, 0.11, 0.0);
    r.run(19.0, off);
    CHECK(r.jumped == 1);
    CHECK(r.moved < 1e-9); // still waiting
    r.run(7.0, off); // the quarantine ends after 20 s, then five steady seconds
    CHECK(r.started == 1);
    CHECK(r.run(12.0, off).norm() < 0.006);
}

TEST(a_gap_that_is_still_sliding_is_not_fixed)
{
    // growing by 0.5 cm/s without a jump: not steady
    FixRig r;
    r.run(2.0, Eigen::Vector3d::Zero());
    double g = 0.0;
    for (int i = 0; i < 300; i++) {
        g += 0.005 * 0.05;
        r.tick(Eigen::Vector3d(0.0, g, 0.0));
    }
    CHECK(r.started == 0);
    CHECK(r.jumped == 0);
    // once it holds at 7.5 cm, it is fixed
    r.run(15.0, Eigen::Vector3d(0.0, g, 0.0));
    CHECK(r.started == 1);
}

TEST(nothing_is_fixed_while_moving_fast)
{
    FixRig r;
    r.calibrationApplied(k_DOWN8);
    r.run(20.0, k_DOWN8, 0.5);
    CHECK(r.started == 0);
    CHECK(r.moved < 1e-9);
}

TEST(the_fix_is_undone_when_its_cause_goes_away)
{
    // the offset came from something that later set itself right (2026-09-30 00:17: SteamVR moved its
    // base station back): the fix would now be the offset, so it is dropped at once
    FixRig r;
    r.run(1.0, Eigen::Vector3d::Zero());
    r.calibrationApplied(k_DOWN8);
    r.run(12.0, k_DOWN8);
    CHECK(r.offset.norm() > 0.07);
    const Eigen::Vector3d seen = r.tick(Eigen::Vector3d::Zero());
    CHECK(r.snapped == 1);
    CHECK(seen.norm() < 1e-9);
    CHECK(r.fix.offset().norm() < 1e-12);
}

TEST(a_calibration_from_elsewhere_replaces_the_fix)
{
    FixRig r;
    r.run(1.0, Eigen::Vector3d::Zero());
    r.calibrationApplied(k_DOWN8);
    r.run(7.0, k_DOWN8);
    CHECK(r.fix.active());
    // the solver applies a calibration that is right: the gap is gone, and so is the fix
    r.calibrationApplied(Eigen::Vector3d::Zero());
    CHECK(r.interrupted == 1);
    CHECK(!r.fix.active());
    CHECK(r.fix.offset().norm() < 1e-12);
    const double before = r.moved;
    r.run(10.0, Eigen::Vector3d::Zero());
    CHECK(r.moved - before < 1e-9);
    CHECK(r.snapped == 0);
    CHECK(r.jumped == 0);
}

TEST(very_large_gaps_are_left_to_the_solver)
{
    FixRig r;
    r.run(1.0, Eigen::Vector3d::Zero());
    r.calibrationApplied(Eigen::Vector3d(0.0, 0.0, 0.6));
    r.run(30.0, Eigen::Vector3d(0.0, 0.0, 0.6));
    CHECK(r.started == 0);
}

TEST(nothing_happens_while_not_allowed)
{
    HeadMountFix fix;
    HeadMountFix::Input in;
    in.allowed = false;
    in.gap = k_DOWN8;
    in.gapBefore = k_DOWN8;
    for (int i = 0; i < 400; i++) {
        in.t = 0.05 * i;
        const HeadMountFix::Output out = fix.update(in);
        CHECK(out.shift.norm() < 1e-12);
    }
    CHECK(!fix.active());
}

int main()
{
    return spacecal::test::runAll();
}
