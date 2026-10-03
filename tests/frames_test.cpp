// SteamVR moving base stations, per device (fork, src/overlay/trust/frame_corrector.h): a device
// keeps its place when SteamVR moves the station it is tracked from, also when the other devices
// are on another station, and its own motion always passes through.

#include "frame_corrector.h"
#include "test_support.h"

#include <cmath>
#include <vector>

using namespace spacecal::trust;

namespace {
constexpr double k_PI = 3.14159265358979323846;
constexpr double k_DT = 1.0 / 90.0;
constexpr double k_SLIDE_TICK = k_DT * 0.015 + 1e-9; // a held correction starts sliding back in the same tick

Eigen::Isometry3d iso(double x, double y, double z, double yawDeg, double pitchDeg = 0.0)
{
    Eigen::Isometry3d i = Eigen::Isometry3d::Identity();
    i.linear() = (Eigen::AngleAxisd(yawDeg * k_PI / 180.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(pitchDeg * k_PI / 180.0, Eigen::Vector3d::UnitX())).toRotationMatrix();
    i.translation() = Eigen::Vector3d(x, y, z);
    return i;
}

// two base stations, metres apart and facing different ways
const Eigen::Isometry3d k_STATION_A = iso(-0.22, 2.03, -5.36, 152.0, -28.0);
const Eigen::Isometry3d k_STATION_B = iso(2.64, 2.09, -2.39, -95.0, -25.0);

struct Dev {
    uint8_t index;
    bool reference;
    Eigen::Isometry3d W; // SteamVR's map of the station it is tracked from
    Eigen::Vector3d world; // where it really is (raw world of the original map)
    bool tracking = true;
};

// the driver pose P for a device at `world` tracked from a station whose map is `map` while the
// real station sits at `truth`: P is measured against the real station, W is the map
Eigen::Isometry3d poseFor(const Eigen::Isometry3d& truth, const Eigen::Vector3d& world)
{
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    p.translation() = truth.inverse() * world;
    p.linear() = truth.linear().transpose();
    return p;
}

struct Room {
    FrameCorrector fc;
    std::vector<Dev> devs;
    std::vector<Eigen::Isometry3d> truth; // per device: the station it is measured against (true pose)
    std::vector<FrameEvent> events;
    double t = 0.0;

    std::vector<FrameSample> samples() const
    {
        std::vector<FrameSample> out;
        for (size_t i = 0; i < devs.size(); i++) {
            FrameSample s;
            s.index = devs[i].index;
            s.reference = devs[i].reference;
            s.tracking = devs[i].tracking;
            s.wfd = devs[i].W;
            s.pose = poseFor(truth[i], devs[i].world);
            out.push_back(s);
        }
        return out;
    }
    void tick(bool correct = true)
    {
        t += k_DT;
        fc.tick(t, samples(), correct, &events);
    }
    Eigen::Vector3d raw(size_t i) const { return (devs[i].W * poseFor(truth[i], devs[i].world)).translation(); }
    Eigen::Vector3d stable(size_t i) const { return fc.correction(devs[i].index) * raw(i); }
    int count(FrameEventKind k) const
    {
        int n = 0;
        for (const auto& e : events)
            n += e.kind == k ? 1 : 0;
        return n;
    }
};

// head tracker + two trackers on station A, two on station B; maps exact at the start
Room makeRoom()
{
    Room r;
    const Eigen::Vector3d body(1.6, 1.0, -1.3);
    r.devs = {
        { 23, true, k_STATION_A, body + Eigen::Vector3d(0, 0.6, 0) },
        { 13, false, k_STATION_A, body + Eigen::Vector3d(0.2, 0.0, 0) },
        { 14, false, k_STATION_A, body + Eigen::Vector3d(-0.2, 0.0, 0) },
        { 24, false, k_STATION_B, body + Eigen::Vector3d(0.1, -0.8, 0.3) },
        { 16, false, k_STATION_B, body + Eigen::Vector3d(-0.1, -0.8, 0.3) },
    };
    r.truth = { k_STATION_A, k_STATION_A, k_STATION_A, k_STATION_B, k_STATION_B };
    for (int i = 0; i < 5; i++)
        r.tick();
    return r;
}

// SteamVR moves a station on its map: every device on it gets the new map, the real station stays
void moveMap(Room& r, const Eigen::Isometry3d& oldMap, const Eigen::Isometry3d& delta)
{
    for (Dev& d : r.devs)
        if (d.W.isApprox(oldMap, 1e-12))
            d.W = delta * oldMap;
}
}

TEST(steamvr_moving_the_head_trackers_station_moves_nothing_and_fixes_the_calibration)
{
    Room r = makeRoom();
    std::vector<Eigen::Vector3d> before;
    for (size_t i = 0; i < r.devs.size(); i++)
        before.push_back(r.stable(i));
    const Eigen::Isometry3d delta = iso(0.11, 0.02, -0.06, 0.9);
    moveMap(r, k_STATION_A, delta);
    r.tick();
    for (size_t i = 0; i < 3; i++)
        CHECK_NEAR((r.stable(i) - before[i]).norm(), 0.0, 1e-9); // on the head tracker's station: exact
    for (size_t i = 3; i < r.devs.size(); i++)
        CHECK_NEAR((r.stable(i) - before[i]).norm(), 0.0, k_SLIDE_TICK); // on station B: SteamVR's map moved station B against station A, they start sliding
    CHECK_EQ(r.count(FrameEventKind::REFERENCE_MOVED), 1);
    CHECK_EQ(r.count(FrameEventKind::DEVICE_MOVED), 0); // the reference family moved: that is the calibration fix
    CHECK(r.events.back().corrected);
    CHECK((r.events.back().delta.matrix() - delta.matrix()).norm() < 1e-9);
    CHECK((r.fc.global().matrix() - delta.inverse().matrix()).norm() < 1e-9); // C_new = C_old * delta^-1
    for (size_t i = 0; i < 3; i++)
        CHECK(r.fc.pin(r.devs[i].index).isApprox(Eigen::Isometry3d::Identity(), 1e-9)); // the calibration alone does it
}

TEST(a_station_moved_under_two_trackers_holds_them_and_slides_them_back)
{
    Room r = makeRoom();
    const Eigen::Vector3d before3 = r.stable(3), before4 = r.stable(4), before1 = r.stable(1);
    // SteamVR moves station B on its map under two trackers, the rest stay on station A
    moveMap(r, k_STATION_B, iso(0.06, -0.03, 0.04, 1.2));
    r.tick();
    CHECK_NEAR((r.stable(3) - before3).norm(), 0.0, k_SLIDE_TICK);
    CHECK_NEAR((r.stable(4) - before4).norm(), 0.0, k_SLIDE_TICK);
    CHECK_NEAR((r.stable(1) - before1).norm(), 0.0, 1e-12);
    CHECK_EQ(r.count(FrameEventKind::DEVICE_MOVED), 2);
    CHECK_EQ(r.count(FrameEventKind::REFERENCE_MOVED), 0);
    CHECK(r.fc.global().isApprox(Eigen::Isometry3d::Identity(), 1e-12)); // the calibration is not touched
    // SteamVR's map has them elsewhere now: they slide there, no faster than 1.5 cm/s
    const double gap = (r.fc.global() * r.raw(3) - before3).norm();
    CHECK(gap > 0.03);
    Eigen::Vector3d last = r.stable(3);
    double fastest = 0.0;
    for (int i = 0; i < 90; i++) {
        r.tick();
        fastest = std::max(fastest, (r.stable(3) - last).norm() / k_DT);
        last = r.stable(3);
    }
    CHECK(fastest <= 0.0151);
    CHECK((r.stable(3) - before3).norm() > 0.013); // it did slide
    for (int i = 0; i < 90 * 20; i++)
        r.tick();
    CHECK_NEAR((r.stable(3) - r.fc.global() * r.raw(3)).norm(), 0.0, 1e-3); // back on SteamVR's map
    CHECK_NEAR((r.stable(1) - before1).norm(), 0.0, 1e-9); // the trackers on station A never moved
}

TEST(own_motion_always_passes_through)
{
    Room r = makeRoom();
    const Eigen::Vector3d a = r.stable(1);
    r.devs[1].world += Eigen::Vector3d(0.3, 0.0, 0.0); // a kick between two frames
    r.tick();
    CHECK_NEAR((r.stable(1) - a).norm(), 0.3, 1e-9);
    // moving while SteamVR moves its station: the motion arrives, the map change does not
    const Eigen::Vector3d b = r.stable(3);
    r.devs[3].world += Eigen::Vector3d(0.0, 0.05, 0.0);
    moveMap(r, k_STATION_B, iso(0.05, 0.0, 0.0, 0.5));
    r.tick();
    CHECK_NEAR((r.stable(3) - b).norm(), 0.05, k_SLIDE_TICK);
}

TEST(a_change_of_station_where_the_maps_disagree_is_held_then_slides)
{
    Room r = makeRoom();
    // tracker 24 changes from station B to station A; SteamVR's maps of the two disagree by 6 cm there
    const Eigen::Vector3d before = r.stable(3);
    r.devs[3].W = k_STATION_A;
    r.truth[3] = iso(-0.22, 2.03, -5.30, 152.0, -28.0); // station A's map is 6 cm off from where it really is
    r.tick();
    CHECK_NEAR((r.stable(3) - before).norm(), 0.0, k_SLIDE_TICK);
    CHECK_EQ(r.count(FrameEventKind::SWITCH_ABSORBED), 1);
    for (int i = 0; i < 90 * 10; i++)
        r.tick();
    CHECK_NEAR((r.stable(3) - r.fc.global() * r.raw(3)).norm(), 0.0, 1e-3);
}

TEST(a_big_change_of_station_is_shown)
{
    Room r = makeRoom();
    const Eigen::Vector3d before = r.stable(3);
    r.devs[3].W = k_STATION_A;
    r.truth[3] = iso(-0.22, 2.03, -4.96, 152.0, -28.0); // 40 cm: a tracking failure, not two maps disagreeing
    r.tick();
    CHECK((r.stable(3) - before).norm() > 0.3);
    CHECK_EQ(r.count(FrameEventKind::SWITCH_SHOWN), 1);
}

TEST(the_head_trackers_own_change_of_station_is_shown_and_its_station_becomes_the_reference)
{
    Room r = makeRoom();
    const Eigen::Vector3d before = r.stable(0);
    r.devs[0].W = k_STATION_B;
    r.truth[0] = iso(2.64, 2.09, -2.34, -95.0, -25.0); // 5 cm disagreement
    r.tick();
    CHECK((r.stable(0) - before).norm() > 0.04); // the trust layer sees the head tracker's step
    CHECK_EQ(r.count(FrameEventKind::REFERENCE_SWITCHED), 1);
    CHECK(r.fc.global().isApprox(Eigen::Isometry3d::Identity(), 1e-12));
    // now a station B move is the calibration fix
    const Eigen::Vector3d h = r.stable(0);
    moveMap(r, k_STATION_B, iso(0.04, 0.0, 0.02, 0.3));
    r.tick();
    CHECK_EQ(r.count(FrameEventKind::REFERENCE_MOVED), 1);
    CHECK_NEAR((r.stable(0) - h).norm(), 0.0, 1e-9);
}

TEST(a_lagging_device_does_not_move_the_reference_back)
{
    Room r = makeRoom();
    const Eigen::Isometry3d delta = iso(0.08, 0.0, 0.0, 0.4);
    // only the head tracker and one tracker get the new map; tracker 14 keeps the old one
    r.devs[0].W = delta * k_STATION_A;
    r.devs[1].W = delta * k_STATION_A;
    r.tick();
    CHECK_EQ(r.count(FrameEventKind::REFERENCE_MOVED), 1);
    const Eigen::Vector3d s2 = r.stable(2);
    for (int i = 0; i < 90; i++)
        r.tick();
    CHECK_NEAR((r.stable(2) - s2).norm(), 0.0, 1e-9); // on the old version of the same station: exact, no slide
    // a tracker that comes over from station B onto the old version is no new version either
    r.devs[3].W = k_STATION_A;
    r.truth[3] = k_STATION_A;
    r.tick();
    CHECK_EQ(r.count(FrameEventKind::REFERENCE_MOVED), 1);
    CHECK((r.fc.global().matrix() - delta.inverse().matrix()).norm() < 1e-9);
}

TEST(back_from_a_loss_a_device_is_on_steamvrs_map)
{
    Room r = makeRoom();
    moveMap(r, k_STATION_B, iso(0.1, 0.0, 0.0, 0.0));
    r.tick();
    CHECK((r.stable(3) - r.fc.global() * r.raw(3)).norm() > 0.05); // held
    r.devs[3].tracking = false;
    for (int i = 0; i < 90; i++)
        r.tick();
    r.devs[3].tracking = true;
    r.tick();
    CHECK_NEAR((r.stable(3) - r.fc.global() * r.raw(3)).norm(), 0.0, 1e-9);
}

TEST(report_only_corrects_nothing)
{
    Room r;
    r.devs = makeRoom().devs;
    r.truth = { k_STATION_A, k_STATION_A, k_STATION_A, k_STATION_B, k_STATION_B };
    for (int i = 0; i < 5; i++)
        r.tick(false);
    moveMap(r, k_STATION_A, iso(0.1, 0.0, 0.0, 1.0));
    r.tick(false);
    CHECK_EQ(r.count(FrameEventKind::REFERENCE_MOVED), 1);
    CHECK(!r.events.back().corrected);
    CHECK(r.fc.global().isApprox(Eigen::Isometry3d::Identity(), 1e-12));
    for (size_t i = 0; i < r.devs.size(); i++)
        CHECK(r.fc.pin(r.devs[i].index).isApprox(Eigen::Isometry3d::Identity(), 1e-12));
}

int main()
{
    return spacecal::test::runAll();
}
