// SteamVR universe shifts (fork, src/overlay/trust/universe_watch.h): the shared WorldFromDriver of
// the lighthouse devices changes, the watch confirms the shift and keeps a stable frame.

#include "test_support.h"
#include "universe_watch.h"

#include <cmath>
#include <vector>

using namespace spacecal::trust;

namespace {
constexpr double k_PI = 3.14159265358979323846;

Eigen::Isometry3d iso(double x, double y, double z, double yawDeg)
{
    Eigen::Isometry3d i = Eigen::Isometry3d::Identity();
    i.linear() = Eigen::AngleAxisd(yawDeg * k_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    i.translation() = Eigen::Vector3d(x, y, z);
    return i;
}

// the frame of the origin base station, as on 2026-10-02 (roughly)
const Eigen::Isometry3d k_W0 = iso(-0.17, 2.25, -5.26, 152.0);
// another station's frame, metres away and turned
const Eigen::Isometry3d k_OTHER = iso(2.52, 2.31, -2.31, -95.0);

std::vector<Eigen::Vector3d> driverPositions()
{
    // six devices in driver space (a body standing about 4 m from the origin station)
    std::vector<Eigen::Vector3d> p;
    const Eigen::Vector3d body(0.4, -1.2, 4.0);
    const Eigen::Vector3d offsets[] = { { 0, 1.6, 0 }, { 0.2, 1.0, 0 }, { -0.2, 1.0, 0 }, { 0.1, 0.1, 0 }, { -0.1, 0.1, 0 }, { 0, 1.1, 0.1 } };
    for (const auto& o : offsets)
        p.push_back(body + o);
    return p;
}

std::vector<UniverseDevice> devicesIn(const std::vector<Eigen::Isometry3d>& wfd, const std::vector<Eigen::Vector3d>& d)
{
    std::vector<UniverseDevice> out;
    for (size_t i = 0; i < d.size(); i++) {
        UniverseDevice u;
        u.index = static_cast<uint8_t>(10 + i);
        u.wfd = wfd[i];
        u.driverPos = d[i];
        out.push_back(u);
    }
    return out;
}

// where the trust layer sees a device: stable = correction * raw
Eigen::Vector3d stable(const UniverseWatch& w, const UniverseDevice& u)
{
    return w.correction(u.index) * (u.wfd * u.driverPos);
}
}

TEST(universe_shift_is_confirmed_and_the_stable_frame_does_not_move)
{
    UniverseWatch w;
    const auto d = driverPositions();
    std::vector<Eigen::Isometry3d> wfd(d.size(), k_W0);
    UniverseShift shift;
    for (int i = 0; i < 5; i++)
        CHECK(!w.tick(i / 60.0, devicesIn(wfd, d), true, &shift));
    std::vector<Eigen::Vector3d> before;
    for (const auto& u : devicesIn(wfd, d))
        before.push_back(stable(w, u));

    // SteamVR re-solves: 18 cm / 0.84 deg, five devices this frame, the sixth one frame later
    const Eigen::Isometry3d delta = iso(-0.17, 0.026, -0.044, 0.84);
    for (size_t i = 0; i < 5; i++)
        wfd[i] = delta * k_W0;
    CHECK(w.tick(5 / 60.0, devicesIn(wfd, d), true, &shift));
    CHECK_EQ(shift.devices, 5);
    CHECK(shift.corrected);
    CHECK_NEAR((shift.delta.translation() - delta.translation()).norm(), 0.0, 1e-9);
    CHECK(shift.moveAtDevices > 0.05 && shift.moveAtDevices < 0.5);
    auto now = devicesIn(wfd, d);
    for (size_t i = 0; i < now.size(); i++)
        CHECK_NEAR((stable(w, now[i]) - before[i]).norm(), 0.0, 1e-9); // the lagging one included

    wfd[5] = delta * k_W0;
    CHECK(!w.tick(6 / 60.0, devicesIn(wfd, d), true, &shift)); // the late one is part of the same shift
    now = devicesIn(wfd, d);
    for (size_t i = 0; i < now.size(); i++)
        CHECK_NEAR((stable(w, now[i]) - before[i]).norm(), 0.0, 1e-9);
    CHECK_EQ(w.shifts(), 1u);

    // the calibration correction keeps calibrated poses: C_new * raw_new == C_old * raw_old
    const Eigen::Isometry3d cOld = iso(-1.24, 0.08, 2.18, -43.6);
    const Eigen::Isometry3d cNew = cOld * shift.delta.inverse();
    const Eigen::Vector3d rawOld = k_W0 * d[0];
    const Eigen::Vector3d rawNew = wfd[0] * d[0];
    CHECK_NEAR((cNew * rawNew - cOld * rawOld).norm(), 0.0, 1e-9);
    // and the trust layer's calibration stays the same: C_trust = C_applied * global^-1
    CHECK_NEAR((cNew * w.global().inverse() * stable(w, now[0]) - cOld * before[0]).norm(), 0.0, 1e-9);
}

TEST(a_tracker_in_another_stations_frame_is_not_a_shift)
{
    UniverseWatch w;
    const auto d = driverPositions();
    std::vector<Eigen::Isometry3d> wfd(d.size(), k_W0);
    UniverseShift shift;
    w.tick(0.0, devicesIn(wfd, d), true, &shift);
    // one tracker (seen 2026-10-02) is solved against another base station: its transform jumps by metres
    wfd[3] = k_OTHER;
    CHECK(!w.tick(1 / 60.0, devicesIn(wfd, d), true, &shift));
    CHECK(w.correction(13).isApprox(w.global())); // its jump stays visible to the trust layer
    // two of them in that frame are not a shift either: too far to be a re-solve
    wfd[4] = k_OTHER;
    CHECK(!w.tick(2 / 60.0, devicesIn(wfd, d), true, &shift));
    CHECK_EQ(w.shifts(), 0u);
}

TEST(a_lone_device_in_a_new_frame_is_held_only_briefly)
{
    UniverseWatch w;
    const auto d = driverPositions();
    std::vector<Eigen::Isometry3d> wfd(d.size(), k_W0);
    UniverseShift shift;
    w.tick(0.0, devicesIn(wfd, d), true, &shift);
    const Eigen::Vector3d before = stable(w, devicesIn(wfd, d)[2]);
    wfd[2] = iso(0.08, 0.0, 0.0, 0.0) * k_W0;
    CHECK(!w.tick(0.02, devicesIn(wfd, d), true, &shift));
    CHECK_NEAR((stable(w, devicesIn(wfd, d)[2]) - before).norm(), 0.0, 1e-9); // held: maybe the start of a shift
    CHECK(!w.tick(0.5, devicesIn(wfd, d), true, &shift)); // nobody followed
    CHECK((stable(w, devicesIn(wfd, d)[2]) - before).norm() > 0.03); // now it counts as its own jump
    CHECK_EQ(w.shifts(), 0u);
}

TEST(without_correction_the_shift_is_reported_but_not_hidden)
{
    UniverseWatch w;
    const auto d = driverPositions();
    std::vector<Eigen::Isometry3d> wfd(d.size(), k_W0);
    UniverseShift shift;
    w.tick(0.0, devicesIn(wfd, d), false, &shift);
    for (auto& x : wfd)
        x = iso(0.0, 0.05, 0.0, 0.3) * k_W0;
    CHECK(w.tick(0.02, devicesIn(wfd, d), false, &shift));
    CHECK(!shift.corrected);
    CHECK(w.global().isApprox(Eigen::Isometry3d::Identity()));
}

TEST(the_universe_follows_a_new_origin_station)
{
    UniverseWatch w;
    const auto d = driverPositions();
    std::vector<Eigen::Isometry3d> wfd(d.size(), k_W0);
    UniverseShift shift;
    w.tick(0.0, devicesIn(wfd, d), true, &shift);
    // the origin station is gone: every device lives in another station's frame from now on
    for (auto& x : wfd)
        x = k_OTHER;
    for (int i = 1; i <= 200; i++)
        CHECK(!w.tick(i / 60.0, devicesIn(wfd, d), true, &shift));
    // a re-solve of that frame is still caught
    for (auto& x : wfd)
        x = iso(0.03, 0.0, 0.02, 0.4) * k_OTHER;
    CHECK(w.tick(201 / 60.0, devicesIn(wfd, d), true, &shift));
    CHECK_EQ(shift.devices, 6);
}

int main()
{
    return spacecal::test::runAll();
}
