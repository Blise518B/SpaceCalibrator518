// Live tab data (fork, src/overlay/guard/live_stats.h): head-frame error, yaw, spike-preserving
// decimation and the five-minute buffers.

#include "live_stats.h"
#include "test_support.h"

#include <cmath>
#include <vector>

using namespace spacecal::guard;

namespace {
constexpr double k_PI = 3.14159265358979323846;

Eigen::Matrix3d rotY(double deg)
{
    return Eigen::AngleAxisd(deg * k_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

Eigen::Matrix3d rotX(double deg)
{
    return Eigen::AngleAxisd(deg * k_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
}
}

TEST(head_frame_follows_where_the_head_faces)
{
    // headset facing -z (OpenVR forward): forward, right, up
    Eigen::Vector3d e = headFrame(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, -0.1));
    CHECK_NEAR(e.x(), 0.1, 1e-9);
    CHECK_NEAR(e.y(), 0.0, 1e-9);
    e = headFrame(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.1, 0.0, 0.0));
    CHECK_NEAR(e.y(), 0.1, 1e-9);
    e = headFrame(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.1, 0.0));
    CHECK_NEAR(e.z(), 0.1, 1e-9);
    // turned 90 deg to the left: forward is -x now
    e = headFrame(rotY(90.0), Eigen::Vector3d(-0.1, 0.0, 0.0));
    CHECK_NEAR(e.x(), 0.1, 1e-9);
    // looking down a bit: forward stays horizontal, up stays world up
    e = headFrame(rotY(90.0) * rotX(-40.0), Eigen::Vector3d(-0.1, 0.05, 0.0));
    CHECK_NEAR(e.x(), 0.1, 1e-9);
    CHECK_NEAR(e.z(), 0.05, 1e-9);
    // looking straight down: the top of the head gives the heading
    e = headFrame(rotX(-90.0), Eigen::Vector3d(0.0, 0.0, -0.1));
    CHECK_NEAR(e.x(), 0.1, 1e-9);
    // straight up: the opposite way
    e = headFrame(rotX(90.0), Eigen::Vector3d(0.0, 0.0, -0.1));
    CHECK_NEAR(e.x(), 0.1, 1e-9);
}

TEST(yaw_of_a_rotation)
{
    CHECK_NEAR(yawDegrees(Eigen::Quaterniond::Identity()), 0.0, 1e-9);
    CHECK_NEAR(yawDegrees(Eigen::Quaterniond(rotY(90.0))), 90.0, 1e-6);
    CHECK_NEAR(yawDegrees(Eigen::Quaterniond(rotY(-30.0) * rotX(20.0))), -30.0, 1e-6);
}

TEST(decimation_keeps_spikes_and_gaps)
{
    std::vector<double> xs, ys, ox, oy;
    for (int i = 0; i < 10000; i++) {
        xs.push_back(i * 0.01);
        ys.push_back(i == 5000 ? 100.0 : (i == 7000 ? std::nan("") : 1.0 + 0.001 * (i % 7)));
    }
    decimateMinMax(xs, ys, 1000, ox, oy);
    CHECK(ox.size() <= 1100);
    CHECK(ox.size() >= 500);
    bool spike = false, gap = false;
    for (size_t i = 0; i < oy.size(); i++) {
        spike = spike || oy[i] == 100.0;
        gap = gap || std::isnan(oy[i]);
        if (i > 0)
            CHECK(ox[i] >= ox[i - 1]);
    }
    CHECK(spike);
    CHECK(gap);
    // short series pass through untouched
    std::vector<double> sx = { 0, 1, 2 }, sy = { 3, 4, 5 };
    decimateMinMax(sx, sy, 1000, ox, oy);
    CHECK_EQ(oy.size(), 3u);
}

TEST(live_buffers_keep_five_minutes)
{
    LiveStats live;
    live.init();
    CHECK(LiveStats::getInstance() == &live);
    for (int i = 0; i < 4000; i++) {
        LiveHeadSample h;
        h.t = i * 0.1;
        h.valid = true;
        h.errorCm = 1.0f;
        live.pushHead(h);
    }
    CHECK_NEAR(live.now(), 399.9, 1e-9);
    CHECK(live.head().front().t >= live.now() - LiveStats::k_KEEP_SECONDS - 1e-9);
    CHECK(live.head().size() <= static_cast<size_t>(LiveStats::k_KEEP_SECONDS * 10.0) + 2);

    // an unchanged calibration ten times a second, a change at once
    for (int i = 0; i < 100; i++) {
        LiveCalibrationSample c;
        c.t = 400.0 + i * 0.01;
        c.xCm = 5.0f;
        live.pushCalibration(c);
    }
    CHECK(live.calibration().size() <= 11u);
    const size_t before = live.calibration().size();
    LiveCalibrationSample moved;
    moved.t = live.calibration().back().t + 0.001;
    moved.xCm = 7.0f;
    live.pushCalibration(moved);
    CHECK_EQ(live.calibration().size(), before + 1);

    // solves and events without a time get the latest one
    LiveSolve s;
    s.outcome = 1;
    live.pushSolve(s);
    CHECK_NEAR(live.solves().back().t, live.now(), 1e-9);
    live.pushEvent({ 0.0, LiveEventKind::PLAYSPACE_SHIFT, 12.0f, "corrected" });
    live.pushEvent({ 0.0, LiveEventKind::MARKER, 0.0f, "hotkey" });
    CHECK_EQ(live.playspaceShiftsTotal(), 1u);
    CHECK_EQ(live.events().size(), 2u);

    // a device's buffer is trimmed on its own pushes
    LiveDeviceInfo info;
    info.known = true;
    info.label = "Head tracker";
    for (int i = 0; i < 400; i++) {
        LiveDeviceSample d;
        d.t = 100.0 + i;
        live.pushDevice(20, info, d);
    }
    CHECK(live.device(20).front().t >= live.now() - LiveStats::k_KEEP_SECONDS - 1e-9);
    CHECK(live.deviceInfo(20).label == "Head tracker");
}

int main()
{
    return spacecal::test::runAll();
}
