// End-to-end test of the replay path (fork): a synthetic session is written with the real
// BlackBox recorder (POSE, WORLD_FROM_DRIVER, DEVICE records) plus an overlay.json, then read
// back with the real reader and pushed through the replay core. The 30 cm head-tracker step must
// produce the same decisions the trust tests expect.

#include "black_box.h"
#include "pose_record.h"
#include "replay_core.h"
#include "test_support.h"
#include "trust/trust_params.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <openvr_driver.h>
#include <thread>

using namespace spacecal;
namespace fs = std::filesystem;

namespace {

Eigen::Isometry3d makeIso(double yawDeg, const Eigen::Vector3d& t)
{
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = Eigen::AngleAxisd(yawDeg * trust::k_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    iso.translation() = t;
    return iso;
}

vr::DriverPose_t poseFromIso(const Eigen::Isometry3d& iso, const Eigen::Vector3d& v)
{
    vr::DriverPose_t p = {};
    p.poseIsValid = true;
    p.deviceIsConnected = true;
    p.result = vr::TrackingResult_Running_OK;
    p.qWorldFromDriverRotation = { 1, 0, 0, 0 };
    p.qDriverFromHeadRotation = { 1, 0, 0, 0 };
    const Eigen::Quaterniond q(iso.linear());
    p.qRotation = { q.w(), q.x(), q.y(), q.z() };
    p.vecPosition[0] = iso.translation().x();
    p.vecPosition[1] = iso.translation().y();
    p.vecPosition[2] = iso.translation().z();
    p.vecVelocity[0] = v.x();
    p.vecVelocity[1] = v.y();
    p.vecVelocity[2] = v.z();
    return p;
}

} // namespace

TEST(replay_reproduces_head_tracker_step)
{
    const fs::path dir = fs::temp_directory_path() / "spacecal_replay_test";
    std::error_code ec;
    fs::remove_all(dir, ec);
    const Eigen::Isometry3d C_true = makeIso(30.0, Eigen::Vector3d(1.0, 0.0, -0.5));
    const Eigen::Isometry3d C_L = makeIso(2.0, Eigen::Vector3d(0.0, 0.08, -0.05));

    // ---- write a session with the real recorder --------------------------------------------
    {
        blackbox::BlackBox bb;
        blackbox::Params p;
        p.chunkSeconds = 3600;
        p.liveWindowSeconds = 7200;
        CHECK(bb.start(dir / "bb", "replay-test", p, 0.05));
        const double t0 = 1000.0;
        // device table
        bb.recordText(blackbox::RecordType::DEVICE, 0, blackbox::TextKind::DEVICE_INFO, "class=1;role=0;sys=oculus;model=Quest Pro;serial=HMD-1", t0);
        bb.recordText(blackbox::RecordType::DEVICE, 3, blackbox::TextKind::DEVICE_INFO, "class=3;role=0;sys=lighthouse;model=VIVE Tracker 3.0;serial=LHR-HEAD", t0);
        bb.recordText(blackbox::RecordType::DEVICE, 4, blackbox::TextKind::DEVICE_INFO, "class=3;role=0;sys=lighthouse;model=VIVE Tracker 3.0;serial=LHR-HIP", t0);
        bb.recordText(blackbox::RecordType::DEVICE, 5, blackbox::TextKind::DEVICE_INFO, "class=3;role=0;sys=lighthouse;model=VIVE Tracker 3.0;serial=LHR-FOOTL", t0);
        bb.recordText(blackbox::RecordType::DEVICE, 6, blackbox::TextKind::DEVICE_INFO, "class=3;role=0;sys=lighthouse;model=VIVE Tracker 3.0;serial=LHR-FOOTR", t0);
        bb.recordText(blackbox::RecordType::DEVICE, 9, blackbox::TextKind::DEVICE_INFO, "class=4;role=0;sys=lighthouse;model=Base Station;serial=LHB-1", t0);
        const double wfdT[3] = { 0, 0, 0 };
        const double wfdQ[4] = { 1, 0, 0, 0 };
        for (uint8_t d : { 0, 3, 4, 5, 6 })
            bb.recordWorldFromDriver(d, wfdT, wfdQ, t0);

        const double dt = 0.01; // 100 Hz poses
        auto bodyAt = [&](double t, Eigen::Isometry3d& hmd, Eigen::Isometry3d& head, Eigen::Isometry3d& hip, Eigen::Isometry3d& fl, Eigen::Isometry3d& fr) {
            const Eigen::Vector3d c(0.3 * std::sin(0.5 * t), 0.0, 0.3 * std::cos(0.3 * t));
            hmd = makeIso(20.0 * std::sin(0.4 * t), c + Eigen::Vector3d(0.0, 1.7, 0.0));
            head = hmd * C_L;
            hip = makeIso(0.0, c + Eigen::Vector3d(0.0, 1.0, 0.0));
            fl = makeIso(0.0, c + Eigen::Vector3d(-0.15, 0.05, 0.1 * std::sin(2.0 * t)));
            fr = makeIso(0.0, c + Eigen::Vector3d(0.15, 0.05, -0.1 * std::sin(2.0 * t)));
        };
        for (double rel = 0.0; rel < 30.0; rel += dt) {
            const double t = t0 + rel;
            Eigen::Isometry3d hmd, head, hip, fl, fr, hmdP, headP, hipP, flP, frP;
            bodyAt(rel, hmd, head, hip, fl, fr);
            bodyAt(rel - dt, hmdP, headP, hipP, flP, frP);
            auto rec = [&](uint8_t idx, const Eigen::Isometry3d& worldR, const Eigen::Isometry3d& worldRPrev, bool lighthouse, const Eigen::Vector3d& glitch) {
                Eigen::Isometry3d raw = lighthouse ? Eigen::Isometry3d(C_true.inverse() * worldR) : worldR;
                Eigen::Isometry3d rawP = lighthouse ? Eigen::Isometry3d(C_true.inverse() * worldRPrev) : worldRPrev;
                const Eigen::Vector3d v = (raw.translation() - rawP.translation()) / dt;
                raw.translation() += glitch;
                bb.push(blackbox::makePoseRecord(idx, poseFromIso(raw, v), t));
            };
            const Eigen::Vector3d glitch = (rel >= 10.0 && rel < 20.0) ? Eigen::Vector3d(0.3, 0.0, 0.0) : Eigen::Vector3d(0.0, 0.0, 0.0);
            rec(0, hmd, hmdP, false, Eigen::Vector3d(0, 0, 0));
            rec(3, head, headP, true, glitch);
            rec(4, hip, hipP, true, Eigen::Vector3d(0, 0, 0));
            rec(5, fl, flP, true, Eigen::Vector3d(0, 0, 0));
            rec(6, fr, frP, true, Eigen::Vector3d(0, 0, 0));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        bb.stop();
    }

    // the "event folder" is the live folder here; add the overlay.json the overlay would write
    const fs::path eventDir = dir / "bb" / "live";
    {
        const Eigen::Quaterniond q(C_true.linear());
        std::ofstream f(eventDir / "overlay.json");
        f << "{\n  \"format\": \"spacecal-blackbox-event-overlay\", \"source\": \"hotkey\", \"t_mark_unix\": 0,\n  \"calibrations\": [ {\n";
        f << "    \"active\": true, \"state\": \"continuous\", \"continuous\": true, \"relative\": false, \"valid\": true,\n";
        f << "    \"reference\": { \"index\": 0, \"tracking_system\": \"oculus\", \"model\": \"Quest Pro\", \"serial\": \"HMD-1\" },\n";
        f << "    \"target\": { \"index\": 3, \"tracking_system\": \"lighthouse\", \"model\": \"VIVE Tracker 3.0\", \"serial\": \"LHR-HEAD\" },\n";
        f << "    \"translation_m\": [" << C_true.translation().x() << ", " << C_true.translation().y() << ", " << C_true.translation().z() << "],\n";
        f << "    \"rotation_quat_wxyz\": [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]\n";
        f << "  } ]\n}\n";
    }

    // ---- read back and replay -------------------------------------------------------------
    blackbox::Recording rec;
    CHECK(blackbox::loadRecording(eventDir, rec));
    CHECK(rec.devices.size() == 6);
    CHECK(rec.devices[3].at("serial") == "LHR-HEAD");

    replay::OverlayJson overlay;
    std::string err;
    CHECK(replay::loadOverlayJson(eventDir / "overlay.json", overlay, &err));
    CHECK(overlay.calibrations.size() == 1);
    CHECK(overlay.calibrations[0].target.index == 3);

    guard::GuardConfig cfg;
    replay::ReplayResult r = replay::replay(rec, overlay.calibrations[0], replay::deviceMetaFromRecording(rec), trust::paramsFromConfig(cfg.trust), 20.0, false, fs::path {});
    for (const auto& tr : r.transitions)
        std::printf("    t=+%.2f dev %d %s -> %s (%s) r=%.3f\n", tr.t - r.firstT, tr.device, trust::stateName(tr.from), trust::stateName(tr.to), trust::reasonName(tr.reason), tr.residual);

    auto count = [&](uint8_t dev, trust::State to, trust::Reason reason) {
        int n = 0;
        for (const auto& tr : r.transitions)
            if (tr.device == dev && tr.to == to && tr.reason == reason)
                n++;
        return n;
    };
    CHECK(r.ticks > 500);
    CHECK_EQ(count(3, trust::State::SUSPECT, trust::Reason::JUMP_SINGLE), 1);
    CHECK_EQ(count(3, trust::State::UNTRUSTED, trust::Reason::CONFIRM_TIMEOUT), 1);
    CHECK_EQ(count(3, trust::State::RECOVERING, trust::Reason::JUMP_BACK), 1);
    CHECK_EQ(count(3, trust::State::TRUSTED, trust::Reason::SOLVE_AGREES), 1);
    for (uint8_t d : { 0, 4, 5, 6 })
        CHECK_EQ(count(d, trust::State::SUSPECT, trust::Reason::JUMP_SINGLE) + count(d, trust::State::SUSPECT, trust::Reason::JUMP_AMBIGUOUS), 0);
    double suspectAt = -1.0;
    for (const auto& tr : r.transitions)
        if (tr.device == 3 && tr.to == trust::State::SUSPECT)
            suspectAt = tr.t - r.firstT;
    CHECK(suspectAt > 9.9 && suspectAt < 10.2);

    fs::remove_all(dir, ec); // error_code overload: Windows throws on a still-open handle

}

int main()
{
    return spacecal::test::runAll();
}
