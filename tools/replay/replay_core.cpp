#include "replay_core.h"
#include "live_stats.h" // headFrame: the error vector in the head's frame, as the Live tab shows it

#include <glaze/glaze.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>

namespace spacecal::replay {

namespace fs = std::filesystem;


bool applyOverride(guard::GuardConfig::Trust& t, const std::string& key, double value)
{
    std::string k = key;
    if (k.rfind("trust.", 0) == 0)
        k = k.substr(6);
#define P(name)                  \
    if (k == #name) {            \
        t.name = decltype(t.name)(value); \
        return true;             \
    }
    P(v_max) P(j_tol) P(v_err_max) P(a_max) P(rot_max_dps) P(rot_tol_deg) P(t_stale) P(t_confirm) P(r_ok) P(t_recover)
    P(d_agree) P(a_agree_deg) P(jump_fit_residual) P(r_high) P(t_residual_high) P(n_jurors) P(t_device_lost) P(c_l_smoothing)
    P(dt_max) P(back_on_path_fraction) P(v_unexplained) P(rot_unexplained_dps) P(v_body_max) P(w_body_max_dps) P(t_shift_window) P(ref_jump_min)
#undef P
    return false;
}



Eigen::Isometry3d isoFromVectors(const std::vector<double>& t, const std::vector<double>& q)
{
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    if (t.size() == 3)
        iso.translation() = Eigen::Vector3d(t[0], t[1], t[2]);
    if (q.size() == 4)
        iso.linear() = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized().toRotationMatrix();
    return iso;
}

ReplayResult replay(const blackbox::Recording& rec, const OverlayCalibration& cal, const std::map<uint8_t, DeviceMeta>& meta, const trust::ConsensusParams& params, double hz, bool relative, const fs::path& csvPath, CalibrationMode calMode, const ReplayFrames& frames)
{
    const bool ownCalibration = calMode == CalibrationMode::OWN;
    const bool corrected = calMode == CalibrationMode::CORRECTED;
    ReplayResult result;
    trust::Consensus consensus;
    consensus.setParams(params);
    trust::UniverseWatch universeWatch; // as the overlay's TrustManager does it
    trust::FrameCorrector frameCorrector;
    frameCorrector.setParams(frames.params);

    struct Latest {
        bool has = false;
        blackbox::Record pose;
        bool hasWfd = false;
        blackbox::Record wfd;
    };
    std::map<uint8_t, Latest> latest;

    const auto& records = rec.records;
    if (records.empty())
        return result;
    // the replayed span is the one covered by poses; SESSION / TEXT records carry the recorder's own
    // clock, which need not match synthetic pose times (a test recording spanned days of uptime)
    bool haveSpan = false;
    for (const blackbox::Record& r : records) {
        if (r.type != static_cast<uint8_t>(blackbox::RecordType::POSE))
            continue;
        if (!haveSpan) {
            result.firstT = r.t;
            haveSpan = true;
        }
        result.lastT = r.t;
    }
    if (!haveSpan)
        return result;
    const double dt = 1.0 / hz;

    std::ofstream csv;
    if (!csvPath.empty()) {
        csv.open(csvPath);
        csv << "t,residual_valid,residual_m,hold_target,target_state,target_flag,err_forward_m,err_right_m,err_up_m\n";
    }

    // WorldFromDriver is recorded only when it changes; recorders since 2026-09-30 also repeat it at the
    // start of every chunk. In older recordings a folder of chunks may start with poses whose transform
    // comes later or never (copy the last one from earlier chunks into the folder). Poses before a
    // device's first WFD record use that first one (without it the head tracker sat 0.7 m off in an
    // archive segment).
    for (const blackbox::Record& r : records) {
        if (r.type == static_cast<uint8_t>(blackbox::RecordType::WORLD_FROM_DRIVER) && !latest[r.device].hasWfd) {
            latest[r.device].hasWfd = true;
            latest[r.device].wfd = r;
        }
    }

    size_t cursor = 0;
    bool pendingSolve = false; // the overlay would run a solve while RECOVERING; the replay accepts it
    const uint8_t refIndex = static_cast<uint8_t>(cal.reference.index >= 0 && cal.reference.index < 64 ? cal.reference.index : 255);
    const uint8_t tgtIndex = static_cast<uint8_t>(cal.target.index >= 0 && cal.target.index < 64 ? cal.target.index : 255);
    const Eigen::Isometry3d calIso = isoFromVectors(cal.translation_m, cal.rotation_quat_wxyz);
    Eigen::Isometry3d liveCal = calIso;
    Eigen::Isometry3d recordedCal = calIso; // CORRECTED: the last recorded calibration
    Eigen::Isometry3d correction = Eigen::Isometry3d::Identity(); // CORRECTED: our fixes the live run did not make
    bool haveCorrection = false;
    double lastRecordedJumpT = -1e9; // CORRECTED: the recorded calibration last moved the head tracker by > 3 cm
    // the head tracker's position in the driver's world, where calibrations are compared
    auto headProbe = [&latest, tgtIndex]() {
        Eigen::Vector3d probe(0.0, 1.5, 0.0);
        auto lt = latest.find(tgtIndex);
        if (lt != latest.end() && lt->second.has) {
            probe = Eigen::Vector3d(lt->second.pose.v[0], lt->second.pose.v[1], lt->second.pose.v[2]);
            if (lt->second.hasWfd) {
                const Eigen::Quaterniond wq(lt->second.wfd.v[3], lt->second.wfd.v[4], lt->second.wfd.v[5], lt->second.wfd.v[6]);
                probe = Eigen::Vector3d(lt->second.wfd.v[0], lt->second.wfd.v[1], lt->second.wfd.v[2]) + wq * probe;
            }
        }
        return probe;
    };
    if (!relative) {
        // the calibration in effect when the recording starts is closer to the first one the driver
        // applied in it than to overlay.json, which may be from much later (archive segments)
        for (const blackbox::Record& r : records) {
            if (r.type == static_cast<uint8_t>(blackbox::RecordType::APPLIED) && r.device == tgtIndex) {
                liveCal = Eigen::Isometry3d::Identity();
                liveCal.translation() = Eigen::Vector3d(r.v[0], r.v[1], r.v[2]);
                liveCal.linear() = Eigen::Quaterniond(r.v[3], r.v[4], r.v[5], r.v[6]).normalized().toRotationMatrix();
                recordedCal = liveCal;
                break;
            }
        }
    }

    for (double t = result.firstT; t <= result.lastT + 1e-9; t += dt) {
        while (cursor < records.size() && records[cursor].t <= t) {
            const blackbox::Record& r = records[cursor];
            if (r.type == static_cast<uint8_t>(blackbox::RecordType::POSE)) {
                latest[r.device].has = true;
                latest[r.device].pose = r;
            } else if (r.type == static_cast<uint8_t>(blackbox::RecordType::WORLD_FROM_DRIVER)) {
                latest[r.device].hasWfd = true;
                latest[r.device].wfd = r;
            } else if (r.type == static_cast<uint8_t>(blackbox::RecordType::APPLIED) && r.device == tgtIndex && !relative && !ownCalibration) {
                // fixed mode: the driver logged the world calibration it applied; follow it
                Eigen::Isometry3d applied = Eigen::Isometry3d::Identity();
                applied.translation() = Eigen::Vector3d(r.v[0], r.v[1], r.v[2]);
                applied.linear() = Eigen::Quaterniond(r.v[3], r.v[4], r.v[5], r.v[6]).normalized().toRotationMatrix();
                const Eigen::Vector3d probe = headProbe();
                if (corrected && (applied * probe - recordedCal * probe).norm() > 0.03)
                    lastRecordedJumpT = r.t;
                if (corrected && haveCorrection) {
                    // did the live solver catch up with our correction? compare at the head tracker
                    const double caughtUp = (applied * probe - (recordedCal * correction) * probe).norm();
                    const double notYet = (applied * probe - recordedCal * probe).norm();
                    if (caughtUp < notYet) {
                        correction = Eigen::Isometry3d::Identity();
                        haveCorrection = false;
                    }
                }
                recordedCal = applied;
                liveCal = corrected ? Eigen::Isometry3d(recordedCal * correction) : applied;
            }
            cursor++;
        }

        trust::TickInput in;
        in.t = t;
        std::vector<trust::UniverseDevice> universeDevices;
        std::vector<trust::FrameSample> frameSamples;
        for (auto& [idx, l] : latest) {
            if (!l.has)
                continue;
            auto it = meta.find(idx);
            const int cls = it != meta.end() ? it->second.deviceClass : 0;
            const bool bodyWorn = cls == 1 || cls == 2 || cls == 3;
            if (!bodyWorn)
                continue;
            trust::DeviceInput d;
            d.index = idx;
            d.bodyWorn = true;
            const std::string sys = it != meta.end() ? it->second.sys : "";
            d.universe = sys == cal.reference.tracking_system ? trust::Universe::REFERENCE : (sys == cal.target.tracking_system ? trust::Universe::TARGET : trust::Universe::OTHER);

            const blackbox::Record& p = l.pose;
            trust::PoseSample s;
            s.t = p.t;
            s.tracking = (p.b & blackbox::k_POSE_FLAG_VALID) && (p.b & blackbox::k_POSE_FLAG_CONNECTED) && p.a == 200;
            Eigen::Vector3d pos(p.v[0], p.v[1], p.v[2]);
            Eigen::Quaterniond q(p.v[3], p.v[4], p.v[5], p.v[6]);
            Eigen::Vector3d v(p.v[7], p.v[8], p.v[9]);
            if (l.hasWfd) {
                const Eigen::Vector3d wt(l.wfd.v[0], l.wfd.v[1], l.wfd.v[2]);
                const Eigen::Quaterniond wq(l.wfd.v[3], l.wfd.v[4], l.wfd.v[5], l.wfd.v[6]);
                pos = wt + wq * pos;
                q = (wq * q).normalized();
                v = wq * v;
            }
            s.p = pos;
            s.q = q.normalized();
            s.v = v;
            s.w = Eigen::Vector3d(p.v[10], p.v[11], p.v[12]);
            // the sample time is the tick time so histories see uniform dt like the overlay does
            s.t = t;
            d.sample = s;
            in.devices.push_back(d);
            if (d.universe == trust::Universe::TARGET && l.hasWfd) {
                trust::FrameSample f;
                f.index = idx;
                f.tracking = s.tracking;
                f.reference = idx == tgtIndex;
                f.wfd = isoFromVectors({ l.wfd.v[0], l.wfd.v[1], l.wfd.v[2] }, { l.wfd.v[3], l.wfd.v[4], l.wfd.v[5], l.wfd.v[6] });
                f.pose = isoFromVectors({ p.v[0], p.v[1], p.v[2] }, { p.v[3], p.v[4], p.v[5], p.v[6] });
                frameSamples.push_back(f);
                trust::UniverseDevice u;
                u.index = idx;
                u.wfd = isoFromVectors({ l.wfd.v[0], l.wfd.v[1], l.wfd.v[2] }, { l.wfd.v[3], l.wfd.v[4], l.wfd.v[5], l.wfd.v[6] });
                u.driverPos = Eigen::Vector3d(p.v[0], p.v[1], p.v[2]);
                universeDevices.push_back(u);
            }
        }
        {
            // SteamVR universe shifts: corrected only where the replay corrects (the live run before
            // 2026-10-02 did not; its next solves caught up, see the CORRECTED bookkeeping above)
            trust::UniverseShift shift;
            if (frames.perDevice) {
                std::vector<trust::FrameEvent> events;
                frameCorrector.tick(t, frameSamples, corrected && !relative, &events);
                for (const trust::FrameEvent& e : events) {
                    result.frameEvents.push_back(e);
                    if (e.kind == trust::FrameEventKind::REFERENCE_MOVED && e.corrected) {
                        correction = correction * e.delta.inverse();
                        haveCorrection = true;
                        liveCal = recordedCal * correction;
                    }
                }
            } else if (universeWatch.tick(t, universeDevices, corrected && !relative, &shift)) {
                result.universeShifts.push_back(shift);
                if (shift.corrected) {
                    correction = correction * shift.delta.inverse();
                    haveCorrection = true;
                    liveCal = recordedCal * correction;
                }
            }
            for (trust::DeviceInput& d : in.devices) {
                if (d.universe != trust::Universe::TARGET)
                    continue;
                const Eigen::Isometry3d a = frames.perDevice ? frameCorrector.correction(d.index) : universeWatch.correction(d.index);
                d.sample.p = a * d.sample.p;
                d.sample.q = Eigen::Quaterniond(a.linear() * d.sample.q.toRotationMatrix()).normalized();
                d.sample.v = a.linear() * d.sample.v;
            }
        }

        in.calib.valid = cal.valid && cal.active;
        in.calib.relativeMode = relative;
        if (pendingSolve) {
            in.calib.solveAttempted = true;
            in.calib.solveValid = true;
            in.calib.solveAgrees = true;
            pendingSolve = false;
        }
        in.calib.referenceIndex = refIndex;
        in.calib.targetIndex = tgtIndex;
        if (relative) {
            in.calib.C_L_valid = in.calib.valid;
            in.calib.C_L = calIso;
        } else {
            in.calib.C_world_valid = in.calib.valid;
            in.calib.C_world = liveCal * (frames.perDevice ? frameCorrector.global() : universeWatch.global()).inverse();
        }

        trust::TickOutput out;
        consensus.tick(in, out);
        result.ticks++;
        for (const auto& tr : out.transitions)
            result.transitions.push_back(tr);
        if (out.event != trust::Event::NONE) {
            result.events.push_back({ t, out.event });
            result.eventShift.push_back(out.event == trust::Event::PLAYSPACE_JUMP ? out.eventDelta.translation().norm() : 0.0);
            if (out.event == trust::Event::PLAYSPACE_JUMP && ownCalibration && !relative)
                liveCal = liveCal * out.eventDelta.inverse(); // what the overlay's playspace-jump fix applies
            // the live run may have corrected the same shift a moment earlier (upstream's fix reacts to the
            // WorldFromDriver change within a frame); correcting it again doubled it (seg 140, 03:02: 20 cm)
            if (out.event == trust::Event::PLAYSPACE_JUMP && corrected && !relative && t - lastRecordedJumpT > 0.5) {
                correction = correction * out.eventDelta.inverse();
                haveCorrection = true;
                liveCal = recordedCal * correction;
            }
        }
        if (out.targetRecovering) {
            pendingSolve = true; // documented limitation: recovery solves are assumed to agree
        }
        if (csv.is_open()) {
            const trust::DeviceStatus st = consensus.status(tgtIndex);
            csv << t << ',' << (out.targetResidualValid ? 1 : 0) << ',' << out.targetResidual << ',' << (out.holdTarget ? 1 : 0) << ',' << trust::stateName(st.state) << ',' << trust::flagName(st.lastFlag);
            // calibrated minus expected head tracker position, split into forward / right / up of the headset
            auto lr = latest.find(refIndex);
            if (out.targetResidualValid && lr != latest.end() && lr->second.has) {
                const blackbox::Record& hp = lr->second.pose;
                Eigen::Quaterniond hq(hp.v[3], hp.v[4], hp.v[5], hp.v[6]);
                if (lr->second.hasWfd)
                    hq = Eigen::Quaterniond(lr->second.wfd.v[3], lr->second.wfd.v[4], lr->second.wfd.v[5], lr->second.wfd.v[6]) * hq;
                const Eigen::Vector3d f = guard::headFrame(hq.normalized().toRotationMatrix(), out.targetResidualVec);
                csv << ',' << f.x() << ',' << f.y() << ',' << f.z() << '\n';
            } else {
                csv << ",,,\n";
            }
        }
    }
    result.anchors = consensus.anchorCount();
    return result;
}


bool loadOverlayJson(const std::filesystem::path& file, OverlayJson& out, std::string* error)
{
    constexpr glz::opts opts { .error_on_unknown_keys = false, .error_on_missing_keys = false };
    auto ec = glz::read_file_json<opts>(out, file.string(), std::string {});
    if (ec) {
        if (error)
            *error = glz::format_error(ec, std::string {});
        return false;
    }
    return true;
}

std::map<uint8_t, DeviceMeta> deviceMetaFromRecording(const blackbox::Recording& rec)
{
    std::map<uint8_t, DeviceMeta> meta;
    for (const auto& [idx, kv] : rec.devices) {
        DeviceMeta m;
        auto get = [&](const char* k) { auto it = kv.find(k); return it != kv.end() ? it->second : std::string(); };
        m.deviceClass = std::atoi(get("class").c_str());
        m.sys = get("sys");
        m.model = get("model");
        m.serial = get("serial");
        meta[idx] = m;
    }
    return meta;
}

} // namespace spacecal::replay
