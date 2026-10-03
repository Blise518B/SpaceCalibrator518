// spacecal-replay: feeds a saved black-box event through the same plausibility + consensus
// code the overlay runs, and prints every trust decision (fork, docs/DESIGN.md section 10).
//
//   spacecal-replay run <event folder> [--guard guard.json] [--hz 20] [--csv out.csv]
//                       [--set trust.<param>=<value> ...] [--relative|--fixed]
//                       [--own-calibration|--corrected-calibration] [--quiet] [--shared-frame]
//   spacecal-replay sweep <event folder> trust.<param>=<lo>:<hi>:<step> [--guard guard.json]
//   spacecal-replay info <event folder>
//
// The calibration (reference / target device, anchor mode, transform) is read from the event's
// overlay.json; the device table from the DEVICE records. The overlay's live recompute of the
// relative correction is not replayed: the residual uses the calibration as stored at mark time,
// which is what the frozen calibration would be anyway.

#include "replay_core.h"
#include "trust/trust_params.h"

#include <glaze/glaze.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using namespace spacecal;
namespace rp = spacecal::replay;
namespace fs = std::filesystem;

namespace {

struct Options {
    std::string command;
    fs::path folder;
    fs::path guardJson;
    double hz = 20.0;
    fs::path csv;
    std::vector<std::pair<std::string, double>> overrides;
    int forceMode = 0; // 0 as recorded, 1 relative, 2 fixed
    std::string sweepSpec;
    rp::CalibrationMode calMode = rp::CalibrationMode::RECORDED; // fixed mode, see replay_core.h
    bool quiet = false; // summary only
    bool sharedFrame = false; // SteamVR moving base stations: one shared frame (universe_watch.h) instead of per device
};

bool parseArgs(int argc, char** argv, Options& o)
{
    if (argc < 3)
        return false;
    o.command = argv[1];
    o.folder = argv[2];
    for (int i = 3; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&](std::string& v) { if (i + 1 >= argc) return false; v = argv[++i]; return true; };
        std::string v;
        if (a == "--guard" && next(v))
            o.guardJson = v;
        else if (a == "--hz" && next(v))
            o.hz = std::atof(v.c_str());
        else if (a == "--csv" && next(v))
            o.csv = v;
        else if (a == "--set" && next(v)) {
            const size_t eq = v.find('=');
            if (eq == std::string::npos)
                return false;
            o.overrides.push_back({ v.substr(0, eq), std::atof(v.c_str() + eq + 1) });
        } else if (a == "--relative")
            o.forceMode = 1;
        else if (a == "--fixed")
            o.forceMode = 2;
        else if (a == "--own-calibration")
            o.calMode = rp::CalibrationMode::OWN;
        else if (a == "--corrected-calibration")
            o.calMode = rp::CalibrationMode::CORRECTED;
        else if (a == "--quiet")
            o.quiet = true;
        else if (a == "--shared-frame")
            o.sharedFrame = true;
        else if (o.command == "sweep" && o.sweepSpec.empty())
            o.sweepSpec = a;
        else
            return false;
    }
    return true;
}


void printInfo(const blackbox::Recording& rec, const rp::OverlayJson* overlay)
{
    std::printf("%zu chunks, %zu records, %.1f s\n", rec.chunks.size(), rec.records.size(), rec.records.empty() ? 0.0 : rec.records.back().t - rec.records.front().t);
    std::map<uint8_t, size_t> poses;
    std::map<uint8_t, size_t> types;
    for (const auto& r : rec.records) {
        types[r.type]++;
        if (r.type == static_cast<uint8_t>(blackbox::RecordType::POSE))
            poses[r.device]++;
    }
    for (auto& [t, n] : types)
        std::printf("  %-18s %zu\n", blackbox::recordTypeName(static_cast<blackbox::RecordType>(t)), n);
    for (auto& [d, n] : poses) {
        auto it = rec.devices.find(d);
        auto field = [&](const char* k) -> std::string {
            if (it == rec.devices.end())
                return "?";
            auto f = it->second.find(k);
            return f != it->second.end() ? f->second : "";
        };
        std::printf("  device %2d: %7zu poses  %s %s %s\n", d, n, field("sys").c_str(), field("model").c_str(), field("serial").c_str());
    }
    for (const auto& r : rec.records) {
        if (r.type == static_cast<uint8_t>(blackbox::RecordType::MARKER))
            std::printf("  marker t=%.3f source=%s label=%u\n", r.t, blackbox::markerSourceName(static_cast<blackbox::MarkerSource>(r.a)), r.code);
        if (r.type == static_cast<uint8_t>(blackbox::RecordType::TRUST))
            std::printf("  recorded trust t=%.3f device %d %d -> %d reason %u residual %.3f\n", r.t, r.device, r.a, r.b, r.code, r.f0);
    }
    if (overlay) {
        std::printf("overlay.json: source=%s, %zu calibration(s)\n", overlay->source.c_str(), overlay->calibrations.size());
        for (const auto& c : overlay->calibrations)
            std::printf("  ref [%lld] %s / target [%lld] %s %s, state=%s relative=%d valid=%d\n", static_cast<long long>(c.reference.index), c.reference.tracking_system.c_str(), static_cast<long long>(c.target.index), c.target.model.c_str(), c.target.serial.c_str(), c.state.c_str(), c.relative ? 1 : 0, c.valid ? 1 : 0);
    }
}

} // namespace

int main(int argc, char** argv)
{
    Options opt;
    if (!parseArgs(argc, argv, opt) || (opt.command != "run" && opt.command != "sweep" && opt.command != "info")) {
        std::fprintf(stderr, "usage:\n  spacecal-replay run <event folder> [--guard guard.json] [--hz 20] [--csv out.csv] [--set trust.<param>=<value>] [--relative|--fixed]\n  spacecal-replay sweep <event folder> trust.<param>=<lo>:<hi>:<step> [--guard guard.json]\n  spacecal-replay info <event folder>\n");
        return 2;
    }

    blackbox::Recording rec;
    if (!blackbox::loadRecording(opt.folder, rec)) {
        std::fprintf(stderr, "no chunks found in %s\n", opt.folder.string().c_str());
        return 1;
    }

    rp::OverlayJson overlay;
    bool haveOverlay = false;
    {
        const fs::path p = opt.folder / "overlay.json";
        if (fs::exists(p)) {
            std::string err;
            haveOverlay = rp::loadOverlayJson(p, overlay, &err);
            if (!haveOverlay)
                std::fprintf(stderr, "warning: overlay.json could not be parsed (%s)\n", err.c_str());
        }
    }

    if (opt.command == "info") {
        printInfo(rec, haveOverlay ? &overlay : nullptr);
        return 0;
    }
    if (!haveOverlay || overlay.calibrations.empty()) {
        std::fprintf(stderr, "overlay.json with at least one calibration is required for replay\n");
        return 1;
    }

    // params
    guard::GuardConfig cfg;
    if (!opt.guardJson.empty()) {
        constexpr glz::opts opts { .comments = true, .error_on_unknown_keys = false, .error_on_missing_keys = false };
        auto ec = glz::read_file_jsonc<opts>(cfg, opt.guardJson.string(), std::string {});
        if (ec) {
            std::fprintf(stderr, "cannot read %s\n", opt.guardJson.string().c_str());
            return 1;
        }
    }
    for (const auto& [k, v] : opt.overrides) {
        if (!rp::applyOverride(cfg.trust, k, v)) {
            std::fprintf(stderr, "unknown parameter %s\n", k.c_str());
            return 2;
        }
    }

    // pick the calibration: first continuous one
    const rp::OverlayCalibration* cal = nullptr;
    for (const auto& c : overlay.calibrations)
        if (c.continuous) {
            cal = &c;
            break;
        }
    if (!cal)
        cal = &overlay.calibrations.front();
    const bool relative = opt.forceMode == 1 ? true : (opt.forceMode == 2 ? false : cal->relative);

    std::map<uint8_t, rp::DeviceMeta> meta = rp::deviceMetaFromRecording(rec);
    if (meta.empty()) {
        std::fprintf(stderr, "warning: no DEVICE records in this recording; universes cannot be assigned, every device counts as OTHER\n");
    }

    auto printResult = [&](const rp::ReplayResult& r) {
        std::printf("%zu ticks over %.1f s, %zu transitions, %zu events, %llu re-anchors\n", r.ticks, r.lastT - r.firstT, r.transitions.size(), r.events.size(), static_cast<unsigned long long>(r.anchors));
        std::map<std::string, size_t> summary;
        for (const auto& tr : r.transitions) {
            char key[96];
            std::snprintf(key, sizeof(key), "device %2d %-10s -> %-10s %s", tr.device, trust::stateName(tr.from), trust::stateName(tr.to), trust::reasonName(tr.reason));
            summary[key]++;
        }
        for (const auto& [k, n] : summary)
            std::printf("  %5zu  %s\n", n, k.c_str());
        // time outside TRUSTED per device (a count alone hides a device that never came back)
        std::map<int, double> leftAt, outside;
        for (const auto& tr : r.transitions) {
            if (tr.from == trust::State::TRUSTED && tr.to != trust::State::TRUSTED)
                leftAt[tr.device] = tr.t;
            else if (tr.to == trust::State::TRUSTED && leftAt.count(tr.device)) {
                outside[tr.device] += tr.t - leftAt[tr.device];
                leftAt.erase(tr.device);
            }
        }
        for (const auto& [d, t] : leftAt)
            outside[d] += r.lastT - t;
        for (const auto& [d, t] : outside)
            std::printf("  device %2d not trusted for %.1f s%s\n", d, t, leftAt.count(d) ? " (still at the end)" : "");
        for (size_t i = 0; i < r.events.size(); i++)
            std::printf("  t=%9.3f (+%7.3f) event %s shift=%.3f m\n", r.events[i].first, r.events[i].first - r.firstT,
                r.events[i].second == trust::Event::PLAYSPACE_JUMP ? "PLAYSPACE_JUMP" : "REFERENCE_DISCONTINUITY", i < r.eventShift.size() ? r.eventShift[i] : 0.0);
        {
            std::map<std::string, int> kinds;
            for (const auto& e : r.frameEvents)
                kinds[trust::frameEventName(e.kind)]++;
            for (const auto& [k, n] : kinds)
                std::printf("  %5d  frames %s\n", n, k.c_str());
            for (const auto& e : r.frameEvents) {
                if (opt.quiet && e.kind != trust::FrameEventKind::REFERENCE_MOVED)
                    continue;
                auto it = meta.find(e.index);
                std::printf("  t=%9.3f (+%7.3f) frames %-18s %-12s %.3f m / %.2f deg%s\n", e.t, e.t - r.firstT, trust::frameEventName(e.kind),
                    it != meta.end() ? it->second.serial.c_str() : "?", e.move, e.angleDeg, e.corrected ? ", corrected" : "");
            }
        }
        for (const auto& u : r.universeShifts)
            std::printf("  t=%9.3f (+%7.3f) SteamVR universe shift %.3f m / %.2f deg, %d devices%s\n", u.t, u.t - r.firstT, u.moveAtDevices, u.angleDeg, u.devices,
                u.corrected ? ", corrected" : "");
        if (opt.quiet)
            return;
        for (const auto& tr : r.transitions) {
            auto it = meta.find(tr.device);
            std::printf("  t=%9.3f (+%7.3f) device %2d %-12s %-10s -> %-10s %-22s residual=%.3f n=[%.3f %.3f %.3f]\n", tr.t, tr.t - r.firstT, tr.device,
                it != meta.end() ? it->second.serial.c_str() : "?", trust::stateName(tr.from), trust::stateName(tr.to), trust::reasonName(tr.reason), tr.residual, tr.numbers[0], tr.numbers[1], tr.numbers[2]);
        }
    };

    if (opt.command == "run") {
        rp::ReplayFrames frames;
        frames.perDevice = cfg.trust.per_device_frames && !opt.sharedFrame;
        frames.params.slide_mps = cfg.trust.frames_slide_cm_s / 100.0;
        frames.params.slide_dps = cfg.trust.frames_slide_deg_s;
        frames.params.absorb_switches = cfg.trust.frames_absorb_switches;
        rp::ReplayResult r = rp::replay(rec, *cal, meta, trust::paramsFromConfig(cfg.trust), opt.hz, relative, opt.csv, opt.calMode, frames);
        printResult(r);
        // compare with what the live overlay recorded
        size_t recorded = 0, matched = 0;
        for (const auto& rr : rec.records) {
            if (rr.type != static_cast<uint8_t>(blackbox::RecordType::TRUST))
                continue;
            recorded++;
            for (const auto& tr : r.transitions) {
                if (tr.device == rr.device && static_cast<uint8_t>(tr.to) == rr.b && std::abs(tr.t - rr.t) < 0.25) {
                    matched++;
                    break;
                }
            }
        }
        if (recorded)
            std::printf("recorded live transitions: %zu, reproduced within 0.25 s: %zu\n", recorded, matched);
        return 0;
    }

    // sweep
    const size_t eq = opt.sweepSpec.find('=');
    if (eq == std::string::npos) {
        std::fprintf(stderr, "sweep needs trust.<param>=<lo>:<hi>:<step>\n");
        return 2;
    }
    const std::string key = opt.sweepSpec.substr(0, eq);
    double lo = 0, hi = 0, step = 0;
    if (std::sscanf(opt.sweepSpec.c_str() + eq + 1, "%lf:%lf:%lf", &lo, &hi, &step) != 3 || step <= 0) {
        std::fprintf(stderr, "sweep range must be lo:hi:step\n");
        return 2;
    }
    std::printf("%-12s %-11s %-11s %-11s %s\n", key.c_str(), "suspect", "untrusted", "events", "first suspect (s after start)");
    for (double v = lo; v <= hi + 1e-12; v += step) {
        guard::GuardConfig::Trust t = cfg.trust;
        if (!rp::applyOverride(t, key, v)) {
            std::fprintf(stderr, "unknown parameter %s\n", key.c_str());
            return 2;
        }
        rp::ReplayResult r = rp::replay(rec, *cal, meta, trust::paramsFromConfig(t), opt.hz, relative, fs::path {}, opt.calMode);
        size_t suspects = 0, untrusted = 0;
        double first = -1.0;
        for (const auto& tr : r.transitions) {
            if (tr.to == trust::State::SUSPECT) {
                suspects++;
                if (first < 0.0)
                    first = tr.t - r.firstT;
            }
            if (tr.to == trust::State::UNTRUSTED)
                untrusted++;
        }
        std::printf("%-12.4f %-11zu %-11zu %-11zu %s\n", v, suspects, untrusted, r.events.size(), first < 0.0 ? "-" : std::to_string(first).c_str());
    }
    return 0;
}
