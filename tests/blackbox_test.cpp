// Black-box recorder test (fork). Runs the recorder with second-long chunks against a temp
// directory, sets a marker, and checks the chunk files, the promoted event folder and the
// record encoding with a reader that mirrors tools/blackbox.py.
//
// Build: cmake -DSPACECAL_BUILD_TESTS=ON .. && cmake --build . --target spacecal_tests
// Run:   ./spacecal_tests  (exit code 0 = all passed)

#include "black_box.h"
#include "marker_notes.h"
#include "pose_record.h"
#include "transform_carry.h"
#include "test_support.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <openvr_driver.h>
#include <thread>
#include <vector>

using namespace spacecal::blackbox;
namespace fs = std::filesystem;

namespace {

struct ParsedChunk {
    ChunkHeader header {};
    std::vector<Record> records;
    bool truncatedTail = false;
};

bool readChunk(const fs::path& path, ParsedChunk& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.read(reinterpret_cast<char*>(&out.header), sizeof(ChunkHeader));
    if (!f || out.header.magic != k_MAGIC || out.header.recordSize != k_RECORD_SIZE || out.header.headerSize != k_HEADER_SIZE)
        return false;
    f.seekg(out.header.headerSize, std::ios::beg);
    Record r;
    while (f.read(reinterpret_cast<char*>(&r), sizeof(Record))) {
        out.records.push_back(r);
    }
    out.truncatedTail = f.gcount() != 0;
    return true;
}

vr::DriverPose_t makePose(double x, double y, double z)
{
    vr::DriverPose_t p = {};
    p.poseIsValid = true;
    p.deviceIsConnected = true;
    p.result = vr::TrackingResult_Running_OK;
    p.qWorldFromDriverRotation = { 1, 0, 0, 0 };
    p.qDriverFromHeadRotation = { 1, 0, 0, 0 };
    p.qRotation = { 1, 0, 0, 0 };
    p.vecPosition[0] = x;
    p.vecPosition[1] = y;
    p.vecPosition[2] = z;
    p.vecVelocity[0] = 0.5;
    p.vecAngularVelocity[2] = 0.25;
    p.poseTimeOffset = -0.004;
    return p;
}

std::string reassembleText(const std::vector<Record>& records, RecordType type, uint8_t kind)
{
    std::string out;
    for (const Record& r : records) {
        if (r.type != static_cast<uint8_t>(type) || r.a != kind)
            continue;
        const char* p = r.text();
        size_t n = 0;
        while (n < k_TEXT_PAYLOAD_SIZE && p[n] != '\0')
            n++;
        out.append(p, n);
    }
    return out;
}


// file helpers that are safe on Windows: the stream is closed before the tree is removed, and
// a directory that still has an open handle (sharing violation) reports instead of throwing
std::string readFile(const fs::path& path)
{
    std::ifstream f(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

void removeTree(const fs::path& dir)
{
    std::error_code ec;
    fs::remove_all(dir, ec);
    if (ec)
        std::printf("  (could not remove %s: %s)\n", dir.string().c_str(), ec.message().c_str());
}

} // namespace

TEST(format_sizes)
{
    CHECK_EQ(sizeof(Record), 80u);
    CHECK_EQ(sizeof(ChunkHeader), 64u);
    Record r;
    CHECK_EQ(reinterpret_cast<char*>(&r.f0) - reinterpret_cast<char*>(&r), 16);
}

TEST(text_record_split)
{
    // 100 chars -> 2 records of 64 + 36
    std::string text(100, 'x');
    for (size_t i = 0; i < text.size(); i++)
        text[i] = static_cast<char>('a' + (i % 26));

    fs::path dir = fs::temp_directory_path() / "spacecal_bb_text";
    removeTree(dir);
    BlackBox bb;
    Params p;
    p.chunkSeconds = 5;
    p.liveWindowSeconds = 10;
    CHECK(bb.start(dir, "test", p, 0.02));
    bb.recordText(RecordType::DEVICE, 3, TextKind::DEVICE_INFO, text, monoNow());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bb.stop();

    std::vector<fs::path> chunks;
    for (auto& e : fs::directory_iterator(dir / "live"))
        if (e.path().extension() == k_CHUNK_EXTENSION)
            chunks.push_back(e.path());
    CHECK_EQ(chunks.size(), 1u);
    ParsedChunk c;
    CHECK(readChunk(chunks[0], c));
    int deviceRecords = 0;
    for (const Record& r : c.records)
        if (r.type == static_cast<uint8_t>(RecordType::DEVICE)) {
            deviceRecords++;
            CHECK_EQ(r.device, 3);
            CHECK_EQ(r.code, 2);
        }
    CHECK_EQ(deviceRecords, 2);
    CHECK_EQ(reassembleText(c.records, RecordType::DEVICE, static_cast<uint8_t>(TextKind::DEVICE_INFO)), text);
    removeTree(dir);
}

TEST(rolling_chunks_marker_and_event)
{
    fs::path dir = fs::temp_directory_path() / "spacecal_bb_event";
    removeTree(dir);

    BlackBox bb;
    Params p;
    p.chunkSeconds = 1;
    p.liveWindowSeconds = 3;
    p.preSeconds = 2;
    p.postSeconds = 2;
    p.maxEventsPerSession = 5;
    CHECK(bb.start(dir, "v-test", p, 0.05));
    CHECK(bb.isRunning());

    // 2.5 s of poses for 3 devices at ~200 Hz
    const double t0 = monoNow();
    double lastT = 0.0;
    int pushed = 0;
    while (monoNow() - t0 < 2.5) {
        const double t = monoNow();
        for (uint8_t d = 0; d < 3; d++) {
            bb.push(makePoseRecord(d, makePose(d, t - t0, 0.0), t));
            pushed++;
        }
        lastT = t;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // marker in the middle of the run
    const std::string folder = "2026-01-01_00-00-00_hotkey";
    CHECK(bb.markEvent(MarkerSource::HOTKEY, 7, folder));
    CHECK(!bb.markEvent(MarkerSource::HOTKEY, 7, "../evil")); // unsafe names are refused
    const double tMark = monoNow();
    // one calibration attempt inside the event window (also lands in the Python fixture)
    const float calib[13] = { 0.02f, 100.0f, 0.01f, 0.5f, 1.0f, 2.0f, 3.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.9f, 0.8f };
    bb.recordCalibration(3, 0, static_cast<uint8_t>(CalibrationTrigger::CONTINUOUS),
        static_cast<uint8_t>(CalibrationOutcome::APPLIED) | k_CALIB_FLAG_CONTINUOUS, 0, 0.015f, calib, tMark);

    // keep recording through the post window plus a chunk so the event can close
    while (monoNow() - tMark < 3.5) {
        const double t = monoNow();
        bb.push(makePoseRecord(0, makePose(0, t - t0, 0.0), t));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // wait for the event to be finalised by the writer
    const double waitStart = monoNow();
    while (bb.getStats().pendingEvents > 0 && monoNow() - waitStart < 5.0)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    Stats s = bb.getStats();
    CHECK_EQ(s.pendingEvents, 0u);
    CHECK_EQ(s.eventsSaved, 1u);
    CHECK_EQ(s.recordsDropped, 0u);
    CHECK(s.recordsWritten >= static_cast<uint64_t>(pushed));
    bb.stop();
    CHECK(!bb.isRunning());

    // live window: at most ceil(3 s / 1 s) closed chunks + the open one + slack
    size_t liveChunks = 0;
    for (auto& e : fs::directory_iterator(dir / "live"))
        if (e.path().extension() == k_CHUNK_EXTENSION)
            liveChunks++;
    CHECK(liveChunks >= 3);
    CHECK(liveChunks <= 6);

    // event folder
    const fs::path evDir = dir / "events" / folder;
    CHECK(fs::exists(evDir / "driver.json"));
    std::vector<fs::path> evChunks;
    for (auto& e : fs::directory_iterator(evDir))
        if (e.path().extension() == k_CHUNK_EXTENSION)
            evChunks.push_back(e.path());
    // 2 s before + 2 s after with 1 s chunks -> 4 to 6 files
    CHECK(evChunks.size() >= 4);
    CHECK(evChunks.size() <= 7);
    std::sort(evChunks.begin(), evChunks.end()); // chunk names sort chronologically

    // every event chunk parses; marker + folder text appear exactly once; times are monotonic
    int markers = 0;
    int calibrations = 0;
    std::string markerFolder;
    double prevT = 0.0;
    uint64_t poseRecords = 0;
    for (const fs::path& path : evChunks) {
        ParsedChunk c;
        CHECK(readChunk(path, c));
        CHECK_EQ(c.header.formatVersion, k_FORMAT_VERSION);
        CHECK_EQ(std::string(c.header.version), std::string("v-test"));
        CHECK(!c.records.empty());
        CHECK_EQ(c.records.front().type, static_cast<uint8_t>(RecordType::SESSION));
        for (const Record& r : c.records) {
            if (r.type == static_cast<uint8_t>(RecordType::POSE)) {
                poseRecords++;
                CHECK(r.t >= prevT - 1e-9 || r.device != 0);
                if (r.device == 0)
                    prevT = r.t;
                CHECK_EQ(r.a, static_cast<uint8_t>(vr::TrackingResult_Running_OK));
                CHECK_EQ(r.b & k_POSE_FLAG_VALID, k_POSE_FLAG_VALID);
                CHECK(std::abs(r.f0 - (-0.004f)) < 1e-6f);
                CHECK(std::abs(r.v[7] - 0.5f) < 1e-6f); // velocity x
                CHECK(std::abs(r.v[12] - 0.25f) < 1e-6f); // angular velocity z
            } else if (r.type == static_cast<uint8_t>(RecordType::MARKER)) {
                markers++;
                CHECK_EQ(r.a, static_cast<uint8_t>(MarkerSource::HOTKEY));
                CHECK_EQ(r.code, 7);
            } else if (r.type == static_cast<uint8_t>(RecordType::CALIBRATION)) {
                calibrations++;
                CHECK_EQ(r.device, 3);
                CHECK_EQ(r.reserved, 0);
                CHECK_EQ(r.a, static_cast<uint8_t>(CalibrationTrigger::CONTINUOUS));
                CHECK_EQ(r.b & k_CALIB_OUTCOME_MASK, static_cast<uint8_t>(CalibrationOutcome::APPLIED));
                CHECK((r.b & k_CALIB_FLAG_CONTINUOUS) != 0);
                CHECK(std::abs(r.f0 - 0.015f) < 1e-6f);
                CHECK(std::abs(r.v[3] - 0.5f) < 1e-6f);
            }
        }
        markerFolder += reassembleText(c.records, RecordType::TEXT, static_cast<uint8_t>(TextKind::MARKER_FOLDER));
    }
    CHECK_EQ(markers, 1);
    CHECK_EQ(calibrations, 1);
    CHECK_EQ(markerFolder, folder);
    CHECK(poseRecords > 500);

    // driver.json mentions the chunk files and the marker source
    const std::string json = readFile(evDir / "driver.json");
    CHECK(json.find("\"source\": \"hotkey\"") != std::string::npos);
    CHECK(json.find("\"incomplete\": false") != std::string::npos);
    CHECK(json.find(evChunks[0].filename().string()) != std::string::npos);

    // optionally keep the event folder as a fixture for the Python reader tests
    if (const char* fixtureOut = std::getenv("SPACECAL_TEST_FIXTURE_OUT")) {
        std::error_code ec;
        fs::remove_all(fixtureOut, ec);
        fs::create_directories(fixtureOut, ec);
        fs::copy(evDir, fixtureOut, fs::copy_options::recursive | fs::copy_options::overwrite_existing, ec);
        std::printf("  fixture written to %s (%s)\n", fixtureOut, ec ? ec.message().c_str() : "ok");
    }

    removeTree(dir);
}

TEST(shutdown_promotes_open_chunk)
{
    fs::path dir = fs::temp_directory_path() / "spacecal_bb_shutdown";
    removeTree(dir);
    BlackBox bb;
    Params p;
    p.chunkSeconds = 30;
    p.liveWindowSeconds = 60;
    p.preSeconds = 5;
    p.postSeconds = 60;
    CHECK(bb.start(dir, "test", p, 0.05));
    for (int i = 0; i < 50; i++) {
        bb.push(makePoseRecord(1, makePose(i, 0, 0), monoNow()));
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    CHECK(bb.markEvent(MarkerSource::OVERLAY_BUTTON, 0, "shutdown_evt"));
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    bb.stop(); // long before the post window ends

    const fs::path evDir = dir / "events" / "shutdown_evt";
    CHECK(fs::exists(evDir / "driver.json"));
    const std::string json = readFile(evDir / "driver.json");
    CHECK(json.find("\"incomplete\": true") != std::string::npos);
    size_t chunks = 0;
    for (auto& e : fs::directory_iterator(evDir))
        if (e.path().extension() == k_CHUNK_EXTENSION)
            chunks++;
    CHECK_EQ(chunks, 1u);
    removeTree(dir);
}

TEST(disabled_recorder_drops_nothing_and_refuses_markers)
{
    fs::path dir = fs::temp_directory_path() / "spacecal_bb_disabled";
    removeTree(dir);
    BlackBox bb;
    Params p;
    p.enabled = false;
    CHECK(bb.start(dir, "test", p, 0.02));
    for (int i = 0; i < 100; i++)
        bb.push(makePoseRecord(1, makePose(i, 0, 0), monoNow()));
    CHECK(!bb.markEvent(MarkerSource::HOTKEY, 0, "nope"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    Stats s = bb.getStats();
    CHECK(!s.enabled);
    CHECK_EQ(s.recordsDropped, 0u);
    bb.stop();
    CHECK(!fs::exists(dir / "events" / "nope"));
    removeTree(dir);
}

namespace {

std::vector<fs::path> chunkFiles(const fs::path& dir)
{
    std::vector<fs::path> out;
    std::error_code ec;
    if (!fs::exists(dir, ec))
        return out;
    for (auto& e : fs::directory_iterator(dir, ec))
        if (e.path().extension() == k_CHUNK_EXTENSION)
            out.push_back(e.path());
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<fs::path> subdirs(const fs::path& dir)
{
    std::vector<fs::path> out;
    std::error_code ec;
    if (!fs::exists(dir, ec))
        return out;
    for (auto& e : fs::directory_iterator(dir, ec))
        if (e.is_directory(ec))
            out.push_back(e.path());
    return out;
}

bool isNtfsCompressed(const fs::path& p)
{
#ifdef _WIN32
    const DWORD attrs = GetFileAttributesW(p.c_str());
    return attrs != INVALID_FILE_ATTRIBUTES && (attrs & FILE_ATTRIBUTE_COMPRESSED) != 0;
#else
    (void)p;
    return false;
#endif
}

} // namespace

TEST(keep_all_archives_every_chunk)
{
    fs::path dir = fs::temp_directory_path() / "spacecal_bb_keepall";
    removeTree(dir);
    BlackBox bb;
    Params p;
    p.chunkSeconds = 1;
    p.liveWindowSeconds = 2;
    p.keepAll = true;
    CHECK(bb.start(dir, "test", p, 0.05));
    const double t0 = monoNow();
    uint64_t pushed = 0;
    while (monoNow() - t0 < 5.5) {
        bb.push(makePoseRecord(0, makePose(monoNow() - t0, 0, 0), monoNow()));
        pushed++;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const Stats mid = bb.getStats();
    CHECK(mid.keepAll);
    CHECK(mid.archivedChunks >= 2u);
    CHECK(mid.archiveBytes > 0u);
    bb.stop();
    // the rest stays in live/ for the next recorder of this session; a recorder of a later
    // session (SteamVR restarted) archives it into the same session folder
    CHECK(chunkFiles(dir / "live").size() >= 1u);
    {
        BlackBox later;
        Params lp;
        lp.keepAll = true;
        CHECK(later.start(dir, "test", lp, 0.05));
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        later.stop();
        CHECK_EQ(chunkFiles(dir / "live").size(), 1u); // only the later session's own chunk
    }
    const auto sessions = subdirs(dir / "sessions");
    CHECK_EQ(sessions.size(), 1u);
    if (sessions.size() != 1)
        return;
    const auto files = chunkFiles(sessions[0]);
    CHECK(files.size() >= 5u);
    uint64_t poses = 0;
    int starts = 0, stops = 0;
    for (const fs::path& f : files) {
        ParsedChunk c;
        CHECK(readChunk(f, c));
        for (const Record& r : c.records) {
            if (r.type == static_cast<uint8_t>(RecordType::POSE)) {
                poses++;
                CHECK_EQ(r.code, static_cast<uint16_t>(vr::TrackingResult_Running_OK));
            } else if (r.type == static_cast<uint8_t>(RecordType::LIFECYCLE)) {
                starts += r.a == static_cast<uint8_t>(LifecycleKind::RECORDER_START);
                stops += r.a == static_cast<uint8_t>(LifecycleKind::RECORDER_STOP);
            }
        }
    }
    CHECK_EQ(poses, pushed); // nothing lost between live and archive
    CHECK_EQ(starts, 1);
    CHECK_EQ(stops, 1);
#ifdef _WIN32
    // chunks archived while running are NTFS-compressed by the writer
    size_t compressed = 0;
    for (const fs::path& f : files)
        compressed += isNtfsCompressed(f) ? 1 : 0;
    CHECK(compressed >= 1u);
#endif
    removeTree(dir);
}

TEST(vanished_live_chunk_never_deletes_the_archived_copy)
{
    // 2026-09-30: two recorders ran on one session. One archived a chunk; the other then failed to
    // find it in live/ and "cleaned up" by deleting the archive's copy (6 minutes lost). A chunk
    // that vanished from live/ must be dropped from the list, and the archive left alone.
    fs::path dir = fs::temp_directory_path() / "spacecal_bb_vanished";
    removeTree(dir);
    BlackBox bb;
    Params p;
    p.chunkSeconds = 1;
    p.liveWindowSeconds = 2;
    p.keepAll = true;
    CHECK(bb.start(dir, "test", p, 0.05));
    const double t0 = monoNow();
    fs::path planted;
    uintmax_t plantedSize = 0;
    while (monoNow() - t0 < 7.0) {
        bb.push(makePoseRecord(0, makePose(monoNow() - t0, 0, 0), monoNow()));
        if (planted.empty() && bb.getStats().archivedChunks >= 1u) {
            const auto sessions = subdirs(dir / "sessions");
            auto live = chunkFiles(dir / "live");
            std::sort(live.begin(), live.end());
            if (sessions.size() == 1 && live.size() >= 3) {
                // the second-oldest closed chunk: "another recorder" archives it before this one does
                const fs::path victim = live[1];
                planted = sessions[0] / victim.filename();
                std::error_code ec;
                fs::copy_file(victim, planted, fs::copy_options::overwrite_existing, ec);
                plantedSize = fs::file_size(planted, ec);
                fs::remove(victim, ec);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    bb.stop();
    CHECK(!planted.empty());
    std::error_code ec;
    CHECK(fs::exists(planted, ec));
    CHECK(plantedSize > 0u);
    CHECK_EQ(fs::file_size(planted, ec), plantedSize);
    removeTree(dir);
}

TEST(nothing_leaves_live_before_the_overlay_sends_settings)
{
    fs::path dir = fs::temp_directory_path() / "spacecal_bb_policy";
    removeTree(dir);
    BlackBox bb;
    Params p;
    p.chunkSeconds = 1;
    p.liveWindowSeconds = 2;
    CHECK(bb.start(dir, "test", p, 0.05, /* waitForPolicy */ true));
    const double t0 = monoNow();
    while (monoNow() - t0 < 4.5) {
        bb.push(makePoseRecord(0, makePose(0, 0, 0), monoNow()));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    // with the settings final, chunks older than 2 s would be gone by now
    CHECK(chunkFiles(dir / "live").size() >= 4u);
    CHECK_EQ(bb.getStats().archivedChunks, 0u);
    bb.stop();
    removeTree(dir);
}

TEST(leftovers_of_a_crashed_session)
{
    for (const bool keepAll : { true, false }) {
        fs::path dir = fs::temp_directory_path() / (keepAll ? "spacecal_bb_leftover_keep" : "spacecal_bb_leftover_drop");
        removeTree(dir);
        // session A stops without keepAll, so its chunk stays in live/ like after a crash
        uint32_t sidA = 0;
        {
            BlackBox a;
            Params p;
            p.chunkSeconds = 60;
            p.liveWindowSeconds = 600;
            CHECK(a.start(dir, "test", p, 0.05));
            for (int i = 0; i < 20; i++)
                a.push(makePoseRecord(1, makePose(i, 0, 0), monoNow()));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            a.stop();
            const auto files = chunkFiles(dir / "live");
            CHECK_EQ(files.size(), 1u);
            ParsedChunk c;
            if (!files.empty() && readChunk(files[0], c))
                sidA = c.header.sessionId;
        }
        CHECK(sidA != 0u);

        // session B finds it, waits for the settings, then archives or deletes it
        BlackBox b;
        CHECK(b.start(dir, "test", Params {}, 0.05, /* waitForPolicy */ true));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        CHECK_EQ(chunkFiles(dir / "live").size(), 2u); // A's chunk untouched so far, plus B's own
        Params settings;
        settings.keepAll = keepAll;
        b.setParams(settings);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        CHECK_EQ(chunkFiles(dir / "live").size(), 1u); // only B's
        char suffix[16];
        std::snprintf(suffix, sizeof(suffix), "_%08x", sidA);
        const auto sessions = subdirs(dir / "sessions");
        if (keepAll) {
            CHECK_EQ(sessions.size(), 1u);
            if (sessions.size() == 1) {
                const std::string name = sessions[0].filename().string();
                CHECK(name.size() > 9 && name.substr(name.size() - 9) == suffix);
                CHECK_EQ(chunkFiles(sessions[0]).size(), 1u);
            }
        } else {
            CHECK_EQ(sessions.size(), 0u);
        }
        b.stop();
        removeTree(dir);
    }
}

TEST(transform_carry_repeats_the_transforms_in_effect_at_a_chunk_start)
{
    TransformCarry carry;
    const double q[4] = { 1.0, 0.0, 0.0, 0.0 };
    const double a[3] = { 1.0, 2.0, 3.0 };
    const double b[3] = { 1.0, 2.0, 3.1 };
    auto pull = [&carry](std::vector<Record> pulled, bool newChunk, double chunkStart, uint64_t exclude) {
        std::vector<Record> out;
        if (newChunk)
            carry.beginChunk(exclude);
        for (const Record& r : pulled)
            if (carry.accept(r))
                out.push_back(r);
        carry.endPull(out, 0, chunkStart);
        return out;
    };
    Record pose;
    pose.type = static_cast<uint8_t>(RecordType::POSE);
    pose.device = 4;

    // first chunk: nothing to carry yet, and an unchanged WorldFromDriver is dropped
    std::vector<Record> out = pull({ makeWorldFromDriverRecord(4, a, q, 1.0), makeWorldFromDriverRecord(4, a, q, 1.5),
                                       makeAppliedRecord(4, a, q, 0, 1.0, 1.2) },
        true, 0.9, 0);
    CHECK_EQ(out.size(), 2u);

    // second chunk: a pose, then the WorldFromDriver changes. The one in effect at the start is carried
    // (ahead of the chunk start and of the pose), the change stays a change.
    pose.t = 60.5;
    out = pull({ pose, makeWorldFromDriverRecord(4, b, q, 61.0) }, true, 60.0, 0);
    CHECK_EQ(out.size(), 4u);
    int carriedWfd = 0, carriedApplied = 0, changes = 0;
    for (const Record& r : out) {
        if (r.type == static_cast<uint8_t>(RecordType::WORLD_FROM_DRIVER) && r.b == k_RECORD_CARRIED) {
            carriedWfd++;
            CHECK(r.t < 60.0);
            CHECK(std::abs(r.v[2] - 3.0f) < 1e-6f);
        } else if (r.type == static_cast<uint8_t>(RecordType::APPLIED) && r.b == k_RECORD_CARRIED) {
            carriedApplied++;
            CHECK(r.t < 60.0);
        } else if (r.type == static_cast<uint8_t>(RecordType::WORLD_FROM_DRIVER)) {
            changes++;
            CHECK(std::abs(r.v[2] - 3.1f) < 1e-6f);
        }
    }
    CHECK_EQ(carriedWfd, 1);
    CHECK_EQ(carriedApplied, 1);
    CHECK_EQ(changes, 1);

    // later pulls into the same chunk carry nothing; the next chunk carries the changed value
    pose.t = 70.0;
    CHECK_EQ(pull({ pose }, false, 60.0, 0).size(), 1u);
    pose.t = 120.2;
    out = pull({ pose }, true, 120.0, 0);
    CHECK_EQ(out.size(), 3u);
    for (const Record& r : out)
        if (r.type == static_cast<uint8_t>(RecordType::WORLD_FROM_DRIVER))
            CHECK(std::abs(r.v[2] - 3.1f) < 1e-6f);

    // a device left out of the recording is not carried either
    CHECK_EQ(pull({}, true, 180.0, 1ull << 4).size(), 0u);
    // a new SteamVR session forgets everything
    carry.reset();
    CHECK_EQ(pull({}, true, 240.0, 0).size(), 0u);
}

TEST(marker_notes)
{
    namespace notes = spacecal::guard::notes;
    // one line, trimmed, control characters gone
    CHECK_EQ(notes::sanitize("  hip   jumped\r\n20 cm\twhen sitting  "), std::string("hip jumped 20 cm when sitting"));
    CHECK_EQ(notes::sanitize(std::string("a\x01" "b")), std::string("ab"));
    CHECK_EQ(notes::sanitize("   "), std::string());
    // cut at the limit without splitting a UTF-8 character (ü is 2 bytes)
    std::string longText(notes::k_MAX_NOTE_BYTES - 1, 'x');
    longText += "\xC3\xBC" "yyy";
    const std::string cut = notes::sanitize(longText);
    CHECK_EQ(cut.size(), notes::k_MAX_NOTE_BYTES - 1);
    CHECK_EQ(cut.back(), 'x');

    // notes.txt skips markers without a note
    std::vector<notes::Entry> entries = { { 1790000000.25, "01:50:35", "triggers", "left foot jumped \"30 cm\"" }, { 1790000010.0, "01:50:45", "hotkey", "" } };
    const std::string file = notes::notesFile("2026-10-01_01-50-35_triggers", entries);
    CHECK(file.find("01:50:35  triggers  left foot jumped \"30 cm\"\n") != std::string::npos);
    CHECK(file.find("01:50:45") == std::string::npos);

    // overlay.json: inserted after "note" in an older file, then replaced in place
    std::string json = "{\r\n  \"source\": \"hotkey\",\r\n  \"note\": \"\",\r\n  \"devices\": [\r\n  ]\r\n}\r\n";
    CHECK(notes::replaceOverlayJsonLine(json, notes::overlayJsonLine(entries)));
    CHECK(json.find("  \"note\": \"\",\r\n  \"user_notes\": [{ \"t_mark_unix\": 1790000000.250, \"time\": \"01:50:35\", \"source\": \"triggers\", \"note\": \"left foot jumped \\\"30 cm\\\"\" }],\r\n  \"devices\"") != std::string::npos);
    entries[0].note = "changed";
    CHECK(notes::replaceOverlayJsonLine(json, notes::overlayJsonLine(entries)));
    CHECK(json.find("\"note\": \"changed\" }],\r\n  \"devices\"") != std::string::npos);
    CHECK(json.find("30 cm") == std::string::npos);
    std::string noNote = "{\n  \"x\": 1\n}\n";
    CHECK(!notes::replaceOverlayJsonLine(noNote, notes::overlayJsonLine(entries)));

    // the record payload fits one TEXT message (8 x 64 bytes) even at the longest note
    const std::string payload = notes::recordPayload(12345.678901, std::string(notes::k_MAX_NOTE_BYTES, 'n'));
    CHECK(payload.rfind("12345.678901 ", 0) == 0);
    CHECK(payload.size() <= 8 * k_TEXT_PAYLOAD_SIZE);
}

TEST(connection_and_lifecycle_records)
{
    fs::path dir = fs::temp_directory_path() / "spacecal_bb_state";
    removeTree(dir);
    BlackBox bb;
    CHECK(bb.start(dir, "test", Params {}, 0.02));
    const double t = monoNow();
    // the driver writes these into the pose ring; here they go straight into the recorder
    bb.push(makeDeviceStateRecord(4, true, 255, 200, t));
    bb.push(makeDeviceStateRecord(4, false, 1, 1, t));
    const float v[2] = { 1.0f, 2.0f };
    bb.recordLifecycle(LifecycleKind::OVERLAY_CONNECTED, 7, 0.0f, v, 2, t);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bb.stop();
    const auto files = chunkFiles(dir / "live");
    CHECK_EQ(files.size(), 1u);
    ParsedChunk c;
    CHECK(!files.empty() && readChunk(files[0], c));
    std::vector<Record> states;
    int overlay = 0;
    for (const Record& r : c.records) {
        if (r.type == static_cast<uint8_t>(RecordType::DEVICE_STATE))
            states.push_back(r);
        if (r.type == static_cast<uint8_t>(RecordType::LIFECYCLE) && r.a == static_cast<uint8_t>(LifecycleKind::OVERLAY_CONNECTED)) {
            overlay++;
            CHECK_EQ(r.code, 7);
            CHECK(std::abs(r.v[1] - 2.0f) < 1e-6f);
        }
    }
    CHECK_EQ(states.size(), 2u);
    if (states.size() == 2) {
        CHECK_EQ(states[0].device, 4);
        CHECK_EQ(states[0].a, 1);
        CHECK_EQ(states[0].b, 255);
        CHECK_EQ(states[0].code, 200);
        CHECK_EQ(states[1].a, 0);
        CHECK_EQ(states[1].b, 1);
    }
    CHECK_EQ(overlay, 1);
    removeTree(dir);
}

int main()
{
    return spacecal::test::runAll();
}
