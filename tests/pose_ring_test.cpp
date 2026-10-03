// Pose ring test (fork, docs/DESIGN.md section 13): the shared-memory path from the thin driver
// to the overlay's recorder. Runs writer and reader in one process on private ring names.

#include "black_box.h"
#include "pose_record.h"
#include "pose_ring.h"
#include "transform_carry.h"
#include "test_support.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <openvr_driver.h>
#include <set>
#include <thread>
#include <vector>

using namespace spacecal::blackbox;
namespace fs = std::filesystem;

namespace {

std::wstring ringName()
{
    static int n = 0;
#ifdef _WIN32
    const unsigned long pid = GetCurrentProcessId();
#else
    const unsigned long pid = 0;
#endif
    return L"Local\\SC518RingTest_" + std::to_wstring(pid) + L"_" + std::to_wstring(n++);
}

Record numbered(uint64_t i, uint8_t device = 1)
{
    Record r;
    r.t = 1000.0 + static_cast<double>(i) * 0.001;
    r.type = static_cast<uint8_t>(RecordType::POSE);
    r.device = device;
    r.v[0] = static_cast<float>(i % 100000);
    r.v[1] = static_cast<float>(i / 100000);
    return r;
}

uint64_t numberOf(const Record& r)
{
    return static_cast<uint64_t>(r.v[1]) * 100000 + static_cast<uint64_t>(r.v[0]);
}

vr::DriverPose_t pose(double x)
{
    vr::DriverPose_t p = {};
    p.poseIsValid = true;
    p.deviceIsConnected = true;
    p.result = vr::TrackingResult_Running_OK;
    p.qRotation = { 1, 0, 0, 0 };
    p.vecPosition[0] = x;
    return p;
}

void removeTree(const fs::path& dir)
{
    std::error_code ec;
    fs::remove_all(dir, ec);
}

struct Parsed {
    ChunkHeader header {};
    std::vector<Record> records;
};

bool readChunkFile(const fs::path& path, Parsed& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f.read(reinterpret_cast<char*>(&out.header), sizeof(ChunkHeader)))
        return false;
    Record r;
    while (f.read(reinterpret_cast<char*>(&r), sizeof(Record)))
        out.records.push_back(r);
    return true;
}

} // namespace

#ifdef _WIN32

TEST(roundtrip_in_order)
{
    const std::wstring name = ringName();
    PoseRingWriter w;
    CHECK(w.create(name, 1024, 0x1234, 1.7e9, 100.0, "test"));
    PoseRingReader r;
    CHECK(r.attach(name));
    CHECK_EQ(r.sessionId(), 0x1234u);
    CHECK_EQ(r.resume(), 0u);
    for (uint64_t i = 0; i < 1000; i++)
        w.write(numbered(i));
    std::vector<Record> out;
    uint64_t lost = 0;
    CHECK_EQ(r.read(out, 100000, lost), 1000u);
    CHECK_EQ(lost, 0u);
    bool ordered = true;
    for (size_t i = 0; i < out.size(); i++)
        ordered = ordered && numberOf(out[i]) == i;
    CHECK(ordered);
    CHECK_EQ(r.read(out, 100000, lost), 0u); // nothing new
}

TEST(overrun_counts_what_was_lost)
{
    const std::wstring name = ringName();
    PoseRingWriter w;
    CHECK(w.create(name, 1024, 7, 0, 0, "test"));
    PoseRingReader r;
    CHECK(r.attach(name));
    r.resume();
    for (uint64_t i = 0; i < 5000; i++)
        w.write(numbered(i));
    std::vector<Record> out;
    uint64_t lost = 0;
    r.read(out, 100000, lost);
    CHECK_EQ(out.size(), 1024u);
    CHECK_EQ(lost, 5000u - 1024u);
    CHECK(!out.empty() && numberOf(out.front()) == 5000u - 1024u);
}

TEST(a_restarted_reader_resumes_at_the_committed_cursor)
{
    const std::wstring name = ringName();
    PoseRingWriter w;
    CHECK(w.create(name, 4096, 9, 0, 0, "test"));
    std::vector<Record> first;
    {
        PoseRingReader r;
        CHECK(r.attach(name));
        r.resume();
        for (uint64_t i = 0; i < 500; i++)
            w.write(numbered(i));
        uint64_t lost = 0;
        r.read(first, 100000, lost);
        r.commit(); // "on disk"
    } // overlay closes
    for (uint64_t i = 500; i < 800; i++)
        w.write(numbered(i)); // while no overlay runs
    PoseRingReader again;
    CHECK(again.attach(name));
    CHECK_EQ(again.resume(), 0u);
    std::vector<Record> second;
    uint64_t lost = 0;
    again.read(second, 100000, lost);
    CHECK_EQ(first.size(), 500u);
    CHECK_EQ(second.size(), 300u);
    CHECK(!second.empty() && numberOf(second.front()) == 500u);
}

TEST(a_resume_after_too_long_reports_the_gap)
{
    const std::wstring name = ringName();
    PoseRingWriter w;
    CHECK(w.create(name, 1024, 11, 0, 0, "test"));
    {
        PoseRingReader r;
        CHECK(r.attach(name));
        r.resume();
        r.commit(); // cursor 0
    }
    for (uint64_t i = 0; i < 3000; i++)
        w.write(numbered(i));
    PoseRingReader again;
    CHECK(again.attach(name));
    CHECK_EQ(again.resume(), 3000u - 1024u);
}
TEST(a_new_driver_session_is_seen)
{
    const std::wstring name = ringName();
    PoseRingReader r;
    {
        PoseRingWriter w;
        CHECK(w.create(name, 1024, 21, 0, 0, "test"));
        CHECK(r.attach(name));
        CHECK(!r.sessionChanged());
    } // SteamVR stops; the reader still holds the mapping
    PoseRingWriter w2; // SteamVR starts again: same name, the mapping is reused
    CHECK(w2.create(name, 1024, 22, 0, 0, "test"));
    CHECK(r.sessionChanged());
    PoseRingReader fresh;
    CHECK(fresh.attach(name));
    CHECK_EQ(fresh.sessionId(), 22u);
}

TEST(concurrent_writers_never_hand_out_torn_records)
{
    const std::wstring name = ringName();
    PoseRingWriter w;
    CHECK(w.create(name, 1u << 14, 31, 0, 0, "test"));
    PoseRingReader r;
    CHECK(r.attach(name));
    r.resume();
    constexpr int k_THREADS = 4;
    constexpr uint64_t k_PER_THREAD = 50000;
    std::vector<std::thread> writers;
    for (int tIdx = 0; tIdx < k_THREADS; tIdx++) {
        writers.emplace_back([&w, tIdx] {
            for (uint64_t i = 0; i < k_PER_THREAD; i++) {
                Record rec = numbered(i, static_cast<uint8_t>(tIdx));
                rec.f0 = static_cast<float>(i % 1000); // must stay consistent with v[0] within one record
                w.write(rec);
            }
        });
    }
    uint64_t got = 0, lost = 0, torn = 0;
    std::vector<Record> out;
    auto drain = [&] {
        out.clear();
        r.read(out, 1u << 20, lost);
        for (const Record& rec : out) {
            got++;
            if (static_cast<uint64_t>(rec.f0) != numberOf(rec) % 1000 || rec.device >= k_THREADS)
                torn++;
        }
    };
    for (int spin = 0; spin < 2000; spin++) {
        drain();
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    for (auto& t : writers)
        t.join();
    drain();
    CHECK_EQ(torn, 0u);
    CHECK_EQ(got + lost, static_cast<uint64_t>(k_THREADS) * k_PER_THREAD);
    std::printf("  read %llu, lost %llu to overruns\n", (unsigned long long)got, (unsigned long long)lost);
}

TEST(recorder_continues_its_session_across_an_overlay_restart)
{
    const std::wstring name = ringName();
    const fs::path dir = fs::temp_directory_path() / "spacecal_ring_session";
    removeTree(dir);
    PoseRingWriter w;
    CHECK(w.create(name, 1u << 16, 0x5151, unixNow(), monoNow(), "test"));

    uint64_t written = 0;
    auto writeFor = [&](double seconds) {
        const double t0 = monoNow();
        while (monoNow() - t0 < seconds) {
            w.write(makePoseRecord(2, pose(static_cast<double>(written)), monoNow()));
            written++;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    };
    auto runRecorder = [&](double seconds) {
        PoseRingReader r;
        CHECK(r.attach(name));
        CHECK_EQ(r.resume(), 0u);
        BlackBox bb;
        StartOptions o;
        o.flushIntervalSec = 0.05;
        o.sessionId = r.sessionId();
        o.sessionStartUnix = 1.7e9;
        o.pull = [&r](std::vector<Record>& out, bool, double) {
            uint64_t lost = 0;
            r.read(out, 1u << 20, lost);
        };
        o.committed = [&r] { r.commit(); };
        Params p;
        p.chunkSeconds = 1;
        p.liveWindowSeconds = 600;
        CHECK(bb.start(dir, "test", p, o));
        writeFor(seconds);
        bb.stop();
    };

    runRecorder(1.6); // first overlay run
    writeFor(0.5); // overlay closed, the driver keeps publishing
    runRecorder(1.6); // restarted overlay

    // every pose exactly once, one session, chunk indices unique
    std::set<uint32_t> indices;
    std::set<double> times;
    uint64_t poses = 0, duplicates = 0;
    int resumedStarts = 0;
    for (auto& e : fs::directory_iterator(dir / "live")) {
        Parsed c;
        CHECK(readChunkFile(e.path(), c));
        CHECK_EQ(c.header.sessionId, 0x5151u);
        CHECK(indices.insert(c.header.chunkIndex).second);
        for (const Record& rec : c.records) {
            if (rec.type == static_cast<uint8_t>(RecordType::POSE)) {
                poses++;
                if (!times.insert(rec.t).second)
                    duplicates++;
            } else if (rec.type == static_cast<uint8_t>(RecordType::LIFECYCLE) && rec.a == static_cast<uint8_t>(LifecycleKind::RECORDER_START) && rec.f0 == 1.0f) {
                resumedStarts++;
            }
        }
    }
    CHECK_EQ(poses, written);
    CHECK_EQ(duplicates, 0u);
    CHECK_EQ(resumedStarts, 1);
    removeTree(dir);
}

TEST(every_chunk_carries_the_transforms_in_effect_at_its_start)
{
    const std::wstring name = ringName();
    const fs::path dir = fs::temp_directory_path() / "spacecal_ring_carry";
    removeTree(dir);
    PoseRingWriter w;
    CHECK(w.create(name, 1u << 16, 0x5252, unixNow(), monoNow(), "test"));
    const double trans[3] = { 1.0, 2.0, 3.0 };
    const double quat[4] = { 1.0, 0.0, 0.0, 0.0 };
    w.write(makeWorldFromDriverRecord(2, trans, quat, monoNow())); // once, like the driver while nothing changes

    PoseRingReader r;
    CHECK(r.attach(name));
    CHECK_EQ(r.resume(), 0u);
    TransformCarry carry;
    BlackBox bb;
    StartOptions o;
    o.flushIntervalSec = 0.05;
    o.sessionId = r.sessionId();
    o.sessionStartUnix = 1.7e9;
    o.pull = [&r, &carry](std::vector<Record>& out, bool newChunk, double chunkStart) {
        const size_t before = out.size();
        if (newChunk)
            carry.beginChunk(0);
        uint64_t lost = 0;
        r.read(out, 1u << 20, lost);
        size_t k = before;
        for (size_t i = before; i < out.size(); i++)
            if (carry.accept(out[i]))
                out[k++] = out[i];
        out.resize(k);
        carry.endPull(out, before, chunkStart);
    };
    o.committed = [&r] { r.commit(); };
    Params p;
    p.chunkSeconds = 1;
    p.liveWindowSeconds = 600;
    CHECK(bb.start(dir, "test", p, o));
    uint64_t written = 0;
    const double t0 = monoNow();
    while (monoNow() - t0 < 3.3) {
        w.write(makePoseRecord(2, pose(static_cast<double>(written)), monoNow()));
        written++;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    bb.stop();

    // every chunk with poses holds the WorldFromDriver before its first pose; one real change overall
    int chunks = 0, carried = 0, changes = 0;
    for (auto& e : fs::directory_iterator(dir / "live")) {
        Parsed c;
        CHECK(readChunkFile(e.path(), c));
        bool havePose = false, placedFirst = false;
        for (const Record& rec : c.records) {
            if (rec.type == static_cast<uint8_t>(RecordType::WORLD_FROM_DRIVER) && rec.device == 2) {
                placedFirst = placedFirst || !havePose;
                if (rec.b == k_RECORD_CARRIED)
                    carried++;
                else
                    changes++;
                CHECK(std::abs(rec.v[0] - 1.0f) < 1e-6f);
            } else if (rec.type == static_cast<uint8_t>(RecordType::POSE)) {
                havePose = true;
            }
        }
        if (havePose) {
            chunks++;
            CHECK(placedFirst);
        }
    }
    CHECK(chunks >= 3);
    CHECK_EQ(changes, 1);
    CHECK(carried >= chunks - 1);
    removeTree(dir);
}

#else

TEST(pose_ring_is_windows_only)
{
    PoseRingWriter w;
    CHECK(!w.create(L"x", 1024, 1, 0, 0, "test"));
}

#endif

int main()
{
    return spacecal::test::runAll();
}