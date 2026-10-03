#include "live_stats.h"

#include <algorithm>
#include <cmath>

namespace spacecal::guard {

LiveStats* LiveStats::s_instance = nullptr;

namespace {
    template <typename T>
    void trim(std::deque<T>& d, double oldest)
    {
        while (!d.empty() && d.front().t < oldest)
            d.pop_front();
    }
}

void LiveStats::init()
{
    s_instance = this;
    clear();
}

void LiveStats::clear()
{
    m_now = -1.0;
    m_head.clear();
    for (auto& d : m_devices)
        d.clear();
    m_deviceInfo = {};
    m_calibration.clear();
    m_solves.clear();
    m_events.clear();
    m_shiftsTotal = 0;
}

void LiveStats::advance(double t)
{
    if (t <= m_now)
        return;
    m_now = t;
    const double oldest = t - k_KEEP_SECONDS;
    trim(m_head, oldest);
    trim(m_calibration, oldest);
    trim(m_solves, oldest);
    trim(m_events, oldest);
}

void LiveStats::pushHead(const LiveHeadSample& s)
{
    advance(s.t);
    m_head.push_back(s);
}

void LiveStats::pushDevice(uint8_t index, const LiveDeviceInfo& info, const LiveDeviceSample& s)
{
    if (index >= 64)
        return;
    advance(s.t);
    m_deviceInfo[index] = info;
    auto& d = m_devices[index];
    d.push_back(s);
    trim(d, m_now - k_KEEP_SECONDS);
}

void LiveStats::pushCalibration(const LiveCalibrationSample& s)
{
    advance(s.t);
    if (!m_calibration.empty()) {
        const LiveCalibrationSample& last = m_calibration.back();
        const bool changed = last.xCm != s.xCm || last.yCm != s.yCm || last.zCm != s.zCm || last.yawDeg != s.yawDeg;
        // a change is kept at once (the line steps where it happened), an unchanged value 10 times a second
        if (!changed && s.t - last.t < 1.0 / k_CAL_RATE_HZ)
            return;
    }
    m_calibration.push_back(s);
}

void LiveStats::pushSolve(LiveSolve s)
{
    if (s.t <= 0.0)
        s.t = m_now;
    advance(s.t);
    m_solves.push_back(std::move(s));
}

void LiveStats::pushEvent(LiveEvent e)
{
    if (e.t <= 0.0)
        e.t = m_now;
    advance(e.t);
    if (e.kind == LiveEventKind::PLAYSPACE_SHIFT)
        m_shiftsTotal++;
    m_events.push_back(std::move(e));
}

Eigen::Vector3d headFrame(const Eigen::Matrix3d& hmdRotation, const Eigen::Vector3d& errorWorld)
{
    const Eigen::Vector3d up(0.0, 1.0, 0.0);
    const Eigen::Vector3d face = hmdRotation * Eigen::Vector3d(0.0, 0.0, -1.0);
    Eigen::Vector3d facing(face.x(), 0.0, face.z());
    if (facing.norm() < 0.2) {
        // looking straight down: the top of the head points the way the face would; looking up: the opposite way
        const Eigen::Vector3d top = hmdRotation * Eigen::Vector3d(0.0, 1.0, 0.0);
        facing = Eigen::Vector3d(top.x(), 0.0, top.z());
        if (face.y() > 0.0)
            facing = -facing;
    }
    if (facing.norm() < 1e-9)
        facing = Eigen::Vector3d(0.0, 0.0, -1.0);
    facing.normalize();
    const Eigen::Vector3d right = facing.cross(up); // OpenVR is right-handed: forward x up = right
    return Eigen::Vector3d(errorWorld.dot(facing), errorWorld.dot(right), errorWorld.dot(up));
}

double yawDegrees(const Eigen::Quaterniond& q)
{
    const Eigen::Vector3d f = q.normalized() * Eigen::Vector3d(0.0, 0.0, -1.0);
    return std::atan2(-f.x(), -f.z()) * 180.0 / 3.14159265358979323846;
}

void decimateMinMax(const std::vector<double>& xs, const std::vector<double>& ys, size_t maxPoints, std::vector<double>& outX, std::vector<double>& outY)
{
    outX.clear();
    outY.clear();
    const size_t n = std::min(xs.size(), ys.size());
    if (n <= maxPoints || maxPoints < 4) {
        outX.assign(xs.begin(), xs.begin() + static_cast<std::ptrdiff_t>(n));
        outY.assign(ys.begin(), ys.begin() + static_cast<std::ptrdiff_t>(n));
        return;
    }
    const size_t buckets = maxPoints / 2;
    outX.reserve(maxPoints + buckets);
    outY.reserve(maxPoints + buckets);
    for (size_t b = 0; b < buckets; b++) {
        const size_t from = b * n / buckets;
        const size_t to = (b + 1) * n / buckets;
        size_t lo = to, hi = to;
        bool gap = false;
        for (size_t i = from; i < to; i++) {
            if (std::isnan(ys[i])) {
                gap = true;
                continue;
            }
            if (lo == to || ys[i] < ys[lo])
                lo = i;
            if (hi == to || ys[i] > ys[hi])
                hi = i;
        }
        if (lo != to) {
            const size_t first = std::min(lo, hi), second = std::max(lo, hi);
            outX.push_back(xs[first]);
            outY.push_back(ys[first]);
            if (second != first) {
                outX.push_back(xs[second]);
                outY.push_back(ys[second]);
            }
        }
        if (gap) {
            outX.push_back(xs[to - 1]);
            outY.push_back(std::numeric_limits<double>::quiet_NaN());
        }
    }
}

} // namespace spacecal::guard
