#include "universe_watch.h"

#include <algorithm>
#include <cmath>

namespace spacecal::trust {

namespace {
    constexpr double k_RAD_TO_DEG = 57.29577951308232;
    constexpr size_t k_PREVIOUS_FRAMES = 3;
    constexpr double k_REBASE_AFTER = 2.0; // s without any device in the current frame before another shared frame takes over
}

void UniverseWatch::reset()
{
    m_haveFrame = false;
    m_frame = Eigen::Isometry3d::Identity();
    m_global = Eigen::Isometry3d::Identity();
    m_previous.clear();
    m_candidate = {};
    for (auto& c : m_correction)
        c = Eigen::Isometry3d::Identity();
    m_shifts = 0;
    m_lastInFrameT = -1.0;
}

double UniverseWatch::angleDeg(const Eigen::Isometry3d& a)
{
    const Eigen::AngleAxisd aa(a.linear());
    return std::abs(aa.angle()) * k_RAD_TO_DEG;
}

bool UniverseWatch::same(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) const
{
    if ((a.translation() - b.translation()).norm() > m_params.same_pos)
        return false;
    return angleDeg(Eigen::Isometry3d(a.linear().transpose() * b.linear())) <= m_params.same_deg;
}

Eigen::Isometry3d UniverseWatch::correction(uint8_t index) const
{
    return index < 64 ? m_correction[index] : m_global;
}

bool UniverseWatch::tick(double t, const std::vector<UniverseDevice>& devices, bool correct, UniverseShift* out)
{
    if (devices.empty())
        return false;

    // groups of devices sharing one transform
    std::vector<int> group(devices.size(), -1);
    std::vector<size_t> groupFirst;
    for (size_t i = 0; i < devices.size(); i++) {
        for (size_t g = 0; g < groupFirst.size() && group[i] < 0; g++)
            if (same(devices[i].wfd, devices[groupFirst[g]].wfd))
                group[i] = static_cast<int>(g);
        if (group[i] < 0) {
            group[i] = static_cast<int>(groupFirst.size());
            groupFirst.push_back(i);
        }
    }
    std::vector<int> groupSize(groupFirst.size(), 0);
    for (int g : group)
        groupSize[g]++;

    if (!m_haveFrame) {
        // the universe's frame is the transform most devices share
        const size_t best = static_cast<size_t>(std::max_element(groupSize.begin(), groupSize.end()) - groupSize.begin());
        m_frame = devices[groupFirst[best]].wfd;
        m_haveFrame = true;
        m_lastInFrameT = t;
    }

    // which group is the current frame, which ones are refinements of it (a re-solve), which are
    // other base stations' frames
    int current = -1;
    int bestShift = -1;
    std::vector<double> groupMove(groupFirst.size(), 0.0);
    for (size_t g = 0; g < groupFirst.size(); g++) {
        const UniverseDevice& d = devices[groupFirst[g]];
        if (same(d.wfd, m_frame)) {
            current = static_cast<int>(g);
            continue;
        }
        const Eigen::Isometry3d delta = d.wfd * m_frame.inverse();
        double move = 0.0;
        int n = 0;
        for (size_t i = 0; i < devices.size(); i++) {
            if (group[i] != static_cast<int>(g))
                continue;
            const Eigen::Vector3d rawOld = m_frame * devices[i].driverPos;
            move += (delta * rawOld - rawOld).norm();
            n++;
        }
        move /= std::max(1, n);
        groupMove[g] = move;
        const bool refinement = move >= m_params.min_shift && move <= m_params.max_shift && angleDeg(delta) <= m_params.max_shift_deg;
        if (refinement && (bestShift < 0 || groupSize[g] > groupSize[bestShift]))
            bestShift = static_cast<int>(g);
    }
    if (current >= 0)
        m_lastInFrameT = t;

    bool shifted = false;
    if (bestShift >= 0) {
        const Eigen::Isometry3d& wfdNew = devices[groupFirst[bestShift]].wfd;
        if (!m_candidate.active || !same(m_candidate.wfd, wfdNew)) {
            m_candidate.active = true;
            m_candidate.wfd = wfdNew;
            m_candidate.since = t;
        }
        if (groupSize[bestShift] >= m_params.min_devices) {
            const Eigen::Isometry3d delta = wfdNew * m_frame.inverse();
            if (out) {
                out->t = t;
                out->delta = delta;
                out->moveAtDevices = groupMove[bestShift];
                out->angleDeg = angleDeg(delta);
                out->devices = groupSize[bestShift];
                out->corrected = correct;
            }
            m_previous.push_back({ m_frame, m_global });
            if (m_previous.size() > k_PREVIOUS_FRAMES)
                m_previous.erase(m_previous.begin());
            if (correct)
                m_global = m_global * delta.inverse(); // the stable frame stays where it was
            m_frame = wfdNew;
            m_candidate.active = false;
            m_lastInFrameT = t;
            m_shifts++;
            shifted = true;
        }
    } else if (current < 0 && t - m_lastInFrameT > k_REBASE_AFTER) {
        // nobody has been in the frame for a while (its base station is gone): the universe now
        // lives in another station's frame; follow it without a correction (raw stays continuous)
        const size_t best = static_cast<size_t>(std::max_element(groupSize.begin(), groupSize.end()) - groupSize.begin());
        if (groupSize[best] >= m_params.min_devices) {
            m_previous.push_back({ m_frame, m_global });
            if (m_previous.size() > k_PREVIOUS_FRAMES)
                m_previous.erase(m_previous.begin());
            m_frame = devices[groupFirst[best]].wfd;
            m_lastInFrameT = t;
        }
    }

    // per-device corrections into the stable frame
    for (const UniverseDevice& d : devices) {
        if (d.index >= 64)
            continue;
        Eigen::Isometry3d c = m_global; // current frame, or another station's frame: raw as SteamVR has it
        if (!same(d.wfd, m_frame)) {
            bool matched = false;
            for (auto it = m_previous.rbegin(); it != m_previous.rend() && !matched; ++it) {
                if (same(d.wfd, it->wfd)) {
                    c = it->global; // its transform lags behind the shift: it is still in the old frame
                    matched = true;
                }
            }
            if (!matched && m_candidate.active && t - m_candidate.since <= m_params.window && same(d.wfd, m_candidate.wfd)) {
                // first device in a new frame: most likely the start of a shift, hold it where it was
                // until the others follow (or the window runs out and it counts as alone)
                c = m_global * (m_candidate.wfd * m_frame.inverse()).inverse();
            }
        }
        m_correction[d.index] = c;
    }
    return shifted;
}

} // namespace spacecal::trust
