#include "calibration_record.h"

#include "blackbox_format.h"
#include "calibration.h"
#include "live_stats.h"
#include "recorder/recorder_host.h"

#include <cmath>
#include <limits>

namespace spacecal::guard {

namespace {
    constexpr double k_RAD_TO_DEG = 57.29577951308232;

    uint8_t deviceByte(vr::TrackedDeviceIndex_t id)
    {
        return id < 255 ? static_cast<uint8_t>(id) : 255;
    }
}

CalibrationAttempt::CalibrationAttempt(TrackingSystemCalibration& calibration, bool forced, uint8_t trigger)
    : m_calibration(calibration)
    , m_forced(forced)
    , m_trigger(trigger)
    , m_prevRotation(calibration.calibratedRotation)
    , m_prevTranslation(calibration.calibratedTranslation)
{
    if (m_trigger == 0) {
        using T = blackbox::CalibrationTrigger;
        if (!calibration.isContinuousCalibration()) {
            m_trigger = static_cast<uint8_t>(T::STANDARD);
        } else if (forced || calibration.forkForceTrigger == static_cast<uint8_t>(T::STARTUP)) {
            // the pending trigger names this solve, also when the startup force was waived
            m_trigger = calibration.forkForceTrigger != 0 ? calibration.forkForceTrigger : static_cast<uint8_t>(T::CONTINUOUS);
            m_consumesTrigger = true;
        } else {
            m_trigger = static_cast<uint8_t>(T::CONTINUOUS);
        }
    }
}

void CalibrationAttempt::setPrevious(double rms, double axisVariance)
{
    m_prevRms = std::isfinite(rms) ? rms : -1.0;
    m_prevAxisVariance = axisVariance;
}

void CalibrationAttempt::skipped(size_t samples)
{
    send(static_cast<uint8_t>(blackbox::CalibrationOutcome::SKIPPED), static_cast<uint8_t>(CalibrationError::DeviceUntrusted), std::numeric_limits<double>::quiet_NaN(), 0.0, m_prevRotation, m_prevTranslation, samples);
}

void CalibrationAttempt::finished(uint8_t error, bool applied, double rms, double axisVariance, const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation, size_t samples)
{
    using O = blackbox::CalibrationOutcome;
    const O outcome = !applied ? O::REJECTED : (error == 0 ? O::APPLIED : O::FORCED);
    send(static_cast<uint8_t>(outcome), error, rms, axisVariance, rotation, translation, samples);
}

void CalibrationAttempt::corrected()
{
    correctedTo(m_calibration.calibratedRotation, m_calibration.calibratedTranslation);
}

void CalibrationAttempt::setBefore(const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation)
{
    m_prevRotation = rotation;
    m_prevTranslation = translation;
}

void CalibrationAttempt::correctedTo(const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation)
{
    send(static_cast<uint8_t>(blackbox::CalibrationOutcome::CORRECTED), 0, std::numeric_limits<double>::quiet_NaN(), 0.0, rotation, translation, 0);
}

void CalibrationAttempt::send(uint8_t outcome, uint8_t error, double rms, double axisVariance, const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation, size_t samples)
{
    const bool continuous = m_calibration.isContinuousCalibration();
    uint8_t b = outcome;
    if (continuous)
        b |= blackbox::k_CALIB_FLAG_CONTINUOUS;
    if (m_calibration.isRelativeCalibration)
        b |= blackbox::k_CALIB_FLAG_RELATIVE;

    float values[13];
    values[0] = static_cast<float>(m_prevRms);
    values[1] = static_cast<float>(samples);
    values[2] = static_cast<float>((translation - m_prevTranslation).norm());
    values[3] = static_cast<float>(m_prevRotation.angularDistance(rotation) * k_RAD_TO_DEG);
    values[4] = static_cast<float>(translation.x());
    values[5] = static_cast<float>(translation.y());
    values[6] = static_cast<float>(translation.z());
    values[7] = static_cast<float>(rotation.w());
    values[8] = static_cast<float>(rotation.x());
    values[9] = static_cast<float>(rotation.y());
    values[10] = static_cast<float>(rotation.z());
    values[11] = static_cast<float>(axisVariance);
    values[12] = static_cast<float>(m_prevAxisVariance);
    const float rmsValue = std::isfinite(rms) ? static_cast<float>(rms) : std::numeric_limits<float>::quiet_NaN();

    if (auto* host = recorder::RecorderHost::getInstance()) {
        host->blackBox().recordCalibration(deviceByte(m_calibration.targetDevice.deviceId), deviceByte(m_calibration.referenceDevice.deviceId), m_trigger, b, error,
            rmsValue, values, blackbox::monoNow());
    }

    // the Live tab's solver graphs
    if (auto* live = LiveStats::getInstance()) {
        using O = blackbox::CalibrationOutcome;
        LiveSolve s;
        s.trigger = m_trigger;
        s.outcome = outcome;
        if (outcome == static_cast<uint8_t>(O::REJECTED) || outcome == static_cast<uint8_t>(O::SKIPPED) || outcome == static_cast<uint8_t>(O::FORCED)) {
            const CalibrationErrorMapping mapping = getCalibrationErrorMapping(static_cast<CalibrationError>(error));
            s.errorKey = mapping.szTranslationKey;
            s.errorText = mapping.szLogString;
        }
        s.rmsMm = static_cast<float>(rms * 1000.0); // NaN stays NaN
        s.currentRmsMm = m_prevRms >= 0.0 ? static_cast<float>(m_prevRms * 1000.0) : std::numeric_limits<float>::quiet_NaN();
        s.axisVariance = static_cast<float>(axisVariance);
        s.xCm = static_cast<float>(translation.x() * 100.0);
        s.yCm = static_cast<float>(translation.y() * 100.0);
        s.zCm = static_cast<float>(translation.z() * 100.0);
        s.yawDeg = static_cast<float>(yawDegrees(rotation));
        s.changeCm = values[2] * 100.0f;
        s.changeDeg = values[3];
        live->pushSolve(std::move(s));
    }

    // a solve that took the pending trigger consumes it; the next force names its own
    if (m_consumesTrigger && continuous)
        m_calibration.forkForceTrigger = 0;
}

} // namespace spacecal::guard
