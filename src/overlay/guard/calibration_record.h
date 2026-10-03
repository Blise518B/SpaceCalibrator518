#pragma once

// Fork: every calibration attempt of the overlay's solver goes into the driver's black box as a
// CALIBRATION record (layout in blackbox_format.h): what triggered it, what came of it and the
// numbers behind it, so a recording shows when and why the calibration moved.

#include "blackbox_format.h"
#include <Eigen/Dense>
#include <cstddef>
#include <cstdint>

namespace spacecal {
class TrackingSystemCalibration;
}

namespace spacecal::guard {

class CalibrationAttempt {
public:
    // trigger: a blackbox::CalibrationTrigger; 0 (UNKNOWN) derives it from the calibration's state
    // and the force flag. Captures the active calibration as the "before" value.
    CalibrationAttempt(TrackingSystemCalibration& calibration, bool forced, uint8_t trigger = 0);
    // the active calibration's error on the current samples, before the solve may replace it
    void setPrevious(double rms, double axisVariance);
    // nothing solved: the target device is not trusted
    void skipped(size_t samples);
    // a solve ran; error = the overlay's CalibrationError, applied = the result became active
    void finished(uint8_t error, bool applied, double rms, double axisVariance, const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation, size_t samples);
    // nothing solved: the active calibration was shifted by a measured jump (call after the shift)
    void corrected();

private:
    void send(uint8_t outcome, uint8_t error, double rms, double axisVariance, const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation, size_t samples);

    TrackingSystemCalibration& m_calibration;
    bool m_forced = false;
    bool m_consumesTrigger = false;
    uint8_t m_trigger = 0;
    Eigen::Quaterniond m_prevRotation;
    Eigen::Vector3d m_prevTranslation;
    double m_prevRms = -1.0;
    double m_prevAxisVariance = 0.0;
};

} // namespace spacecal::guard
