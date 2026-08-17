#include "cs_calibration/calibration_consumer.hpp"

#include <memory>

namespace cs_calibration {
CalibrationConsumer::CalibrationConsumer() : calibrated_(false) {}

CalibrationConsumer::~CalibrationConsumer() {}

bool CalibrationConsumer::consume(const ELITE::vector6d_t& d, const ELITE::vector6d_t& a, const ELITE::vector6d_t& alpha) {
    ELITE::vector6d_t theta{0};
    DHRobot my_robot;
    for (size_t i = 0; i < 5; ++i) {
        my_robot.segments_.push_back(DHSegment(d[i], a[i + 1], theta[i], alpha[i + 1]));
    }
    my_robot.segments_.push_back(DHSegment(d[5], 0.0, theta[5], 0.0));
    Calibration calibration(my_robot);

    calibration_parameters_ = calibration.toYaml();
    calibrated_ = true;

    return true;
}

YAML::Node CalibrationConsumer::getCalibrationParameters() const {
    if (!calibrated_) {
        throw(std::runtime_error("Cannot get calibration, as no calibration data received yet"));
    }
    return calibration_parameters_;
}
}  // namespace cs_calibration
