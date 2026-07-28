/**
 * @author Chen Shichao
 * @date 2024-06-27
 */

#ifndef CS_CALIBRATION__CALIBRATION_CONSUMER_HPP_
#define CS_CALIBRATION__CALIBRATION_CONSUMER_HPP_

#include <memory>
#include <Elite/DataType.hpp>

#include "cs_calibration/calibration.hpp"

namespace cs_calibration
{
class CalibrationConsumer
{
public:
    CalibrationConsumer();
    ~CalibrationConsumer();

    bool consume(const ELITE::vector6d_t& d,
                 const ELITE::vector6d_t& a,
                 const ELITE::vector6d_t& alpha);

    bool isCalibrated() const {
      return calibrated_;
    }

    YAML::Node getCalibrationParameters() const;

private:
    bool calibrated_;
    YAML::Node calibration_parameters_;
};
}  // namespace cs_calibration
#endif  // CS_CALIBRATION__CALIBRATION_CONSUMER_HPP_
