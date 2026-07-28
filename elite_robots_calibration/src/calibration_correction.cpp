/**
 * @author Chen Shichao
 * @date 2024-06-27
 */

#include "cs_calibration/calibration_consumer.hpp"

#include <Elite/PrimaryPortInterface.hpp>
#include <Elite/RobotConfPackage.hpp>
#include <filesystem>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/exceptions/exceptions.hpp"

namespace fs = std::filesystem;

class CalibrationCorrection : public rclcpp::Node {
public:
    CalibrationCorrection()
        : Node("cs_calibration",
               rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)) {
        std::string output_package_name;

        try {
            // The robot's IP address
            robot_ip_ = this->get_parameter("robot_ip").as_string();
            // The target file where the calibration data is written to
            output_filename_ = this->get_parameter("output_filename").as_string();
        } catch (rclcpp::exceptions::ParameterNotDeclaredException& e) {
            RCLCPP_FATAL_STREAM(this->get_logger(), e.what());
            exit(1);
        }
    }

    virtual ~CalibrationCorrection() = default;

    void run() {
        cs_calibration::CalibrationConsumer consumer;

        auto kin = std::make_shared<ELITE::KinematicsInfo>();
        std::unique_ptr<ELITE::PrimaryPortInterface> primary;
        primary.reset(new ELITE::PrimaryPortInterface);
        if (!primary->connect(robot_ip_)) {
            RCLCPP_FATAL_STREAM(this->get_logger(), "Connect to robot fail");
            return;
        }
        if (!primary->getPackage(kin, 5000)) {
            primary->disconnect();
            RCLCPP_FATAL_STREAM(this->get_logger(), "Connect to robot fail");
            return;
        }
        primary->disconnect();

        consumer.consume(kin->dh_d_, kin->dh_a_, kin->dh_alpha_);

        while (!consumer.isCalibrated()) {
            rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.1).to_chrono<std::chrono::nanoseconds>());
        }
        calibration_data_.reset(new YAML::Node);
        *calibration_data_ = consumer.getCalibrationParameters();
    }

    bool writeCalibrationData() {
        if (calibration_data_ == nullptr) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Calibration data not yet set.");
            return false;
        }

        fs::path out_path = fs::absolute(output_filename_);

        fs::path dst_path = out_path.parent_path();
        if (!fs::exists(dst_path)) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Parent folder " << dst_path << " does not exist.");
            return false;
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Writing calibration data to " << out_path);
        if (fs::exists(output_filename_)) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Output file " << output_filename_ << " already exists. Overwriting.");
        }
        std::ofstream file(output_filename_);
        if (file.is_open()) {
            file << *calibration_data_;
        } else {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed writing the file. Do you have the correct rights?");
            return false;
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Wrote output.");

        return true;
    }

private:
    std::string robot_ip_;
    std::string output_filename_;
    std::shared_ptr<YAML::Node> calibration_data_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    try {
        auto calib_node = std::make_shared<CalibrationCorrection>();
        calib_node->run();
        if (!calib_node->writeCalibrationData()) {
            RCLCPP_ERROR_STREAM(calib_node->get_logger(), "Failed writing calibration data. See errors above for details.");
            return -1;
        }
        RCLCPP_INFO_STREAM(calib_node->get_logger(), "Calibration correction done");
    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("cs_calibration"), e.what());
    }

    rclcpp::shutdown();

    return 0;
}
