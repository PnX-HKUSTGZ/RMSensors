#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "camera_calibration/camera_calibration_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<camera_calibration::CameraCalibrationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
