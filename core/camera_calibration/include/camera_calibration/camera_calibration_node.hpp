#ifndef CAMERA_CALIBRATION__CAMERA_CALIBRATION_NODE_HPP_
#define CAMERA_CALIBRATION__CAMERA_CALIBRATION_NODE_HPP_

#pragma once

#include <memory>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "camera_calibration/camera_calibration_parameters.hpp"

namespace camera_calibration
{

class CameraCalibrationNode : public rclcpp::Node
{
public:
    explicit CameraCalibrationNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void initialize_calibration_pattern();
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    std::shared_ptr<ParamListener> param_listener_;
    Params params_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    std::vector<std::vector<cv::Point3f>> object_points_;
    std::vector<std::vector<cv::Point2f>> image_points_;
    std::vector<cv::Point3f> object_pattern_;
};

}  // namespace camera_calibration

#endif  // CAMERA_CALIBRATION__CAMERA_CALIBRATION_NODE_HPP_
