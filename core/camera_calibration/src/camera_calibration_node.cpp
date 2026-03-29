#include "camera_calibration/camera_calibration_node.hpp"

#include <sstream>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace camera_calibration
{

CameraCalibrationNode::CameraCalibrationNode(const rclcpp::NodeOptions & options)
    : Node("calibrate_camera", options)
{
    param_listener_ = std::make_shared<ParamListener>(
        get_node_parameters_interface(), get_logger());
    params_ = param_listener_->get_params();

    initialize_calibration_pattern();

    subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        params_.subscription.image_topic,
        rclcpp::QoS(static_cast<size_t>(params_.subscription.queue_size)),
        std::bind(&CameraCalibrationNode::image_callback, this, std::placeholders::_1));
}

void CameraCalibrationNode::initialize_calibration_pattern()
{
    object_pattern_.clear();
    object_pattern_.reserve(
        static_cast<size_t>(params_.target.width * params_.target.height));

    for (int i = 0; i < params_.target.height; ++i) {
        for (int j = 0; j < params_.target.width; ++j) {
            object_pattern_.emplace_back(
                static_cast<float>(j * params_.target.spacing_m),
                static_cast<float>(i * params_.target.spacing_m),
                0.0F);
        }
    }
}

void CameraCalibrationNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!rclcpp::ok()) {
        rclcpp::shutdown();
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image = cv_ptr->image;
    cv::Mat grayscale_image;
    cv::cvtColor(image, grayscale_image, cv::COLOR_BGR2GRAY);
    RCLCPP_INFO(get_logger(), "Get Image");

    const cv::Size board_size(params_.target.width, params_.target.height);
    std::vector<cv::Point2f> corner_points;
    const bool success = cv::findCirclesGrid(grayscale_image, board_size, corner_points);

    if (!success) {
        RCLCPP_WARN(get_logger(), "fail to find circls grid");
        return;
    }

    RCLCPP_INFO(get_logger(), "find circls grid successfully");
    cv::drawChessboardCorners(image, board_size, corner_points, success);

    object_points_.push_back(object_pattern_);
    image_points_.push_back(corner_points);

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat rotation_vectors;
    cv::Mat translation_vectors;

    if (object_points_.size() >= static_cast<size_t>(params_.sampling.required_frames)) {
        std::stringstream stream;
        cv::calibrateCamera(
            object_points_,
            image_points_,
            cv::Size(grayscale_image.rows, grayscale_image.cols),
            camera_matrix,
            dist_coeffs,
            rotation_vectors,
            translation_vectors);
        stream << "cameraMatrix : " << camera_matrix << std::endl;
        stream << "distCoeffs : " << dist_coeffs << std::endl;
        stream << "Rotation vector : " << rotation_vectors << std::endl;
        stream << "Translation vector : " << translation_vectors << std::endl;

        RCLCPP_INFO(get_logger(), "finish!");
        RCLCPP_INFO(get_logger(), "%s", stream.str().c_str());

        object_points_.clear();
        image_points_.clear();
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(get_logger(), "get %zu frames", object_points_.size());
    cv::imshow("get", image);
    cv::waitKey(params_.sampling.display_wait_ms);
}

}  // namespace camera_calibration
