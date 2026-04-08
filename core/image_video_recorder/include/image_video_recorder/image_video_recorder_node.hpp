#ifndef IMAGE_VIDEO_RECORDER__IMAGE_VIDEO_RECORDER_NODE_HPP_
#define IMAGE_VIDEO_RECORDER__IMAGE_VIDEO_RECORDER_NODE_HPP_

#pragma once

#include <atomic>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "image_video_recorder/image_video_recorder_parameters.hpp"
#include "image_video_recorder/recording_session.hpp"

namespace image_video_recorder
{

class ImageVideoRecorderNode : public rclcpp::Node
{
public:
    explicit ImageVideoRecorderNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~ImageVideoRecorderNode() override;

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void finalize_and_log();

    std::shared_ptr<ParamListener> param_listener_;
    Params params_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::shared_ptr<RecordingSession> session_;
    std::atomic_bool display_conversion_warning_logged_{false};
    std::atomic_bool finalize_logged_{false};
    std::atomic_bool resize_warning_logged_{false};
};

}  // namespace image_video_recorder

#endif  // IMAGE_VIDEO_RECORDER__IMAGE_VIDEO_RECORDER_NODE_HPP_
