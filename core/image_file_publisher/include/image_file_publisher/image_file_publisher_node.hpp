#ifndef IMAGE_FILE_PUBLISHER__IMAGE_FILE_PUBLISHER_NODE_HPP_
#define IMAGE_FILE_PUBLISHER__IMAGE_FILE_PUBLISHER_NODE_HPP_

#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "image_file_publisher/image_file_publisher_parameters.hpp"

namespace image_file_publisher
{

enum class SourceKind
{
    image,
    directory,
    video,
};

class ImageFilePublisherNode : public rclcpp::Node
{
public:
    explicit ImageFilePublisherNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    bool initialize_source(std::string * error_message);
    bool open_video_capture(std::string * error_message);
    double resolve_publish_rate_hz();
    void timer_callback();
    bool read_next_frame(cv::Mat * frame);
    bool read_next_directory_frame(cv::Mat * frame);
    bool read_next_video_frame(cv::Mat * frame);
    bool load_image_file(
        const std::string & path,
        cv::Mat * image,
        std::string * error_message) const;
    bool convert_to_bgr8(
        const cv::Mat & input,
        cv::Mat * output,
        std::string * error_message) const;
    void publish_frame(const cv::Mat & frame);

    std::shared_ptr<ParamListener> param_listener_;
    Params params_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    SourceKind source_kind_{SourceKind::image};
    cv::Mat static_image_;
    std::vector<std::string> image_paths_;
    std::size_t next_image_index_{0};
    cv::VideoCapture video_capture_;
    double video_source_fps_{0.0};
};

}  // namespace image_file_publisher

#endif  // IMAGE_FILE_PUBLISHER__IMAGE_FILE_PUBLISHER_NODE_HPP_
