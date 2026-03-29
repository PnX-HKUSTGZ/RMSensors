#include "image_file_publisher/image_file_publisher_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <string>
#include <unordered_set>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include "image_file_publisher/parameter_validation.hpp"

namespace image_file_publisher
{

namespace
{

constexpr double kDefaultImageRateHz = 10.0;

const std::unordered_set<std::string> kImageExtensions = {
    ".bmp", ".dib", ".jpe", ".jpeg", ".jpg", ".jp2", ".png", ".pbm",
    ".pgm", ".ppm", ".ras", ".sr", ".tif", ".tiff", ".webp"};

std::string to_lower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool is_supported_image_path(const std::filesystem::path & path)
{
    return kImageExtensions.count(to_lower(path.extension().string())) > 0;
}

const char * to_string(const SourceKind kind)
{
    switch (kind) {
        case SourceKind::image:
            return "image";
        case SourceKind::directory:
            return "directory";
        case SourceKind::video:
            return "video";
    }

    return "unknown";
}

}  // namespace

ImageFilePublisherNode::ImageFilePublisherNode(const rclcpp::NodeOptions & options)
    : Node("image_file_publisher_node", options)
{
    param_listener_ = std::make_shared<ParamListener>(
        get_node_parameters_interface(), get_logger());
    params_ = param_listener_->get_params();

    std::string error_message;
    if (!validate_parameters(params_, &error_message)) {
        RCLCPP_ERROR(get_logger(), "Invalid parameters: %s", error_message.c_str());
        rclcpp::shutdown();
        return;
    }

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        params_.publish.image_topic,
        rclcpp::QoS(static_cast<std::size_t>(params_.publish.queue_size)));

    if (!initialize_source(&error_message)) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize source: %s", error_message.c_str());
        rclcpp::shutdown();
        return;
    }

    const double publish_rate_hz = resolve_publish_rate_hz();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz));

    timer_ = create_wall_timer(
        period,
        std::bind(&ImageFilePublisherNode::timer_callback, this));

    RCLCPP_INFO(
        get_logger(),
        "image_file_publisher started, source=%s, topic=%s, frame_id=%s, rate_hz=%.3f, loop=%s",
        to_string(source_kind_),
        params_.publish.image_topic.c_str(),
        params_.publish.frame_id.c_str(),
        publish_rate_hz,
        params_.playback.loop ? "true" : "false");
}

bool ImageFilePublisherNode::initialize_source(std::string * error_message)
{
    const std::filesystem::path source_path(params_.source.path);
    const std::string source_type = to_lower(params_.source.type);

    if (source_type == "auto") {
        if (std::filesystem::is_directory(source_path)) {
            source_kind_ = SourceKind::directory;
        } else if (is_supported_image_path(source_path)) {
            source_kind_ = SourceKind::image;
        } else {
            source_kind_ = SourceKind::video;
        }
    } else if (source_type == "image") {
        source_kind_ = SourceKind::image;
    } else if (source_type == "directory") {
        source_kind_ = SourceKind::directory;
    } else {
        source_kind_ = SourceKind::video;
    }

    if (source_kind_ == SourceKind::image) {
        return load_image_file(source_path.string(), &static_image_, error_message);
    }

    if (source_kind_ == SourceKind::directory) {
        image_paths_.clear();
        for (const auto & entry : std::filesystem::directory_iterator(source_path)) {
            if (!entry.is_regular_file()) {
                continue;
            }
            if (is_supported_image_path(entry.path())) {
                image_paths_.push_back(entry.path().string());
            }
        }
        std::sort(image_paths_.begin(), image_paths_.end());
        next_image_index_ = 0;
        if (image_paths_.empty()) {
            if (error_message != nullptr) {
                *error_message = "source.path directory does not contain any supported image files";
            }
            return false;
        }
        return true;
    }

    return open_video_capture(error_message);
}

bool ImageFilePublisherNode::open_video_capture(std::string * error_message)
{
    video_capture_.release();
    video_capture_.open(params_.source.path);
    if (!video_capture_.isOpened()) {
        if (error_message != nullptr) {
            *error_message = "unable to open video source";
        }
        return false;
    }

    video_source_fps_ = video_capture_.get(cv::CAP_PROP_FPS);
    if (!std::isfinite(video_source_fps_) || video_source_fps_ <= 0.0) {
        video_source_fps_ = 0.0;
    }

    return true;
}

double ImageFilePublisherNode::resolve_publish_rate_hz()
{
    if (params_.playback.rate_hz > 0.0) {
        return params_.playback.rate_hz;
    }

    if (source_kind_ == SourceKind::video && video_source_fps_ > 0.0) {
        return video_source_fps_;
    }

    if (source_kind_ == SourceKind::video) {
        RCLCPP_WARN(
            get_logger(),
            "Video source FPS is unavailable, falling back to %.1f Hz",
            kDefaultImageRateHz);
    }

    return kDefaultImageRateHz;
}

void ImageFilePublisherNode::timer_callback()
{
    if (!rclcpp::ok()) {
        rclcpp::shutdown();
        return;
    }

    cv::Mat frame;
    if (!read_next_frame(&frame)) {
        return;
    }

    publish_frame(frame);
}

bool ImageFilePublisherNode::read_next_frame(cv::Mat * frame)
{
    if (source_kind_ == SourceKind::image) {
        *frame = static_image_;
        return true;
    }

    if (source_kind_ == SourceKind::directory) {
        return read_next_directory_frame(frame);
    }

    return read_next_video_frame(frame);
}

bool ImageFilePublisherNode::read_next_directory_frame(cv::Mat * frame)
{
    std::size_t attempts = 0;
    while (attempts < image_paths_.size()) {
        if (next_image_index_ >= image_paths_.size()) {
            if (!params_.playback.loop) {
                RCLCPP_INFO(get_logger(), "Directory source reached the end, shutting down");
                rclcpp::shutdown();
                return false;
            }
            next_image_index_ = 0;
        }

        std::string error_message;
        const std::string & path = image_paths_[next_image_index_++];
        ++attempts;
        if (load_image_file(path, frame, &error_message)) {
            return true;
        }

        RCLCPP_ERROR(
            get_logger(),
            "Failed to load image '%s': %s",
            path.c_str(),
            error_message.c_str());
    }

    RCLCPP_ERROR(get_logger(), "No readable image files remain in the directory source");
    rclcpp::shutdown();
    return false;
}

bool ImageFilePublisherNode::read_next_video_frame(cv::Mat * frame)
{
    cv::Mat raw_frame;
    if (video_capture_.read(raw_frame) && !raw_frame.empty()) {
        std::string error_message;
        if (!convert_to_bgr8(raw_frame, frame, &error_message)) {
            RCLCPP_ERROR(get_logger(), "Failed to convert video frame: %s", error_message.c_str());
            return false;
        }
        return true;
    }

    if (!params_.playback.loop) {
        RCLCPP_INFO(get_logger(), "Video source reached the end, shutting down");
        rclcpp::shutdown();
        return false;
    }

    std::string error_message;
    if (!open_video_capture(&error_message)) {
        RCLCPP_ERROR(get_logger(), "Failed to reopen video source: %s", error_message.c_str());
        rclcpp::shutdown();
        return false;
    }

    if (!video_capture_.read(raw_frame) || raw_frame.empty()) {
        RCLCPP_ERROR(get_logger(), "Video source is empty after reopening");
        rclcpp::shutdown();
        return false;
    }

    if (!convert_to_bgr8(raw_frame, frame, &error_message)) {
        RCLCPP_ERROR(get_logger(), "Failed to convert looped video frame: %s", error_message.c_str());
        rclcpp::shutdown();
        return false;
    }

    return true;
}

bool ImageFilePublisherNode::load_image_file(
    const std::string & path,
    cv::Mat * image,
    std::string * error_message) const
{
    const cv::Mat raw_image = cv::imread(path, cv::IMREAD_UNCHANGED);
    if (raw_image.empty()) {
        if (error_message != nullptr) {
            *error_message = "OpenCV failed to decode the image";
        }
        return false;
    }

    return convert_to_bgr8(raw_image, image, error_message);
}

bool ImageFilePublisherNode::convert_to_bgr8(
    const cv::Mat & input,
    cv::Mat * output,
    std::string * error_message) const
{
    if (input.empty()) {
        if (error_message != nullptr) {
            *error_message = "input frame is empty";
        }
        return false;
    }

    cv::Mat eight_bit_image;
    if (input.depth() == CV_8U) {
        eight_bit_image = input;
    } else if (input.depth() == CV_16U) {
        input.convertTo(eight_bit_image, CV_8U, 1.0 / 256.0);
    } else {
        if (error_message != nullptr) {
            *error_message = "only 8-bit or 16-bit images are supported";
        }
        return false;
    }

    if (eight_bit_image.channels() == 1) {
        cv::cvtColor(eight_bit_image, *output, cv::COLOR_GRAY2BGR);
        return true;
    }
    if (eight_bit_image.channels() == 3) {
        eight_bit_image.copyTo(*output);
        return true;
    }
    if (eight_bit_image.channels() == 4) {
        cv::cvtColor(eight_bit_image, *output, cv::COLOR_BGRA2BGR);
        return true;
    }

    if (error_message != nullptr) {
        *error_message = "unsupported channel count";
    }
    return false;
}

void ImageFilePublisherNode::publish_frame(const cv::Mat & frame)
{
    if (image_pub_ == nullptr || frame.empty()) {
        return;
    }

    auto image_msg = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::BGR8,
        frame).toImageMsg();
    image_msg->header.stamp = now();
    image_msg->header.frame_id = params_.publish.frame_id;
    image_pub_->publish(*image_msg);
}

}  // namespace image_file_publisher
