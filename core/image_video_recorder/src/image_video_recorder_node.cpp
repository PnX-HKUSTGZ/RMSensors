#include "image_video_recorder/image_video_recorder_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "image_video_recorder/image_conversion.hpp"
#include "image_video_recorder/parameter_validation.hpp"

namespace image_video_recorder
{

namespace
{

void log_finalize_start(const rclcpp::Logger & logger, const RecordingSession & session)
{
    if (session.frame_count() == 0U) {
        RCLCPP_INFO(
            logger,
            "Finalizing recording with no frames; cleaning temporary data in %s",
            session.temp_directory().string().c_str());
        return;
    }

    RCLCPP_INFO(
        logger,
        "Finalizing recording: encoding %zu frames to %s",
        session.frame_count(),
        session.output_path().string().c_str());
}

void log_finalize_result(const rclcpp::Logger & logger, const FinalizeResult & result)
{
    if (!result.finalize_invoked) {
        return;
    }

    if (result.succeeded && result.wrote_output) {
        RCLCPP_INFO(
            logger,
            "Recording finalized: frames=%zu, output=%s, success=true",
            result.frame_count,
            result.output_path.string().c_str());
        return;
    }

    if (result.succeeded && !result.wrote_output) {
        RCLCPP_WARN(
            logger,
            "Recording finalized: frames=0, output=%s, success=true, wrote_output=false",
            result.output_path.string().c_str());
        return;
    }

    RCLCPP_ERROR(
        logger,
        "Recording finalized: frames=%zu, output=%s, success=false, temp_directory=%s, error=%s",
        result.frame_count,
        result.output_path.string().c_str(),
        result.temp_directory.string().c_str(),
        result.error_message.c_str());
}

}  // namespace

ImageVideoRecorderNode::ImageVideoRecorderNode(const rclcpp::NodeOptions & options)
    : Node("image_video_recorder_node", options)
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

    RecordingSessionOptions session_options;
    session_options.output_directory = params_.output.directory;
    session_options.filename_prefix = params_.output.filename_prefix;
    session_options.fps = params_.recording.fps;
    session_options.temp_root = params_.runtime.temp_root;
    session_options.start_time = std::chrono::system_clock::now();

    session_ = std::make_shared<RecordingSession>(std::move(session_options));
    if (!session_->initialize(&error_message)) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize recording session: %s", error_message.c_str());
        rclcpp::shutdown();
        return;
    }

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        params_.input.image_topic,
        rclcpp::QoS(static_cast<std::size_t>(params_.input.queue_size)),
        std::bind(&ImageVideoRecorderNode::image_callback, this, std::placeholders::_1));

    rclcpp::on_shutdown(
        [this]() {
            if (session_ != nullptr) {
                finalize_and_log();
            }
        },
        get_node_base_interface()->get_context());

    RCLCPP_INFO(
        get_logger(),
        "image_video_recorder started, topic=%s, output_directory=%s, fps=%.3f, temp_directory=%s",
        params_.input.image_topic.c_str(),
        params_.output.directory.c_str(),
        params_.recording.fps,
        session_->temp_directory().string().c_str());
}

ImageVideoRecorderNode::~ImageVideoRecorderNode()
{
    finalize_and_log();
}

void ImageVideoRecorderNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    if (session_ == nullptr) {
        return;
    }

    cv::Mat bgr_frame;
    bool used_display_conversion = false;
    std::string error_message;
    if (!convert_image_message_to_bgr8(
            msg,
            &bgr_frame,
            &used_display_conversion,
            &error_message))
    {
        RCLCPP_ERROR(get_logger(), "Failed to convert image frame: %s", error_message.c_str());
        return;
    }

    if (used_display_conversion && !display_conversion_warning_logged_.exchange(true)) {
        RCLCPP_WARN(
            get_logger(),
            "Recording non-color image encoding '%s' using display conversion",
            msg->encoding.c_str());
    }

    const AppendFrameResult result = session_->append_frame(bgr_frame);
    if (!result.accepted) {
        if (rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Failed to record image frame: %s", result.error_message.c_str());
        }
        return;
    }

    if (result.resized && !resize_warning_logged_.exchange(true)) {
        RCLCPP_WARN(
            get_logger(),
            "Image size changed during recording; resizing mismatched frames to the first frame size");
    }
}

void ImageVideoRecorderNode::finalize_and_log()
{
    if (session_ == nullptr || finalize_logged_.exchange(true)) {
        return;
    }

    log_finalize_start(get_logger(), *session_);
    log_finalize_result(get_logger(), session_->finalize());
}

}  // namespace image_video_recorder
