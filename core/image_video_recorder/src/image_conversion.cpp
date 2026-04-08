#include "image_video_recorder/image_conversion.hpp"

#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace image_video_recorder
{

namespace
{

void set_error_message(std::string * error_message, const std::string & message)
{
    if (error_message != nullptr) {
        *error_message = message;
    }
}

}  // namespace

bool convert_image_message_to_bgr8(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    cv::Mat * image,
    bool * used_display_conversion,
    std::string * error_message)
{
    if (used_display_conversion != nullptr) {
        *used_display_conversion = false;
    }

    if (msg == nullptr) {
        set_error_message(error_message, "input image message is null");
        return false;
    }

    if (image == nullptr) {
        set_error_message(error_message, "output image pointer is null");
        return false;
    }

    std::string direct_conversion_error;
    try {
        const auto cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        *image = cv_image->image;
        return true;
    } catch (const cv_bridge::Exception & e) {
        direct_conversion_error = e.what();
    }

    try {
        const auto source_image = cv_bridge::toCvCopy(msg);
        cv_bridge::CvtColorForDisplayOptions options;
        options.do_dynamic_scaling = true;

        const auto display_image = cv_bridge::cvtColorForDisplay(
            source_image,
            sensor_msgs::image_encodings::BGR8,
            options);

        if (display_image == nullptr || display_image->image.empty()) {
            set_error_message(
                error_message,
                "display conversion returned an empty image for encoding '" + msg->encoding + "'");
            return false;
        }

        *image = display_image->image;
        if (used_display_conversion != nullptr) {
            *used_display_conversion = true;
        }
        return true;
    } catch (const cv_bridge::Exception & e) {
        set_error_message(
            error_message,
            "failed to convert encoding '" + msg->encoding + "' to BGR8; direct conversion error: " +
                direct_conversion_error + "; display conversion error: " + e.what());
        return false;
    } catch (const cv::Exception & e) {
        set_error_message(
            error_message,
            "failed to convert encoding '" + msg->encoding +
                "' to BGR8 during display conversion: " + e.what());
        return false;
    }
}

}  // namespace image_video_recorder
