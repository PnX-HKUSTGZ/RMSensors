#ifndef IMAGE_VIDEO_RECORDER__IMAGE_CONVERSION_HPP_
#define IMAGE_VIDEO_RECORDER__IMAGE_CONVERSION_HPP_

#pragma once

#include <string>

#include <opencv2/core/mat.hpp>

#include <sensor_msgs/msg/image.hpp>

namespace image_video_recorder
{

bool convert_image_message_to_bgr8(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    cv::Mat * image,
    bool * used_display_conversion,
    std::string * error_message);

}  // namespace image_video_recorder

#endif  // IMAGE_VIDEO_RECORDER__IMAGE_CONVERSION_HPP_
