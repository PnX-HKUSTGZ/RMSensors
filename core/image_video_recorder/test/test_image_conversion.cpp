#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "image_video_recorder/image_conversion.hpp"

namespace
{

sensor_msgs::msg::Image::ConstSharedPtr make_bgr8_message()
{
    auto message = std::make_shared<sensor_msgs::msg::Image>();
    message->height = 2;
    message->width = 2;
    message->encoding = sensor_msgs::image_encodings::BGR8;
    message->is_bigendian = false;
    message->step = 6;
    message->data = {
        0, 0, 255, 0, 255, 0,
        255, 0, 0, 255, 255, 255};
    return message;
}

sensor_msgs::msg::Image::ConstSharedPtr make_16uc1_message()
{
    auto message = std::make_shared<sensor_msgs::msg::Image>();
    message->height = 2;
    message->width = 2;
    message->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    message->is_bigendian = false;
    message->step = 4;

    const std::vector<std::uint16_t> pixels = {0U, 1000U, 2000U, 4000U};
    message->data.resize(pixels.size() * sizeof(std::uint16_t));
    std::memcpy(message->data.data(), pixels.data(), message->data.size());
    return message;
}

}  // namespace

TEST(ImageConversionTest, ConvertsBgr8WithoutDisplayFallback)
{
    cv::Mat image;
    bool used_display_conversion = true;
    std::string error_message;

    EXPECT_TRUE(image_video_recorder::convert_image_message_to_bgr8(
        make_bgr8_message(),
        &image,
        &used_display_conversion,
        &error_message));
    EXPECT_TRUE(error_message.empty());
    EXPECT_FALSE(used_display_conversion);
    ASSERT_EQ(image.type(), CV_8UC3);
    EXPECT_EQ(image.rows, 2);
    EXPECT_EQ(image.cols, 2);
}

TEST(ImageConversionTest, Converts16UC1UsingDisplayFallback)
{
    cv::Mat image;
    bool used_display_conversion = false;
    std::string error_message;

    EXPECT_TRUE(image_video_recorder::convert_image_message_to_bgr8(
        make_16uc1_message(),
        &image,
        &used_display_conversion,
        &error_message));
    EXPECT_TRUE(error_message.empty());
    EXPECT_TRUE(used_display_conversion);
    ASSERT_EQ(image.type(), CV_8UC3);
    EXPECT_EQ(image.rows, 2);
    EXPECT_EQ(image.cols, 2);
    EXPECT_NE(image.at<cv::Vec3b>(0, 0), image.at<cv::Vec3b>(1, 1));
}
