#include <gtest/gtest.h>

#include <string>

#include "hk_camera_driver/parameter_validation.hpp"

namespace
{

hk_camera_driver::Params make_default_params()
{
    hk_camera_driver::Params params;
    return params;
}

}  // namespace

TEST(HkCameraDriverParameterValidationTest, RejectsEmptyImageTopic)
{
    auto params = make_default_params();
    params.publish.image_topic.clear();

    std::string error_message;
    EXPECT_FALSE(hk_camera_driver::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("publish.image_topic"), std::string::npos);
}

TEST(HkCameraDriverParameterValidationTest, RejectsEmptyFrameId)
{
    auto params = make_default_params();
    params.publish.frame_id.clear();

    std::string error_message;
    EXPECT_FALSE(hk_camera_driver::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("publish.frame_id"), std::string::npos);
}

TEST(HkCameraDriverParameterValidationTest, RejectsInvalidAutoExposureRange)
{
    auto params = make_default_params();
    params.camera.exposure_auto = true;
    params.camera.auto_exposure_lower_us = 30000;
    params.camera.auto_exposure_upper_us = 1000;

    std::string error_message;
    EXPECT_FALSE(hk_camera_driver::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("auto_exposure_upper_us"), std::string::npos);
}

TEST(HkCameraDriverParameterValidationTest, RejectsInvalidManualExposure)
{
    auto params = make_default_params();
    params.camera.exposure_auto = false;
    params.camera.manual_exposure_us = 0.0;

    std::string error_message;
    EXPECT_FALSE(hk_camera_driver::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("manual_exposure_us"), std::string::npos);
}

TEST(HkCameraDriverParameterValidationTest, RejectsEmptyStaticTfParentFrame)
{
    auto params = make_default_params();
    params.tf.publish_static_tf = true;
    params.tf.parent_frame.clear();

    std::string error_message;
    EXPECT_FALSE(hk_camera_driver::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("tf.parent_frame"), std::string::npos);
}

TEST(HkCameraDriverParameterValidationTest, RejectsTooLargeBayerQuality)
{
    auto params = make_default_params();
    params.conversion.bayer_quality = 4;

    std::string error_message;
    EXPECT_FALSE(hk_camera_driver::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("bayer_quality"), std::string::npos);
}

TEST(HkCameraDriverParameterValidationTest, RejectsEmptyStaticTfChildFrame)
{
    auto params = make_default_params();
    params.tf.publish_static_tf = true;
    params.tf.child_frame.clear();

    std::string error_message;
    EXPECT_FALSE(hk_camera_driver::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("tf.child_frame"), std::string::npos);
}
