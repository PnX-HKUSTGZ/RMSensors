#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "image_video_recorder/parameter_validation.hpp"

namespace
{

image_video_recorder::Params make_default_params()
{
    image_video_recorder::Params params;
    return params;
}

}  // namespace

TEST(ImageVideoRecorderParameterValidationTest, AcceptsDefaultParameters)
{
    const auto params = make_default_params();

    std::string error_message;
    EXPECT_TRUE(image_video_recorder::validate_parameters(params, &error_message));
    EXPECT_TRUE(error_message.empty());
}

TEST(ImageVideoRecorderParameterValidationTest, RejectsEmptyImageTopic)
{
    auto params = make_default_params();
    params.input.image_topic.clear();

    std::string error_message;
    EXPECT_FALSE(image_video_recorder::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("input.image_topic"), std::string::npos);
}

TEST(ImageVideoRecorderParameterValidationTest, RejectsEmptyOutputDirectory)
{
    auto params = make_default_params();
    params.output.directory.clear();

    std::string error_message;
    EXPECT_FALSE(image_video_recorder::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("output.directory"), std::string::npos);
}

TEST(ImageVideoRecorderParameterValidationTest, RejectsEmptyFilenamePrefix)
{
    auto params = make_default_params();
    params.output.filename_prefix.clear();

    std::string error_message;
    EXPECT_FALSE(image_video_recorder::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("output.filename_prefix"), std::string::npos);
}

TEST(ImageVideoRecorderParameterValidationTest, RejectsNonFiniteFps)
{
    auto params = make_default_params();
    params.recording.fps = std::numeric_limits<double>::quiet_NaN();

    std::string error_message;
    EXPECT_FALSE(image_video_recorder::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("recording.fps"), std::string::npos);
}

TEST(ImageVideoRecorderParameterValidationTest, RejectsNonPositiveFps)
{
    auto params = make_default_params();
    params.recording.fps = 0.0;

    std::string error_message;
    EXPECT_FALSE(image_video_recorder::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("recording.fps"), std::string::npos);
}
