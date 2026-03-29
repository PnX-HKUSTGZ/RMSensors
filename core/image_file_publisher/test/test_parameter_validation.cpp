#include <gtest/gtest.h>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

#include "image_file_publisher/parameter_validation.hpp"

namespace
{

class ScopedTempDirectory
{
public:
    ScopedTempDirectory()
    {
        std::array<char, 64> temp_directory_template{};
        const std::string template_path =
            (std::filesystem::temp_directory_path() / "image_file_publisher_test_XXXXXX").string();
        std::snprintf(
            temp_directory_template.data(),
            temp_directory_template.size(),
            "%s",
            template_path.c_str());

        char * created_directory = mkdtemp(temp_directory_template.data());
        if (created_directory == nullptr) {
            throw std::runtime_error("failed to create temporary directory");
        }

        path_ = created_directory;
    }

    ~ScopedTempDirectory()
    {
        std::error_code error_code;
        std::filesystem::remove_all(path_, error_code);
    }

    const std::filesystem::path & path() const
    {
        return path_;
    }

private:
    std::filesystem::path path_;
};

image_file_publisher::Params make_default_params()
{
    image_file_publisher::Params params;
    return params;
}

}  // namespace

TEST(ImageFilePublisherParameterValidationTest, RejectsEmptySourcePath)
{
    auto params = make_default_params();
    params.source.path.clear();

    std::string error_message;
    EXPECT_FALSE(image_file_publisher::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("source.path"), std::string::npos);
}

TEST(ImageFilePublisherParameterValidationTest, RejectsEmptyImageTopic)
{
    auto params = make_default_params();
    params.source.type = "directory";

    ScopedTempDirectory temp_directory;
    std::ofstream(temp_directory.path() / "frame_001.png").put('\n');
    params.source.path = temp_directory.path().string();
    params.publish.image_topic.clear();

    std::string error_message;
    EXPECT_FALSE(image_file_publisher::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("publish.image_topic"), std::string::npos);
}

TEST(ImageFilePublisherParameterValidationTest, RejectsEmptyFrameId)
{
    auto params = make_default_params();
    params.source.type = "directory";

    ScopedTempDirectory temp_directory;
    std::ofstream(temp_directory.path() / "frame_001.png").put('\n');
    params.source.path = temp_directory.path().string();
    params.publish.frame_id.clear();

    std::string error_message;
    EXPECT_FALSE(image_file_publisher::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("publish.frame_id"), std::string::npos);
}

TEST(ImageFilePublisherParameterValidationTest, RejectsNonFinitePlaybackRate)
{
    auto params = make_default_params();
    params.source.type = "directory";

    ScopedTempDirectory temp_directory;
    std::ofstream(temp_directory.path() / "frame_001.png").put('\n');
    params.source.path = temp_directory.path().string();
    params.playback.rate_hz = std::numeric_limits<double>::quiet_NaN();

    std::string error_message;
    EXPECT_FALSE(image_file_publisher::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("playback.rate_hz"), std::string::npos);
}

TEST(ImageFilePublisherParameterValidationTest, RejectsSourceTypeMismatch)
{
    auto params = make_default_params();
    params.source.type = "video";

    ScopedTempDirectory temp_directory;
    params.source.path = temp_directory.path().string();

    std::string error_message;
    EXPECT_FALSE(image_file_publisher::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("source.path"), std::string::npos);
}

TEST(ImageFilePublisherParameterValidationTest, RejectsEmptyDirectorySource)
{
    auto params = make_default_params();
    params.source.type = "directory";

    ScopedTempDirectory temp_directory;
    params.source.path = temp_directory.path().string();

    std::string error_message;
    EXPECT_FALSE(image_file_publisher::validate_parameters(params, &error_message));
    EXPECT_NE(error_message.find("supported image files"), std::string::npos);
}
