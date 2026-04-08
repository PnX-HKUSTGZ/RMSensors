#include <gtest/gtest.h>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

#include "image_video_recorder/recording_session.hpp"

namespace
{

class ScopedTempDirectory
{
public:
    ScopedTempDirectory()
    {
        std::array<char, 64> temp_directory_template{};
        const std::string template_path =
            (std::filesystem::temp_directory_path() / "image_video_recorder_test_XXXXXX").string();
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

image_video_recorder::RecordingSessionOptions make_options(
    const std::filesystem::path & output_directory,
    const std::filesystem::path & temp_root)
{
    image_video_recorder::RecordingSessionOptions options;
    options.output_directory = output_directory;
    options.filename_prefix = "recording";
    options.fps = 30.0;
    options.temp_root = temp_root;
    options.start_time = std::chrono::system_clock::from_time_t(1712361600);
    return options;
}

cv::Mat make_image(int width, int height, const cv::Scalar & color)
{
    return cv::Mat(height, width, CV_8UC3, color).clone();
}

}  // namespace

TEST(RecordingSessionTest, BuildOutputPathUsesExpectedPattern)
{
    std::string error_message;
    const auto output_path = image_video_recorder::RecordingSession::build_output_path(
        "/tmp/out",
        "recording",
        std::chrono::system_clock::from_time_t(1712361600),
        &error_message);

    EXPECT_TRUE(error_message.empty());
    EXPECT_EQ(output_path.parent_path(), std::filesystem::path("/tmp/out"));
    EXPECT_TRUE(std::regex_match(
        output_path.filename().string(),
        std::regex(R"(recording_\d{8}_\d{6}\.mp4)")));
}

TEST(RecordingSessionTest, InitializeFailsWhenOutputDirectoryIsAFile)
{
    ScopedTempDirectory temp_directory;
    const std::filesystem::path output_file_path = temp_directory.path() / "not_a_directory";
    std::ofstream(output_file_path).put('\n');

    auto options = make_options(output_file_path, temp_directory.path());
    image_video_recorder::RecordingSession session(options);

    std::string error_message;
    EXPECT_FALSE(session.initialize(&error_message));
    EXPECT_NE(error_message.find("not a directory"), std::string::npos);
}

TEST(RecordingSessionTest, InitializeFailsWhenTempRootIsAFile)
{
    ScopedTempDirectory temp_directory;
    const std::filesystem::path temp_root_file = temp_directory.path() / "temp_root_file";
    std::ofstream(temp_root_file).put('\n');

    auto options = make_options(temp_directory.path() / "output", temp_root_file);
    image_video_recorder::RecordingSession session(options);

    std::string error_message;
    EXPECT_FALSE(session.initialize(&error_message));
    EXPECT_NE(error_message.find("not a directory"), std::string::npos);
}

TEST(RecordingSessionTest, FinalizeWithoutFramesDoesNotGenerateOutput)
{
    ScopedTempDirectory temp_directory;
    auto options = make_options(temp_directory.path() / "output", temp_directory.path() / "temp");
    image_video_recorder::RecordingSession session(options);

    std::string error_message;
    ASSERT_TRUE(session.initialize(&error_message)) << error_message;

    const auto output_path = session.output_path();
    const auto temporary_directory = session.temp_directory();
    const auto result = session.finalize();

    EXPECT_TRUE(result.finalize_invoked);
    EXPECT_TRUE(result.succeeded);
    EXPECT_FALSE(result.wrote_output);
    EXPECT_EQ(result.frame_count, 0U);
    EXPECT_FALSE(std::filesystem::exists(output_path));
    EXPECT_FALSE(std::filesystem::exists(temporary_directory));
}

TEST(RecordingSessionTest, FinalizeOverwritesExistingOutputFile)
{
    ScopedTempDirectory temp_directory;
    auto options = make_options(temp_directory.path() / "output", temp_directory.path() / "temp");
    image_video_recorder::RecordingSession session(options);

    std::string error_message;
    ASSERT_TRUE(session.initialize(&error_message)) << error_message;

    const auto output_path = session.output_path();
    {
        std::ofstream old_output(output_path, std::ios::binary);
        old_output << "old";
    }

    ASSERT_TRUE(session.append_frame(make_image(64, 48, cv::Scalar(0, 255, 0))).accepted);
    ASSERT_TRUE(session.append_frame(make_image(64, 48, cv::Scalar(255, 0, 0))).accepted);

    const auto result = session.finalize();

    ASSERT_TRUE(result.succeeded) << result.error_message;
    EXPECT_TRUE(result.wrote_output);
    EXPECT_TRUE(std::filesystem::exists(output_path));
    EXPECT_GT(std::filesystem::file_size(output_path), 3U);
}

TEST(RecordingSessionTest, FinalizeFailurePreservesTemporaryFrames)
{
    ScopedTempDirectory temp_directory;
    auto options = make_options(temp_directory.path() / "output", temp_directory.path() / "temp");
    image_video_recorder::RecordingSession session(options);

    std::string error_message;
    ASSERT_TRUE(session.initialize(&error_message)) << error_message;
    ASSERT_TRUE(session.append_frame(make_image(64, 48, cv::Scalar(0, 255, 0))).accepted);

    const auto temporary_directory = session.temp_directory();
    std::filesystem::remove(temporary_directory / "frame_000001.png");

    const auto result = session.finalize();

    EXPECT_TRUE(result.finalize_invoked);
    EXPECT_FALSE(result.succeeded);
    EXPECT_FALSE(result.wrote_output);
    EXPECT_TRUE(std::filesystem::exists(temporary_directory));
    EXPECT_FALSE(result.error_message.empty());
}

TEST(RecordingSessionTest, MismatchedFrameSizesAreResized)
{
    ScopedTempDirectory temp_directory;
    auto options = make_options(temp_directory.path() / "output", temp_directory.path() / "temp");
    image_video_recorder::RecordingSession session(options);

    std::string error_message;
    ASSERT_TRUE(session.initialize(&error_message)) << error_message;

    const auto first_result = session.append_frame(make_image(64, 48, cv::Scalar(0, 255, 0)));
    const auto second_result = session.append_frame(make_image(32, 24, cv::Scalar(255, 0, 0)));

    ASSERT_TRUE(first_result.accepted);
    ASSERT_TRUE(second_result.accepted);
    EXPECT_FALSE(first_result.resized);
    EXPECT_TRUE(second_result.resized);
    EXPECT_EQ(session.resized_frame_count(), 1U);

    const auto result = session.finalize();
    EXPECT_TRUE(result.succeeded) << result.error_message;
    EXPECT_TRUE(result.wrote_output);
    EXPECT_EQ(result.resized_frame_count, 1U);
}
