#ifndef IMAGE_VIDEO_RECORDER__RECORDING_SESSION_HPP_
#define IMAGE_VIDEO_RECORDER__RECORDING_SESSION_HPP_

#pragma once

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace image_video_recorder
{

struct RecordingSessionOptions
{
    std::filesystem::path output_directory;
    std::string filename_prefix;
    double fps{30.0};
    std::filesystem::path temp_root;
    std::chrono::system_clock::time_point start_time{std::chrono::system_clock::now()};
};

struct AppendFrameResult
{
    bool accepted{false};
    bool resized{false};
    std::size_t frame_count{0};
    std::string error_message;
};

struct FinalizeResult
{
    bool finalize_invoked{false};
    bool succeeded{false};
    bool wrote_output{false};
    std::size_t frame_count{0};
    std::size_t resized_frame_count{0};
    std::filesystem::path output_path;
    std::filesystem::path temp_directory;
    std::string error_message;
};

class RecordingSession
{
public:
    explicit RecordingSession(RecordingSessionOptions options);

    bool initialize(std::string * error_message);
    AppendFrameResult append_frame(const cv::Mat & bgr_frame);
    FinalizeResult finalize();

    std::filesystem::path output_path() const;
    std::filesystem::path temp_directory() const;
    std::size_t frame_count() const;
    std::size_t resized_frame_count() const;

    static std::filesystem::path build_output_path(
        const std::filesystem::path & output_directory,
        const std::string & filename_prefix,
        const std::chrono::system_clock::time_point & start_time,
        std::string * error_message);

private:
    static bool ensure_directory_exists(
        const std::filesystem::path & directory_path,
        std::string * error_message);
    static bool create_unique_temp_directory(
        const std::filesystem::path & base_directory,
        std::filesystem::path * temp_directory,
        std::string * error_message);
    static std::filesystem::path build_frame_path(
        const std::filesystem::path & temp_directory,
        std::size_t frame_index);
    static void set_error_message(std::string * error_message, const std::string & message);

    bool write_frame_png(
        const cv::Mat & bgr_frame,
        const std::filesystem::path & frame_path,
        std::string * error_message) const;

    mutable std::mutex mutex_;
    RecordingSessionOptions options_;
    std::filesystem::path output_path_;
    std::filesystem::path temp_directory_;
    std::vector<std::filesystem::path> frame_paths_;
    cv::Size frame_size_;
    bool initialized_{false};
    bool finalizing_{false};
    bool finalized_{false};
    std::size_t resized_frame_count_{0};
    FinalizeResult final_result_;
};

}  // namespace image_video_recorder

#endif  // IMAGE_VIDEO_RECORDER__RECORDING_SESSION_HPP_
