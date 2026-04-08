#include "image_video_recorder/recording_session.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace image_video_recorder
{

namespace
{

std::string format_timestamp(
    const std::chrono::system_clock::time_point & time_point,
    std::string * error_message)
{
    const auto set_error_message = [error_message](const std::string & message) {
        if (error_message != nullptr) {
            *error_message = message;
        }
    };

    const std::time_t raw_time = std::chrono::system_clock::to_time_t(time_point);
    std::tm local_time{};
    if (localtime_r(&raw_time, &local_time) == nullptr) {
        set_error_message("failed to convert start time to local time");
        return {};
    }

    std::ostringstream stream;
    stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return stream.str();
}

void remove_partial_output(const std::filesystem::path & output_path)
{
    std::error_code error_code;
    if (std::filesystem::exists(output_path, error_code)) {
        std::filesystem::remove(output_path, error_code);
    }
}

}  // namespace

RecordingSession::RecordingSession(RecordingSessionOptions options)
    : options_(std::move(options))
{
}

bool RecordingSession::initialize(std::string * error_message)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_) {
        return true;
    }

    if (finalized_ || finalizing_) {
        set_error_message(error_message, "recording session is already closing");
        return false;
    }

    if (!ensure_directory_exists(options_.output_directory, error_message)) {
        return false;
    }

    std::filesystem::path output_path = build_output_path(
        options_.output_directory,
        options_.filename_prefix,
        options_.start_time,
        error_message);
    if (output_path.empty()) {
        return false;
    }

    std::filesystem::path temp_root = options_.temp_root;
    if (temp_root.empty()) {
        try {
            temp_root = std::filesystem::temp_directory_path();
        } catch (const std::filesystem::filesystem_error & e) {
            set_error_message(error_message, e.what());
            return false;
        }
    }

    std::filesystem::path temp_directory;
    if (!create_unique_temp_directory(temp_root, &temp_directory, error_message)) {
        return false;
    }

    output_path_ = std::move(output_path);
    temp_directory_ = std::move(temp_directory);
    initialized_ = true;
    return true;
}

AppendFrameResult RecordingSession::append_frame(const cv::Mat & bgr_frame)
{
    std::lock_guard<std::mutex> lock(mutex_);

    AppendFrameResult result;
    result.frame_count = frame_paths_.size();

    if (!initialized_) {
        result.error_message = "recording session is not initialized";
        return result;
    }

    if (finalizing_ || finalized_) {
        result.error_message = "recording session is closing";
        return result;
    }

    if (bgr_frame.empty()) {
        result.error_message = "input frame is empty";
        return result;
    }

    if (bgr_frame.type() != CV_8UC3) {
        result.error_message = "input frame must be BGR8";
        return result;
    }

    cv::Mat frame_to_write = bgr_frame;
    if (frame_paths_.empty()) {
        frame_size_ = bgr_frame.size();
    } else if (bgr_frame.size() != frame_size_) {
        cv::resize(bgr_frame, frame_to_write, frame_size_, 0.0, 0.0, cv::INTER_LINEAR);
        result.resized = true;
        ++resized_frame_count_;
    }

    const std::filesystem::path frame_path = build_frame_path(temp_directory_, frame_paths_.size() + 1U);
    std::string error_message;
    if (!write_frame_png(frame_to_write, frame_path, &error_message)) {
        result.error_message = error_message;
        return result;
    }

    frame_paths_.push_back(frame_path);
    result.accepted = true;
    result.frame_count = frame_paths_.size();
    return result;
}

FinalizeResult RecordingSession::finalize()
{
    FinalizeResult result;
    std::vector<std::filesystem::path> frame_paths;
    std::filesystem::path output_path;
    std::filesystem::path temp_directory;
    cv::Size frame_size;
    double fps{0.0};

    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (finalized_) {
            return final_result_;
        }

        result.finalize_invoked = true;
        result.frame_count = frame_paths_.size();
        result.resized_frame_count = resized_frame_count_;
        result.output_path = output_path_;
        result.temp_directory = temp_directory_;

        if (!initialized_) {
            result.error_message = "recording session was not initialized";
            final_result_ = result;
            finalized_ = true;
            return final_result_;
        }

        finalizing_ = true;
        frame_paths = frame_paths_;
        output_path = output_path_;
        temp_directory = temp_directory_;
        frame_size = frame_size_;
        fps = options_.fps;
    }

    if (frame_paths.empty()) {
        std::error_code error_code;
        std::filesystem::remove_all(temp_directory, error_code);
        if (error_code) {
            result.error_message = error_code.message();
        } else {
            result.succeeded = true;
        }
    } else {
        std::error_code error_code;
        if (std::filesystem::exists(output_path, error_code)) {
            std::filesystem::remove(output_path, error_code);
            if (error_code) {
                result.error_message = error_code.message();
            }
        }

        if (result.error_message.empty()) {
            cv::VideoWriter writer(
                output_path.string(),
                cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                fps,
                frame_size);

            if (!writer.isOpened()) {
                result.error_message = "failed to open MP4 output writer";
            } else {
                for (const auto & frame_path : frame_paths) {
                    cv::Mat frame = cv::imread(frame_path.string(), cv::IMREAD_COLOR);
                    if (frame.empty()) {
                        result.error_message = "failed to read temporary frame '" + frame_path.string() + "'";
                        break;
                    }
                    writer.write(frame);
                }
                writer.release();
            }
        }

        if (result.error_message.empty()) {
            std::error_code file_error;
            if (!std::filesystem::exists(output_path, file_error)) {
                result.error_message = "MP4 output file was not created";
            } else if (file_error) {
                result.error_message = file_error.message();
            } else if (std::filesystem::file_size(output_path, file_error) == 0U) {
                result.error_message = file_error ? file_error.message() : "MP4 output file is empty";
            }
        }

        if (result.error_message.empty()) {
            std::error_code cleanup_error;
            std::filesystem::remove_all(temp_directory, cleanup_error);
            if (cleanup_error) {
                result.error_message = cleanup_error.message();
                result.wrote_output = true;
            } else {
                result.succeeded = true;
                result.wrote_output = true;
            }
        } else {
            remove_partial_output(output_path);
        }
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        finalizing_ = false;
        finalized_ = true;
        final_result_ = result;
    }

    return result;
}

std::filesystem::path RecordingSession::output_path() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return output_path_;
}

std::filesystem::path RecordingSession::temp_directory() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return temp_directory_;
}

std::size_t RecordingSession::frame_count() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return frame_paths_.size();
}

std::size_t RecordingSession::resized_frame_count() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return resized_frame_count_;
}

std::filesystem::path RecordingSession::build_output_path(
    const std::filesystem::path & output_directory,
    const std::string & filename_prefix,
    const std::chrono::system_clock::time_point & start_time,
    std::string * error_message)
{
    const std::string timestamp = format_timestamp(start_time, error_message);
    if (timestamp.empty()) {
        return {};
    }

    return output_directory / (filename_prefix + "_" + timestamp + ".mp4");
}

bool RecordingSession::ensure_directory_exists(
    const std::filesystem::path & directory_path,
    std::string * error_message)
{
    if (directory_path.empty()) {
        set_error_message(error_message, "directory path must not be empty");
        return false;
    }

    try {
        if (std::filesystem::exists(directory_path)) {
            if (!std::filesystem::is_directory(directory_path)) {
                set_error_message(
                    error_message,
                    "'" + directory_path.string() + "' exists but is not a directory");
                return false;
            }
            return true;
        }

        std::filesystem::create_directories(directory_path);
        return true;
    } catch (const std::filesystem::filesystem_error & e) {
        set_error_message(error_message, e.what());
        return false;
    }
}

bool RecordingSession::create_unique_temp_directory(
    const std::filesystem::path & base_directory,
    std::filesystem::path * temp_directory,
    std::string * error_message)
{
    if (!ensure_directory_exists(base_directory, error_message)) {
        return false;
    }

    std::string template_path = (base_directory / "image_video_recorder_XXXXXX").string();
    std::vector<char> writable_path(template_path.begin(), template_path.end());
    writable_path.push_back('\0');

    errno = 0;
    char * created_path = mkdtemp(writable_path.data());
    if (created_path == nullptr) {
        set_error_message(
            error_message,
            "failed to create temporary directory: " + std::string(std::strerror(errno)));
        return false;
    }

    *temp_directory = created_path;
    return true;
}

std::filesystem::path RecordingSession::build_frame_path(
    const std::filesystem::path & temp_directory,
    std::size_t frame_index)
{
    std::ostringstream file_name;
    file_name << "frame_" << std::setfill('0') << std::setw(6) << frame_index << ".png";
    return temp_directory / file_name.str();
}

void RecordingSession::set_error_message(std::string * error_message, const std::string & message)
{
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool RecordingSession::write_frame_png(
    const cv::Mat & bgr_frame,
    const std::filesystem::path & frame_path,
    std::string * error_message) const
{
    try {
        if (!cv::imwrite(frame_path.string(), bgr_frame)) {
            set_error_message(
                error_message,
                "failed to write temporary frame '" + frame_path.string() + "'");
            return false;
        }
    } catch (const cv::Exception & e) {
        set_error_message(error_message, e.what());
        return false;
    }

    return true;
}

}  // namespace image_video_recorder
