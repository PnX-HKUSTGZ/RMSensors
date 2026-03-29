#include "image_file_publisher/parameter_validation.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>

namespace image_file_publisher
{

namespace
{

using ImagePathList = std::vector<std::filesystem::path>;

constexpr char kSourceTypeAuto[] = "auto";
constexpr char kSourceTypeImage[] = "image";
constexpr char kSourceTypeDirectory[] = "directory";
constexpr char kSourceTypeVideo[] = "video";

const std::unordered_set<std::string> kImageExtensions = {
    ".bmp", ".dib", ".jpe", ".jpeg", ".jpg", ".jp2", ".png", ".pbm",
    ".pgm", ".ppm", ".ras", ".sr", ".tif", ".tiff", ".webp"};

const std::unordered_set<std::string> kVideoExtensions = {
    ".avi", ".flv", ".m4v", ".mkv", ".mov", ".mp4", ".mpeg",
    ".mpg", ".mts", ".ts", ".webm", ".wmv"};

void set_error_message(std::string * error_message, const std::string & message)
{
    if (error_message != nullptr) {
        *error_message = message;
    }
}

std::string to_lower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool has_extension(
    const std::filesystem::path & path,
    const std::unordered_set<std::string> & extensions)
{
    return extensions.count(to_lower(path.extension().string())) > 0;
}

bool collect_directory_images(
    const std::filesystem::path & directory_path,
    ImagePathList * image_paths,
    std::string * error_message)
{
    image_paths->clear();

    try {
        for (const auto & entry : std::filesystem::directory_iterator(directory_path)) {
            if (!entry.is_regular_file()) {
                continue;
            }
            if (has_extension(entry.path(), kImageExtensions)) {
                image_paths->push_back(entry.path());
            }
        }
    } catch (const std::filesystem::filesystem_error & e) {
        set_error_message(error_message, e.what());
        return false;
    }

    std::sort(image_paths->begin(), image_paths->end());
    if (image_paths->empty()) {
        set_error_message(
            error_message,
            "source.path directory does not contain any supported image files");
        return false;
    }

    return true;
}

bool validate_source_path(
    const std::filesystem::path & source_path,
    const std::string & source_type,
    std::string * error_message)
{
    try {
        if (!std::filesystem::exists(source_path)) {
            set_error_message(error_message, "source.path does not exist");
            return false;
        }

        if (source_type == kSourceTypeAuto) {
            if (std::filesystem::is_directory(source_path)) {
                ImagePathList image_paths;
                return collect_directory_images(source_path, &image_paths, error_message);
            }

            if (!std::filesystem::is_regular_file(source_path)) {
                set_error_message(
                    error_message,
                    "source.path must be a regular file or directory");
                return false;
            }

            if (
                has_extension(source_path, kImageExtensions) ||
                has_extension(source_path, kVideoExtensions))
            {
                return true;
            }

            set_error_message(
                error_message,
                "source.path file extension is not a supported image or video type");
            return false;
        }

        if (source_type == kSourceTypeDirectory) {
            if (!std::filesystem::is_directory(source_path)) {
                set_error_message(
                    error_message,
                    "source.type is directory but source.path is not a directory");
                return false;
            }

            ImagePathList image_paths;
            return collect_directory_images(source_path, &image_paths, error_message);
        }

        if (!std::filesystem::is_regular_file(source_path)) {
            set_error_message(
                error_message,
                "source.path must be a regular file for image or video sources");
            return false;
        }

        if (source_type == kSourceTypeImage) {
            if (!has_extension(source_path, kImageExtensions)) {
                set_error_message(
                    error_message,
                    "source.type is image but source.path does not use a supported image extension");
                return false;
            }
            return true;
        }

        if (source_type == kSourceTypeVideo) {
            if (!has_extension(source_path, kVideoExtensions)) {
                set_error_message(
                    error_message,
                    "source.type is video but source.path does not use a supported video extension");
                return false;
            }
            return true;
        }
    } catch (const std::filesystem::filesystem_error & e) {
        set_error_message(error_message, e.what());
        return false;
    }

    set_error_message(error_message, "source.type must be one of auto/image/directory/video");
    return false;
}

}  // namespace

bool validate_parameters(const Params & params, std::string * error_message)
{
    if (params.source.path.empty()) {
        set_error_message(error_message, "source.path must not be empty");
        return false;
    }
    if (params.publish.image_topic.empty()) {
        set_error_message(error_message, "publish.image_topic must not be empty");
        return false;
    }
    if (params.publish.frame_id.empty()) {
        set_error_message(error_message, "publish.frame_id must not be empty");
        return false;
    }
    if (!std::isfinite(params.playback.rate_hz)) {
        set_error_message(error_message, "playback.rate_hz must be finite");
        return false;
    }

    const std::string source_type = to_lower(params.source.type);
    return validate_source_path(std::filesystem::path(params.source.path), source_type, error_message);
}

}  // namespace image_file_publisher
