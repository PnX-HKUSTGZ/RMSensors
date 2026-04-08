#include "image_video_recorder/parameter_validation.hpp"

#include <cmath>
#include <string>

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

bool validate_parameters(const Params & params, std::string * error_message)
{
    if (params.input.image_topic.empty()) {
        set_error_message(error_message, "input.image_topic must not be empty");
        return false;
    }
    if (params.output.directory.empty()) {
        set_error_message(error_message, "output.directory must not be empty");
        return false;
    }
    if (params.output.filename_prefix.empty()) {
        set_error_message(error_message, "output.filename_prefix must not be empty");
        return false;
    }
    if (!std::isfinite(params.recording.fps) || params.recording.fps <= 0.0) {
        set_error_message(error_message, "recording.fps must be finite and greater than zero");
        return false;
    }

    return true;
}

}  // namespace image_video_recorder
