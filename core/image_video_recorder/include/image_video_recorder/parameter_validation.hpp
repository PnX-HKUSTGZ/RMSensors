#ifndef IMAGE_VIDEO_RECORDER__PARAMETER_VALIDATION_HPP_
#define IMAGE_VIDEO_RECORDER__PARAMETER_VALIDATION_HPP_

#pragma once

#include <string>

#include "image_video_recorder/image_video_recorder_parameters.hpp"

namespace image_video_recorder
{

bool validate_parameters(const Params & params, std::string * error_message);

}  // namespace image_video_recorder

#endif  // IMAGE_VIDEO_RECORDER__PARAMETER_VALIDATION_HPP_
