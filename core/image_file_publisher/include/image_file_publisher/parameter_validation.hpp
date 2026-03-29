#ifndef IMAGE_FILE_PUBLISHER__PARAMETER_VALIDATION_HPP_
#define IMAGE_FILE_PUBLISHER__PARAMETER_VALIDATION_HPP_

#pragma once

#include <string>

#include "image_file_publisher/image_file_publisher_parameters.hpp"

namespace image_file_publisher
{

bool validate_parameters(const Params & params, std::string * error_message);

}  // namespace image_file_publisher

#endif  // IMAGE_FILE_PUBLISHER__PARAMETER_VALIDATION_HPP_
