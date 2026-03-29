#ifndef HK_CAMERA_DRIVER__PARAMETER_VALIDATION_HPP_
#define HK_CAMERA_DRIVER__PARAMETER_VALIDATION_HPP_

#pragma once

#include <string>

#include "hk_camera_driver/hk_camera_driver_parameters.hpp"

namespace hk_camera_driver
{

// 跨字段校验单独抽出来，既方便节点初始化复用，也方便做纯单元测试。
bool validate_parameters(const Params & params, std::string * error_message);

}  // namespace hk_camera_driver

#endif  // HK_CAMERA_DRIVER__PARAMETER_VALIDATION_HPP_
