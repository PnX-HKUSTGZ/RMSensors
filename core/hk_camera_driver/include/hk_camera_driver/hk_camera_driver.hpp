#ifndef HK_CAMERA_DRIVER__HK_CAMERA_DRIVER_HPP_
#define HK_CAMERA_DRIVER__HK_CAMERA_DRIVER_HPP_

#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "MvCameraControl.h"

#include "hk_camera_driver/hk_camera_driver_parameters.hpp"
#include "hk_camera_driver/parameter_validation.hpp"

namespace hk_camera_driver
{

// 驱动运行状态：
// 1. idle: 仅完成节点基础构造；
// 2. starting: 正在尝试枚举设备并启动取流；
// 3. running: 已成功打开设备并持续发布图像；
// 4. reconnect_wait: 运行期故障后等待下一次重连；
// 5. config_error: 参数错误，已请求全局 shutdown；
// 6. fatal_error: 内部状态异常，无法继续工作；
// 7. stopping: 析构阶段，停止接受新回调。
enum class DriverState
{
    idle,
    starting,
    running,
    reconnect_wait,
    config_error,
    fatal_error,
    stopping
};

// 节点职责：
// 1. 读取 generate_parameter_library 生成的参数；
// 2. 通过 MVS SDK 枚举、打开并持续取流；
// 3. 将图像统一转换成 BGR8 后发布到 ROS 2；
// 4. 运行期断连后自动重连；
// 5. 按需发布相机静态 TF。
class HkCameraDriver : public rclcpp::Node
{
public:
    explicit HkCameraDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~HkCameraDriver() override;

private:
    bool load_parameters(std::string * error_message);
    bool validate_parameters(std::string * error_message) const;
    void update_calibration_cache();
    void initialize_publishers();
    void initialize_timers();
    void publish_static_transform();
    void begin_shutdown() noexcept;
    bool is_shutdown_requested() const noexcept;
    void stop_runtime_timers() noexcept;

    bool initialize_camera(std::string * error_message);
    void attempt_start_camera() noexcept;
    void request_reconnect(const std::string & reason) noexcept;
    void handle_watchdog() noexcept;

    void cleanup_camera() noexcept;
    void cleanup_camera_locked() noexcept;
    void unregister_callbacks(void * camera_handle) noexcept;
    void unregister_callbacks_locked() noexcept;
    void wait_for_callbacks() noexcept;

    void apply_camera_configuration_locked();
    void configure_gige_packet_size_locked(const MV_CC_DEVICE_INFO & device_info);
    void append_warning_locked(const std::string & warning);

    const MV_CC_DEVICE_INFO * select_device(const MV_CC_DEVICE_INFO_LIST & device_list) const;
    bool device_matches_selection(const MV_CC_DEVICE_INFO & device_info) const;
    std::string describe_device(const MV_CC_DEVICE_INFO & device_info) const;
    std::string extract_model_name(const MV_CC_DEVICE_INFO & device_info) const;
    std::string extract_serial_number(const MV_CC_DEVICE_INFO & device_info) const;
    std::string extract_user_defined_name(const MV_CC_DEVICE_INFO & device_info) const;

    void handle_image_callback(
        const unsigned char * data,
        const MV_FRAME_OUT_INFO_EX & frame_info) noexcept;
    void handle_exception_callback(unsigned int message_type) noexcept;
    bool convert_frame_to_bgr_locked(
        const unsigned char * data,
        const MV_FRAME_OUT_INFO_EX & frame_info,
        cv::Mat & bgr_image,
        int * sdk_result = nullptr);
    void publish_frame(const cv::Mat & bgr_image, const rclcpp::Time & stamp);

    bool check_result(int result, const std::string & action, bool warn_only = false) const;
    bool set_enum_value_locked(const char * key, unsigned int value, bool warn_only = false);
    bool set_enum_value_by_string_locked(const char * key, const char * value, bool warn_only = false);
    bool set_float_value_locked(const char * key, float value, bool warn_only = false);
    bool set_int_value_locked(const char * key, int64_t value, bool warn_only = false);

    bool enter_callback() noexcept;
    void leave_callback() noexcept;

    static void __stdcall image_callback(
        unsigned char * data,
        MV_FRAME_OUT_INFO_EX * frame_info,
        void * user) noexcept;
    static void __stdcall exception_callback(unsigned int message_type, void * user) noexcept;

    // 参数监听器和当前参数快照。
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;
    rclcpp::Context::SharedPtr context_;
    rclcpp::PreShutdownCallbackHandle pre_shutdown_callback_handle_;

    // ROS 发布与定时器资源。
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // 相机句柄和驱动状态统一由该锁保护。
    std::mutex camera_mutex_;
    void * camera_handle_{nullptr};
    bool sdk_acquired_{false};
    bool camera_open_{false};
    bool grabbing_{false};
    DriverState state_{DriverState::idle};
    bool camera_ready_{false};
    bool degraded_mode_{false};
    std::string last_error_;
    std::string last_warning_;
    rclcpp::Time last_frame_time_{0, 0, RCL_SYSTEM_TIME};
    std::atomic<bool> reconnect_pending_{false};

    // 回调与析构之间需要显式同步，避免 SDK 回调访问悬空对象。
    std::mutex callback_mutex_;
    std::condition_variable callbacks_drained_cv_;
    std::atomic<uint32_t> active_callbacks_{0};
    std::atomic<bool> callbacks_enabled_{false};
    std::atomic<bool> shutting_down_{false};

    // 去畸变参数缓存，以及复用的 SDK 像素转换输出缓冲区。
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::vector<unsigned char> converted_buffer_;
};

}  // namespace hk_camera_driver

#endif  // HK_CAMERA_DRIVER__HK_CAMERA_DRIVER_HPP_
