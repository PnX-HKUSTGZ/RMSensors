#include "hk_camera_driver/hk_camera_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <mutex>
#include <sstream>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/calib3d.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "CameraParams.h"
#include "MvErrorDefine.h"

namespace hk_camera_driver
{

namespace
{

constexpr unsigned int kTransportLayerMask =
    MV_GIGE_DEVICE |
    MV_USB_DEVICE |
    MV_GENTL_GIGE_DEVICE |
    MV_GENTL_CAMERALINK_DEVICE |
    MV_GENTL_CXP_DEVICE |
    MV_GENTL_XOF_DEVICE;

std::mutex g_sdk_mutex;
size_t g_sdk_reference_count = 0;

struct CleanupResources
{
    void * camera_handle{nullptr};
    bool camera_open{false};
    bool sdk_acquired{false};
};

template<typename CharT, size_t N>
std::string bytes_to_string(const CharT (&data)[N])
{
    const auto * begin = reinterpret_cast<const char *>(data);
    const auto * end = begin;
    const auto * limit = begin + N;
    while (end != limit && *end != '\0') {
        ++end;
    }
    return std::string(begin, end);
}

void set_error_message(std::string * error_message, std::string message)
{
    if (error_message != nullptr) {
        *error_message = std::move(message);
    }
}

bool is_gige_device(const unsigned int device_type)
{
    return device_type == MV_GIGE_DEVICE || device_type == MV_GENTL_GIGE_DEVICE;
}

unsigned int frame_width(const MV_FRAME_OUT_INFO_EX & frame_info)
{
    return frame_info.nExtendWidth > 0 ? frame_info.nExtendWidth : frame_info.nWidth;
}

unsigned int frame_height(const MV_FRAME_OUT_INFO_EX & frame_info)
{
    return frame_info.nExtendHeight > 0 ? frame_info.nExtendHeight : frame_info.nHeight;
}

unsigned int frame_length(const MV_FRAME_OUT_INFO_EX & frame_info)
{
    if (
        frame_info.nFrameLenEx > 0 &&
        frame_info.nFrameLenEx <= std::numeric_limits<unsigned int>::max())
    {
        return static_cast<unsigned int>(frame_info.nFrameLenEx);
    }
    return frame_info.nFrameLen;
}

bool acquire_sdk(const rclcpp::Logger & logger, std::string * error_message)
{
    std::lock_guard<std::mutex> lock(g_sdk_mutex);
    if (g_sdk_reference_count == 0) {
        const int result = MV_CC_Initialize();
        if (result != MV_OK) {
            RCLCPP_ERROR(logger, "MV_CC_Initialize failed with code 0x%x", result);
            set_error_message(error_message, "failed to initialize MVS SDK");
            return false;
        }
    }

    ++g_sdk_reference_count;
    return true;
}

void release_sdk(const rclcpp::Logger & logger) noexcept
{
    std::lock_guard<std::mutex> lock(g_sdk_mutex);
    if (g_sdk_reference_count == 0) {
        return;
    }

    --g_sdk_reference_count;
    if (g_sdk_reference_count > 0) {
        return;
    }

    const int result = MV_CC_Finalize();
    if (result != MV_OK) {
        RCLCPP_WARN(logger, "MV_CC_Finalize failed with code 0x%x", result);
    }
}

bool is_reconnect_required_error(const int result)
{
    switch (static_cast<unsigned int>(result)) {
    case MV_E_HANDLE:
    case MV_E_PRECONDITION:
    case MV_E_NORESPONSE:
    case MV_E_BUSY:
    case MV_E_PACKET:
    case MV_E_NETER:
    case MV_E_USB_READ:
    case MV_E_USB_WRITE:
    case MV_E_USB_DEVICE:
    case MV_E_USB_DRIVER:
        return true;
    default:
        return false;
    }
}

}  // namespace

HkCameraDriver::HkCameraDriver(const rclcpp::NodeOptions & options)
    : Node("hk_camera_driver", options),
      last_frame_time_(0, 0, get_clock()->get_clock_type())
{
    context_ = get_node_base_interface()->get_context();

    std::string error_message;
    if (!load_parameters(&error_message)) {
        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            state_ = DriverState::config_error;
            camera_ready_ = false;
            last_error_ = error_message;
        }

        RCLCPP_FATAL(get_logger(), "parameter validation failed: %s", error_message.c_str());
        rclcpp::shutdown();
        return;
    }

    update_calibration_cache();
    initialize_publishers();
    initialize_timers();

    if (context_ != nullptr) {
        pre_shutdown_callback_handle_ = context_->add_pre_shutdown_callback(
            [this]() noexcept {
                begin_shutdown();
            });
    }

    publish_static_transform();
    attempt_start_camera();
}

HkCameraDriver::~HkCameraDriver()
{
    if (context_ != nullptr) {
        (void)context_->remove_pre_shutdown_callback(pre_shutdown_callback_handle_);
    }

    begin_shutdown();
    cleanup_camera();
    wait_for_callbacks();
}

void HkCameraDriver::begin_shutdown() noexcept
{
    if (shutting_down_.exchange(true, std::memory_order_acq_rel)) {
        return;
    }

    reconnect_pending_.store(false, std::memory_order_release);
    callbacks_enabled_.store(false, std::memory_order_release);
    stop_runtime_timers();

    std::lock_guard<std::mutex> lock(camera_mutex_);
    state_ = DriverState::stopping;
    camera_ready_ = false;
    grabbing_ = false;
}

bool HkCameraDriver::load_parameters(std::string * error_message)
{
    try {
        param_listener_ = std::make_shared<ParamListener>(
            get_node_parameters_interface(), get_logger());
        params_ = param_listener_->get_params();
    } catch (const std::exception & e) {
        set_error_message(
            error_message,
            std::string("failed to load parameters: ") + e.what());
        return false;
    }

    return validate_parameters(error_message);
}

bool HkCameraDriver::validate_parameters(std::string * error_message) const
{
    return hk_camera_driver::validate_parameters(params_, error_message);
}

void HkCameraDriver::update_calibration_cache()
{
    // generate_parameter_library 的 fixed array 会生成 rsl::StaticVector，这里转成标准容器后再填 OpenCV。
    const auto matrix = rsl::to_vector(params_.calibration.camera_matrix);
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (size_t i = 0; i < matrix.size(); ++i) {
        camera_matrix_.at<double>(static_cast<int>(i / 3), static_cast<int>(i % 3)) = matrix[i];
    }

    if (params_.calibration.dist_coeffs.empty()) {
        dist_coeffs_.release();
        return;
    }

    dist_coeffs_ = cv::Mat(
        1,
        static_cast<int>(params_.calibration.dist_coeffs.size()),
        CV_64F);
    for (size_t i = 0; i < params_.calibration.dist_coeffs.size(); ++i) {
        dist_coeffs_.at<double>(0, static_cast<int>(i)) = params_.calibration.dist_coeffs[i];
    }
}

void HkCameraDriver::initialize_publishers()
{
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        params_.publish.image_topic,
        rclcpp::QoS(static_cast<size_t>(params_.publish.queue_size)));
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

void HkCameraDriver::initialize_timers()
{
    const auto reconnect_period =
        std::chrono::milliseconds(params_.reconnect.retry_interval_ms);
    const auto watchdog_period = std::chrono::milliseconds(
        std::max<int64_t>(100, params_.reconnect.frame_timeout_ms / 3));

    // 重连定时器默认关闭，仅在运行期故障后开启。
    reconnect_timer_ = create_wall_timer(
        reconnect_period,
        [this]() {
            if (is_shutdown_requested()) {
                begin_shutdown();
                return;
            }
            attempt_start_camera();
        });
    reconnect_timer_->cancel();

    // 帧看门狗始终运行，但只有 running 状态下才会触发重连。
    watchdog_timer_ = create_wall_timer(
        watchdog_period,
        [this]() {
            if (is_shutdown_requested()) {
                begin_shutdown();
                return;
            }
            handle_watchdog();
        });
}

bool HkCameraDriver::is_shutdown_requested() const noexcept
{
    return shutting_down_.load(std::memory_order_acquire) ||
           context_ == nullptr ||
           !context_->is_valid();
}

void HkCameraDriver::stop_runtime_timers() noexcept
{
    if (reconnect_timer_ != nullptr) {
        reconnect_timer_->cancel();
    }
    if (watchdog_timer_ != nullptr) {
        watchdog_timer_->cancel();
    }
}

void HkCameraDriver::publish_static_transform()
{
    if (!params_.tf.publish_static_tf) {
        return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = params_.tf.parent_frame;
    transform.child_frame_id = params_.tf.child_frame;
    transform.transform.translation.x = params_.tf.translation.x;
    transform.transform.translation.y = params_.tf.translation.y;
    transform.transform.translation.z = params_.tf.translation.z;
    transform.transform.rotation.x = params_.tf.rotation.x;
    transform.transform.rotation.y = params_.tf.rotation.y;
    transform.transform.rotation.z = params_.tf.rotation.z;
    transform.transform.rotation.w = params_.tf.rotation.w;

    static_tf_broadcaster_->sendTransform(transform);
    RCLCPP_INFO(
        get_logger(),
        "published static TF %s -> %s",
        params_.tf.parent_frame.c_str(),
        params_.tf.child_frame.c_str());
}

void HkCameraDriver::attempt_start_camera() noexcept
{
    if (is_shutdown_requested()) {
        begin_shutdown();
        return;
    }

    bool needs_cleanup = false;
    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (
            state_ == DriverState::starting ||
            state_ == DriverState::running ||
            state_ == DriverState::config_error ||
            state_ == DriverState::stopping)
        {
            return;
        }

        state_ = DriverState::starting;
        degraded_mode_ = false;
        last_warning_.clear();
        needs_cleanup =
            reconnect_pending_.exchange(false) ||
            camera_handle_ != nullptr ||
            camera_open_ ||
            grabbing_ ||
            sdk_acquired_;
    }

    // 断连后的资源清理放到 ROS 线程执行，避免在 SDK 回调栈里直接销毁句柄。
    if (needs_cleanup) {
        cleanup_camera();
        wait_for_callbacks();
    }

    std::string error_message;
    try {
        if (!initialize_camera(&error_message)) {
            if (is_shutdown_requested()) {
                begin_shutdown();
                return;
            }

            request_reconnect(
                error_message.empty() ? "failed to initialize camera" : error_message);
            return;
        }

        std::string warning_message;
        bool should_cleanup = false;
        bool should_restart = false;
        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            if (is_shutdown_requested() || state_ == DriverState::stopping) {
                state_ = DriverState::stopping;
                should_cleanup = true;
            } else if (state_ != DriverState::starting || reconnect_pending_.load()) {
                should_restart = true;
            }

            if (!should_cleanup && !should_restart) {
                state_ = DriverState::running;
                last_error_.clear();
                warning_message = last_warning_;
            }
        }

        if (should_cleanup) {
            begin_shutdown();
            cleanup_camera();
            wait_for_callbacks();
            return;
        }

        if (should_restart) {
            cleanup_camera();
            wait_for_callbacks();
            if (reconnect_timer_ != nullptr) {
                reconnect_timer_->reset();
            }
            return;
        }

        // 重连定时器成功后不再 cancel，避免和新的 reset 发生竞态后把重连任务误取消。
        if (warning_message.empty()) {
            RCLCPP_INFO(
                get_logger(),
                "hk_camera_driver started, topic=%s, frame_id=%s, undistort=%s",
                params_.publish.image_topic.c_str(),
                params_.publish.frame_id.c_str(),
                params_.publish.enable_undistort ? "true" : "false");
        } else {
            RCLCPP_WARN(
                get_logger(),
                "hk_camera_driver started with warnings, topic=%s, frame_id=%s, warning=%s",
                params_.publish.image_topic.c_str(),
                params_.publish.frame_id.c_str(),
                warning_message.c_str());
        }
    } catch (const std::exception & e) {
        request_reconnect(std::string("unexpected start error: ") + e.what());
    } catch (...) {
        request_reconnect("unexpected start error: unknown exception");
    }
}

bool HkCameraDriver::initialize_camera(std::string * error_message)
{
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (is_shutdown_requested() || state_ == DriverState::stopping) {
        set_error_message(error_message, "driver is stopping");
        return false;
    }

    if (camera_ready_) {
        return true;
    }

    if (!acquire_sdk(get_logger(), error_message)) {
        return false;
    }
    sdk_acquired_ = true;
    if (is_shutdown_requested() || state_ == DriverState::stopping) {
        set_error_message(error_message, "driver is stopping");
        cleanup_camera_locked();
        return false;
    }

    MV_CC_DEVICE_INFO_LIST device_list;
    std::memset(&device_list, 0, sizeof(device_list));
    if (!check_result(MV_CC_EnumDevices(kTransportLayerMask, &device_list), "MV_CC_EnumDevices")) {
        set_error_message(error_message, "failed to enumerate Hikrobot cameras");
        cleanup_camera_locked();
        return false;
    }
    if (device_list.nDeviceNum == 0) {
        set_error_message(error_message, "no Hikrobot camera was detected");
        cleanup_camera_locked();
        return false;
    }
    if (is_shutdown_requested() || state_ == DriverState::stopping) {
        set_error_message(error_message, "driver is stopping");
        cleanup_camera_locked();
        return false;
    }

    // 启动阶段先把所有可见设备打印出来，便于按索引或序列号定位问题。
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
        if (device_list.pDeviceInfo[i] == nullptr) {
            continue;
        }
        RCLCPP_INFO(
            get_logger(),
            "camera[%u]: %s",
            i,
            describe_device(*device_list.pDeviceInfo[i]).c_str());
    }

    const MV_CC_DEVICE_INFO * selected_device = select_device(device_list);
    if (selected_device == nullptr) {
        std::ostringstream oss;
        if (!params_.device.user_defined_name.empty() || !params_.device.serial_number.empty()) {
            oss << "failed to find camera";
            if (!params_.device.user_defined_name.empty()) {
                oss << " with user_defined_name=" << params_.device.user_defined_name;
            }
            if (!params_.device.serial_number.empty()) {
                oss << " serial_number=" << params_.device.serial_number;
            }
        } else {
            oss << "camera index " << params_.device.index << " is out of range";
        }

        set_error_message(error_message, oss.str());
        cleanup_camera_locked();
        return false;
    }
    if (is_shutdown_requested() || state_ == DriverState::stopping) {
        set_error_message(error_message, "driver is stopping");
        cleanup_camera_locked();
        return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "selected camera: %s",
        describe_device(*selected_device).c_str());

    if (!check_result(
            MV_CC_CreateHandle(&camera_handle_, const_cast<MV_CC_DEVICE_INFO *>(selected_device)),
            "MV_CC_CreateHandle"))
    {
        set_error_message(error_message, "failed to create camera handle");
        cleanup_camera_locked();
        return false;
    }
    if (is_shutdown_requested() || state_ == DriverState::stopping) {
        set_error_message(error_message, "driver is stopping");
        cleanup_camera_locked();
        return false;
    }

    if (!check_result(
            MV_CC_OpenDevice(camera_handle_, MV_ACCESS_Exclusive, 0),
            "MV_CC_OpenDevice"))
    {
        set_error_message(error_message, "failed to open camera");
        cleanup_camera_locked();
        return false;
    }
    camera_open_ = true;
    if (is_shutdown_requested() || state_ == DriverState::stopping) {
        set_error_message(error_message, "driver is stopping");
        cleanup_camera_locked();
        return false;
    }

    // 打开设备后先做链路与曝光配置，再注册回调并开始取流。
    configure_gige_packet_size_locked(*selected_device);
    apply_camera_configuration_locked();
    if (is_shutdown_requested() || state_ == DriverState::stopping) {
        set_error_message(error_message, "driver is stopping");
        cleanup_camera_locked();
        return false;
    }

    // 注册成功前不接受新回调，避免退出时回调计数被反复拉高。
    callbacks_enabled_.store(true, std::memory_order_release);

    if (!check_result(
            MV_CC_RegisterExceptionCallBack(
                camera_handle_,
                &HkCameraDriver::exception_callback,
                this),
            "MV_CC_RegisterExceptionCallBack"))
    {
        set_error_message(error_message, "failed to register exception callback");
        cleanup_camera_locked();
        return false;
    }

    if (!check_result(
            MV_CC_RegisterImageCallBackEx(
                camera_handle_,
                &HkCameraDriver::image_callback,
                this),
            "MV_CC_RegisterImageCallBackEx"))
    {
        set_error_message(error_message, "failed to register image callback");
        cleanup_camera_locked();
        return false;
    }

    if (!check_result(MV_CC_StartGrabbing(camera_handle_), "MV_CC_StartGrabbing")) {
        set_error_message(error_message, "failed to start grabbing");
        cleanup_camera_locked();
        return false;
    }

    grabbing_ = true;
    if (is_shutdown_requested() || state_ == DriverState::stopping) {
        set_error_message(error_message, "driver is stopping");
        return false;
    }
    camera_ready_ = true;
    last_frame_time_ = now();
    return true;
}

void HkCameraDriver::request_reconnect(const std::string & reason) noexcept
{
    if (is_shutdown_requested()) {
        begin_shutdown();
        return;
    }

    try {
        bool should_schedule_timer = false;
        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            if (
                state_ == DriverState::reconnect_wait ||
                state_ == DriverState::config_error ||
                state_ == DriverState::stopping)
            {
                return;
            }

            last_error_ = reason;
            camera_ready_ = false;
            grabbing_ = false;
            callbacks_enabled_.store(false, std::memory_order_release);
            state_ = DriverState::reconnect_wait;
            reconnect_pending_.store(true);
            last_frame_time_ = rclcpp::Time(0, 0, last_frame_time_.get_clock_type());
            should_schedule_timer = true;
        }

        RCLCPP_ERROR(
            get_logger(),
            "camera enters reconnect_wait, cleanup will run on reconnect timer, retry in %ld ms, reason=%s",
            params_.reconnect.retry_interval_ms,
            reason.c_str());

        if (should_schedule_timer && reconnect_timer_ != nullptr) {
            reconnect_timer_->reset();
        }
    } catch (const std::exception & e) {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        state_ = DriverState::fatal_error;
        camera_ready_ = false;
        last_error_ = std::string("failed to schedule reconnect: ") + e.what();
        RCLCPP_FATAL(get_logger(), "%s", last_error_.c_str());
    } catch (...) {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        state_ = DriverState::fatal_error;
        camera_ready_ = false;
        last_error_ = "failed to schedule reconnect: unknown exception";
        RCLCPP_FATAL(get_logger(), "%s", last_error_.c_str());
    }
}

void HkCameraDriver::handle_watchdog() noexcept
{
    if (is_shutdown_requested()) {
        begin_shutdown();
        return;
    }

    try {
        bool should_reconnect = false;
        int64_t elapsed_ms = 0;
        const auto current_time = now();

        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            if (state_ != DriverState::running || !camera_ready_ || !grabbing_) {
                return;
            }

            elapsed_ms = (current_time - last_frame_time_).nanoseconds() / 1000000LL;
            should_reconnect = elapsed_ms >= params_.reconnect.frame_timeout_ms;
        }

        if (should_reconnect) {
            request_reconnect(
                "frame watchdog timeout after " + std::to_string(elapsed_ms) + " ms");
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "watchdog error: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "watchdog error: unknown exception");
    }
}

void HkCameraDriver::cleanup_camera() noexcept
{
    CleanupResources resources;
    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        camera_ready_ = false;
        grabbing_ = false;
        callbacks_enabled_.store(false, std::memory_order_release);
        resources.camera_handle = camera_handle_;
        resources.camera_open = camera_open_;
        resources.sdk_acquired = sdk_acquired_;
        camera_handle_ = nullptr;
        camera_open_ = false;
        sdk_acquired_ = false;
        converted_buffer_.clear();
        last_frame_time_ = rclcpp::Time(0, 0, last_frame_time_.get_clock_type());
    }

    if (resources.camera_handle != nullptr) {
        // 这里已经脱离 camera_mutex_，即使 SDK 停流内部等待回调结束，也不会再和回调尾部争锁。
        check_result(MV_CC_StopGrabbing(resources.camera_handle), "MV_CC_StopGrabbing", true);
        unregister_callbacks(resources.camera_handle);
        wait_for_callbacks();

        if (resources.camera_open) {
            check_result(MV_CC_CloseDevice(resources.camera_handle), "MV_CC_CloseDevice", true);
        }

        check_result(MV_CC_DestroyHandle(resources.camera_handle), "MV_CC_DestroyHandle", true);
    }

    if (resources.sdk_acquired) {
        release_sdk(get_logger());
    }
}

void HkCameraDriver::cleanup_camera_locked() noexcept
{
    // 清理阶段先封住回调入口，避免新回调让等待阶段一直无法归零。
    callbacks_enabled_.store(false, std::memory_order_release);
    camera_ready_ = false;
    grabbing_ = false;

    if (camera_handle_ != nullptr) {
        check_result(MV_CC_StopGrabbing(camera_handle_), "MV_CC_StopGrabbing", true);
        unregister_callbacks_locked();

        if (camera_open_) {
            check_result(MV_CC_CloseDevice(camera_handle_), "MV_CC_CloseDevice", true);
            camera_open_ = false;
        }

        check_result(MV_CC_DestroyHandle(camera_handle_), "MV_CC_DestroyHandle", true);
        camera_handle_ = nullptr;
    }

    if (sdk_acquired_) {
        release_sdk(get_logger());
        sdk_acquired_ = false;
    }

    converted_buffer_.clear();
    last_frame_time_ = rclcpp::Time(0, 0, last_frame_time_.get_clock_type());
}

void HkCameraDriver::unregister_callbacks(void * camera_handle) noexcept
{
    if (camera_handle == nullptr) {
        return;
    }

    check_result(
        MV_CC_RegisterImageCallBackEx(
            camera_handle,
            static_cast<MvImageCallbackEx>(nullptr),
            nullptr),
        "MV_CC_RegisterImageCallBackEx(nullptr)",
        true);
    check_result(
        MV_CC_RegisterExceptionCallBack(
            camera_handle,
            static_cast<MvExceptionCallback>(nullptr),
            nullptr),
        "MV_CC_RegisterExceptionCallBack(nullptr)",
        true);
}

void HkCameraDriver::unregister_callbacks_locked() noexcept
{
    unregister_callbacks(camera_handle_);
}

void HkCameraDriver::wait_for_callbacks() noexcept
{
    std::unique_lock<std::mutex> lock(callback_mutex_);
    if (active_callbacks_.load(std::memory_order_acquire) == 0U) {
        return;
    }

    callbacks_drained_cv_.wait(
        lock,
        [this]() {
            return active_callbacks_.load(std::memory_order_acquire) == 0U;
        });
}

void HkCameraDriver::apply_camera_configuration_locked()
{
    // 当前驱动只保留最核心的可配置项：触发模式、增益、曝光和 Bayer 转换质量。
    if (!set_enum_value_locked("TriggerMode", 0U, true)) {
        append_warning_locked("failed to set TriggerMode=Off");
    }
    if (!set_enum_value_by_string_locked("GainAuto", "Off", true)) {
        append_warning_locked("failed to set GainAuto=Off");
    }
    if (!set_float_value_locked("Gain", static_cast<float>(params_.camera.gain), true)) {
        append_warning_locked("failed to set Gain");
    }

    if (!check_result(
            MV_CC_SetBayerCvtQuality(
                camera_handle_,
                static_cast<unsigned int>(params_.conversion.bayer_quality)),
            "MV_CC_SetBayerCvtQuality",
            true))
    {
        append_warning_locked("failed to set Bayer conversion quality");
    }
    if (!check_result(
            MV_CC_SetBayerFilterEnable(camera_handle_, params_.conversion.bayer_filter_enable),
            "MV_CC_SetBayerFilterEnable",
            true))
    {
        append_warning_locked("failed to set Bayer filter");
    }

    if (params_.camera.exposure_auto) {
        if (!set_enum_value_by_string_locked("ExposureAuto", "Continuous", true)) {
            append_warning_locked("failed to set ExposureAuto=Continuous");
        }
        if (!check_result(
                MV_CC_SetAutoExposureTimeLower(
                    camera_handle_,
                    static_cast<unsigned int>(params_.camera.auto_exposure_lower_us)),
                "MV_CC_SetAutoExposureTimeLower",
                true))
        {
            append_warning_locked("failed to set auto exposure lower bound");
        }
        if (!check_result(
                MV_CC_SetAutoExposureTimeUpper(
                    camera_handle_,
                    static_cast<unsigned int>(params_.camera.auto_exposure_upper_us)),
                "MV_CC_SetAutoExposureTimeUpper",
                true))
        {
            append_warning_locked("failed to set auto exposure upper bound");
        }
    } else {
        if (!set_enum_value_by_string_locked("ExposureAuto", "Off", true)) {
            append_warning_locked("failed to set ExposureAuto=Off");
        }
        if (!set_float_value_locked(
                "ExposureTime",
                static_cast<float>(params_.camera.manual_exposure_us),
                true))
        {
            append_warning_locked("failed to set ExposureTime");
        }
    }
}

void HkCameraDriver::configure_gige_packet_size_locked(const MV_CC_DEVICE_INFO & device_info)
{
    if (!is_gige_device(device_info.nTLayerType) || camera_handle_ == nullptr) {
        return;
    }

    // 允许用户显式指定包大小；未指定时回退到 SDK 推荐值。
    int packet_size = static_cast<int>(params_.device.gige_packet_size);
    if (packet_size <= 0) {
        packet_size = MV_CC_GetOptimalPacketSize(camera_handle_);
        if (packet_size <= 0) {
            RCLCPP_WARN(
                get_logger(),
                "failed to query optimal GigE packet size, SDK returned %d",
                packet_size);
            append_warning_locked("failed to query optimal GigE packet size");
            return;
        }
    }

    if (!set_int_value_locked("GevSCPSPacketSize", packet_size, true)) {
        append_warning_locked("failed to set GigE packet size");
    }
}

void HkCameraDriver::append_warning_locked(const std::string & warning)
{
    degraded_mode_ = true;
    if (last_warning_.empty()) {
        last_warning_ = warning;
        return;
    }

    last_warning_ += "; ";
    last_warning_ += warning;
}

const MV_CC_DEVICE_INFO * HkCameraDriver::select_device(
    const MV_CC_DEVICE_INFO_LIST & device_list) const
{
    // 有名字或序列号时优先按显式标识选择，避免设备顺序变化导致索引漂移。
    if (!params_.device.user_defined_name.empty() || !params_.device.serial_number.empty()) {
        for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
            const auto * device_info = device_list.pDeviceInfo[i];
            if (device_info != nullptr && device_matches_selection(*device_info)) {
                return device_info;
            }
        }
        return nullptr;
    }

    if (
        params_.device.index < 0 ||
        static_cast<unsigned int>(params_.device.index) >= device_list.nDeviceNum)
    {
        return nullptr;
    }

    return device_list.pDeviceInfo[static_cast<size_t>(params_.device.index)];
}

bool HkCameraDriver::device_matches_selection(const MV_CC_DEVICE_INFO & device_info) const
{
    if (
        !params_.device.user_defined_name.empty() &&
        extract_user_defined_name(device_info) != params_.device.user_defined_name)
    {
        return false;
    }

    if (
        !params_.device.serial_number.empty() &&
        extract_serial_number(device_info) != params_.device.serial_number)
    {
        return false;
    }

    return true;
}

std::string HkCameraDriver::describe_device(const MV_CC_DEVICE_INFO & device_info) const
{
    std::ostringstream oss;
    oss << "model=" << extract_model_name(device_info);

    const auto user_defined_name = extract_user_defined_name(device_info);
    if (!user_defined_name.empty()) {
        oss << ", user_defined_name=" << user_defined_name;
    }

    const auto serial_number = extract_serial_number(device_info);
    if (!serial_number.empty()) {
        oss << ", serial_number=" << serial_number;
    }

    if (is_gige_device(device_info.nTLayerType)) {
        const auto & gige_info = device_info.SpecialInfo.stGigEInfo;
        oss << ", current_ip="
            << ((gige_info.nCurrentIp & 0xff000000) >> 24) << "."
            << ((gige_info.nCurrentIp & 0x00ff0000) >> 16) << "."
            << ((gige_info.nCurrentIp & 0x0000ff00) >> 8) << "."
            << (gige_info.nCurrentIp & 0x000000ff);
    }

    return oss.str();
}

std::string HkCameraDriver::extract_model_name(const MV_CC_DEVICE_INFO & device_info) const
{
    switch (device_info.nTLayerType) {
    case MV_GIGE_DEVICE:
    case MV_GENTL_GIGE_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stGigEInfo.chModelName);
    case MV_USB_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stUsb3VInfo.chModelName);
    case MV_GENTL_CAMERALINK_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stCMLInfo.chModelName);
    case MV_GENTL_CXP_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stCXPInfo.chModelName);
    case MV_GENTL_XOF_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stXoFInfo.chModelName);
    default:
        return "unknown";
    }
}

std::string HkCameraDriver::extract_serial_number(const MV_CC_DEVICE_INFO & device_info) const
{
    switch (device_info.nTLayerType) {
    case MV_GIGE_DEVICE:
    case MV_GENTL_GIGE_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stGigEInfo.chSerialNumber);
    case MV_USB_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stUsb3VInfo.chSerialNumber);
    case MV_GENTL_CAMERALINK_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stCMLInfo.chSerialNumber);
    case MV_GENTL_CXP_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stCXPInfo.chSerialNumber);
    case MV_GENTL_XOF_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stXoFInfo.chSerialNumber);
    default:
        return {};
    }
}

std::string HkCameraDriver::extract_user_defined_name(const MV_CC_DEVICE_INFO & device_info) const
{
    switch (device_info.nTLayerType) {
    case MV_GIGE_DEVICE:
    case MV_GENTL_GIGE_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stGigEInfo.chUserDefinedName);
    case MV_USB_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stUsb3VInfo.chUserDefinedName);
    case MV_GENTL_CAMERALINK_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stCMLInfo.chUserDefinedName);
    case MV_GENTL_CXP_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stCXPInfo.chUserDefinedName);
    case MV_GENTL_XOF_DEVICE:
        return bytes_to_string(device_info.SpecialInfo.stXoFInfo.chUserDefinedName);
    default:
        return {};
    }
}

void HkCameraDriver::handle_image_callback(
    const unsigned char * data,
    const MV_FRAME_OUT_INFO_EX & frame_info) noexcept
{
    if (is_shutdown_requested()) {
        return;
    }

    try {
        cv::Mat bgr_image;
        bool should_reconnect = false;
        int sdk_result = MV_OK;

        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            if (camera_handle_ == nullptr || !grabbing_ || !camera_ready_) {
                return;
            }

            // SDK 原始帧统一先转成 BGR8，后续 ROS 发布和 OpenCV 处理都走这一种格式。
            if (!convert_frame_to_bgr_locked(data, frame_info, bgr_image, &sdk_result)) {
                should_reconnect = is_reconnect_required_error(sdk_result);
            }
        }

        if (should_reconnect) {
            request_reconnect("pixel conversion failed because device became unavailable");
            return;
        }
        if (bgr_image.empty()) {
            return;
        }

        // 去畸变放在锁外完成，避免拖慢 SDK 回调线程。
        if (params_.publish.enable_undistort && !dist_coeffs_.empty()) {
            cv::Mat undistorted_image;
            cv::undistort(bgr_image, undistorted_image, camera_matrix_, dist_coeffs_);
            publish_frame(undistorted_image, now());
            return;
        }

        publish_frame(bgr_image, now());
    } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "image callback dropped a frame due to exception: %s",
            e.what());
    } catch (...) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "image callback dropped a frame due to unknown exception");
    }
}

void HkCameraDriver::handle_exception_callback(const unsigned int message_type) noexcept
{
    try {
        if (message_type == MV_EXCEPTION_DEV_DISCONNECT) {
            request_reconnect("camera disconnected from MVS SDK");
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "camera exception callback received message type 0x%x",
            message_type);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "exception callback failed: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "exception callback failed: unknown exception");
    }
}

bool HkCameraDriver::convert_frame_to_bgr_locked(
    const unsigned char * data,
    const MV_FRAME_OUT_INFO_EX & frame_info,
    cv::Mat & bgr_image,
    int * sdk_result)
{
    if (sdk_result != nullptr) {
        *sdk_result = MV_OK;
    }

    const unsigned int width = frame_width(frame_info);
    const unsigned int height = frame_height(frame_info);
    if (width == 0 || height == 0 || data == nullptr) {
        return false;
    }

    const size_t required_size = static_cast<size_t>(width) * static_cast<size_t>(height) * 3U;
    if (converted_buffer_.size() < required_size) {
        // 复用转换缓冲区，避免每帧重复申请释放内存。
        converted_buffer_.resize(required_size);
    }

    MV_CC_PIXEL_CONVERT_PARAM_EX convert_param;
    std::memset(&convert_param, 0, sizeof(convert_param));
    convert_param.nWidth = width;
    convert_param.nHeight = height;
    convert_param.enSrcPixelType = frame_info.enPixelType;
    convert_param.pSrcData = const_cast<unsigned char *>(data);
    convert_param.nSrcDataLen = frame_length(frame_info);
    convert_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    convert_param.pDstBuffer = converted_buffer_.data();
    convert_param.nDstBufferSize = static_cast<unsigned int>(converted_buffer_.size());

    const int result = MV_CC_ConvertPixelTypeEx(camera_handle_, &convert_param);
    if (sdk_result != nullptr) {
        *sdk_result = result;
    }
    if (result != MV_OK) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "MV_CC_ConvertPixelTypeEx failed, code=0x%x, pixel_type=0x%x",
            result,
            static_cast<unsigned int>(frame_info.enPixelType));
        return false;
    }

    cv::Mat converted(
        static_cast<int>(height),
        static_cast<int>(width),
        CV_8UC3,
        converted_buffer_.data());
    converted.copyTo(bgr_image);
    return true;
}

void HkCameraDriver::publish_frame(const cv::Mat & bgr_image, const rclcpp::Time & stamp)
{
    if (image_pub_ == nullptr || bgr_image.empty()) {
        return;
    }
    if (is_shutdown_requested()) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (
            state_ != DriverState::running ||
            !camera_ready_ ||
            !grabbing_ ||
            reconnect_pending_.load(std::memory_order_acquire))
        {
            return;
        }
    }

    auto image_msg = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::BGR8,
        bgr_image).toImageMsg();
    image_msg->header.stamp = stamp;
    image_msg->header.frame_id = params_.publish.frame_id;
    image_pub_->publish(*image_msg);

    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (
            state_ != DriverState::running ||
            !camera_ready_ ||
            !grabbing_ ||
            reconnect_pending_.load(std::memory_order_acquire))
        {
            return;
        }
        last_frame_time_ = stamp;
    }
}

bool HkCameraDriver::check_result(
    const int result,
    const std::string & action,
    const bool warn_only) const
{
    if (result == MV_OK) {
        return true;
    }

    if (warn_only) {
        RCLCPP_WARN(get_logger(), "%s failed with code 0x%x", action.c_str(), result);
    } else {
        RCLCPP_ERROR(get_logger(), "%s failed with code 0x%x", action.c_str(), result);
    }
    return false;
}

bool HkCameraDriver::set_enum_value_locked(
    const char * key,
    const unsigned int value,
    const bool warn_only)
{
    return check_result(MV_CC_SetEnumValue(camera_handle_, key, value), key, warn_only);
}

bool HkCameraDriver::set_enum_value_by_string_locked(
    const char * key,
    const char * value,
    const bool warn_only)
{
    return check_result(MV_CC_SetEnumValueByString(camera_handle_, key, value), key, warn_only);
}

bool HkCameraDriver::set_float_value_locked(
    const char * key,
    const float value,
    const bool warn_only)
{
    return check_result(MV_CC_SetFloatValue(camera_handle_, key, value), key, warn_only);
}

bool HkCameraDriver::set_int_value_locked(
    const char * key,
    const int64_t value,
    const bool warn_only)
{
    return check_result(MV_CC_SetIntValueEx(camera_handle_, key, value), key, warn_only);
}

bool HkCameraDriver::enter_callback() noexcept
{
    if (!callbacks_enabled_.load(std::memory_order_acquire)) {
        return false;
    }

    active_callbacks_.fetch_add(1, std::memory_order_acq_rel);
    if (callbacks_enabled_.load(std::memory_order_acquire)) {
        return true;
    }

    leave_callback();
    return false;
}

void HkCameraDriver::leave_callback() noexcept
{
    const auto previous_count = active_callbacks_.fetch_sub(1, std::memory_order_acq_rel);
    if (previous_count == 1U) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callbacks_drained_cv_.notify_all();
    }
}

void __stdcall HkCameraDriver::image_callback(
    unsigned char * data,
    MV_FRAME_OUT_INFO_EX * frame_info,
    void * user) noexcept
{
    if (data == nullptr || frame_info == nullptr || user == nullptr) {
        return;
    }

    auto * driver = static_cast<HkCameraDriver *>(user);
    if (!driver->enter_callback()) {
        return;
    }

    struct CallbackScopeExit
    {
        HkCameraDriver * driver;

        ~CallbackScopeExit()
        {
            driver->leave_callback();
        }
    } scope_exit{driver};

    if (driver->is_shutdown_requested()) {
        return;
    }

    driver->handle_image_callback(data, *frame_info);
}

void __stdcall HkCameraDriver::exception_callback(
    unsigned int message_type,
    void * user) noexcept
{
    if (user == nullptr) {
        return;
    }

    auto * driver = static_cast<HkCameraDriver *>(user);
    if (!driver->enter_callback()) {
        return;
    }

    struct CallbackScopeExit
    {
        HkCameraDriver * driver;

        ~CallbackScopeExit()
        {
            driver->leave_callback();
        }
    } scope_exit{driver};

    if (driver->is_shutdown_requested()) {
        return;
    }

    driver->handle_exception_callback(message_type);
}

}  // namespace hk_camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(hk_camera_driver::HkCameraDriver)
