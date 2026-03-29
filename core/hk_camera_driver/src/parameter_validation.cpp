#include "hk_camera_driver/parameter_validation.hpp"

namespace hk_camera_driver
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
    if (params.publish.image_topic.empty()) {
        set_error_message(error_message, "publish.image_topic must not be empty");
        return false;
    }
    if (params.publish.frame_id.empty()) {
        set_error_message(error_message, "publish.frame_id must not be empty");
        return false;
    }
    if (
        params.camera.exposure_auto &&
        params.camera.auto_exposure_upper_us < params.camera.auto_exposure_lower_us)
    {
        set_error_message(
            error_message,
            "camera.auto_exposure_upper_us must be greater than or equal to "
            "camera.auto_exposure_lower_us");
        return false;
    }
    if (!params.camera.exposure_auto && params.camera.manual_exposure_us <= 0.0) {
        set_error_message(error_message, "camera.manual_exposure_us must be positive");
        return false;
    }
    if (params.conversion.bayer_quality > 3) {
        set_error_message(
            error_message,
            "conversion.bayer_quality must be less than or equal to 3");
        return false;
    }
    if (params.tf.publish_static_tf && params.tf.parent_frame.empty()) {
        set_error_message(
            error_message,
            "tf.parent_frame must not be empty when static TF is enabled");
        return false;
    }
    if (params.tf.publish_static_tf && params.tf.child_frame.empty()) {
        set_error_message(
            error_message,
            "tf.child_frame must not be empty when static TF is enabled");
        return false;
    }

    return true;
}

}  // namespace hk_camera_driver
