#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "image_video_recorder/image_video_recorder_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<image_video_recorder::ImageVideoRecorderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
