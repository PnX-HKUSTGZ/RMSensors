#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "image_file_publisher/image_file_publisher_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<image_file_publisher::ImageFilePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
