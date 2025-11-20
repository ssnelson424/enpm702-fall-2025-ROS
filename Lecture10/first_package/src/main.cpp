#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hello");
  rclcpp::spin(node);
//   RCLCPP_INFO_STREAM(node->get_logger(), "Hello from ROS 2");
//   RCLCPP_WARN_STREAM(node->get_logger(), "Hello from ROS 2");
//   RCLCPP_ERROR_STREAM(node->get_logger(), "Hello from ROS 2");
  RCLCPP_FATAL_STREAM(node->get_logger(), "Hello from ROS 2");
  rclcpp::shutdown();
}