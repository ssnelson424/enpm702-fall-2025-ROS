#pragma once

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class PublisherDemo : public rclcpp::Node {
 public:
  explicit PublisherDemo(const std::string& node_name)
      : Node(node_name), count_{0} {
    // Create publisher with message type, topic name, and QoS depth
    publisher_ = this->create_publisher<std_msgs::msg::String>("leia", 10);

    // Create timer for periodic publishing (2Hz = 500ms)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PublisherDemo::publish_leia_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Publisher initialized");
  }

 private:
  void publish_leia_timer_callback();

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};