#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SubscriberDemo : public rclcpp::Node {
 public:
  explicit SubscriberDemo(const std::string& node_name)
      : Node(node_name) {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "leia",  // Topic name
        10,      // QoS queue depth
        std::bind(&SubscriberDemo::leia_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Subscriber initialized, listening to 'leia'");
  }

 private:
  void leia_callback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};