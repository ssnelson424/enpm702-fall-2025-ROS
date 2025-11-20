#include "subscriber_demo/subscriber_demo.hpp"



void SubscriberDemo::leia_callback(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    
    // Process the message as needed:
    // - Store data for later use
    // - Trigger other actions
    // - Update internal state
    // - Publish transformed data
}