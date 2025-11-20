#include "publisher_demo/publisher_demo.hpp"



void PublisherDemo::publish_leia_timer_callback(){
    // Create message
    auto message = std_msgs::msg::String();
    
    // Populate message data
    message.data = "Help me Obi-Wan Kenobi, you're my only hope #" + 
                   std::to_string(count_++);
    
    // Log what we're publishing (optional)
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    
    // Publish the message
    publisher_->publish(message);
}