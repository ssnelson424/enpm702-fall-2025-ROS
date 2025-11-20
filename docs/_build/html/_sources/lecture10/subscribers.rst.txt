====================================================
Subscribers
====================================================

Creating a Subscriber Node
--------------------------

Basic steps:

1. Create a node.
2. Create a subscription on a topic with a callback.
3. Spin the node to process received messages.

Example Subscriber (C++)
------------------------

.. code-block:: cpp

   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/string.hpp"

   class MinimalSubscriber : public rclcpp::Node
   {
   public:
     MinimalSubscriber()
       : Node("minimal_subscriber")
     {
       subscription_ = this->create_subscription<std_msgs::msg::String>(
         "chatter",
         10,
         std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
     }

   private:
     void topic_callback(const std_msgs::msg::String::SharedPtr msg)
     {
       RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
     }

     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
   };

   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<MinimalSubscriber>());
     rclcpp::shutdown();
     return 0;
   }

Matching Topic Name and Type
----------------------------

- The subscriber must use the **same topic name** as the publisher.
- The subscriber must use the **same message type** as the publisher.

If either of these does not match:

- No messages will be delivered.
- Tools like ``ros2 topic info`` and ``ros2 node info`` can help you debug.

Testing Publisher and Subscriber Together
-----------------------------------------

1. Run the publisher:

   .. code-block:: bash

      ros2 run my_package minimal_publisher

2. In another terminal, run the subscriber:

   .. code-block:: bash

      ros2 run my_package minimal_subscriber

You should see the messages printed by the subscriber.
