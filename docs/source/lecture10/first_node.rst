====================================================
Writing Your First ROS 2 Node
====================================================

Goal
----

Create a minimal C++ node that:

- Initializes the ROS 2 client library.
- Creates a node object.
- Spins until shut down.

Minimal C++ Node
----------------

.. code-block:: cpp

   #include "rclcpp/rclcpp.hpp"

   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);

     auto node = std::make_shared<rclcpp::Node>("minimal_node");

     RCLCPP_INFO(node->get_logger(), "Node has been started.");

     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
   }

Key Points
----------

- ``rclcpp::init``

  - Initializes the ROS 2 system with command-line arguments.

- ``rclcpp::Node``

  - Represents a ROS 2 node.
  - Takes a node name such as ``"minimal_node"``.

- ``rclcpp::spin``

  - Enters a loop that processes callbacks (timers, subscriptions, etc.).
  - Keeps the node alive until interrupted (Ctrl+C).

- ``rclcpp::shutdown``

  - Cleans up ROS 2 resources before exiting.

CMake Integration
-----------------

In the package’s ``CMakeLists.txt``, you typically:

- Find ``rclcpp``.
- Add an executable.
- Link the target against ``rclcpp``.

.. code-block:: cmake

   find_package(rclcpp REQUIRED)

   add_executable(minimal_node src/minimal_node.cpp)
   ament_target_dependencies(minimal_node rclcpp)

   install(TARGETS
     minimal_node
     DESTINATION lib/${PROJECT_NAME})

After building with ``colcon build`` and sourcing the workspace:

.. code-block:: bash

   ros2 run my_package minimal_node
