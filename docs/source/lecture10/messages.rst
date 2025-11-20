====================================================
ROS 2 Messages
====================================================

Message Types
-------------

ROS 2 messages are strongly-typed structures defined in ``.msg`` files.  
Examples:

- ``std_msgs/msg/String``
- ``geometry_msgs/msg/Twist``
- ``sensor_msgs/msg/LaserScan``

Standard Message Packages
-------------------------

Common message packages you will encounter include:

- ``std_msgs`` – basic types such as strings, integers, and booleans.
- ``geometry_msgs`` – vectors, poses, twists, etc.
- ``sensor_msgs`` – common sensor data types.
- ``nav_msgs`` – navigation-related messages such as ``OccupancyGrid``.

Message Introspection
---------------------

Use the ROS 2 CLI to inspect messages and topics:

- List topics:

  .. code-block:: bash

     ros2 topic list

- Show the type of a topic:

  .. code-block:: bash

     ros2 topic info /cmd_vel

- Inspect a message definition:

  .. code-block:: bash

     ros2 interface show geometry_msgs/msg/Twist

Using Messages in C++
---------------------

Example: publishing a ``geometry_msgs/msg/Twist``:

.. code-block:: cpp

   #include "geometry_msgs/msg/twist.hpp"

   geometry_msgs::msg::Twist cmd;
   cmd.linear.x  = 0.5;
   cmd.angular.z = 0.0;

   publisher_->publish(cmd);

Remember to:

- Include the appropriate message header.
- Add the package to ``ament_target_dependencies`` in ``CMakeLists.txt``.
