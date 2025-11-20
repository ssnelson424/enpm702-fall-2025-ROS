====================================================
ROS 2 Fundamentals and Architecture
====================================================

What is ROS 2?
--------------

ROS 2 (Robot Operating System 2) is a framework for building modular robot software.  
Instead of writing one large monolithic program, ROS 2 encourages you to build many small \emph{nodes} that communicate with each other.

Motivation and Use Cases
------------------------

ROS 2 is widely used in:

- **Autonomous vehicles**

  - Simulation platforms such as CARLA and Autoware-based stacks.
  - Lane keeping, perception, sensor fusion, and behavior planners.

- **Manufacturing**

  - Industrial robot arms (e.g., KUKA, ABB).
  - Collaborative robots in flexible production lines.
  - Automated warehouses and logistics robots.

- **Other domains**

  - Drones and aerial robotics.
  - Maritime autonomous vessels.
  - Rail and transportation automation.

Core Concepts
-------------

- **Node**

  - A process that uses the ROS 2 client library (e.g. ``rclcpp`` in C++).
  - Performs a focused task such as reading a sensor or computing a control command.

- **Topic**

  - A named communication channel.
  - Nodes publish messages to a topic and/or subscribe to messages from it.
  - Typical example: ``/camera/image_raw``, ``/cmd_vel``.

- **Message**

  - A strongly-typed data structure (e.g. ``sensor_msgs/msg/Image``, ``geometry_msgs/msg/Twist``).
  - Defined in ``.msg`` files and compiled into C++ types.

- **Service**

  - Synchronous request/response communication between nodes.
  - Node A sends a request; Node B replies with a response.

- **Parameters**

  - Name/value configurations for nodes (e.g. ``max_speed``, ``frame_id``).
  - Can often be changed at startup or dynamically at runtime.

- **ROS 2 Daemon**

  - A background process that maintains a system-wide graph of nodes and topics.
  - Tools like ``ros2 node list`` and ``ros2 topic list`` consult this daemon.

Communication Layer (DDS)
-------------------------

Under the hood, ROS 2 uses a Data Distribution Service (DDS) implementation to:

- Handle discovery (finding nodes and topics on the network).
- Deliver messages with configurable QoS (Quality of Service) policies.
- Enable communication across different machines and networks when correctly configured.
