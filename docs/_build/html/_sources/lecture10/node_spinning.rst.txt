====================================================
Node Spinning and Execution Model
====================================================

Why Spinning Matters
--------------------

Spinning is how a ROS 2 node processes incoming data and scheduled callbacks.  
Without spinning, subscriptions, timers, and services will never be triggered.

Single-Threaded Spinner
-----------------------

The simplest pattern uses the single-threaded spinner:

.. code-block:: cpp

   rclcpp::spin(node);

- Processes callbacks one at a time.
- Easy to reason about and debug.
- Suitable for many small nodes.

Multi-Threaded Spinner
----------------------

For nodes that must handle many callbacks in parallel (for example, high-rate sensors and multiple services), ROS 2 also offers multi-threaded spinning:

.. code-block:: cpp

   rclcpp::executors::MultiThreadedExecutor executor;
   executor.add_node(node);
   executor.spin();

- Uses multiple threads internally.
- Allows callbacks to run concurrently.
- Requires careful consideration of shared data and thread safety.

Node Lifecycle and Best Practices
---------------------------------

- Always call ``rclcpp::shutdown()`` before exiting ``main``.
- Use RAII and smart pointers for node ownership.
- Keep callbacks small and focused; offload heavy work to helper classes or worker threads when needed.
- Prefer composition: encapsulate functionality inside classes that are used by your node, instead of writing one very large node class.
