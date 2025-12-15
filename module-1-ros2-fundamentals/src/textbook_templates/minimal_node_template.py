#!/usr/bin/env python3
"""
Minimal ROS 2 Node Template
Physical AI & Humanoid Robotics Textbook - Module 1

Purpose: Reusable template for creating ROS 2 nodes with best practices
Features: Publisher, Subscriber, Exception handling, Graceful shutdown

Usage:
1. Copy this template to your package
2. Rename class and customize __init__ parameters
3. Implement your logic in timer_callback() or topic_callback()
4. Update package.xml with dependencies
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with your message type


class MinimalNodeTemplate(Node):
    """
    Template for ROS 2 nodes using rclpy.

    This class demonstrates:
    - Node subclass pattern (ROS 2 best practice)
    - Publisher creation with specified QoS
    - Subscriber creation with callback
    - Timer for periodic publishing
    - Proper initialization and cleanup
    """

    def __init__(self):
        """
        Initialize the node with publishers, subscribers, and timers.
        """
        super().__init__('minimal_node_template')  # Node name - change this!

        # Publisher: Sends messages to a topic
        self.publisher_ = self.create_publisher(
            String,           # Message type
            'example_topic',  # Topic name - change this!
            10               # Queue size (QoS depth)
        )

        # Subscriber: Receives messages from a topic
        self.subscription = self.create_subscription(
            String,                  # Message type
            'input_topic',           # Topic name - change this!
            self.topic_callback,     # Callback function
            10                      # Queue size (QoS depth)
        )
        self.subscription  # Prevent unused variable warning

        # Timer: Triggers periodic callbacks (publishes at 1 Hz here)
        timer_period = 1.0  # seconds - change to your desired frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for example purposes
        self.counter = 0

        self.get_logger().info('Minimal Node Template initialized!')

    def timer_callback(self):
        """
        Called periodically by the timer.
        Use this for periodic publishing (e.g., sensor readings, status updates).
        """
        msg = String()
        msg.data = f'Hello, ROS 2! Counter: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

    def topic_callback(self, msg):
        """
        Called when a message is received on the subscribed topic.

        Args:
            msg: The received message (type matches subscription)
        """
        self.get_logger().info(f'Received: "{msg.data}"')
        # Process the message here (e.g., control logic, data processing)


def main(args=None):
    """
    Main entry point for the node.
    Handles initialization, spinning, and graceful shutdown.
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create node instance
    node = MinimalNodeTemplate()

    try:
        # Spin the node (keeps it running and processing callbacks)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info('Keyboard interrupt detected, shutting down...')
    except Exception as e:
        # Handle unexpected errors
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        # Cleanup: Destroy node and shutdown ROS 2
        node.destroy_node()
        rclpy.shutdown()
        print('Node shut down successfully.')


if __name__ == '__main__':
    main()
