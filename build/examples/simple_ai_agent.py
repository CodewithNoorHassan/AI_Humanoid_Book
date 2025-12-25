#!/usr/bin/env python3

"""
Simple AI Agent Example for ROS 2
This example demonstrates how to connect a Python-based AI agent to robot controllers
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class SimpleAIAgent(Node):
    def __init__(self):
        super().__init__('simple_ai_agent')

        # Create publisher for velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscription for laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Create a timer to run the AI decision loop
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Store the latest scan data
        self.latest_scan = None

        self.get_logger().info('Simple AI Agent initialized')

    def scan_callback(self, msg):
        """Callback function to handle incoming laser scan data"""
        self.latest_scan = msg

    def timer_callback(self):
        """Timer callback to make AI decisions based on sensor data"""
        if self.latest_scan is not None:
            cmd_vel = self.simple_navigation_logic(self.latest_scan)
            self.publisher.publish(cmd_vel)

    def simple_navigation_logic(self, scan):
        """Simple navigation logic based on laser scan data"""
        cmd = Twist()

        # Find the minimum distance in the front sector (avoiding direct front)
        front_left = min(scan.ranges[0:30])  # First 30 degrees
        front_right = min(scan.ranges[330:360])  # Last 30 degrees
        front_center = min(scan.ranges[350:10])  # Crosses 0 degree boundary

        min_front = min(front_left, front_center, front_right)

        # Simple obstacle avoidance
        if min_front > 1.0:
            # No immediate obstacles, move forward
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
        elif min_front > 0.5:
            # Obstacle detected, slow down and turn
            cmd.linear.x = 0.2
            cmd.angular.z = 0.3
        else:
            # Very close obstacle, stop and turn more aggressively
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8

        return cmd


def main(args=None):
    """Main function to run the AI agent node"""
    rclpy.init(args=args)
    ai_agent = SimpleAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        print("AI Agent stopped by user")
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()