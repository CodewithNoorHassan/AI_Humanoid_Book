#!/usr/bin/env python3

"""
Machine Learning Agent Example for ROS 2
This example demonstrates how to integrate ML models with robot controllers
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState
from geometry_msgs.msg import Twist
import numpy as np
import tensorflow as tf


class MLAgentNode(Node):
    def __init__(self):
        super().__init__('ml_agent_node')

        # Create publisher for velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscriptions for sensor data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )

        # Create a timer to run the ML decision loop
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialize variables to store sensor data
        self.latest_scan = None
        self.latest_joints = None

        # Load pre-trained model (assuming it exists)
        # In practice, you would train and save your model first
        try:
            self.model = self.create_simple_model()
            self.get_logger().info('ML Model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.model = None

        self.get_logger().info('ML Agent initialized')

    def create_simple_model(self):
        """
        Create a simple model for demonstration purposes.
        In practice, you would load a pre-trained model.
        """
        model = tf.keras.Sequential([
            tf.keras.layers.Dense(64, activation='relu', input_shape=(360,)),  # Assuming 360 laser readings
            tf.keras.layers.Dense(32, activation='relu'),
            tf.keras.layers.Dense(2)  # 2 outputs: linear.x and angular.z
        ])
        model.compile(optimizer='adam', loss='mse')
        return model

    def scan_callback(self, msg):
        """Callback function to handle incoming laser scan data"""
        # Store the scan ranges, handling invalid values
        ranges = []
        for r in msg.ranges:
            if not (math.isnan(r) or math.isinf(r)):
                ranges.append(min(r, 10.0))  # Cap at 10m
            else:
                ranges.append(10.0)  # Treat as max range if invalid

        self.latest_scan = np.array(ranges)

    def joint_callback(self, msg):
        """Callback function to handle incoming joint state data"""
        self.latest_joints = msg

    def timer_callback(self):
        """Timer callback to make ML-based decisions"""
        if self.latest_scan is not None and len(self.latest_scan) > 0:
            cmd_vel = self.ml_decision_logic()
            if cmd_vel is not None:
                self.publisher.publish(cmd_vel)

    def ml_decision_logic(self):
        """ML-based decision making"""
        if self.model is None:
            # Fallback behavior if model is not available
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd

        try:
            # Preprocess sensor data for the model
            processed_data = self.preprocess_sensor_data(self.latest_scan)

            # Make prediction with the model
            prediction = self.model.predict(processed_data, verbose=0)

            # Convert prediction to Twist command
            cmd = Twist()
            cmd.linear.x = float(prediction[0][0])
            cmd.angular.z = float(prediction[0][1])

            # Apply safety limits
            cmd.linear.x = max(-1.0, min(1.0, cmd.linear.x))  # Limit linear velocity
            cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))  # Limit angular velocity

            return cmd

        except Exception as e:
            self.get_logger().error(f'ML decision error: {e}')
            # Return safe command if prediction fails
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd

    def preprocess_sensor_data(self, raw_data):
        """Preprocess raw sensor data for ML model input"""
        # Normalize the data to [0, 1] range
        max_range = 10.0
        normalized = raw_data / max_range

        # Reshape for model input (batch_size, features)
        reshaped = normalized.reshape(1, -1)

        return reshaped


def main(args=None):
    """Main function to run the ML agent node"""
    rclpy.init(args=args)
    ml_agent = MLAgentNode()

    try:
        rclpy.spin(ml_agent)
    except KeyboardInterrupt:
        print("ML Agent stopped by user")
    finally:
        ml_agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()