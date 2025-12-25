---
sidebar_label: ROS 2 Communication with Python Agents
sidebar_position: 3
---

# ROS 2 Communication with Python Agents

This chapter explains how to bridge Python-based AI agents to robot controllers using client libraries, focusing on practical examples of AI-to-robot communication patterns.

## Learning Objectives

By the end of this chapter, you will be able to:

- Identify and use appropriate client libraries for ROS 2 integration with Python
- Implement Python-based AI agents that connect to robot controllers
- Apply different communication patterns (topics, services, actions) for AI-robot interaction
- Create practical examples of AI-to-robot communication
- Follow best practices for AI-robot communication in ROS 2
- Troubleshoot common connection issues between AI agents and robots

## Introduction to Client Libraries for ROS 2 Integration

ROS 2 provides client libraries that enable programs written in various languages to interact with the ROS 2 ecosystem. The most commonly used libraries include:

- **rclpy**: Python client library for ROS 2
- **rclcpp**: C++ client library for ROS 2
- **rclnodejs**: Node.js client library for ROS 2
- **rclc**: C client library for microcontrollers

For Python-based AI agents, **rclpy** is the primary choice as it provides a Pythonic interface to ROS 2 concepts and integrates well with the Python data science ecosystem.

## Bridging Python-based AI Agents to Robot Controllers

Connecting AI agents written in Python to robot controllers involves several key steps:

### 1. Setting up the Python Environment

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
```

### 2. Creating a ROS 2 Node for the AI Agent

The AI agent runs as a ROS 2 node that can subscribe to sensor data and publish commands:

```python
class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')
        self.subscription = self.create_subscription(
            JointState,
            'robot/joint_states',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg):
        # Process sensor data and generate AI decision
        ai_decision = self.process_sensor_data(msg)
        self.publisher.publish(ai_decision)

    def process_sensor_data(self, sensor_data):
        # AI logic implementation
        pass
```

### 3. Integrating with AI Frameworks

Python-based AI agents can leverage popular frameworks like TensorFlow, PyTorch, or scikit-learn:

```python
import tensorflow as tf
import numpy as np

class MLAgentNode(Node):
    def __init__(self):
        super().__init__('ml_agent_node')
        # Load pre-trained model
        self.model = tf.keras.models.load_model('robot_control_model.h5')

    def process_sensor_data(self, sensor_data):
        # Preprocess sensor data for the model
        processed_data = self.preprocess(sensor_data)
        # Get prediction from model
        prediction = self.model.predict(processed_data)
        # Convert to ROS message
        return self.convert_to_ros_msg(prediction)
```

## Practical Examples of AI-to-Robot Communication Patterns

### Publisher-Subscriber Pattern for Sensor Data

AI agents typically subscribe to sensor topics to receive information about the robot's state and environment:

```python
# Subscribing to sensor data
self.laser_subscription = self.create_subscription(
    LaserScan,
    'scan',
    self.laser_callback,
    10)

def laser_callback(self, msg):
    # Process laser scan data for AI decision making
    obstacles = self.detect_obstacles(msg)
    self.make_navigation_decision(obstacles)
```

### Service Client Pattern for High-Level Commands

For request-response interactions, AI agents can use services:

```python
from my_robot_msgs.srv import NavigationGoal

def send_navigation_goal(self, x, y, theta):
    # Create a service client
    self.nav_client = self.create_client(NavigationGoal, 'navigation_goal')

    # Wait for service to be available
    while not self.nav_client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('Service not available, waiting again...')

    # Create and send request
    request = NavigationGoal.Request()
    request.target_pose.x = x
    request.target_pose.y = y
    request.target_pose.theta = theta

    # Call the service asynchronously
    future = self.nav_client.call_async(request)
    return future
```

### Action Client Pattern for Long-Running Tasks

For complex tasks with feedback and status, actions are more appropriate:

```python
from rclpy.action import ActionClient
from robot_behavior_msgs.action import ExecuteBehavior

class AIPlannerNode(Node):
    def __init__(self):
        super().__init__('ai_planner_node')
        self._action_client = ActionClient(
            self,
            ExecuteBehavior,
            'execute_behavior'
        )

    def execute_behavior(self, behavior_name):
        goal_msg = ExecuteBehavior.Goal()
        goal_msg.behavior_name = behavior_name

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        # Handle feedback from the behavior execution
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.progress}')
```

## Step-by-Step Tutorial for Basic AI-to-Robot Connection

### Step 1: Install Dependencies

```bash
pip install rclpy
# For AI frameworks
pip install tensorflow pytorch scikit-learn
```

### Step 2: Create the AI Agent Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleAIAgent(Node):
    def __init__(self):
        super().__init__('simple_ai_agent')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg):
        # Store latest scan data
        self.latest_scan = msg

    def timer_callback(self):
        if hasattr(self, 'latest_scan'):
            cmd_vel = self.simple_navigation_logic(self.latest_scan)
            self.publisher.publish(cmd_vel)

    def simple_navigation_logic(self, scan):
        cmd = Twist()
        # Simple obstacle avoidance
        min_distance = min(scan.ranges)
        if min_distance > 1.0:
            cmd.linear.x = 0.5  # Move forward
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0  # Stop
            cmd.angular.z = 0.5  # Turn
        return cmd

def main(args=None):
    rclpy.init(args=args)
    ai_agent = SimpleAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Launch the AI Agent

Create a launch file to run the AI agent:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ai_agent',
            executable='simple_ai_agent',
            name='simple_ai_agent',
            output='screen'
        )
    ])
```

## Best Practices for AI-Robot Communication

### 1. Asynchronous Processing
Handle ROS 2 callbacks asynchronously to avoid blocking the AI computation:

```python
import asyncio

class AsyncAIAgent(Node):
    def __init__(self):
        super().__init__('async_ai_agent')
        self.ai_loop = asyncio.new_event_loop()

    def sensor_callback(self, msg):
        # Offload AI processing to a separate thread
        self.ai_loop.run_in_executor(None, self.process_with_ai, msg)
```

### 2. Data Preprocessing
Ensure sensor data is properly preprocessed before feeding it to AI models:

```python
def preprocess_sensor_data(self, raw_data):
    # Normalize data
    normalized = (raw_data - self.mean) / self.std
    # Reshape for model input
    reshaped = normalized.reshape(1, -1)
    return reshaped
```

### 3. Error Handling
Implement robust error handling for both ROS 2 communication and AI inference:

```python
def safe_ai_inference(self, sensor_data):
    try:
        # Validate input data
        if not self.validate_input(sensor_data):
            return self.fallback_behavior()

        # Perform AI inference
        result = self.ai_model.predict(sensor_data)
        return self.convert_to_robot_command(result)

    except Exception as e:
        self.get_logger().error(f'AI inference error: {e}')
        return self.safety_fallback()
```

## Troubleshooting Common Connection Issues

### Topic Connection Issues
- **Symptom**: No data received from topics
- **Solution**: Check topic names with `ros2 topic list` and verify correct namespace usage

### Performance Issues
- **Symptom**: Slow response from AI agent
- **Solution**: Profile the AI inference and optimize the model or use a more efficient implementation

### Network Configuration
- **Symptom**: Nodes on different machines cannot communicate
- **Solution**: Ensure proper DDS configuration and network settings (ROS_DOMAIN_ID, RMW implementation)

## Hands-on Exercise

1. **Environment Setup**: Set up a Python environment with rclpy installed. If you don't have ROS 2 installed, create a virtual environment and install a mock library to simulate the experience.

2. **Simple Node Creation**: Create a simple Python ROS 2 node that publishes a "hello world" message to a topic every second. Use the node creation pattern shown in this chapter.

3. **AI Integration Practice**: Modify the simple node to simulate an AI decision by randomly choosing between two commands (e.g., "move forward" or "turn left") and publishing the decision.

## Summary

This chapter covered the essential concepts for connecting Python-based AI agents to robot controllers using ROS 2. You learned about client libraries, communication patterns, practical implementation examples, and best practices. In the previous chapter, [ROS 2 Fundamentals](./ros2-fundamentals), you learned about the core concepts of ROS 2 architecture. In the next chapter, [Humanoid Modeling with URDF](./urdf-modeling), you'll explore robot structure definition using URDF.