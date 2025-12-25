---
sidebar_label: Glossary
sidebar_position: 5
---

# Glossary of ROS 2 Terms

This glossary provides definitions for key terms used throughout the ROS 2 as Robotic Nervous System module.

## A

**Action** - A communication pattern in ROS 2 for long-running tasks that provides feedback, status, and the ability to cancel the task while it's running.

**Action Client** - A node that sends goals to an action server and monitors the progress of the goal execution.

**Action Server** - A node that accepts goals from action clients, executes them, and provides feedback and status updates.

## C

**Client Library (rcl)** - Software libraries that enable programs in various languages to interface with ROS 2 systems (e.g., rclpy for Python, rclcpp for C++).

**Communication Pattern** - The method by which nodes in ROS 2 exchange information, including topics, services, and actions.

## D

**DDS (Data Distribution Service)** - The middleware implementation that underlies ROS 2, providing the communication infrastructure for topics, services, and actions.

**Domain ID** - A number used to separate different ROS 2 networks, allowing multiple isolated ROS 2 systems on the same network.

## J

**Joint** - In URDF, a connection between two links that allows relative motion. Types include fixed, revolute, continuous, prismatic, floating, and planar.

## L

**Link** - In URDF, a rigid component of the robot that has physical properties like mass, inertia, and geometry.

## M

**Middleware** - Software that provides common services and capabilities to applications beyond what's offered by the operating system, in ROS 2's case handling communication between nodes.

**Middleware Nervous System** - A conceptual model where ROS 2 serves as the communication infrastructure that connects different components of a robot system, similar to how a nervous system connects different parts of a biological organism.

## N

**Node** - A basic unit of computation in ROS 2 that performs specific functions within the robot system. Nodes communicate with each other through topics, services, or actions.

## Q

**QoS (Quality of Service)** - A set of policies that define the behavior of ROS 2 communication, including reliability, durability, history, deadline, and liveliness.

## S

**Service** - A synchronous, request-response communication pattern between nodes in ROS 2.

**Service Client** - A node that sends requests to a service server.

**Service Server** - A node that receives requests from service clients and sends back responses.

## T

**Topic** - An asynchronous, many-to-many communication mechanism in ROS 2 where publishers send messages to topics and subscribers receive them.

**TF (Transforms)** - The system in ROS that keeps track of coordinate frames in a robot system and allows transformations between them.

## U

**URDF (Unified Robot Description Format)** - An XML-based format used in ROS to describe robot models, including links, joints, and their properties.

## X

**Xacro** - An XML macro language that extends URDF with features like variables, properties, mathematical expressions, and macro definitions to simplify complex robot descriptions.