---
sidebar_label: ROS 2 Fundamentals
sidebar_position: 2
---

# ROS 2 Fundamentals

This chapter introduces the fundamental concepts of ROS 2 as a middleware nervous system that connects AI decision-making to physical robot control.

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain the role of middleware in robotics and how ROS 2 serves as a communication layer
- Compare ROS 2 architecture with traditional software architectures
- Describe the function of nodes as autonomous computational units
- Understand the publisher-subscriber pattern for topic-based communication
- Explain the request-response pattern for service-based communication
- Describe how ROS 2 enables real-time, distributed robot systems

## Conceptual Overview of Middleware in Robotics

Middleware in robotics serves as the communication layer that enables different software components to interact seamlessly. ROS 2, as a middleware framework, provides:

- **Communication abstraction**: Hides the complexity of network protocols and data serialization
- **Process isolation**: Allows components to run independently and potentially on different machines
- **Language independence**: Supports multiple programming languages through client libraries
- **Distributed computing**: Enables systems to span multiple computers and devices

## ROS 2 vs Traditional Software Architectures

Traditional software architectures often use monolithic or simple client-server patterns. ROS 2 introduces a more sophisticated distributed architecture:

- **Decentralized**: No single point of failure or control
- **Peer-to-peer**: Nodes communicate directly without a central broker (in DDS-based implementation)
- **Data-centric**: Communication is based on topics and data types rather than direct function calls
- **Real-time capable**: Designed to meet timing constraints for robot control

## Nodes as Autonomous Functional Units

In ROS 2, a node is a fundamental unit of computation that:

- Performs a specific function within the robot system
- Communicates with other nodes through topics, services, or actions
- Can be written in any supported language (C++, Python, etc.)
- Runs as a separate process that can be started or stopped independently

### Creating a Node

A basic node structure includes:
- Node initialization and destruction
- Subscription and publication setup
- Callback functions for handling messages
- Spin loop to process callbacks

## Topics and Message-Based Communication

Topics enable asynchronous, many-to-many communication between nodes:

- **Publisher-subscriber pattern**: Publishers send messages to topics, subscribers receive them
- **Anonymous publishing**: Publishers don't know who subscribes
- **Queuing**: Messages are queued for delivery to subscribers
- **Message types**: Strongly typed messages defined in `.msg` files

### Topic Characteristics

- **Asynchronous**: Publishers and subscribers don't need to run simultaneously
- **Unidirectional**: Data flows from publisher to subscriber
- **Many-to-many**: Multiple publishers and subscribers can use the same topic
- **Data-driven**: Communication is triggered by data availability

## Services and Request-Response Patterns

Services provide synchronous, request-response communication:

- **Client-server pattern**: Client sends request, server responds
- **Synchronous**: Client waits for response before continuing
- **One-to-one**: Typically one server serves one client at a time
- **Request-response types**: Strongly typed requests and responses defined in `.srv` files

### Service Characteristics

- **Synchronous**: Client blocks until response received
- **Bidirectional**: Request goes one way, response returns
- **One-to-one**: Each request gets a dedicated response
- **Stateless**: Each request-response pair is independent

## How ROS 2 Enables Real-Time, Distributed Robot Systems

ROS 2 is designed to support real-time and distributed robotics applications through:

- **DDS (Data Distribution Service) abstraction**: Provides quality of service controls
- **Quality of Service (QoS) settings**: Configure reliability, durability, and performance
- **Multi-machine deployment**: Nodes can run on different computers
- **Language interoperability**: Different nodes can be written in different languages

### Quality of Service Features

- **Reliability**: Reliable (guaranteed delivery) or best-effort (no guarantee)
- **Durability**: Volatile (only new messages) or transient-local (historical messages)
- **History**: Keep-all or keep-last N messages
- **Deadline**: Maximum time between consecutive messages
- **Liveliness**: How to determine if a participant is alive

## Hands-on Exercise

1. **Node Discovery**: Use `ros2 node list` to discover active nodes on your system. If you don't have ROS 2 installed, research what this command would show.

2. **Topic Exploration**: Use `ros2 topic list` to see available topics, then use `ros2 topic echo <topic_name>` to observe messages on a topic. Note the message structure.

3. **Concept Mapping**: Draw a simple robot system with 3 nodes (sensor, controller, actuator) and identify how they would communicate using topics and services.

## Summary

This chapter covered the fundamental concepts of ROS 2 as a middleware nervous system. You learned about nodes, topics, services, and how they enable communication between robotic components. In the next chapter, [ROS 2 Communication with Python Agents](./python-agents), you'll learn how to implement these concepts by connecting Python-based AI agents to robot controllers.