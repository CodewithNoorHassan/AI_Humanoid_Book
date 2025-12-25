---
sidebar_label: Humanoid Modeling with URDF
sidebar_position: 4
---

# Humanoid Modeling with URDF

This chapter explains how humanoid robot structure is defined using URDF (Unified Robot Description Format), covering links, joints, and physical properties that define robot structure.

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain the purpose and structure of URDF files in robotics
- Identify and describe the components of robot structure: links and joints
- Create and interpret practical examples of URDF files for humanoid robots
- Understand the physical and kinematic properties defined in URDF
- Explain how URDF models integrate with ROS 2 systems
- Follow a systematic approach to reading and understanding URDF files

## Introduction to URDF and Robot Description Format

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and kinematic properties of a robot, including:

- **Links**: Rigid components of the robot (e.g., base, arms, legs, head)
- **Joints**: Connections between links that allow relative motion
- **Visual**: How the robot appears in simulation and visualization
- **Collision**: How the robot interacts with its environment for collision detection
- **Inertial**: Physical properties for dynamics simulation

URDF is fundamental to ROS robotics as it enables:
- Robot simulation in Gazebo
- Visualization in RViz
- Kinematic analysis with tools like MoveIt
- Motion planning and control

## Links and Joints that Define Robot Structure

### Links

A link represents a rigid body in the robot. Each link has:

- A unique name
- Inertial properties (mass, center of mass, moment of inertia)
- Visual properties (geometry, material, origin)
- Collision properties (geometry for collision detection)

```xml
<link name="base_link">
  <inertial>
    <mass value="1.0" />
    <origin xyz="0 0 0" rpy="0 0 0" />
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <cylinder length="0.2" radius="0.1" />
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1" />
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <cylinder length="0.2" radius="0.1" />
    </geometry>
  </collision>
</link>
```

### Joints

A joint connects two links and defines how they can move relative to each other. Joints have:

- A unique name
- A type (fixed, continuous, revolute, prismatic, floating, planar)
- Parent and child links
- Joint limits (for revolute and prismatic joints)
- Origin (position and orientation of the joint)

```xml
<joint name="base_to_wheel" type="continuous">
  <parent link="base_link" />
  <child link="wheel_link" />
  <origin xyz="0.1 0 0" rpy="0 0 0" />
  <axis xyz="0 1 0" />
</joint>
```

## Practical Examples of URDF Files for Humanoid Robots

### Simple Humanoid Torso Example

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5" />
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0" />
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5" />
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1" />
      </geometry>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0" />
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1" />
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002" />
    </inertial>
  </link>

  <!-- Joint connecting torso to head -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso" />
    <child link="head" />
    <origin xyz="0 0 0.35" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>
</robot>
```

### Complete Humanoid Example with Arms

```xml
<?xml version="1.0"?>
<robot name="complete_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5" />
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0" />
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5" />
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
    </inertial>
  </link>

  <!-- Left Upper Arm -->
  <link name="upper_arm_left">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05" />
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0" />
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05" />
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 -0.15" />
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0005" />
    </inertial>
  </link>

  <!-- Joint connecting torso to left upper arm -->
  <joint name="shoulder_left" type="revolute">
    <parent link="torso" />
    <child link="upper_arm_left" />
    <origin xyz="0.2 0.1 0.1" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>

  <!-- Left Lower Arm -->
  <link name="lower_arm_left">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04" />
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0" />
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5" />
      <origin xyz="0 0 -0.125" />
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.0002" />
    </inertial>
  </link>

  <!-- Joint connecting upper arm to lower arm -->
  <joint name="elbow_left" type="revolute">
    <parent link="upper_arm_left" />
    <child link="lower_arm_left" />
    <origin xyz="0 0 -0.3" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="5.0" velocity="1.0" />
  </joint>
</robot>
```

## Physical and Kinematic Properties in URDF

### Inertial Properties

The inertial properties define how a link behaves dynamically:

- **Mass**: The mass of the link in kilograms
- **Origin**: The center of mass location relative to the link frame
- **Inertia**: The 3x3 inertia matrix (only diagonal elements in URDF: ixx, ixy, ixz, iyy, iyz, izz)

### Visual and Collision Properties

- **Visual**: Defines how the link appears in visualization tools
- **Collision**: Defines the geometry used for collision detection
- **Geometry types**: box, cylinder, sphere, mesh
- **Materials**: Color and texture properties

## How URDF Relates to ROS 2 Systems

URDF integrates with ROS 2 systems in several ways:

### Robot State Publisher

The `robot_state_publisher` node reads a URDF and publishes the state of the robot's joints to the `/tf` topic, allowing other nodes to understand the robot's configuration.

### TF (Transforms)

URDF defines the static transforms between robot links, which are published to the TF system for spatial relationships.

### Simulation

In Gazebo simulation, URDF models are loaded to create virtual robots that behave according to their physical properties.

### Motion Planning

Planning frameworks like MoveIt use URDF to understand the robot's kinematic structure for motion planning.

## Step-by-Step Tutorial for Reading URDF Files

### Step 1: Understanding the URDF Structure

A URDF file is an XML document with a root `<robot>` tag containing `<link>` and `<joint>` elements.

### Step 2: Identifying Links

Look for `<link>` tags, each with a unique `name` attribute. Each link contains:
- `<inertial>`: Physical properties
- `<visual>`: How it looks
- `<collision>`: Collision geometry

### Step 3: Understanding Joints

Look for `<joint>` tags that connect links:
- `name`: Unique joint name
- `type`: Type of joint (fixed, revolute, continuous, etc.)
- `<parent>`: The link the joint connects FROM
- `<child>`: The link the joint connects TO

### Step 4: Reading Joint Limits

For revolute and prismatic joints, look at the `<limit>` tag:
- `lower` and `upper`: Position limits
- `effort`: Maximum effort/torque
- `velocity`: Maximum velocity

### Step 5: Visualizing URDF

You can visualize a URDF file using RViz or the `check_urdf` command:

```bash
# Check URDF validity
ros2 run urdf check_urdf /path/to/robot.urdf

# Visualize in RViz
# Load the robot model plugin and specify the URDF file
```

## Common URDF Best Practices

### 1. Proper Mass and Inertia Values
Always specify realistic mass and inertia values for proper simulation:

```xml
<inertial>
  <mass value="0.1" />
  <origin xyz="0 0 0" rpy="0 0 0" />
  <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001" />
</inertial>
```

### 2. Appropriate Joint Limits
Set realistic joint limits to prevent damage in simulation and real robots:

```xml
<joint name="joint_name" type="revolute">
  <!-- ... -->
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
</joint>
```

### 3. Correct Origin Definitions
Ensure origins are correctly defined relative to the parent link frame:

```xml
<origin xyz="0.1 0 0" rpy="0 0 0" />
```

### 4. Use Xacro for Complex Robots
For complex robots, use Xacro (XML Macros) to simplify URDF creation:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_name">
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="simple_cylinder" params="name parent x y z mass radius length">
    <link name="${name}">
      <inertial>
        <mass value="${mass}" />
        <origin xyz="0 0 0" />
        <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1" />
      </inertial>
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
          <cylinder radius="${radius}" length="${length}" />
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
          <cylinder radius="${radius}" length="${length}" />
        </geometry>
      </collision>
    </link>
  </xacro:macro>
</robot>
```

## Troubleshooting Common URDF Issues

### 1. Invalid XML
- **Symptom**: Parser errors when loading URDF
- **Solution**: Validate XML syntax, ensure all tags are properly closed

### 2. Missing Joint Connections
- **Symptom**: Robot appears as disconnected parts
- **Solution**: Verify every link (except the root) is connected by a joint

### 3. Incorrect Mass Properties
- **Symptom**: Unstable simulation behavior
- **Solution**: Ensure all links have realistic mass and inertia values

### 4. Self-Collision Issues
- **Symptom**: Robot collides with itself in simulation
- **Solution**: Add appropriate collision filtering or adjust geometry

## Hands-on Exercise

1. **URDF Structure Analysis**: Examine the simple_robot.urdf file in the examples directory. Identify all links, joints, and their properties. Draw the kinematic tree structure.

2. **XML Validation**: Use an XML validator to check the syntax of the humanoid_torso.urdf file. Identify and list all the different types of elements used in the file.

3. **Robot Design Challenge**: Design a simple 2-wheeled robot by creating a basic URDF structure with a base link, two wheel links, and appropriate joints connecting them.

## Summary

URDF is essential for describing robot structure in ROS 2. It defines the physical and kinematic properties of robots, enabling simulation, visualization, and motion planning. Understanding how to read and write URDF files is crucial for robotics development, especially when working with complex humanoid robots. In the previous chapter, [ROS 2 Communication with Python Agents](./python-agents), you learned how to connect AI agents to robot controllers. This concludes the ROS 2 as Robotic Nervous System module, where you've learned about the fundamental concepts, AI integration, and robot modeling.