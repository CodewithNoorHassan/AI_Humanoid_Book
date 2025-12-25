# Physics Simulation with Gazebo

## Introduction to Gazebo for Robotics

Gazebo is a powerful physics simulation environment that provides realistic robot simulation capabilities. It's widely used in the robotics community for testing algorithms, validating behaviors, and developing robotic systems without the need for physical hardware.

## Setting up Gazebo for Digital Twins

### Installing Gazebo

Gazebo can be installed as part of the ROS 2 ecosystem or as a standalone application. For robotics applications, installing through ROS 2 is recommended as it includes integration packages.

On Ubuntu with ROS 2 Humble:
```bash
sudo apt update
sudo apt install ros-humble-gazebo-*
```

### Basic Gazebo Concepts

Gazebo operates using several core concepts:
- **Worlds**: Define the environment including lighting, physics parameters, and objects
- **Models**: 3D representations of objects with physical properties
- **Plugins**: Extend Gazebo's functionality (sensors, controllers, etc.)

## Understanding Gazebo Physics Parameters

### Gravity Configuration

Gravity is defined in the world file and affects all objects in the simulation:

```xml
<world>
  <gravity>0 0 -9.8</gravity>
  <!-- x, y, z components of gravitational acceleration -->
</world>
```

For different environments:
- Earth-like: `<gravity>0 0 -9.8</gravity>`
- Moon-like: `<gravity>0 0 -1.62</gravity>`
- Zero gravity: `<gravity>0 0 0</gravity>`

### Mass and Inertial Properties

Each model component requires mass and inertial properties:

```xml
<link name="link_name">
  <inertial>
    <mass>1.0</mass>  <!-- Mass in kilograms -->
    <inertia>
      <ixx>0.01</ixx>  <!-- Moments of inertia -->
      <iyy>0.01</iyy>
      <izz>0.01</izz>
      <ixy>0</ixy>     <!-- Products of inertia -->
      <ixz>0</ixz>
      <iyz>0</iyz>
    </inertia>
  </inertial>
</link>
```

### Friction Parameters

Friction affects how objects interact when in contact:

```xml
<link name="link_name">
  <collision name="collision">
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>      <!-- Static friction coefficient -->
          <mu2>1.0</mu2>    <!-- Secondary friction coefficient -->
          <slip1>0</slip1>  <!-- Slip in primary direction -->
          <slip2>0</slip2>  <!-- Slip in secondary direction -->
        </ode>
      </friction>
    </surface>
  </collision>
</link>
```

### Damping and Restitution

Damping and restitution control energy loss and bounciness:

```xml
<link name="link_name">
  <inertial>
    <damping>
      <linear>0.01</linear>   <!-- Linear velocity damping -->
      <angular>0.01</angular> <!-- Angular velocity damping -->
    </damping>
  </inertial>
  <collision name="collision">
    <surface>
      <bounce>
        <restitution_coefficient>0.2</restitution_coefficient> <!-- Bounciness -->
        <threshold>100000.0</threshold> <!-- Velocity threshold for bounce -->
      </bounce>
    </surface>
  </collision>
</link>
```

## Implementing Physics Simulation

### Step-by-Step: Creating Your First Gazebo World

Let's create a basic Gazebo world with a simple robot model:

#### Step 1: Create a World File

Create a file named `simple_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Set gravity -->
    <gravity>0 0 -9.8</gravity>

    <!-- Include a default physics profile -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Add a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Define a simple box model -->
    <model name="simple_box">
      <pose>0 0 1 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.1 1</ambient>
            <diffuse>0.8 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

#### Step 2: Launch the World

Save the file and launch it with:
```bash
gazebo simple_world.world
```

### Gravity and Environmental Forces

In any realistic digital twin, gravity is fundamental to creating believable robot behaviors. We'll explore how to configure gravitational parameters in Gazebo to match real-world conditions.

### Collision Detection and Response

Proper collision detection ensures that robots interact realistically with their environment. We'll cover different collision models and how to configure them.

#### Collision Geometry Types

Gazebo supports several collision geometry types:

- **Box**: Rectangular prism, useful for simple objects
- **Sphere**: Perfect spheres for round objects
- **Cylinder**: Cylindrical shapes
- **Mesh**: Complex shapes using 3D mesh files
- **Plane**: Infinite flat surfaces (useful for ground planes)

#### Configuring Collision Properties

```xml
<collision name="collision_name">
  <geometry>
    <box>
      <size>1.0 1.0 1.0</size>
    </box>
  </geometry>
  <surface>
    <contact>
      <ode>
        <max_vel>100</max_vel>           <!-- Maximum contact penetration velocity -->
        <min_depth>0.001</min_depth>     <!-- Penetration depth for contact stabilization -->
      </ode>
    </contact>
    <friction>
      <ode>
        <mu>1.0</mu>                    <!-- Primary friction coefficient -->
        <mu2>1.0</mu2>                  <!-- Secondary friction coefficient -->
        <fdir1>1 0 0</fdir1>            <!-- Friction direction -->
      </ode>
    </friction>
  </surface>
</collision>
```

#### Collision Layers and Filtering

For complex environments, you can use collision layers to control which objects interact:

```xml
<collision name="collision_name">
  <surface>
    <contact>
      <collide_without_contact>0</collide_without_contact>  <!-- Enable/disable collision -->
    </contact>
  </surface>
</collision>
```

### Material Properties and Friction

The physical properties of surfaces and robot components affect how they interact. We'll examine how to configure material properties for realistic interactions.

## Humanoid Motion Simulation

### Joint Dynamics

Humanoid robots have complex joint systems that require careful simulation. We'll explore how to model these dynamics accurately.

#### Joint Types in Gazebo

Gazebo supports several joint types essential for humanoid robots:

- **Revolute**: Rotational joints with a single degree of freedom (e.g., elbow, knee)
- **Prismatic**: Linear joints moving along one axis
- **Fixed**: Rigid connections between links
- **Ball**: Ball-and-socket joints allowing free rotation
- **Universal**: Two degrees of freedom, like a double pendulum

#### Configuring Joint Properties

```xml
<joint name="knee_joint" type="revolute">
  <parent>thigh_link</parent>
  <child>calf_link</child>
  <axis>
    <xyz>0 1 0</xyz>                <!-- Rotation axis -->
    <limit>
      <lower>-1.57</lower>          <!-- Lower limit in radians -->
      <upper>1.57</upper>           <!-- Upper limit in radians -->
      <effort>100.0</effort>        <!-- Maximum effort (N-m) -->
      <velocity>1.0</velocity>      <!-- Maximum velocity (rad/s) -->
    </limit>
    <dynamics>
      <damping>1.0</damping>         <!-- Damping coefficient -->
      <friction>0.1</friction>      <!-- Static friction -->
    </dynamics>
  </axis>
</joint>
```

#### Example: Simple Humanoid Leg

```xml
<!-- Thigh link -->
<link name="thigh">
  <inertial>
    <mass>5.0</mass>
    <inertia>
      <ixx>0.1</ixx>
      <iyy>0.1</iyy>
      <izz>0.1</izz>
    </inertia>
  </inertial>
  <visual name="visual">
    <geometry>
      <cylinder>
        <radius>0.08</radius>
        <length>0.4</length>
      </cylinder>
    </geometry>
  </visual>
  <collision name="collision">
    <geometry>
      <cylinder>
        <radius>0.08</radius>
        <length>0.4</length>
      </cylinder>
    </geometry>
  </collision>
</link>

<!-- Calf link -->
<link name="calf">
  <inertial>
    <mass>3.0</mass>
    <inertia>
      <ixx>0.05</ixx>
      <iyy>0.05</iyy>
      <izz>0.05</izz>
    </inertia>
  </inertial>
  <visual name="visual">
    <geometry>
      <cylinder>
        <radius>0.07</radius>
        <length>0.4</length>
      </cylinder>
    </geometry>
  </visual>
  <collision name="collision">
    <geometry>
      <cylinder>
        <radius>0.07</radius>
        <length>0.4</length>
      </cylinder>
    </geometry>
  </collision>
</link>

<!-- Knee joint -->
<joint name="knee" type="revolute">
  <parent>thigh</parent>
  <child>calf</child>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.5" upper="2.0" effort="200" velocity="1.0"/>
</joint>
```

### Actuator Modeling

Realistic actuator behavior is crucial for believable humanoid motion. We'll cover how to simulate actuator limitations and characteristics.

## Best Practices for Physics Simulation

### Performance Optimization

Balancing simulation accuracy with computational efficiency is crucial for real-time digital twins.

### Validation Techniques

Ensuring that your simulation accurately reflects real-world physics is essential for effective digital twins.

## Exercises

### Exercise 1: Simple Pendulum Simulation
Create a pendulum model in Gazebo with realistic physics:
1. Design a simple pendulum with a fixed joint at the top
2. Configure appropriate mass and inertial properties
3. Implement gravity and observe the pendulum's motion
4. Adjust damping parameters to see how they affect motion
5. Measure the oscillation period and compare to theoretical calculations

### Exercise 2: Collision Response Testing
Design a test environment to explore collision physics:
1. Create a sloped surface in your world file
2. Add objects with different friction coefficients (0.1, 0.5, 1.0)
3. Observe how each object slides down the slope
4. Document how friction affects motion and final position
5. Add restitution coefficients and observe bouncing behavior

### Exercise 3: Humanoid Balance Challenge
Extend the simple leg example to create a basic balancing mechanism:
1. Build a simple biped model with two legs and a torso
2. Implement basic joint controllers to maintain balance
3. Apply external forces to test stability
4. Adjust center of mass and observe effects on stability
5. Document the relationship between base of support and balance

## Summary

Physics simulation forms the foundation of any realistic digital twin. By carefully configuring gravitational forces, collision properties, and joint dynamics, we can create virtual environments that accurately reflect real-world physics. Proper physics simulation is essential for validating robot behaviors before deployment to physical hardware.

This chapter has provided a comprehensive overview of Gazebo physics simulation, from basic setup to advanced configuration of joint dynamics and humanoid motion. You've learned how to configure realistic physics parameters including gravity, mass, friction, and damping, as well as how to create complex humanoid joint systems. The exercises have given you hands-on experience with pendulum simulation, collision response testing, and humanoid balance challenges.

In the next chapter, we'll explore virtual sensor modeling, building on the physics foundation to create realistic sensor simulations that complement the physical behaviors.

## Cross-References

- **Previous Chapter**: [Introduction to Digital Twins](intro.md) - Understanding the fundamental concepts of digital twin systems
- **Next Chapter**: [Virtual Sensor Modeling](virtual-sensors.md) - Building on physics simulation to create realistic sensor models
- **Related Topic**: [Unity-based Human-Robot Interaction](unity-interaction.md) - Visualizing physics-based behaviors in high-fidelity environments

## Hands-on Exercise

Create a simple humanoid robot model in Gazebo and implement basic physics behaviors including gravity response and collision detection.