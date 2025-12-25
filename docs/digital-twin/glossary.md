# Glossary: Digital Twin Terminology

## Core Concepts

### Digital Twin
A virtual representation of a physical robot that mirrors its real-world behaviors, properties, and responses in a simulated environment.

### Physics Simulation
A computational model that replicates real-world physical phenomena including gravity, collisions, friction, and motion dynamics.

### Sensor Simulation
Virtual representations of real sensors that produce data similar to their physical counterparts, including realistic noise and imperfections.

### Visual Fidelity
The degree of realism in visual representation of environments and objects in simulation platforms.

## Gazebo-Specific Terms

### World File
An XML file that defines the environment, including models, lighting, and physics properties for a Gazebo simulation.

### Model
A 3D representation of an object or robot in Gazebo, including its geometry, physics properties, and visual appearance.

### SDF (Simulation Description Format)
The XML-based format used to describe objects and environments in Gazebo.

### Plugin
A piece of code that extends Gazebo's functionality, such as sensor plugins, controller plugins, or physics plugins.

### Physics Engine
The underlying system that calculates physical interactions in the simulation (e.g., ODE, Bullet, DART).

## Unity-Specific Terms

### GameObject
The basic object in Unity that can contain components like meshes, scripts, and other data.

### Component
A part of a GameObject that adds specific functionality (e.g., MeshRenderer, Collider, Script).

### Scene
A container that holds a collection of GameObjects in Unity.

### Prefab
A reusable GameObject template that can be instantiated multiple times in a scene.

### Shader
A program that determines how surfaces are rendered in Unity.

## Robotics-Specific Terms

### LiDAR (Light Detection and Ranging)
A sensor that measures distances by illuminating targets with laser light and measuring the reflection.

### IMU (Inertial Measurement Unit)
A device that measures and reports velocity, orientation, and gravitational forces using accelerometers and gyroscopes.

### URDF (Unified Robot Description Format)
An XML format used to describe robot models, including kinematics, dynamics, and visual properties.

### TF (Transform)
A system in ROS that keeps track of coordinate frames in a tree structure over time.

### Coordinate Frame
A system that defines the position and orientation of objects in 3D space.

## Simulation Concepts

### Real-time Factor
The ratio of simulation time to real time (e.g., 1.0 means simulation runs at real-time speed).

### Collision Mesh
A simplified mesh used for collision detection, often different from the visual mesh for performance reasons.

### Inertial Properties
Physical properties of a model including mass, center of mass, and moment of inertia.

### Sensor Noise
Artificially added randomness to sensor data to simulate real-world sensor imperfections.

### Ground Truth
The actual values in simulation that represent the "true" state of the system, often not available in real-world scenarios.

## Performance Terms

### Frame Rate
The number of frames rendered per second in a simulation or visualization.

### Update Rate
The frequency at which a simulation or sensor updates its state or measurements.

### Timestep
The time interval between simulation updates, affecting both accuracy and performance.

### Deterministic Simulation
A simulation that produces the same results when run multiple times with the same initial conditions.