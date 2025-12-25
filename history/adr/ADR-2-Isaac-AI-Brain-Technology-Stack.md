# ADR-2: Isaac AI Brain Technology Stack

**Status**: Accepted
**Date**: 2025-12-20

## Context

For the Isaac AI Brain educational module, we need to establish a technology stack that will enable the creation of educational content covering advanced robot perception, navigation, and autonomy using NVIDIA Isaac and ROS 2. The solution must be suitable for teaching AI and robotics students about Isaac Sim, Isaac ROS, and Nav2 for humanoid locomotion.

The educational content needs to:
- Cover photorealistic simulation with Isaac Sim
- Include hardware-accelerated perception using Isaac ROS
- Provide navigation and locomotion training with Nav2
- Integrate with the existing Docusaurus documentation framework
- Be accessible to students with varying technical backgrounds

## Decision

We will use the following integrated technology stack:

**Simulation Environment**: NVIDIA Isaac Sim
- Photorealistic rendering capabilities for synthetic data generation
- Integration with robotics workflows and training pipelines
- Support for complex humanoid robot models and environments

**Perception Framework**: Isaac ROS
- Hardware-accelerated perception algorithms optimized for NVIDIA GPUs
- VSLAM capabilities for real-time localization and mapping
- Integration with ROS 2 ecosystem

**Navigation System**: Nav2 (Navigation 2)
- Advanced path planning algorithms
- Adaptable for humanoid locomotion requirements
- Integration with ROS 2 and Isaac ecosystem

**Documentation Platform**: Docusaurus framework
- Consistent with existing project documentation
- Supports educational content with proper navigation
- Markdown-based for easy content creation

## Consequences

**Positive:**
- Students learn industry-standard tools used in advanced robotics
- Isaac Sim-Isaac ROS combination provides comprehensive perception training
- Consistent documentation approach with existing modules
- Strong support for synthetic data generation and hardware acceleration
- Realistic simulation parameters prepare students for advanced robotics challenges

**Negative:**
- Requires students to learn specialized NVIDIA tools (Isaac ecosystem)
- Hardware acceleration may require specific NVIDIA GPU hardware
- Learning curve for Isaac Sim and Isaac ROS integration
- Complex integration between Isaac Sim, Isaac ROS, and Nav2

**Risks:**
- Students may face hardware requirements for Isaac ROS acceleration
- Isaac ecosystem may have licensing considerations for commercial use
- Updates to Isaac tools could break educational examples
- Dependency on NVIDIA-specific technologies

## Alternatives Considered

**Alternative 1**: Pure ROS 2 + Gazebo approach (without Isaac)
- Pros: More hardware-agnostic, broader community support
- Cons: Missing hardware acceleration benefits, less photorealistic simulation

**Alternative 2**: Web-based simulation (Three.js + custom perception tools)
- Pros: Browser-based, no specialized hardware required
- Cons: Lacks hardware acceleration, limited perception capabilities

**Alternative 3**: Different simulation platforms (Webots, PyBullet)
- Pros: Alternative simulation capabilities
- Cons: Missing Isaac's photorealistic rendering and hardware acceleration

## References

- specs/2-isaac-ai-brain/spec.md
- specs/1-digital-twin/spec.md
- specs/1-ros2-nervous-system/spec.md