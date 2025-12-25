# ADR-3: VLA (Vision-Language-Action) Technology Stack

**Status**: Proposed
**Date**: 2025-12-20

## Context

For the Vision-Language-Action (VLA) educational module, we need to establish a technology stack that will enable the creation of educational content covering the integration of large language models with robotic perception and control. The solution must be suitable for teaching advanced AI and robotics students about voice-to-action pipelines, LLM-based cognitive planning, and end-to-end autonomous workflows.

The educational content needs to:
- Cover voice-to-action pipeline fundamentals with speech recognition
- Include LLM-based cognitive planning mapped to ROS 2 actions
- Provide end-to-end autonomous workflow integration with perception, language, planning, and control
- Integrate with the existing Docusaurus documentation framework
- Be accessible to students with prior ROS 2, simulation, and perception knowledge

## Decision

We will use the following integrated technology stack:

**Speech Recognition**: Speech-to-text systems (such as Whisper or similar technology)
- Converts spoken language to text for processing
- Handles various accents, background noise, and audio quality variations
- Integrates with natural language understanding systems

**Large Language Model Integration**: LLM-based cognitive planning systems
- Processes natural language instructions and generates action plans
- Incorporates world knowledge and reasoning capabilities
- Maps high-level goals to low-level robot actions
- Maintains context and handles complex multi-step instructions

**ROS 2 Action Mapping**: Integration with ROS 2 action and service systems
- Translates LLM-generated plans to specific ROS 2 actions
- Handles action execution, feedback, and error recovery
- Maintains compatibility with existing ROS 2 ecosystems

**Perception Integration**: Connection between perception systems and language understanding
- Enables grounded language processing with sensory input
- Provides context-aware instruction interpretation
- Supports multimodal understanding combining vision and language

**Documentation Platform**: Docusaurus framework
- Consistent with existing project documentation
- Supports educational content with proper navigation
- Markdown-based for easy content creation

## Consequences

**Positive:**
- Students learn industry-standard approaches to multimodal AI integration
- VLA approach provides comprehensive understanding of modern robotics autonomy
- Consistent documentation approach with existing modules
- Strong foundation for advanced robotics research and development
- Realistic integration scenarios prepare students for advanced robotics challenges

**Negative:**
- Requires students to have prior knowledge of ROS 2, simulation, and perception
- Complex integration between multiple advanced technologies
- Learning curve for understanding multimodal AI systems
- Dependencies on multiple sophisticated technology stacks

**Risks:**
- Students may struggle with the complexity of integrated systems without proper foundational knowledge
- Rapid evolution of LLM technology may require frequent content updates
- Integration challenges between perception, language, and control systems
- Computational requirements for demonstrating full VLA capabilities

## Alternatives Considered

**Alternative 1**: Simplified approach focusing only on language-to-action mapping (without perception)
- Pros: Reduced complexity, easier to understand initially
- Cons: Missing crucial multimodal integration aspects, less realistic for real robotics applications

**Alternative 2**: Different LLM platforms or approaches (custom models vs. existing LLMs)
- Pros: Potential for more specialized robotics language understanding
- Cons: Greater complexity, less transferable knowledge to industry standards

**Alternative 3**: Different documentation approaches (interactive tutorials vs. conceptual guides)
- Pros: More hands-on learning experience
- Cons: Would violate constraint of focusing on conceptual, system-level understanding

## References

- specs/4-vla-integration/spec.md
- specs/2-isaac-ai-brain/spec.md
- specs/1-digital-twin/spec.md
- specs/1-ros2-nervous-system/spec.md