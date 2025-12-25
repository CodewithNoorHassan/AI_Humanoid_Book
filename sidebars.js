// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    'quickstart',
    {
      type: 'category',
      label: 'ROS 2 as Robotic Nervous System',
      items: [
        'ros2-nervous-system/index',
        'ros2-nervous-system/ros2-fundamentals',
        'ros2-nervous-system/python-agents',
        'ros2-nervous-system/urdf-modeling',
        'ros2-nervous-system/glossary'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      link: {type: 'doc', id: 'digital-twin/index'},
      items: [
        'digital-twin/intro',
        'digital-twin/gazebo-physics',
        'digital-twin/virtual-sensors',
        'digital-twin/unity-interaction'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Isaac AI Brain (NVIDIA Isaac™)',
      link: {type: 'doc', id: 'isaac-ai-brain/index'},
      items: [
        'isaac-ai-brain/intro',
        'isaac-ai-brain/isaac-sim',
        'isaac-ai-brain/isaac-ros',
        'isaac-ai-brain/nav2-humanoid'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) Integration',
      link: {type: 'doc', id: 'vla-integration/index'},
      items: [
        'vla-integration/intro',
        'vla-integration/voice-to-action',
        'vla-integration/llm-planning',
        'vla-integration/vla-systems',
        'vla-integration/capstone-project'
      ],
    },
  ],
};

module.exports = sidebars;