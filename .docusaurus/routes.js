import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/404',
    component: ComponentCreator('/404', '5c5'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '5a3'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '860'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '64a'),
            routes: [
              {
                path: '/docs/digital-twin/',
                component: ComponentCreator('/docs/digital-twin/', '24d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/gazebo-physics',
                component: ComponentCreator('/docs/digital-twin/gazebo-physics', '746'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/glossary',
                component: ComponentCreator('/docs/digital-twin/glossary', 'a5c'),
                exact: true
              },
              {
                path: '/docs/digital-twin/intro',
                component: ComponentCreator('/docs/digital-twin/intro', '354'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/setup',
                component: ComponentCreator('/docs/digital-twin/setup', 'dc9'),
                exact: true
              },
              {
                path: '/docs/digital-twin/unity-interaction',
                component: ComponentCreator('/docs/digital-twin/unity-interaction', '576'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/virtual-sensors',
                component: ComponentCreator('/docs/digital-twin/virtual-sensors', '927'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-ai-brain/',
                component: ComponentCreator('/docs/isaac-ai-brain/', '014'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-ai-brain/intro',
                component: ComponentCreator('/docs/isaac-ai-brain/intro', '2ae'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-ai-brain/isaac-ros',
                component: ComponentCreator('/docs/isaac-ai-brain/isaac-ros', 'b22'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-ai-brain/isaac-sim',
                component: ComponentCreator('/docs/isaac-ai-brain/isaac-sim', '638'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-ai-brain/nav2-humanoid',
                component: ComponentCreator('/docs/isaac-ai-brain/nav2-humanoid', '1fb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/quickstart',
                component: ComponentCreator('/docs/quickstart', '79e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/',
                component: ComponentCreator('/docs/ros2-nervous-system/', '95a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/glossary',
                component: ComponentCreator('/docs/ros2-nervous-system/glossary', '511'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/python-agents',
                component: ComponentCreator('/docs/ros2-nervous-system/python-agents', '57c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/ros2-fundamentals',
                component: ComponentCreator('/docs/ros2-nervous-system/ros2-fundamentals', '8ca'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/urdf-modeling',
                component: ComponentCreator('/docs/ros2-nervous-system/urdf-modeling', '197'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-integration/',
                component: ComponentCreator('/docs/vla-integration/', '03a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-integration/capstone-project',
                component: ComponentCreator('/docs/vla-integration/capstone-project', 'cb2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-integration/intro',
                component: ComponentCreator('/docs/vla-integration/intro', 'c37'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-integration/llm-planning',
                component: ComponentCreator('/docs/vla-integration/llm-planning', '35d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-integration/vla-systems',
                component: ComponentCreator('/docs/vla-integration/vla-systems', '84e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-integration/voice-to-action',
                component: ComponentCreator('/docs/vla-integration/voice-to-action', 'db8'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
