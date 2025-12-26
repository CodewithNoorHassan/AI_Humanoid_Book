import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/Ai_Humanoid_Book/404',
    component: ComponentCreator('/Ai_Humanoid_Book/404', 'b2d'),
    exact: true
  },
  {
    path: '/Ai_Humanoid_Book/docs',
    component: ComponentCreator('/Ai_Humanoid_Book/docs', 'ce8'),
    routes: [
      {
        path: '/Ai_Humanoid_Book/docs',
        component: ComponentCreator('/Ai_Humanoid_Book/docs', 'c5d'),
        routes: [
          {
            path: '/Ai_Humanoid_Book/docs',
            component: ComponentCreator('/Ai_Humanoid_Book/docs', 'f37'),
            routes: [
              {
                path: '/Ai_Humanoid_Book/docs/digital-twin/',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/digital-twin/', '1d0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/digital-twin/gazebo-physics',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/digital-twin/gazebo-physics', 'dec'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/digital-twin/glossary',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/digital-twin/glossary', '66a'),
                exact: true
              },
              {
                path: '/Ai_Humanoid_Book/docs/digital-twin/intro',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/digital-twin/intro', '7a8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/digital-twin/setup',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/digital-twin/setup', '30c'),
                exact: true
              },
              {
                path: '/Ai_Humanoid_Book/docs/digital-twin/unity-interaction',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/digital-twin/unity-interaction', '8b3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/digital-twin/virtual-sensors',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/digital-twin/virtual-sensors', '2dd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/intro',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/intro', '63c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/isaac-ai-brain/',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/isaac-ai-brain/', '8a4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/isaac-ai-brain/intro',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/isaac-ai-brain/intro', '5a2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/isaac-ai-brain/isaac-ros',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/isaac-ai-brain/isaac-ros', '16e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/isaac-ai-brain/isaac-sim',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/isaac-ai-brain/isaac-sim', '9f0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/isaac-ai-brain/nav2-humanoid',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/isaac-ai-brain/nav2-humanoid', 'a04'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/quickstart',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/quickstart', '587'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/ros2-nervous-system/',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/ros2-nervous-system/', 'ad1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/ros2-nervous-system/glossary',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/ros2-nervous-system/glossary', '2c4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/ros2-nervous-system/python-agents',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/ros2-nervous-system/python-agents', '8e8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/ros2-nervous-system/ros2-fundamentals',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/ros2-nervous-system/ros2-fundamentals', 'cd3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/ros2-nervous-system/urdf-modeling',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/ros2-nervous-system/urdf-modeling', 'd5f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/vla-integration/',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/vla-integration/', 'e08'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/vla-integration/capstone-project',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/vla-integration/capstone-project', 'b4a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/vla-integration/intro',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/vla-integration/intro', '0fd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/vla-integration/llm-planning',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/vla-integration/llm-planning', 'd72'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/vla-integration/vla-systems',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/vla-integration/vla-systems', 'cc7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Ai_Humanoid_Book/docs/vla-integration/voice-to-action',
                component: ComponentCreator('/Ai_Humanoid_Book/docs/vla-integration/voice-to-action', '177'),
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
    path: '/Ai_Humanoid_Book/',
    component: ComponentCreator('/Ai_Humanoid_Book/', 'c58'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
