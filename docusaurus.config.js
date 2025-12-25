// @ts-check
// `@type` JSDoc annotations allow IDEs and type-checking tools to autocomplete
// and validate function arguments and return values, enhancing development
// experience and preventing bugs.

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AI-Humanoid Technical Book',
  tagline: 'Understanding ROS 2 as a Robotic Nervous System',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-username.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/Ai_Humanoid_Book/',

  // GitHub pages deployment config.
  organizationName: 'your-username', // Usually your GitHub org/user name.
  projectName: 'Ai_Humanoid_Book', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'AI-Humanoid Book',
        logo: {
          alt: 'AI-Humanoid Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            to: '/docs/intro',
            label: 'Tutorials',
            position: 'left',
          },
          {
            to: '/',
            label: 'Home',
            position: 'left',
          },
          {
            to: '/docs/ros2-nervous-system',
            label: 'ROS 2 Module',
            position: 'left',
          },
          {
            to: '/docs/digital-twin',
            label: 'Digital Twin Module',
            position: 'left',
          },
          {
            to: '/docs/isaac-ai-brain',
            label: 'Isaac AI Brain Module',
            position: 'left',
          },
          {
            to: '/docs/vla-integration',
            label: 'VLA Integration Module',
            position: 'left',
          },
          {
            type: 'search',
            position: 'right',
          },
          {
            href: 'https://github.com/your-username/your-project-name',
            label: 'GitHub',
            position: 'right',
          },
        ],
        style: 'dark',
        hideOnScroll: false,
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Documentation',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'ROS 2 Fundamentals',
                to: '/docs/ros2-nervous-system/ros2-fundamentals',
              },
              {
                label: 'Python Agents',
                to: '/docs/ros2-nervous-system/python-agents',
              },
              {
                label: 'URDF Modeling',
                to: '/docs/ros2-nervous-system/urdf-modeling',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Official',
                href: 'https://docs.ros.org/en/rolling/',
              },
              {
                label: 'Docusaurus',
                href: 'https://docusaurus.io',
              },
              {
                label: 'GitHub Repo',
                href: 'https://github.com/your-username/Ai_Humanoid_Book',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'Report Issues',
                href: 'https://github.com/your-username/Ai_Humanoid_Book/issues',
              },
              {
                label: 'Contribute',
                href: 'https://github.com/your-username/Ai_Humanoid_Book/blob/main/CONTRIBUTING.md',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} AI-Humanoid Technical Book. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;