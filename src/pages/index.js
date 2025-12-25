import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import JarvisInterface from '@site/src/components/JarvisInterface';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <h1 className="hero__title">{siteConfig.title}</h1>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <p className={styles.heroDescription}>
              Master the integration of ROS 2, Digital Twins, Isaac AI, and Vision-Language-Action systems for next-generation humanoid robotics.
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Start Learning - 5 min ⏱️
              </Link>
              <Link
                className="button button--primary button--lg"
                to="/docs/ros2-nervous-system">
                Explore Modules
              </Link>
            </div>
          </div>
          <div className={styles.heroImage}>
            <JarvisInterface />
          </div>
        </div>
      </div>
    </header>
  );
}

function ModuleOverview() {
  const modules = [
    {
      title: 'ROS 2 as Robotic Nervous System',
      description: 'Master ROS 2 fundamentals, Python agents, and URDF modeling for robotic communication and control.',
      icon: '🧠',
      link: '/docs/ros2-nervous-system',
      color: '#3578e5'
    },
    {
      title: 'Digital Twin (Gazebo & Unity)',
      description: 'Create photorealistic simulations with advanced physics engines and virtual sensors.',
      icon: '🔄',
      link: '/docs/digital-twin',
      color: '#00c3ff'
    },
    {
      title: 'Isaac AI Brain (NVIDIA Isaac™)',
      description: 'Implement perception, navigation, and autonomy using NVIDIA Isaac and ROS 2 integration.',
      icon: '🤖',
      link: '/docs/isaac-ai-brain',
      color: '#764abc'
    },
    {
      title: 'Vision-Language-Action (VLA) Integration',
      description: 'Combine vision, language, and action systems for natural language-driven robotics.',
      icon: '👁️',
      link: '/docs/vla-integration',
      color: '#ff6b6b'
    }
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Complete Learning Modules
          </Heading>
          <p className={styles.sectionSubtitle}>
            Four comprehensive modules designed to take you from fundamentals to advanced robotics integration
          </p>
        </div>
        <div className="row">
          {modules.map((module, index) => (
            <div key={index} className="col col--3">
              <div className={styles.moduleCard}>
                <div className={styles.moduleIcon} style={{backgroundColor: module.color}}>
                  {module.icon}
                </div>
                <h3 className={styles.moduleTitle}>{module.title}</h3>
                <p className={styles.moduleDescription}>{module.description}</p>
                <Link className={styles.moduleLink} to={module.link}>
                  Explore Module →
                </Link>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function StatsSection() {
  const stats = [
    { number: '4', label: 'Learning Modules' },
    { number: '15+', label: 'Detailed Chapters' },
    { number: '50+', label: 'Practical Examples' },
    { number: '∞', label: 'Innovation Possibilities' }
  ];

  return (
    <section className={styles.statsSection}>
      <div className="container">
        <div className="row">
          {stats.map((stat, index) => (
            <div key={index} className="col col--3">
              <div className={styles.statItem}>
                <div className={styles.statNumber}>{stat.number}</div>
                <div className={styles.statLabel}>{stat.label}</div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <div className={styles.ctaContent}>
          <h2 className={styles.ctaTitle}>Ready to Build the Future of Robotics?</h2>
          <p className={styles.ctaSubtitle}>
            Join thousands of developers and researchers mastering the next generation of AI-humanoid systems.
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Start Learning Today
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/ros2-nervous-system">
              Explore All Modules
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Comprehensive guide to ROS 2, Digital Twins, Isaac AI, and VLA Integration for humanoid robotics">
      <HomepageHeader />
      <main>
        <StatsSection />
        <ModuleOverview />
        <HomepageFeatures />
        <CTASection />
      </main>
    </Layout>
  );
}