import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Learn ROS 2 Fundamentals',
    Svg: require('@site/static/img/ros2-architecture.svg').default,
    description: (
      <>
        Understand the core concepts of ROS 2 as a middleware nervous system that
        connects AI decision-making to physical robot control.
      </>
    ),
  },
  {
    title: 'Connect AI Agents',
    Svg: require('@site/static/img/ai-robot-communication.svg').default,
    description: (
      <>
        Bridge Python-based AI agents to robot controllers using client libraries
        and practical communication patterns.
      </>
    ),
  },
  {
    title: 'Model Humanoid Robots',
    Svg: require('@site/static/img/urdf-structure.svg').default,
    description: (
      <>
        Define humanoid robot structure using URDF (Unified Robot Description Format)
        and understand physical properties.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}