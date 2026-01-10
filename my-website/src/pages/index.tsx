import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';
import RagChatbot from '../components/RagChatbot';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          Physical AI & Humanoid Robotics
        </Heading>
        <p className="hero__subtitle">Bridging Digital Brain to Physical Body</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning
          </Link>
        </div>
      </div>
    </header>
  );
}

type ModuleCardProps = {
  title: string;
  subchapters: string[];
  link: string;
  className?: string;
};

function ModuleCard({ title, subchapters, link, className }: ModuleCardProps) {
  const cardClassNames = clsx('card', styles.card, className);

  return (
    <div className="col col--3 margin-bottom--lg">
      <div className={cardClassNames}>
        <div className="card__header">
          <h3>{title}</h3>
        </div>
        <div className="card__body">
          <ul>
            {subchapters.map((subchapter, index) => (
              <li key={index}>{subchapter}</li>
            ))}
          </ul>
        </div>
        <div className="card__footer">
          <Link className="button button--primary button--block" to={link}>
            Explore Module
          </Link>
        </div>
      </div>
    </div>
  );
}

function ModulesSection() {
  const modules = [
    {
      title: 'ROS 2',
      subchapters: [
        'Introduction to ROS 2',
        'Nodes & Topics',
        'Services & Actions',
        'URDF Robot Description'
      ],
      link: '/docs/chapters/intro-ros2',
      className: styles['module-card']
    },
    {
      title: 'Digital Twin',
      subchapters: [
        'Digital Twins in Physical AI',
        'Physics Simulation with Gazebo',
        'High-Fidelity Environments with Unity',
        'Sensor Simulation for Humanoid Robots'
      ],
      link: '/docs/modules/digital-twin/chapter-1-digital-twins-in-physical-ai',
      className: styles['digital-twin-card']
    },
    {
      title: 'AI-Robot Brain',
      subchapters: [
        'Introduction to Isaac Sim',
        'Isaac ROS & Visual SLAM',
        'Navigation 2 Path Planning',
        'Integration & Deployment'
      ],
      link: '/docs/module3-ai-robot-brain/intro-isaac-sim',
      className: styles['ai-brain-card']
    },
    {
      title: 'Vision-Language-Action',
      subchapters: [
        'Introduction to VLA',
        'Voice to Action',
        'Cognitive Planning with LLMs',
        'Capstone: Autonomous Humanoid'
      ],
      link: '/docs/module4-vla-physical-ai/intro-vla',
      className: styles['vla-card']
    }
  ];

  return (
    <section className={clsx(styles.featureSection)}>
      <div className="container padding-vert--lg">
        <Heading as="h2" className="text--center margin-bottom--lg">
         <u style={{ color: '#53648cff' }}>Course Modules</u>
        </Heading>

        <div className="row">
          {modules.map((module, index) => (
            <ModuleCard
              key={index}
              title={module.title}
              subchapters={module.subchapters}
              link={module.link}
              className={module.className}
            />
          ))}
        </div>
      </div>
    </section>
  );
}

type HardwareCardProps = {
  title: string;
  description: string;
  price: string;
  className?: string;
};

function HardwareCard({ title, description, price, className }: HardwareCardProps) {
  const cardClassNames = clsx('card', styles.card, className);

  return (
    <div className="col col--4 margin-bottom--lg">
      <div className={cardClassNames}>
        <div className="card__header">
          <h3>{title}</h3>
        </div>
        <div className="card__body">
          <p>{description}</p>
          <p><strong>Price:</strong> {price}</p>
        </div>
      </div>
    </div>
  );
}

function HardwareSection() {
  const hardwareItems = [
    {
      title: 'Digital Twin Workstation',
      description: 'High-performance computing setup for running digital twin simulations and development environments.',
      price: '~$2,000-$5,000',
      className: styles['hardware-card']
    },
    {
      title: 'Physical AI Edge Kit',
      description: 'Complete hardware kit with sensors, actuators, and edge computing modules for physical AI applications.',
      price: '~$1,500-$3,000',
      className: styles['hardware-card-2']
    },
    {
      title: 'Robot Lab',
      description: 'Complete laboratory setup with humanoid robots, safety equipment, and development tools.',
      price: '~$10,000-$50,000',
      className: styles['hardware-card-3']
    }
  ];

  return (
    <section className={clsx(styles.featureSection)}>
      <div className="container padding-vert--lg">
        <Heading as="h2" className="text--center margin-bottom--lg">
         <u style={{ color: '#53648cff' }}>Hardware Requirements</u>
        </Heading>
        <div className="row">
          {hardwareItems.map((item, index) => (
            <HardwareCard
              key={index}
              title={item.title}
              description={item.description}
              price={item.price}
              className={item.className}
            />
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="Bridging Digital Brain to Physical Body - Comprehensive course on Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <div className="container margin-vert--lg">
          <RagChatbot />
        </div>
        <ModulesSection />
        <HardwareSection />
      </main>
    </Layout>
  );
}
