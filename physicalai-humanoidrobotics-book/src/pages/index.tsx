import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import {
  Container,
  Section,
  Grid,
  PrimaryButton,
  SecondaryButton,
  FeatureCard,
  ModuleCard,
  Badge,
  ProgressBar,
} from '@site/src/components';

import styles from './index.module.css';

/**
 * Hero Section Component
 * Professional header with research-grade aesthetics and CTAs
 */
function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <header className={styles.heroBanner}>
      {/* Animated Background Elements */}
      <div className={styles.geometricBackground}>
        <div className={styles.hexGrid}>
          {[...Array(12)].map((_, i) => (
            <div
              key={i}
              className={styles.hexagon}
              style={{
                animationDelay: `${i * 0.1}s`,
                left: `${(i % 4) * 25}%`,
                top: `${Math.floor(i / 4) * 33}%`,
              }}
            />
          ))}
        </div>
        <div className={styles.circuitLayer}>
          <svg className={styles.circuitSvg} viewBox="0 0 1200 600" preserveAspectRatio="none">
            <defs>
              <linearGradient id="circuitGrad" x1="0%" y1="0%" x2="100%" y2="100%">
                <stop offset="0%" stopColor="#3b82f6" stopOpacity="0.1" />
                <stop offset="100%" stopColor="#2563eb" stopOpacity="0.3" />
              </linearGradient>
            </defs>
            <path
              className={styles.circuitPath}
              d="M0,100 L300,100 L300,200 L600,200 L600,300 L900,300 L900,200 L1200,200"
              stroke="url(#circuitGrad)"
              strokeWidth="2"
              fill="none"
            />
            <path
              className={styles.circuitPath}
              d="M0,400 L200,400 L200,300 L500,300 L500,400 L800,400 L800,500 L1200,500"
              stroke="url(#circuitGrad)"
              strokeWidth="2"
              fill="none"
              style={{animationDelay: '1s'}}
            />
          </svg>
        </div>
      </div>

      {/* Robot Illustration */}
      <div className={styles.robotContainer}>
        <svg className={styles.robotIllustration} viewBox="0 0 300 300" xmlns="http://www.w3.org/2000/svg">
          <defs>
            <linearGradient id="robotGrad" x1="0%" y1="0%" x2="100%" y2="100%">
              <stop offset="0%" stopColor="#475569" />
              <stop offset="100%" stopColor="#1e293b" />
            </linearGradient>
            <filter id="glow">
              <feGaussianBlur stdDeviation="4" result="coloredBlur" />
              <feMerge>
                <feMergeNode in="coloredBlur" />
                <feMergeNode in="SourceGraphic" />
              </feMerge>
            </filter>
          </defs>

          <g className={styles.robotBody}>
            <rect x="100" y="140" width="100" height="100" rx="12" fill="url(#robotGrad)" />
            <rect x="70" y="160" width="30" height="60" rx="8" fill="#64748b" className={styles.robotArmLeft} />
            <rect x="200" y="160" width="30" height="60" rx="8" fill="#64748b" className={styles.robotArmRight} />
          </g>

          <g className={styles.robotHead}>
            <rect x="110" y="70" width="80" height="70" rx="8" fill="url(#robotGrad)" />
            <circle cx="130" cy="100" r="10" fill="#3b82f6" className={styles.robotEye} filter="url(#glow)" />
            <circle cx="170" cy="100" r="10" fill="#3b82f6" className={styles.robotEye} filter="url(#glow)" />
            <rect x="145" y="50" width="10" height="20" fill="#64748b" />
            <circle cx="150" cy="45" r="6" fill="#3b82f6" filter="url(#glow)" className={styles.robotAntenna} />
            <path d="M 130 120 Q 150 130 170 120" stroke="#3b82f6" strokeWidth="3" fill="none" strokeLinecap="round" />
          </g>

          <circle className={styles.particle} cx="50" cy="100" r="3" fill="#3b82f6" opacity="0.6" />
          <circle className={styles.particle} cx="250" cy="150" r="2" fill="#3b82f6" opacity="0.4" style={{animationDelay: '0.5s'}} />
          <circle className={styles.particle} cx="80" cy="220" r="2.5" fill="#3b82f6" opacity="0.5" style={{animationDelay: '1s'}} />
        </svg>
      </div>

      {/* Content */}
      <Container size="full">
        <div className={styles.heroContent}>
          <div className={styles.heroLabel}>
            <span className={styles.labelDot} />
            Industry-Leading Robotics Education
          </div>

          <Heading as="h1" className={styles.heroTitle}>
            <span className={styles.titleLine}>
              <span className={styles.titleMain}>{siteConfig.title}</span>
            </span>
            <span className={styles.titleGradient}>Architect. Simulate. Deploy.</span>
          </Heading>

          <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
          <p className={styles.heroDescription}>
            Enterprise-grade curriculum designed by industry veterans. Master ROS 2, digital twins, and
            AI-powered autonomous systems with production-ready implementations.
          </p>

          <div className={styles.heroCtaGroup}>
            <PrimaryButton href="/docs/preface" size="large">
              Explore Ainoxa
            </PrimaryButton>
            <SecondaryButton href="/docs/module2/module-overview" size="large">
              Start Reading
            </SecondaryButton>
          </div>

          <div className={styles.techStack}>
            <div className={styles.stackLabel}>Powered by</div>
            <div className={styles.stackBadges}>
              {['ROS 2 Humble', 'Gazebo Harmonic', 'Unity 2022 LTS', 'Python 3.10+', 'Modern C++17'].map((tech) => (
                <Badge key={tech}>{tech}</Badge>
              ))}
            </div>
          </div>
        </div>
      </Container>

      {/* Scroll Indicator */}
      <div className={styles.scrollIndicator}>
        <div className={styles.scrollMouse}>
          <div className={styles.scrollWheel} />
        </div>
        <span>Scroll to explore</span>
      </div>
    </header>
  );
}

/**
 * Key Features Section
 * Highlights core curriculum features with icons
 */
function FeaturesSection() {
  const features = [
    {
      title: 'ROS 2 Fundamentals',
      description: 'Master distributed middleware, pub-sub patterns, and production-grade architecture',
      icon: '⚙️',
    },
    {
      title: 'Digital Twin Engineering',
      description: 'Build high-fidelity simulations with Gazebo physics and Unity rendering',
      icon: '🤖',
    },
    {
      title: 'Sensor Fusion',
      description: 'Integrate LiDAR, cameras, and IMU data for robust perception systems',
      icon: '📡',
    },
    {
      title: 'Autonomous Navigation',
      description: 'Implement SLAM, path planning, and real-time decision making',
      icon: '🗺️',
    },
  ];

  return (
    <Section variant="default">
      <Container size="lg">
        <div className={styles.sectionHeader}>
          <div className={styles.sectionLabel}>Core Modules</div>
          <Heading as="h2" className={styles.sectionHeading}>
            What You Will Master
          </Heading>
          <p className={styles.sectionText}>
            Progressive curriculum covering foundational concepts to advanced deployment strategies
          </p>
        </div>

        <Grid cols={{mobile: 1, tablet: 2, desktop: 4}} gap="lg">
          {features.map((feature) => (
            <FeatureCard
              key={feature.title}
              title={feature.title}
              description={feature.description}
              icon={<span className={styles.featureIcon}>{feature.icon}</span>}
            />
          ))}
        </Grid>
      </Container>
    </Section>
  );
}

/**
 * Metrics Section
 * Display impressive statistics about the curriculum
 */
function MetricsSection() {
  const metrics = [
    {
      value: '3',
      label: 'Core Modules',
      sublabel: 'Progressive Learning Path',
    },
    {
      value: '15+',
      label: 'Chapters',
      sublabel: 'Hands-On Content',
    },
    {
      value: '50+',
      label: 'Lab Exercises',
      sublabel: 'Real-World Projects',
    },
    {
      value: '100%',
      label: 'Industry Grade',
      sublabel: 'Production Ready',
    },
  ];

  return (
    <Section variant="dark">
      <Container size="xl">
        <Grid cols={{mobile: 1, tablet: 2, desktop: 4}} gap="md">
          {metrics.map((metric) => (
            <div key={metric.label} className={styles.metricCard}>
              <div className={styles.metricValue}>{metric.value}</div>
              <div className={styles.metricLabel}>{metric.label}</div>
              <div className={styles.metricSublabel}>{metric.sublabel}</div>
            </div>
          ))}
        </Grid>
      </Container>
    </Section>
  );
}

/**
 * Modules Section
 * Display available learning modules with progress and metadata
 */
function ModulesSection() {
  const modules = [
    {
      number: 1,
      title: 'Robotic Operating System',
      description: 'Master distributed middleware, pub-sub patterns, service-oriented architecture, and URDF modeling.',
      chapters: 5,
      duration: '8-10 hours',
      difficulty: 'beginner' as const,
      progress: 40,
      tags: ['ROS 2', 'DDS', 'URDF', 'Python', 'C++'],
      status: 'in-progress' as const,
      href: '/docs/module1/module-overview',
    },
    {
      number: 2,
      title: 'Digital Twin Engineering',
      description: 'Build high-fidelity digital twins with Gazebo physics and Unity rendering pipelines.',
      chapters: 6,
      duration: '12-15 hours',
      difficulty: 'intermediate' as const,
      progress: 100,
      tags: ['Gazebo', 'Unity', 'Physics', 'Graphics'],
      status: 'completed' as const,
      href: '/docs/module2/module-overview',
    },
    {
      number: 3,
      title: 'Autonomous Intelligence',
      description: 'Implement perception, SLAM, path planning, and Vision-Language-Action models.',
      chapters: 7,
      duration: '15-20 hours',
      difficulty: 'advanced' as const,
      progress: 0,
      tags: ['CV', 'SLAM', 'Nav2', 'VLA', 'ML'],
      status: 'locked' as const,
      href: '#',
    },
  ];

  return (
    <Section variant="default">
      <Container size="lg">
        <div className={styles.sectionHeader}>
          <div className={styles.sectionLabel}>Curriculum</div>
          <Heading as="h2" className={styles.sectionHeading}>
            Three-Tier Mastery Framework
          </Heading>
          <p className={styles.sectionText}>
            Progressive modules designed for enterprise robotics engineering. Build comprehensive competency from
            foundational concepts to production deployment.
          </p>
        </div>

        <Grid cols={{mobile: 1, tablet: 2, desktop: 3}} gap="lg">
          {modules.map((module) => (
            <ModuleCard
              key={module.number}
              number={module.number}
              title={module.title}
              description={module.description}
              chapters={module.chapters}
              duration={module.duration}
              difficulty={module.difficulty}
              progress={module.progress}
              tags={module.tags}
              status={module.status}
              href={module.href}
            />
          ))}
        </Grid>
      </Container>
    </Section>
  );
}

/**
 * Enterprise Features Section
 * Highlight production-ready features
 */
function EnterpriseSection() {
  const features = [
    {
      title: 'Test-Driven Development',
      description: 'Comprehensive pytest suites, simulation validation, and hardware-in-the-loop testing',
    },
    {
      title: 'Containerized Deployment',
      description: 'Docker-based workflows with multi-stage builds and optimized runtime environments',
    },
    {
      title: 'Real-Time Performance',
      description: 'Optimized for <10ms latency, deterministic execution, and resource-constrained platforms',
    },
    {
      title: 'Security First',
      description: 'ROS 2 security enclaves, encrypted communications, and secure credential management',
    },
  ];

  return (
    <Section variant="dark">
      <Container size="lg">
        <div className={styles.enterpriseGrid}>
          <div className={styles.enterpriseContent}>
            <div className={styles.sectionLabel}>Enterprise-Grade</div>
            <Heading as="h2" className={styles.sectionHeading}>
              Production-Ready Architecture
            </Heading>
            <p className={styles.enterpriseDescription}>
              Built on principles used by Fortune 500 robotics companies. Every module includes comprehensive
              testing, CI/CD integration, and deployment strategies for mission-critical systems.
            </p>

            <div className={styles.enterpriseFeatures}>
              {features.map((feature) => (
                <div key={feature.title} className={styles.enterpriseFeature}>
                  <div className={styles.featureDot} />
                  <div>
                    <h4 className={styles.featureTitle}>{feature.title}</h4>
                    <p className={styles.featureDescription}>{feature.description}</p>
                  </div>
                </div>
              ))}
            </div>

            <div className={styles.enterpriseCta}>
              <PrimaryButton href="/docs/preface" size="large">
                Start Building
              </PrimaryButton>
            </div>
          </div>

          <div className={styles.enterpriseVisual}>
            <div className={styles.codeWindow}>
              <div className={styles.codeHeader}>
                <div className={styles.codeDots}>
                  <span />
                  <span />
                  <span />
                </div>
                <span className={styles.codeTitle}>ros2_workspace/src/</span>
              </div>
              <div className={styles.codeBody}>
                <pre className={styles.codeContent}>{`# Production-grade ROS 2 node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class AutonomousNavigator(Node):
    def __init__(self):
        super().__init__('navigator')

        # Subscribe to LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, 10
        )

        self.get_logger().info(
            'Autonomous system initialized'
        )`}</pre>
              </div>
            </div>
          </div>
        </div>
      </Container>
    </Section>
  );
}

/**
 * Final CTA Section
 */
function CtaSection() {
  return (
    <Section variant="elevated">
      <Container size="md">
        <div className={styles.ctaContainer}>
          <Heading as="h2" className={styles.ctaHeading}>
            Ready to Master Robotics?
          </Heading>
          <p className={styles.ctaText}>
            Join engineers from leading robotics companies learning with our enterprise-grade curriculum.
          </p>
          <div className={styles.ctaButtons}>
            <PrimaryButton href="/docs/preface" size="large">
              Get Started Now
            </PrimaryButton>
          </div>
        </div>
      </Container>
    </Section>
  );
}

/**
 * Home Page
 */
export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title="Home"
      description="Enterprise-grade robotics education. Master ROS 2, digital twins, and autonomous systems."
    >
      <HomepageHeader />
      <FeaturesSection />
      <MetricsSection />
      <ModulesSection />
      <EnterpriseSection />
      <CtaSection />
    </Layout>
  );
}
