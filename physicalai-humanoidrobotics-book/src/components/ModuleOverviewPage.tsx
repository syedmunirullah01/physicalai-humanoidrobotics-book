/**
 * ModuleOverviewPage - Professional module structure component
 * Implements research-grade module presentation with clear hierarchy
 */

import React, { ReactNode } from 'react';
import styles from './ModuleOverviewPage.module.css';
import { Container, Grid, Section, PrimaryButton, SecondaryButton, Badge } from '@site/src/components';

export interface Chapter {
  id: string;
  number: number;
  title: string;
  description: string;
  bloomLevel: string;
  duration: string;
  link?: string;
}

export interface LearningOutcome {
  level: string;
  title: string;
  items: string[];
}

export interface Prerequisite {
  category: string;
  icon: string;
  items: string[];
}

export interface TechStackItem {
  category: string;
  tools: string[];
}

export interface ModuleOverviewPageProps {
  moduleNumber: number;
  moduleTitle: string;
  description: string;
  duration: string;
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  chapters: Chapter[];
  outcomes?: LearningOutcome[];
  prerequisites?: Prerequisite[];
  techStack?: TechStackItem[];
  totalHours?: string;
  ctaText?: string;
  ctaLink?: string;
  ctaSecondaryText?: string;
  ctaSecondaryLink?: string;
}

/**
 * ModuleOverviewPage Component
 *
 * Displays comprehensive module structure with:
 * - Professional header with gradient branding
 * - Chapter progression list with metadata
 * - Learning outcomes organized by Bloom's level
 * - Prerequisites and tech stack
 * - Clear call-to-action to begin module
 *
 * Props:
 *   - moduleNumber: Module identifier (1, 2, 3, etc.)
 *   - moduleTitle: Primary module title
 *   - description: Module summary/overview text
 *   - duration: Expected completion time
 *   - difficulty: Skill level indicator
 *   - chapters: Array of chapter objects with metadata
 *   - outcomes: Learning outcomes by Bloom's level
 *   - prerequisites: Required knowledge/software
 *   - techStack: Tools and technologies used
 *   - totalHours: Total module duration estimate
 *   - ctaText: Primary button text
 *   - ctaLink: Primary button link
 */
export function ModuleOverviewPage({
  moduleNumber,
  moduleTitle,
  description,
  duration,
  difficulty,
  chapters,
  outcomes = [],
  prerequisites = [],
  techStack = [],
  totalHours,
  ctaText = 'Start Module',
  ctaLink = '#',
  ctaSecondaryText = 'View Documentation',
  ctaSecondaryLink = '#',
}: ModuleOverviewPageProps): JSX.Element {
  const difficultyColor =
    difficulty === 'beginner'
      ? 'default'
      : difficulty === 'intermediate'
        ? 'warning'
        : 'custom';

  return (
    <div className={styles.container}>
      {/* ==================== MODULE HEADER ==================== */}
      <div className={styles.moduleHeader}>
        <div className={styles.moduleHeaderContent}>
          <div className={styles.moduleTitle}>
            Module {moduleNumber}: {moduleTitle}
          </div>
          <p className={styles.moduleDescription}>{description}</p>

          {/* Meta Information */}
          <div className={styles.moduleMetaInfo}>
            <div className={styles.metaItem}>
              <span className={styles.metaLabel}>Duration:</span>
              <span className={styles.metaValue}>{duration}</span>
            </div>
            <div className={styles.metaItem}>
              <span className={styles.metaLabel}>Difficulty:</span>
              <Badge variant={difficultyColor}>{difficulty.charAt(0).toUpperCase() + difficulty.slice(1)}</Badge>
            </div>
            {totalHours && (
              <div className={styles.metaItem}>
                <span className={styles.metaLabel}>Total Hours:</span>
                <span className={styles.metaValue}>{totalHours}</span>
              </div>
            )}
            <div className={styles.metaItem}>
              <span className={styles.metaLabel}>Chapters:</span>
              <span className={styles.metaValue}>{chapters.length}</span>
            </div>
          </div>
        </div>
      </div>

      {/* ==================== MAIN CONTENT ==================== */}
      <div className={styles.contentWrapper}>
        <div className={styles.mainContent}>
          {/* ==================== CHAPTERS SECTION ==================== */}
          <section className={styles.chaptersSection}>
            <h2 className={styles.sectionTitle}>
              <span className={styles.sectionTitleIcon}>📚</span>
              Chapter Progression
            </h2>

            <div className={styles.chaptersList}>
              {chapters.map((chapter) => (
                <a
                  key={chapter.id}
                  href={chapter.link || '#'}
                  className={styles.chapterItem}
                  role="article"
                  aria-label={`${chapter.title}, ${chapter.duration}`}
                >
                  <div className={styles.chapterNumber}>Chapter {chapter.number}</div>
                  <div className={styles.chapterTitle}>{chapter.title}</div>
                  <p className={styles.chapterDescription}>{chapter.description}</p>

                  <div className={styles.chapterFooter}>
                    <span className={styles.chapterTime}>⏱️ {chapter.duration}</span>
                    <span className={styles.chapterBloom}>{chapter.bloomLevel}</span>
                  </div>
                </a>
              ))}
            </div>
          </section>

          {/* ==================== LEARNING OUTCOMES ==================== */}
          {outcomes.length > 0 && (
            <section className={styles.outcomesSection}>
              <h2 className={styles.sectionTitle}>
                <span className={styles.sectionTitleIcon}>🎯</span>
                Learning Outcomes
              </h2>

              <div className={styles.outcomesGrid}>
                {outcomes.map((outcome, idx) => (
                  <div key={idx} className={styles.outcomeCard}>
                    <span className={styles.outcomeLevel}>{outcome.level}</span>
                    <h3 className={styles.outcomeTitle}>{outcome.title}</h3>
                    <ul className={styles.outcomeList}>
                      {outcome.items.map((item, itemIdx) => (
                        <li key={itemIdx} className={styles.outcomeItem}>
                          <span className={styles.outcomeCheckmark}>✓</span>
                          <span>{item}</span>
                        </li>
                      ))}
                    </ul>
                  </div>
                ))}
              </div>
            </section>
          )}

          {/* ==================== PREREQUISITES ==================== */}
          {prerequisites.length > 0 && (
            <section className={styles.prerequisitesSection}>
              <h2 className={styles.sectionTitle}>
                <span className={styles.sectionTitleIcon}>✅</span>
                Prerequisites
              </h2>

              <div className={styles.prerequisitesList}>
                {prerequisites.map((prereq, idx) => (
                  <div key={idx} className={styles.prerequisiteCategory}>
                    <h3 className={styles.prerequisiteCategoryTitle}>
                      <span>{prereq.icon}</span>
                      {prereq.category}
                    </h3>
                    <ul className={styles.prerequisiteItems}>
                      {prereq.items.map((item, itemIdx) => (
                        <li key={itemIdx} className={styles.prerequisiteItem}>
                          <span className={styles.prerequisiteIcon}>■</span>
                          <span>{item}</span>
                        </li>
                      ))}
                    </ul>
                  </div>
                ))}
              </div>
            </section>
          )}

          {/* ==================== TECH STACK ==================== */}
          {techStack.length > 0 && (
            <section className={styles.techStackSection}>
              <h2 className={styles.sectionTitle}>
                <span className={styles.sectionTitleIcon}>⚙️</span>
                Tools & Technologies
              </h2>

              <div className={styles.techGrid}>
                {techStack.map((stack, idx) => (
                  <div key={idx} className={styles.techCategory}>
                    <h3 className={styles.techCategoryTitle}>{stack.category}</h3>
                    <ul className={styles.techList}>
                      {stack.tools.map((tool, toolIdx) => (
                        <li key={toolIdx} className={styles.techItem}>
                          {tool}
                        </li>
                      ))}
                    </ul>
                  </div>
                ))}
              </div>
            </section>
          )}

          {/* ==================== CTA SECTION ==================== */}
          <section className={styles.ctaSection}>
            <h2 className={styles.ctaTitle}>Ready to Master This Module?</h2>
            <p className={styles.ctaDescription}>
              Begin your journey through {moduleTitle}. Complete all chapters and the integration project to unlock the next module.
            </p>
            <div className={styles.ctaButtons}>
              <PrimaryButton href={ctaLink} size="large">
                {ctaText} →
              </PrimaryButton>
              <SecondaryButton href={ctaSecondaryLink} size="large">
                {ctaSecondaryText}
              </SecondaryButton>
            </div>
          </section>
        </div>

        {/* ==================== SIDEBAR ==================== */}
        <aside className={styles.sidebar}>
          {/* Quick Stats Card */}
          <div className={styles.sidebarCard}>
            <h3 className={styles.cardTitle}>
              <span className={styles.cardIcon}>📊</span>
              Quick Stats
            </h3>
            <div className={styles.infoText}>
              <div style={{ marginBottom: 'var(--spacing-md)' }}>
                <strong>{chapters.length} Chapters</strong> to complete
              </div>
              <div style={{ marginBottom: 'var(--spacing-md)' }}>
                <strong>{totalHours || 'Variable'}</strong> total hours
              </div>
              <div>
                <strong>{difficulty.charAt(0).toUpperCase() + difficulty.slice(1)}</strong> difficulty
              </div>
            </div>
          </div>

          {/* Prerequisites Quick Check */}
          {prerequisites.length > 0 && (
            <div className={styles.sidebarCard}>
              <h3 className={styles.cardTitle}>
                <span className={styles.cardIcon}>✅</span>
                Prerequisites
              </h3>
              <p className={styles.infoText} style={{ marginBottom: 'var(--spacing-md)' }}>
                Ensure you have the required knowledge before starting.
              </p>
              <a href="#prerequisites" className={styles.linkButton}>
                <span>View all →</span>
              </a>
            </div>
          )}

          {/* Tech Stack Overview */}
          {techStack.length > 0 && (
            <div className={styles.sidebarCard}>
              <h3 className={styles.cardTitle}>
                <span className={styles.cardIcon}>⚙️</span>
                Tech Stack
              </h3>
              <div className={styles.badgeList}>
                {techStack
                  .flatMap((stack) => stack.tools)
                  .slice(0, 4)
                  .map((tool, idx) => (
                    <Badge key={idx} variant="default">
                      {tool}
                    </Badge>
                  ))}
                {techStack.flatMap((stack) => stack.tools).length > 4 && (
                  <Badge variant="default">+{techStack.flatMap((stack) => stack.tools).length - 4} more</Badge>
                )}
              </div>
              <a href="#tech-stack" className={styles.linkButton}>
                <span>View all →</span>
              </a>
            </div>
          )}

          {/* Support Card */}
          <div className={styles.sidebarCard}>
            <h3 className={styles.cardTitle}>
              <span className={styles.cardIcon}>💬</span>
              Need Help?
            </h3>
            <p className={styles.infoText}>
              Join our community Discord or check the troubleshooting guide in each chapter.
            </p>
            <a href="https://discord.gg/your-invite" className={styles.linkButton}>
              <span>Join Discord →</span>
            </a>
          </div>
        </aside>
      </div>
    </div>
  );
}

export type { ModuleOverviewPageProps };
