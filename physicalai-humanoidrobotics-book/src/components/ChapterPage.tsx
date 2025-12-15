/**
 * ChapterPage - Professional chapter reading layout
 * Implements research-grade reading experience with proper hierarchy
 */

import React, { ReactNode, useMemo } from 'react';
import styles from './ChapterPage.module.css';
import { Breadcrumb, type BreadcrumbProps } from '@site/src/components';

export interface TOCItem {
  id: string;
  title: string;
  level: number;
  children?: TOCItem[];
}

export interface NavChapter {
  title: string;
  link?: string;
}

export interface ChapterPageProps {
  moduleNumber: number;
  moduleName: string;
  chapterNumber: number;
  chapterTitle: string;
  description?: string;
  bloomLevel?: 'Understand' | 'Apply' | 'Analyze' | 'Evaluate' | 'Create';
  estimatedTime?: string;
  breadcrumbs?: BreadcrumbProps['items'];
  tableOfContents?: TOCItem[];
  progress?: number;
  previousChapter?: NavChapter;
  nextChapter?: NavChapter;
  children: ReactNode;
}

/**
 * ChapterPage Component
 *
 * Provides comprehensive chapter reading layout with:
 * - Breadcrumb navigation showing learning path
 * - Progress indicator for module completion
 * - Table of contents sidebar with jump links
 * - Semantic article content area
 * - Chapter navigation (previous/next)
 * - Learning resources and support links
 *
 * Props:
 *   - moduleNumber: Module identifier (1, 2, etc.)
 *   - moduleName: Full module title
 *   - chapterNumber: Chapter identifier (1, 2, etc.)
 *   - chapterTitle: Chapter heading
 *   - bloomLevel: Cognitive level (Understand, Apply, etc.)
 *   - estimatedTime: Time to complete chapter
 *   - breadcrumbs: Navigation breadcrumb items
 *   - tableOfContents: Heading structure for TOC sidebar
 *   - progress: Module completion percentage (0-100)
 *   - previousChapter: Previous chapter metadata
 *   - nextChapter: Next chapter metadata
 *   - children: Chapter content (MDX/HTML)
 */
export function ChapterPage({
  moduleNumber,
  moduleName,
  chapterNumber,
  chapterTitle,
  description,
  bloomLevel = 'Apply',
  estimatedTime = '2-3 hours',
  breadcrumbs,
  tableOfContents = [],
  progress = 0,
  previousChapter,
  nextChapter,
  children,
}: ChapterPageProps): JSX.Element {
  // Build default breadcrumbs if not provided
  const defaultBreadcrumbs: BreadcrumbProps['items'] = [
    { label: 'Home', href: '/' },
    { label: 'Learning', href: '/docs' },
    { label: moduleName, href: `/docs/module${moduleNumber}/module-overview` },
    { label: chapterTitle }, // Current page - no href
  ];

  const finalBreadcrumbs = breadcrumbs || defaultBreadcrumbs;

  return (
    <div className={styles.container}>
      {/* ==================== BREADCRUMB NAVIGATION ==================== */}
      <div className={styles.breadcrumbWrapper}>
        <div className={styles.breadcrumbContent}>
          <Breadcrumb items={finalBreadcrumbs} />
        </div>
      </div>

      {/* ==================== MAIN LAYOUT ==================== */}
      <div className={styles.mainLayout}>
        {/* ==================== ARTICLE CONTENT ==================== */}
        <main className={styles.articleContent}>
          {/* Chapter Header */}
          <header className={styles.chapterHeader}>
            <div className={styles.chapterMeta}>
              <span className={styles.chapterNumber}>Module {moduleNumber}</span>
              <span className={styles.chapterModule}>Chapter {chapterNumber}</span>
              {bloomLevel && <span className={styles.chapterBloomTag}>{bloomLevel}</span>}
            </div>

            <h1 className={styles.chapterTitle}>{chapterTitle}</h1>

            {description && <p style={{ color: 'var(--color-text-secondary)', marginTop: 'var(--spacing-md)' }}>{description}</p>}

            {estimatedTime && (
              <div className={styles.chapterEstimate}>
                <span>⏱️</span>
                <span>Estimated time: {estimatedTime}</span>
              </div>
            )}
          </header>

          {/* Article Content */}
          <article className={styles.article}>{children}</article>

          {/* Chapter Navigation */}
          <nav className={styles.navSection}>
            <h2 className={styles.navTitle}>Navigate Chapters</h2>
            <div className={styles.navLinks}>
              {previousChapter ? (
                <a href={previousChapter.link} className={styles.navLink}>
                  <span className={styles.navLinkLabel}>← Previous Chapter</span>
                  <span className={styles.navLinkText}>{previousChapter.title}</span>
                </a>
              ) : (
                <div className={`${styles.navLink} ${styles.navLinkDisabled}`}>
                  <span className={styles.navLinkLabel}>← Previous Chapter</span>
                  <span className={styles.navLinkText}>This is the first chapter</span>
                </div>
              )}

              {nextChapter ? (
                <a href={nextChapter.link} className={styles.navLink}>
                  <span className={styles.navLinkLabel}>Next Chapter →</span>
                  <span className={styles.navLinkText}>{nextChapter.title}</span>
                </a>
              ) : (
                <div className={`${styles.navLink} ${styles.navLinkDisabled}`}>
                  <span className={styles.navLinkLabel}>Next Chapter →</span>
                  <span className={styles.navLinkText}>Coming soon</span>
                </div>
              )}
            </div>
          </nav>
        </main>

        {/* ==================== SIDEBAR ==================== */}
        <aside className={styles.sidebar}>
          {/* Progress Indicator */}
          {progress !== undefined && (
            <div className={styles.progressSection}>
              <div className={styles.progressTitle}>Module Progress</div>
              <div className={styles.progressBar}>
                <div className={styles.progressFill} style={{ width: `${progress}%` }} role="progressbar" aria-valuenow={progress} aria-valuemin={0} aria-valuemax={100} />
              </div>
              <div className={styles.progressText}>{progress}% Complete</div>
            </div>
          )}

          {/* Table of Contents */}
          {tableOfContents.length > 0 && (
            <div className={styles.tocSection}>
              <h3 className={styles.tocTitle}>
                <span>📑</span>
                On This Page
              </h3>
              <TOCList items={tableOfContents} />
            </div>
          )}

          {/* Learning Resources Card */}
          <div className={styles.sidebarCard}>
            <h3 className={styles.cardTitle}>
              <span className={styles.cardIcon}>📚</span>
              Learning Resources
            </h3>
            <p className={styles.cardText}>Enhance your learning with additional materials and external resources.</p>
            <ul className={styles.resourcesList}>
              <li className={styles.resourceItem}>
                <span className={styles.resourceIcon}>🔗</span>
                <a href="/docs" className={styles.resourceLink}>
                  Official Documentation
                </a>
              </li>
              <li className={styles.resourceItem}>
                <span className={styles.resourceIcon}>🎥</span>
                <a href="#" className={styles.resourceLink}>
                  Video Tutorial
                </a>
              </li>
              <li className={styles.resourceItem}>
                <span className={styles.resourceIcon}>💻</span>
                <a href="#" className={styles.resourceLink}>
                  Code Examples
                </a>
              </li>
            </ul>
          </div>

          {/* Support Card */}
          <div className={styles.sidebarCard}>
            <h3 className={styles.cardTitle}>
              <span className={styles.cardIcon}>💬</span>
              Get Help
            </h3>
            <p className={styles.cardText}>Stuck? Reach out to our community or check the troubleshooting section.</p>
            <ul className={styles.resourcesList}>
              <li className={styles.resourceItem}>
                <span className={styles.resourceIcon}>🌐</span>
                <a href="https://discord.gg/your-invite" className={styles.resourceLink}>
                  Join Discord
                </a>
              </li>
              <li className={styles.resourceItem}>
                <span className={styles.resourceIcon}>❓</span>
                <a href="#troubleshooting" className={styles.resourceLink}>
                  Troubleshooting
                </a>
              </li>
            </ul>
          </div>
        </aside>
      </div>
    </div>
  );
}

/**
 * Table of Contents List Component
 * Renders hierarchical heading structure
 */
function TOCList({ items, level = 0 }: { items: TOCItem[]; level?: number }): JSX.Element {
  if (items.length === 0) return <></>;

  const isNested = level > 0;

  return (
    <ul className={isNested ? styles.tocNested : styles.tocList}>
      {items.map((item) => (
        <li key={item.id} className={styles.tocItem}>
          <a href={`#${item.id}`} className={styles.tocLink}>
            {item.title}
          </a>
          {item.children && item.children.length > 0 && <TOCList items={item.children} level={level + 1} />}
        </li>
      ))}
    </ul>
  );
}

export type { ChapterPageProps, TOCItem, NavChapter };
