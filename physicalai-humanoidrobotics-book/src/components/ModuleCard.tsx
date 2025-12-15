import React, { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './ModuleCard.module.css';

interface ModuleCardProps {
  title: string;
  description: string;
  number: number;
  chapters: number;
  duration: string;
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  progress: number; // 0-100
  tags?: string[];
  status?: 'locked' | 'available' | 'in-progress' | 'completed';
  href?: string;
  onClick?: () => void;
  className?: string;
}

const difficultyColors = {
  beginner: '#10b981',
  intermediate: '#f59e0b',
  advanced: '#ef4444',
};

/**
 * ModuleCard Component
 *
 * Comprehensive card for displaying module overviews with progress, metadata, and CTAs.
 * Used in curriculum grids to showcase learning modules.
 *
 * Features:
 * - Gradient header (cyan to purple)
 * - Module metadata (chapters, duration, difficulty)
 * - Progress bar with percentage
 * - Status indicator (locked, available, in-progress, completed)
 * - Technology tags
 * - Clear call-to-action button
 * - Responsive design (2-3 cols desktop, 1 col mobile)
 *
 * @example
 * <ModuleCard
 *   title="ROS 2 Fundamentals"
 *   description="Master the core concepts of Robot Operating System 2"
 *   number={1}
 *   chapters={4}
 *   duration="8 hours"
 *   difficulty="beginner"
 *   progress={45}
 *   tags={['ROS 2', 'Python', 'Linux']}
 *   status="in-progress"
 *   href="/module-1"
 * />
 */
export const ModuleCard = React.forwardRef<HTMLDivElement, ModuleCardProps>(
  (
    {
      title,
      description,
      number,
      chapters,
      duration,
      difficulty,
      progress,
      tags = [],
      status = 'available',
      href,
      onClick,
      className,
    },
    ref
  ) => {
    const isLocked = status === 'locked';
    const isCompleted = status === 'completed';
    const isInProgress = status === 'in-progress';

    const handleClick = () => {
      if (onClick) {
        onClick();
      } else if (href && !isLocked) {
        window.location.href = href;
      }
    };

    const CTA = isCompleted ? 'Review' : isInProgress ? 'Continue' : 'Start';

    return (
      <div
        ref={ref}
        className={clsx(styles.card, styles[status], className)}
        onClick={handleClick}
        role="article"
        aria-label={`Module ${number}: ${title}`}
      >
        {/* Header with Gradient Background */}
        <div className={styles.header}>
          <div className={styles.headerContent}>
            <span className={styles.moduleNumber}>Module {number}</span>
            <h3 className={styles.title}>{title}</h3>
          </div>
          {isLocked && <span className={styles.lockIcon}>🔒</span>}
        </div>

        {/* Description */}
        <p className={styles.description}>{description}</p>

        {/* Metadata Grid */}
        <div className={styles.metaGrid}>
          <div className={styles.metaItem}>
            <span className={styles.metaLabel}>Chapters</span>
            <span className={styles.metaValue}>{chapters}</span>
          </div>
          <div className={styles.metaItem}>
            <span className={styles.metaLabel}>Duration</span>
            <span className={styles.metaValue}>{duration}</span>
          </div>
        </div>

        {/* Tags */}
        {tags.length > 0 && (
          <div className={styles.tags}>
            {tags.map((tag) => (
              <span key={tag} className={styles.tag}>
                {tag}
              </span>
            ))}
          </div>
        )}

        {/* Progress Bar */}
        {(isInProgress || isCompleted) && (
          <div className={styles.progressContainer}>
            <div className={styles.progressBar}>
              <div
                className={styles.progressFill}
                style={{ width: `${Math.min(progress, 100)}%` }}
                aria-valuenow={progress}
                aria-valuemin={0}
                aria-valuemax={100}
                role="progressbar"
              />
            </div>
            <span className={styles.progressLabel}>{Math.round(progress)}%</span>
          </div>
        )}

        {/* CTA Button */}
        <button
          className={clsx(styles.ctaButton, isLocked && styles.disabled)}
          onClick={(e) => {
            e.stopPropagation();
            if (!isLocked) {
              handleClick();
            }
          }}
          disabled={isLocked}
          aria-label={`${CTA} module ${number}`}
        >
          {CTA} Module
          <span className={styles.arrowIcon}>→</span>
        </button>

        {/* Status Badge */}
        {isCompleted && (
          <div className={styles.completedBadge}>
            <span>✓</span> Completed
          </div>
        )}

        {isLocked && (
          <div className={styles.lockedOverlay}>
            <p>Complete prerequisites to unlock</p>
          </div>
        )}
      </div>
    );
  }
);

ModuleCard.displayName = 'ModuleCard';

export default ModuleCard;
