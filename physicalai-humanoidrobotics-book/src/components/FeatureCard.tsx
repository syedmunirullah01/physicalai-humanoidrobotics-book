import React, { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './FeatureCard.module.css';

interface FeatureCardProps {
  icon: ReactNode;
  title: ReactNode;
  description: ReactNode;
  className?: string;
  ariaLabel?: string;
}

/**
 * FeatureCard Component
 *
 * A compact, icon-focused card for showcasing features in grid layouts.
 * Typically used in 3-4 column grids across homepage or sections.
 *
 * Features:
 * - Icon with cyan glow effect
 * - Clear title and description hierarchy
 * - Hover effects (icon glow intensifies)
 * - No borders, minimal visual weight
 * - Responsive sizing
 *
 * Design Notes:
 * - Icon size: 48x48px (adjusts on mobile)
 * - Title: h4 size with primary text color
 * - Description: secondary text color
 * - Suitable for feature grids: 1 col (mobile), 2 cols (tablet), 3-4 cols (desktop)
 *
 * @example
 * <FeatureCard
 *   icon={<RobotIcon />}
 *   title="Advanced Kinematics"
 *   description="Master forward and inverse kinematics algorithms"
 * />
 */
export const FeatureCard = React.forwardRef<HTMLDivElement, FeatureCardProps>(
  ({ icon, title, description, className, ariaLabel }, ref) => {
    return (
      <div ref={ref} className={clsx(styles.card, className)} aria-label={ariaLabel}>
        <div className={styles.iconContainer}>
          <div className={styles.iconWrapper}>{icon}</div>
        </div>

        <div className={styles.content}>
          <h3 className={styles.title}>{title}</h3>
          <p className={styles.description}>{description}</p>
        </div>
      </div>
    );
  }
);

FeatureCard.displayName = 'FeatureCard';

export default FeatureCard;
