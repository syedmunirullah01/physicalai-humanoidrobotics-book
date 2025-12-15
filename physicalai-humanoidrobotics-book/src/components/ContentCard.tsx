import React, { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './ContentCard.module.css';

interface ContentCardProps {
  children: ReactNode;
  title?: ReactNode;
  description?: ReactNode;
  icon?: ReactNode;
  href?: string;
  onClick?: () => void;
  variant?: 'default' | 'feature' | 'elevated';
  hoverable?: boolean;
  className?: string;
}

/**
 * ContentCard Component
 *
 * Versatile card component for displaying content with optional icon, title, and description.
 * Supports hover effects, links, and multiple variants for different use cases.
 *
 * Design: Dark background with cyan border accent, smooth hover transitions
 * Usage: Feature cards, content sections, module listings
 * Accessibility: Semantic structure, keyboard navigable
 *
 * @example
 * <ContentCard
 *   icon={<RobotIcon />}
 *   title="Advanced Kinematics"
 *   description="Master forward and inverse kinematics"
 *   hoverable
 * />
 */
export const ContentCard = React.forwardRef<
  HTMLDivElement | HTMLAnchorElement,
  ContentCardProps
>(
  (
    {
      children,
      title,
      description,
      icon,
      href,
      onClick,
      variant = 'default',
      hoverable = true,
      className,
    },
    ref
  ) => {
    const cardClasses = clsx(
      styles.card,
      styles[variant],
      hoverable && styles.hoverable,
      className
    );

    const content = (
      <>
        {icon && <div className={styles.icon}>{icon}</div>}
        <div className={styles.content}>
          {title && <h3 className={styles.title}>{title}</h3>}
          {description && <p className={styles.description}>{description}</p>}
          {children}
        </div>
      </>
    );

    if (href) {
      return (
        <a
          ref={ref as React.Ref<HTMLAnchorElement>}
          href={href}
          className={cardClasses}
          role="article"
        >
          {content}
        </a>
      );
    }

    return (
      <div
        ref={ref as React.Ref<HTMLDivElement>}
        className={cardClasses}
        onClick={onClick}
        role="article"
      >
        {content}
      </div>
    );
  }
);

ContentCard.displayName = 'ContentCard';

export default ContentCard;
