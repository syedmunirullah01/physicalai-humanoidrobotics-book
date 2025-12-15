import React, { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './Badge.module.css';

type BadgeVariant = 'default' | 'success' | 'warning' | 'error' | 'custom';

interface BadgeProps {
  children: ReactNode;
  variant?: BadgeVariant;
  icon?: ReactNode;
  className?: string;
}

/**
 * Badge Component
 *
 * Small, pill-shaped label for tags, statuses, or metadata.
 * Commonly used for technology tags, difficulty indicators, or status badges.
 *
 * Features:
 * - Multiple color variants (default: cyan, success: green, warning: orange, error: red)
 * - Optional icon support
 * - Semantic color meaning
 * - Compact design (4px 12px padding)
 * - High contrast borders
 *
 * @example
 * <Badge variant="default">Python</Badge>
 * <Badge variant="success">Completed</Badge>
 * <Badge variant="warning">Deprecated</Badge>
 * <Badge icon={<CheckIcon />} variant="success">Done</Badge>
 */
export const Badge = React.forwardRef<HTMLSpanElement, BadgeProps>(
  ({ children, variant = 'default', icon, className }, ref) => {
    return (
      <span ref={ref} className={clsx(styles.badge, styles[variant], className)}>
        {icon && <span className={styles.icon}>{icon}</span>}
        <span className={styles.text}>{children}</span>
      </span>
    );
  }
);

Badge.displayName = 'Badge';

export default Badge;
