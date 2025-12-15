import React from 'react';
import clsx from 'clsx';
import styles from './ProgressBar.module.css';

type ProgressVariant = 'linear' | 'circular' | 'segments';

interface ProgressBarProps {
  value: number; // 0-100
  variant?: ProgressVariant;
  label?: React.ReactNode;
  showPercentage?: boolean;
  animated?: boolean;
  className?: string;
  ariaLabel?: string;
}

/**
 * ProgressBar Component
 *
 * Displays progress or completion status in various formats.
 * Supports linear, circular, and segment variants.
 *
 * @example
 * <ProgressBar value={65} showPercentage />
 * <ProgressBar value={80} variant="circular" />
 */
export const ProgressBar = React.forwardRef<HTMLDivElement, ProgressBarProps>(
  (
    {
      value,
      variant = 'linear',
      label,
      showPercentage = false,
      animated = true,
      className,
      ariaLabel,
    },
    ref
  ) => {
    const clampedValue = Math.min(Math.max(value, 0), 100);

    if (variant === 'circular') {
      const circumference = 2 * Math.PI * 45;
      const strokeDashoffset = circumference - (clampedValue / 100) * circumference;

      return (
        <div
          ref={ref}
          className={clsx(styles.progressCircular, className)}
          role="progressbar"
          aria-valuenow={clampedValue}
          aria-valuemin={0}
          aria-valuemax={100}
          aria-label={ariaLabel}
        >
          <svg className={styles.circularSvg} viewBox="0 0 100 100">
            <circle cx="50" cy="50" r="45" className={styles.circularBg} />
            <circle
              cx="50"
              cy="50"
              r="45"
              className={clsx(styles.circularFill, animated && styles.animated)}
              style={{ strokeDashoffset }}
            />
          </svg>
          {showPercentage && <span className={styles.circularLabel}>{Math.round(clampedValue)}%</span>}
        </div>
      );
    }

    if (variant === 'segments') {
      return (
        <div
          ref={ref}
          className={clsx(styles.progressSegments, className)}
          role="progressbar"
          aria-valuenow={clampedValue}
          aria-valuemin={0}
          aria-valuemax={100}
          aria-label={ariaLabel}
        >
          {[...Array(8)].map((_, i) => (
            <div
              key={i}
              className={clsx(
                styles.segment,
                (i + 1) * 12.5 <= clampedValue && styles.filled
              )}
            />
          ))}
          {label && <span className={styles.segmentLabel}>{label}</span>}
        </div>
      );
    }

    // Default: linear
    return (
      <div
        ref={ref}
        className={clsx(styles.progressLinear, className)}
        role="progressbar"
        aria-valuenow={clampedValue}
        aria-valuemin={0}
        aria-valuemax={100}
        aria-label={ariaLabel}
      >
        <div className={styles.track}>
          <div
            className={clsx(styles.bar, animated && styles.animated)}
            style={{ width: `${clampedValue}%` }}
          />
        </div>
        {showPercentage && <span className={styles.percentage}>{Math.round(clampedValue)}%</span>}
      </div>
    );
  }
);

ProgressBar.displayName = 'ProgressBar';

export default ProgressBar;
