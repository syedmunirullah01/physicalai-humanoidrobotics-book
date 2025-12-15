import React, { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './SecondaryButton.module.css';

interface SecondaryButtonProps {
  children: ReactNode;
  onClick?: () => void;
  href?: string;
  disabled?: boolean;
  size?: 'small' | 'medium' | 'large';
  fullWidth?: boolean;
  icon?: ReactNode;
  iconPosition?: 'left' | 'right';
  className?: string;
  type?: 'button' | 'submit' | 'reset';
  ariaLabel?: string;
}

/**
 * SecondaryButton Component
 *
 * A secondary call-to-action button for alternate actions, cancellations, or less prominent options.
 * Styled with cyan border and transparent background.
 *
 * Features:
 * - Keyboard accessible (Tab, Enter/Space)
 * - Visible focus indicator (cyan outline)
 * - Smooth hover transitions
 * - Multiple size variants
 * - Icon support with configurable position
 * - Optional link rendering (when href provided)
 * - Disabled state support
 * - Respects prefers-reduced-motion
 *
 * @example
 * // Basic usage
 * <SecondaryButton onClick={handleCancel}>Cancel</SecondaryButton>
 *
 * @example
 * // As link
 * <SecondaryButton href="/docs">View Documentation</SecondaryButton>
 *
 * @example
 * // With icon
 * <SecondaryButton icon={<InfoIcon />} iconPosition="left">
 *   Learn More
 * </SecondaryButton>
 */
export const SecondaryButton = React.forwardRef<
  HTMLButtonElement | HTMLAnchorElement,
  SecondaryButtonProps
>(
  (
    {
      children,
      onClick,
      href,
      disabled = false,
      size = 'medium',
      fullWidth = false,
      icon,
      iconPosition = 'left',
      className,
      type = 'button',
      ariaLabel,
    },
    ref
  ) => {
    const buttonClasses = clsx(
      styles.button,
      styles[size],
      fullWidth && styles.fullWidth,
      disabled && styles.disabled,
      className
    );

    const contentClasses = clsx(
      styles.content,
      icon && styles.withIcon,
      icon && styles[`icon${iconPosition === 'right' ? 'Right' : 'Left'}`]
    );

    if (href && !disabled) {
      return (
        <a
          ref={ref as React.ForwardedRef<HTMLAnchorElement>}
          href={href}
          className={buttonClasses}
          aria-label={ariaLabel}
        >
          <span className={contentClasses}>
            {icon && iconPosition === 'left' && <span className={styles.icon}>{icon}</span>}
            <span>{children}</span>
            {icon && iconPosition === 'right' && <span className={styles.icon}>{icon}</span>}
          </span>
        </a>
      );
    }

    return (
      <button
        ref={ref as React.ForwardedRef<HTMLButtonElement>}
        className={buttonClasses}
        onClick={onClick}
        disabled={disabled}
        type={type}
        aria-label={ariaLabel}
        aria-disabled={disabled}
      >
        <span className={contentClasses}>
          {icon && iconPosition === 'left' && <span className={styles.icon}>{icon}</span>}
          <span>{children}</span>
          {icon && iconPosition === 'right' && <span className={styles.icon}>{icon}</span>}
        </span>
      </button>
    );
  }
);

SecondaryButton.displayName = 'SecondaryButton';

export default SecondaryButton;
