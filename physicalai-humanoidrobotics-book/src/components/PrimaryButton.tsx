import React, { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './PrimaryButton.module.css';

interface PrimaryButtonProps {
  children: ReactNode;
  onClick?: () => void;
  href?: string;
  disabled?: boolean;
  size?: 'small' | 'medium' | 'large';
  fullWidth?: boolean;
  icon?: ReactNode;
  iconPosition?: 'left' | 'right';
  type?: 'button' | 'submit' | 'reset';
  className?: string;
}

/**
 * PrimaryButton Component
 *
 * Renders a primary action button with cyan gradient, glow effects, and smooth interactions.
 * Supports both <button> and <a> tags for navigation and form submission.
 *
 * Design: Professional, high-contrast cyan accent with shadow glow
 * Accessibility: Full keyboard support, visible focus indicator, ARIA labels
 *
 * @example
 * <PrimaryButton onClick={handleClick}>Start Learning</PrimaryButton>
 * <PrimaryButton href="/docs" size="large" icon={<ArrowIcon />}>Continue</PrimaryButton>
 */
export const PrimaryButton = React.forwardRef<
  HTMLButtonElement | HTMLAnchorElement,
  PrimaryButtonProps
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
      iconPosition = 'right',
      type = 'button',
      className,
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
      icon && styles.withIcon
    );

    const iconElement = icon && (
      <span className={clsx(styles.icon, styles[`icon-${iconPosition}`])}>
        {icon}
      </span>
    );

    if (href && !disabled) {
      return (
        <a
          ref={ref as React.Ref<HTMLAnchorElement>}
          href={href}
          className={buttonClasses}
          aria-disabled={disabled}
        >
          {iconPosition === 'left' && iconElement}
          <span className={contentClasses}>{children}</span>
          {iconPosition === 'right' && iconElement}
        </a>
      );
    }

    return (
      <button
        ref={ref as React.Ref<HTMLButtonElement>}
        type={type}
        onClick={onClick}
        disabled={disabled}
        className={buttonClasses}
        aria-disabled={disabled}
      >
        {iconPosition === 'left' && iconElement}
        <span className={contentClasses}>{children}</span>
        {iconPosition === 'right' && iconElement}
      </button>
    );
  }
);

PrimaryButton.displayName = 'PrimaryButton';

export default PrimaryButton;
