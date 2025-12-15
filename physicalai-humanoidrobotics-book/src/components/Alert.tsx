import React, { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './Alert.module.css';

type AlertType = 'info' | 'success' | 'warning' | 'error';

interface AlertProps {
  children: ReactNode;
  type?: AlertType;
  title?: string;
  icon?: ReactNode;
  dismissible?: boolean;
  onDismiss?: () => void;
  className?: string;
}

/**
 * Alert Component
 *
 * Displays important information, success messages, warnings, or errors to the user.
 * Supports dismissible alerts, custom icons, and type-based styling.
 *
 * Design: Colored left border, subtle background, left-aligned icon
 * Accessibility: ARIA role="alert", semantic structure, keyboard focus
 *
 * @example
 * <Alert type="warning" title="Deprecated">This API will be removed in v2.0</Alert>
 * <Alert type="success" icon={<CheckIcon />} dismissible onDismiss={handleClose}>
 *   Module completed successfully!
 * </Alert>
 */
export const Alert = React.forwardRef<HTMLDivElement, AlertProps>(
  (
    {
      children,
      type = 'info',
      title,
      icon,
      dismissible = false,
      onDismiss,
      className,
    },
    ref
  ) => {
    const [isDismissed, setIsDismissed] = React.useState(false);

    const handleDismiss = () => {
      setIsDismissed(true);
      onDismiss?.();
    };

    if (isDismissed) return null;

    const alertClasses = clsx(styles.alert, styles[type], className);

    return (
      <div
        ref={ref}
        className={alertClasses}
        role="alert"
        aria-live="polite"
        aria-atomic="true"
      >
        <div className={styles.content}>
          {icon && <span className={styles.icon}>{icon}</span>}

          <div className={styles.text}>
            {title && <p className={styles.title}>{title}</p>}
            <p className={styles.message}>{children}</p>
          </div>
        </div>

        {dismissible && (
          <button
            className={styles.dismissButton}
            onClick={handleDismiss}
            aria-label="Dismiss alert"
            type="button"
          >
            <svg
              width="20"
              height="20"
              viewBox="0 0 20 20"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
            >
              <path d="M15 5L5 15M5 5l10 10" />
            </svg>
          </button>
        )}
      </div>
    );
  }
);

Alert.displayName = 'Alert';

export default Alert;
