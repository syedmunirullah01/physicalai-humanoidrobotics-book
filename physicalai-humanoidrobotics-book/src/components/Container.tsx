import React, { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './Container.module.css';

type ContainerSize = 'sm' | 'md' | 'lg' | 'xl' | 'full';

interface ContainerProps {
  children: ReactNode;
  size?: ContainerSize;
  className?: string;
}

/**
 * Container Component
 *
 * Consistent content width wrapper with horizontal centering.
 * Used for consistent page and section widths.
 *
 * Sizes:
 * - sm: 640px
 * - md: 768px
 * - lg: 1024px (docs content)
 * - xl: 1280px (default for pages)
 * - full: 100% (hero sections, full-width)
 *
 * @example
 * <Container size="lg">
 *   <h1>Learning Module</h1>
 * </Container>
 */
export const Container = React.forwardRef<HTMLDivElement, ContainerProps>(
  ({ children, size = 'xl', className }, ref) => {
    return (
      <div ref={ref} className={clsx(styles.container, styles[size], className)}>
        {children}
      </div>
    );
  }
);

Container.displayName = 'Container';

export default Container;
