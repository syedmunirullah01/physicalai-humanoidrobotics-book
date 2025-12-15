import React, { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './Grid.module.css';

type GridCols = 1 | 2 | 3 | 4;
type GapSize = 'sm' | 'md' | 'lg';

interface GridProps {
  children: ReactNode;
  cols?: {
    mobile?: GridCols;
    tablet?: GridCols;
    desktop?: GridCols;
  };
  gap?: GapSize;
  className?: string;
}

/**
 * Grid Component
 *
 * Responsive multi-column layout grid.
 * Automatically adjusts columns based on breakpoints.
 *
 * @example
 * <Grid cols={{ mobile: 1, tablet: 2, desktop: 3 }} gap="lg">
 *   <Card />
 *   <Card />
 *   <Card />
 * </Grid>
 */
export const Grid = React.forwardRef<HTMLDivElement, GridProps>(
  (
    {
      children,
      cols = { mobile: 1, tablet: 2, desktop: 3 },
      gap = 'md',
      className,
    },
    ref
  ) => {
    const gridStyle = {
      '--grid-cols-mobile': cols.mobile || 1,
      '--grid-cols-tablet': cols.tablet || 2,
      '--grid-cols-desktop': cols.desktop || 3,
    } as React.CSSProperties;

    return (
      <div
        ref={ref}
        className={clsx(styles.grid, styles[`gap-${gap}`], className)}
        style={gridStyle}
      >
        {children}
      </div>
    );
  }
);

Grid.displayName = 'Grid';

export default Grid;
