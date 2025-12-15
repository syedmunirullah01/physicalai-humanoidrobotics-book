import React, { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './Section.module.css';

type SectionVariant = 'default' | 'dark' | 'elevated';

interface SectionProps {
  children: ReactNode;
  variant?: SectionVariant;
  withBorder?: boolean;
  className?: string;
}

/**
 * Section Component
 *
 * Full-width section container with vertical padding and optional border.
 * Used for major page sections with distinct visual separation.
 *
 * Variants:
 * - default: Standard dark background
 * - dark: Darker, elevated appearance
 * - elevated: Contrast against default
 *
 * @example
 * <Section variant="dark" withBorder>
 *   <Container>
 *     <h2>Advanced Topics</h2>
 *   </Container>
 * </Section>
 */
export const Section = React.forwardRef<HTMLDivElement, SectionProps>(
  ({ children, variant = 'default', withBorder = false, className }, ref) => {
    return (
      <section
        ref={ref}
        className={clsx(
          styles.section,
          styles[variant],
          withBorder && styles.withBorder,
          className
        )}
      >
        {children}
      </section>
    );
  }
);

Section.displayName = 'Section';

export default Section;
