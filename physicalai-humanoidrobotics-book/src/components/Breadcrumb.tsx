import React from 'react';
import Link from '@docusaurus/Link';
import styles from './Breadcrumb.module.css';

interface BreadcrumbItem {
  label: string;
  href?: string;
}

interface BreadcrumbProps {
  items: BreadcrumbItem[];
  className?: string;
}

/**
 * Breadcrumb Component
 *
 * Navigation path indicator showing the user's location in the site hierarchy.
 * Example: Home / Module 1 / Chapter 2
 *
 * @example
 * <Breadcrumb
 *   items={[
 *     { label: 'Home', href: '/' },
 *     { label: 'Module 1', href: '/module-1' },
 *     { label: 'Chapter 2' },
 *   ]}
 * />
 */
export const Breadcrumb = React.forwardRef<HTMLNavElement, BreadcrumbProps>(
  ({ items, className }, ref) => {
    return (
      <nav ref={ref} className={`${styles.breadcrumb} ${className || ''}`} aria-label="Breadcrumb">
        <ol className={styles.list}>
          {items.map((item, index) => {
            const isLast = index === items.length - 1;
            return (
              <li key={index} className={styles.item}>
                {!isLast && item.href ? (
                  <>
                    <Link to={item.href} className={styles.link}>
                      {item.label}
                    </Link>
                    <span className={styles.separator}>/</span>
                  </>
                ) : (
                  <span className={styles.current}>{item.label}</span>
                )}
              </li>
            );
          })}
        </ol>
      </nav>
    );
  }
);

Breadcrumb.displayName = 'Breadcrumb';

export default Breadcrumb;
