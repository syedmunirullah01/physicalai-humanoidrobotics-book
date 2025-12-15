# Physical AI & Humanoid Robotics - Implementation Guide

## Overview

This guide provides step-by-step instructions for implementing the professional UI design system across the Physical AI & Humanoid Robotics platform. It covers component implementation, styling strategy, and integration with Docusaurus.

---

## 1. Project Setup & Configuration

### Current State
- Docusaurus 3.9.2
- React 19.0
- TypeScript 5.6
- CSS Modules (primary styling approach)
- Custom CSS with design tokens

### Build & Development
```bash
# Start development server
npm run start

# Build for production
npm run build

# Type checking
npm run typecheck

# Linting (if configured)
npm run lint
```

---

## 2. Design Token Implementation

### CSS Custom Properties Setup

**File**: `src/css/custom.css`

**Current implementation** includes base tokens. **Enhancements needed**:

```css
/* Add comprehensive design tokens */
:root {
  /* ==================== COLOR TOKENS ==================== */

  /* Primary Cyan */
  --color-primary: #00d9ff;
  --color-primary-light: #22d3ee;
  --color-primary-dark: #0ea5e9;
  --color-primary-darker: #0284c7;

  /* Accent Colors */
  --color-accent-purple: #a855f7;
  --color-accent-pink: #ec4899;

  /* Semantic Colors */
  --color-success: #10b981;
  --color-warning: #f59e0b;
  --color-error: #ef4444;
  --color-info: #3b82f6;

  /* Neutral/Gray Scale */
  --color-slate-50: #f8fafc;
  --color-slate-100: #f1f5f9;
  --color-slate-200: #e2e8f0;
  --color-slate-300: #cbd5e1;
  --color-slate-400: #94a3b8;
  --color-slate-500: #64748b;
  --color-slate-600: #475569;
  --color-slate-700: #334155;
  --color-slate-800: #1e293b;
  --color-slate-900: #0f172a;

  /* Backgrounds */
  --color-bg-primary: #000000;
  --color-bg-secondary: #0a0a0a;
  --color-bg-surface: #0f0f0f;

  /* Text */
  --color-text-primary: #f1f5f9;
  --color-text-secondary: #cbd5e1;
  --color-text-tertiary: #64748b;

  /* ==================== SPACING TOKENS ==================== */

  --spacing-xs: 0.25rem;    /* 4px */
  --spacing-sm: 0.5rem;     /* 8px */
  --spacing-md: 1rem;       /* 16px */
  --spacing-lg: 1.5rem;     /* 24px */
  --spacing-xl: 2rem;       /* 32px */
  --spacing-2xl: 3rem;      /* 48px */
  --spacing-3xl: 4rem;      /* 64px */
  --spacing-4xl: 6rem;      /* 96px */

  /* ==================== TYPOGRAPHY TOKENS ==================== */

  --font-family-base: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Roboto', 'Oxygen', 'Ubuntu', 'Cantarell', 'Fira Sans', 'Droid Sans', 'Helvetica Neue', sans-serif;
  --font-family-mono: 'JetBrains Mono', 'SF Mono', 'Menlo', 'Consolas', monospace;

  --font-size-xs: 0.75rem;    /* 12px */
  --font-size-sm: 0.875rem;   /* 14px */
  --font-size-base: 1rem;     /* 16px */
  --font-size-lg: 1.125rem;   /* 18px */
  --font-size-xl: 1.25rem;    /* 20px */
  --font-size-2xl: 1.5rem;    /* 24px */
  --font-size-3xl: 2rem;      /* 32px */
  --font-size-4xl: 2.5rem;    /* 40px */
  --font-size-5xl: 3rem;      /* 48px */

  --line-height-tight: 1.2;
  --line-height-normal: 1.5;
  --line-height-relaxed: 1.75;

  /* ==================== BORDER RADIUS TOKENS ==================== */

  --radius-sm: 0.375rem;    /* 6px */
  --radius-md: 0.75rem;     /* 12px */
  --radius-lg: 1rem;        /* 16px */
  --radius-xl: 1.5rem;      /* 24px */
  --radius-full: 9999px;

  /* ==================== SHADOW TOKENS ==================== */

  --shadow-sm: 0 1px 2px 0 rgba(0, 0, 0, 0.05);
  --shadow-md: 0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -2px rgba(0, 0, 0, 0.1);
  --shadow-lg: 0 10px 15px -3px rgba(0, 0, 0, 0.1), 0 4px 6px -4px rgba(0, 0, 0, 0.1);
  --shadow-xl: 0 20px 25px -5px rgba(0, 0, 0, 0.1), 0 8px 10px -6px rgba(0, 0, 0, 0.1);
  --shadow-2xl: 0 25px 50px -12px rgba(0, 0, 0, 0.25);

  /* Glow Shadows */
  --shadow-glow-cyan: 0 0 20px rgba(0, 217, 255, 0.3), 0 0 40px rgba(0, 217, 255, 0.1);
  --shadow-glow-purple: 0 0 20px rgba(168, 85, 247, 0.3), 0 0 40px rgba(168, 85, 247, 0.1);

  /* ==================== TRANSITION TOKENS ==================== */

  --transition-fast: 150ms cubic-bezier(0.4, 0, 0.2, 1);
  --transition-base: 200ms cubic-bezier(0.4, 0, 0.2, 1);
  --transition-smooth: 300ms cubic-bezier(0.4, 0, 0.2, 1);
  --transition-slow: 500ms cubic-bezier(0.4, 0, 0.2, 1);

  /* ==================== LAYOUT TOKENS ==================== */

  --container-sm: 640px;
  --container-md: 768px;
  --container-lg: 1024px;
  --container-xl: 1280px;

  --navbar-height: 70px;
  --sidebar-width: 280px;
  --sidebar-width-sm: 240px;
}

/* Dark mode adjustments (mostly already set) */
[data-theme='dark'] {
  /* Colors remain the same, but text might need adjustment */
}

/* High contrast mode support */
@media (prefers-contrast: more) {
  :root {
    --shadow-md: 0 4px 8px -1px rgba(0, 0, 0, 0.2);
    --shadow-lg: 0 10px 20px -3px rgba(0, 0, 0, 0.3);
  }
}

/* Reduced motion preferences */
@media (prefers-reduced-motion: reduce) {
  :root {
    --transition-fast: 0ms;
    --transition-base: 0ms;
    --transition-smooth: 0ms;
    --transition-slow: 0ms;
  }
}
```

---

## 3. Component Implementation Workflow

### Step 1: Create Component TypeScript File

**File**: `src/components/ExampleComponent.tsx`

```typescript
import React, { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './ExampleComponent.module.css';

interface ExampleComponentProps {
  children: ReactNode;
  variant?: 'primary' | 'secondary';
  disabled?: boolean;
  className?: string;
}

/**
 * ExampleComponent
 *
 * Brief description of component purpose and usage.
 *
 * @example
 * <ExampleComponent variant="primary">Content</ExampleComponent>
 */
export const ExampleComponent = React.forwardRef<
  HTMLDivElement,
  ExampleComponentProps
>(
  ({ children, variant = 'primary', disabled = false, className }, ref) => {
    const componentClasses = clsx(
      styles.component,
      styles[variant],
      disabled && styles.disabled,
      className
    );

    return (
      <div ref={ref} className={componentClasses} role="presentation">
        {children}
      </div>
    );
  }
);

ExampleComponent.displayName = 'ExampleComponent';

export default ExampleComponent;
```

### Step 2: Create Component CSS Module

**File**: `src/components/ExampleComponent.module.css`

```css
/**
 * ExampleComponent Styles
 *
 * Design Principles:
 * - Performance-first (GPU acceleration)
 * - Accessibility (focus indicators, color contrast)
 * - Responsiveness (mobile-first)
 * - Maintainability (clear structure, comments)
 */

.component {
  /* Base styles */
  display: flex;
  align-items: center;
  gap: var(--spacing-md);

  /* Colors */
  background: var(--color-bg-surface);
  color: var(--color-text-primary);

  /* Spacing */
  padding: var(--spacing-lg);

  /* Visual */
  border-radius: var(--radius-md);
  border: 1px solid rgba(0, 217, 255, 0.2);

  /* Performance */
  will-change: transform, box-shadow;
  transition: all var(--transition-base);
}

/* ==================== VARIANTS ==================== */

.primary {
  background: linear-gradient(135deg, #00d9ff 0%, #22d3ee 100%);
  color: #000000;
}

.secondary {
  background: rgba(0, 217, 255, 0.1);
  border-color: rgba(0, 217, 255, 0.3);
}

/* ==================== STATES ==================== */

.component:hover:not(.disabled) {
  border-color: rgba(0, 217, 255, 0.4);
  box-shadow: var(--shadow-glow-cyan);
}

.component:focus-visible {
  outline: 2px solid var(--color-primary);
  outline-offset: 4px;
}

.disabled {
  opacity: 0.6;
  cursor: not-allowed;
  pointer-events: none;
}

/* ==================== RESPONSIVE ==================== */

@media (max-width: 768px) {
  .component {
    padding: var(--spacing-md);
  }
}

/* ==================== ACCESSIBILITY ==================== */

@media (prefers-reduced-motion: reduce) {
  .component {
    transition: none;
  }
}

@media (prefers-contrast: more) {
  .component {
    border-width: 2px;
  }
}
```

### Step 3: Export from Barrel File

**File**: `src/components/index.tsx`

```typescript
// Export all components for convenience
export { PrimaryButton, type PrimaryButtonProps } from './PrimaryButton';
export { SecondaryButton, type SecondaryButtonProps } from './SecondaryButton';
export { ContentCard, type ContentCardProps } from './ContentCard';
export { FeatureCard, type FeatureCardProps } from './FeatureCard';
export { ModuleCard, type ModuleCardProps } from './ModuleCard';
export { Alert, type AlertProps } from './Alert';
export { Badge, type BadgeProps } from './Badge';
export { CodeBlock, type CodeBlockProps } from './CodeBlock';
export { ProgressBar, type ProgressBarProps } from './ProgressBar';
export { Breadcrumb, type BreadcrumbProps } from './Breadcrumb';
export { Container, type ContainerProps } from './Container';
export { Section, type SectionProps } from './Section';
export { Grid, type GridProps } from './Grid';
```

---

## 4. Docusaurus Integration

### 4.1 Theme Customization

**Swizzle Theme Components** (for advanced customization):

```bash
npm run swizzle @docusaurus/preset-classic -- --list

# Example: Swizzle NavBar
npm run swizzle @docusaurus/preset-classic Layout/Navbar
```

This creates overrideable component files in `src/theme/`.

### 4.2 Page Layout Customization

**File**: `src/theme/Layout/index.tsx` (if swizzled)

```tsx
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import type { WrapperProps } from '@docusaurus/types';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): JSX.Element {
  return <OriginalLayout {...props} />;
}
```

### 4.3 Markdown Components Override

**File**: `src/theme/MDXComponents.tsx`

```typescript
import MDXComponents from '@theme-original/MDXComponents';
import { Alert, Badge, CodeBlock } from '@site/src/components';

export default {
  ...MDXComponents,
  Alert,        // Use Alert for admonitions
  Badge,        // Use Badge for inline tags
  CodeBlock,    // Use CodeBlock for code snippets
  // Add custom components as needed
};
```

---

## 5. CSS Architecture & Organization

### Recommended Structure

```
src/css/
├── custom.css           (global, design tokens)
├── base/
│   ├── reset.css       (reset/normalize)
│   ├── typography.css  (font-face, base styles)
│   └── layout.css      (html, body, containers)
├── components/
│   ├── navbar.css
│   ├── sidebar.css
│   ├── buttons.css
│   ├── cards.css
│   └── forms.css
├── utilities/
│   ├── animations.css  (keyframes)
│   ├── responsive.css  (media queries)
│   └── accessibility.css (a11y utilities)
└── dark-mode.css       (dark theme overrides)
```

**Import Order in custom.css**:

```css
/* 1. Reset & Defaults */
@import url('./base/reset.css');
@import url('./base/typography.css');
@import url('./base/layout.css');

/* 2. Design Tokens & Theme (in custom.css root) */
/* ... token definitions ... */

/* 3. Utilities */
@import url('./utilities/animations.css');
@import url('./utilities/responsive.css');
@import url('./utilities/accessibility.css');

/* 4. Component Overrides */
@import url('./components/navbar.css');
@import url('./components/sidebar.css');
@import url('./components/buttons.css');
@import url('./components/cards.css');
@import url('./components/forms.css');

/* 5. Dark Mode (at end to override) */
@import url('./dark-mode.css');
```

---

## 6. Responsive Design Implementation

### Mobile-First Breakpoints

```css
/* Mobile (375px - default) */
.container { padding: 24px; }

/* Tablet (768px+) */
@media (min-width: 768px) {
  .container { padding: 32px; }
}

/* Desktop (1024px+) */
@media (min-width: 1024px) {
  .container { padding: 40px; }
}

/* Wide (1536px+) */
@media (min-width: 1536px) {
  .container { max-width: 1400px; }
}
```

### Responsive Typography

```css
/* Scale typography for readability */
:root {
  --font-size-base: 16px;  /* Mobile */
  --line-height-body: 1.6;
}

@media (min-width: 768px) {
  :root {
    --font-size-base: 16.5px; /* Tablet */
  }
}

@media (min-width: 1024px) {
  :root {
    --font-size-base: 17px;   /* Desktop */
  }
}

body {
  font-size: var(--font-size-base);
  line-height: var(--line-height-body);
}
```

---

## 7. Accessibility Implementation

### Semantic HTML

```tsx
// Good: Semantic structure
<main role="main">
  <section aria-labelledby="features-heading">
    <h2 id="features-heading">Key Features</h2>
    <nav aria-label="Feature navigation">
      {/* ... */}
    </nav>
  </section>
</main>

// Bad: Non-semantic divs
<div id="main">
  <div class="section">
    <div class="heading">Key Features</div>
    {/* ... */}
  </div>
</div>
```

### Focus Management

```css
/* Visible focus indicators (always) */
button:focus-visible,
a:focus-visible,
input:focus-visible {
  outline: 2px solid var(--color-primary);
  outline-offset: 4px;
}

/* NEVER use outline: none without replacement */
/* This breaks keyboard navigation */
```

### Color Contrast Testing

```bash
# Use tools like WebAIM Contrast Checker
# All text must meet:
# - Normal text: 4.5:1 (AA) or 7:1 (AAA) ← target this
# - Large text (18px+): 3:1 (AA) or 4.5:1 (AAA)

# Test with:
# 1. WebAIM Contrast Checker (webaim.org/resources/contrastchecker/)
# 2. Firefox DevTools: Inspector > A11y Panel
# 3. Axe DevTools Chrome/Firefox extension
```

---

## 8. Performance Optimization Checklist

### CSS Performance

- [ ] Use CSS custom properties for theming (reduces file size)
- [ ] Minimize selectors (avoid `.navbar .navbar-brand .navbar-link`)
- [ ] Use `will-change` sparingly (only on animated elements)
- [ ] GPU-accelerate animations (use `transform` and `opacity`)
- [ ] Inline critical CSS for hero section
- [ ] Defer non-critical CSS loading

### Image Optimization

```html
<!-- Responsive images with srcset -->
<img
  src="image-1024.png"
  srcset="image-512.png 512w, image-1024.png 1024w, image-2048.png 2048w"
  sizes="(max-width: 768px) 100vw, (max-width: 1024px) 75vw, 1024px"
  alt="Descriptive alt text"
  loading="lazy"
  decoding="async"
/>

<!-- WebP with PNG fallback -->
<picture>
  <source srcset="image.webp" type="image/webp">
  <img src="image.png" alt="Description">
</picture>
```

### JavaScript Optimization

```typescript
// Lazy load heavy components
const CodeBlock = lazy(() => import('./CodeBlock'));

// Debounce scroll handlers
const handleScroll = debounce(() => {
  // Intersection Observer preferred for scroll-based effects
}, 200);

// Use Intersection Observer for lazy loading
const observer = new IntersectionObserver((entries) => {
  entries.forEach(entry => {
    if (entry.isIntersecting) {
      // Load content
      observer.unobserve(entry.target);
    }
  });
});
```

---

## 9. Testing Strategy

### Component Testing

```typescript
// example.test.tsx
import { render, screen } from '@testing-library/react';
import { PrimaryButton } from './PrimaryButton';

describe('PrimaryButton', () => {
  it('renders with correct text', () => {
    render(<PrimaryButton>Click me</PrimaryButton>);
    expect(screen.getByText('Click me')).toBeInTheDocument();
  });

  it('handles click events', () => {
    const handleClick = jest.fn();
    render(<PrimaryButton onClick={handleClick}>Click</PrimaryButton>);
    screen.getByRole('button').click();
    expect(handleClick).toHaveBeenCalled();
  });

  it('is accessible with keyboard', () => {
    render(<PrimaryButton>Press Enter</PrimaryButton>);
    const button = screen.getByRole('button');
    expect(button).toHaveProperty('tabIndex', 0);
  });
});
```

### Visual Regression Testing

```bash
npm run test:visual

# Check for unexpected style changes
# Tools: Percy, Chromatic, or manual screenshots
```

### Accessibility Testing

```bash
npm run test:a11y

# Run Axe accessibility audit
# Tools: Axe DevTools, Pa11y, Lighthouse Audit
```

---

## 10. Deployment & Build

### Build Optimization

```bash
# Analyze bundle size
npm run build -- --stats

# Production build
npm run build

# Serve locally before deploy
npm run serve
```

### Performance Budgets

Set up performance budgets in `package.json`:

```json
{
  "scripts": {
    "bundle-analyze": "next build --analyze",
    "lighthouse": "lighthouse https://yoursite.com --output-path=./report.html"
  },
  "budgets": [
    {
      "type": "bundle",
      "name": "main",
      "limits": ["50 kB"]
    },
    {
      "type": "bundle",
      "name": "styles",
      "limits": ["30 kB"]
    }
  ]
}
```

---

## 11. Common Implementation Patterns

### Button with Icon

```tsx
<PrimaryButton icon={<ArrowIcon />} iconPosition="right" size="large">
  Learn More
</PrimaryButton>
```

### Card Grid

```tsx
<Grid cols={{ mobile: 1, tablet: 2, desktop: 3 }} gap="lg">
  {features.map(feature => (
    <FeatureCard
      key={feature.id}
      icon={feature.icon}
      title={feature.title}
      description={feature.description}
    />
  ))}
</Grid>
```

### Code Block with Language

```tsx
<CodeBlock
  language="python"
  title="example.py"
  code={pythonCode}
  showLineNumbers
/>
```

### Alert with Dismiss

```tsx
const [showAlert, setShowAlert] = useState(true);

if (!showAlert) return null;

<Alert
  type="warning"
  title="Deprecation"
  dismissible
  onDismiss={() => setShowAlert(false)}
>
  This API will be removed in v2.0
</Alert>
```

---

## 12. Documentation & Maintenance

### Component Documentation Template

Create a `.mdx` file for each component:

**File**: `docs/components/PrimaryButton.mdx`

```mdx
---
title: Primary Button
description: Main call-to-action button component
---

## Usage

```tsx
import { PrimaryButton } from '@site/src/components';

<PrimaryButton onClick={handleClick}>Click me</PrimaryButton>
```

## Props

| Prop | Type | Default | Description |
|------|------|---------|-------------|
| `children` | ReactNode | required | Button content |
| `onClick` | function | - | Click handler |
| `disabled` | boolean | false | Disabled state |
| `size` | 'small' \| 'medium' \| 'large' | 'medium' | Size variant |

## Variants

### Sizes

<PrimaryButton size="small">Small</PrimaryButton>
<PrimaryButton size="medium">Medium</PrimaryButton>
<PrimaryButton size="large">Large</PrimaryButton>

### States

<PrimaryButton>Normal</PrimaryButton>
<PrimaryButton disabled>Disabled</PrimaryButton>

## Accessibility

- Keyboard accessible (Tab, Enter/Space)
- Clear focus indicator
- Proper ARIA attributes
```

---

## 13. Migration Checklist

For existing pages, follow this order:

- [ ] Update `custom.css` with design tokens
- [ ] Create new component files (`PrimaryButton.tsx`, etc.)
- [ ] Update imports on pages
- [ ] Test responsive design on all breakpoints
- [ ] Run accessibility audit
- [ ] Performance testing
- [ ] Deploy to staging
- [ ] QA review
- [ ] Deploy to production

---

## 14. Resources & References

### Design System
- Figma file (if available): [URL]
- Design tokens: `src/css/custom.css`
- Component library: `src/components/`

### Documentation
- Docusaurus: https://docusaurus.io
- CSS Modules: https://create-react-app.dev/docs/adding-a-css-modules-stylesheet/
- TypeScript: https://www.typescriptlang.org/docs/

### Tools
- Chrome DevTools: For debugging styles and responsive design
- Lighthouse: For performance auditing
- Axe DevTools: For accessibility testing
- WebAIM: For color contrast checking

---

## Document Control

- **Version**: 1.0
- **Last Updated**: December 2025
- **Status**: Active
- **Next Review**: January 2026

---
