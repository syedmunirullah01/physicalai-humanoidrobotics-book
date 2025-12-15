# Component Library - Quick Start Guide

A quick reference for using the professional UI component library in the Physical AI & Humanoid Robotics platform.

---

## Installation & Imports

All components are exported from `src/components/`:

```typescript
import {
  PrimaryButton,
  SecondaryButton,
  ContentCard,
  FeatureCard,
  ModuleCard,
  Alert,
  Badge,
  ProgressBar,
  Breadcrumb,
  Container,
  Grid,
  Section,
} from '@site/src/components';
```

---

## Button Components

### PrimaryButton
Main call-to-action button (cyan gradient, glow shadow).

```tsx
// Simple button
<PrimaryButton onClick={handleClick}>Click Me</PrimaryButton>

// As link
<PrimaryButton href="/learn">Start Learning</PrimaryButton>

// With icon
<PrimaryButton icon={<ArrowIcon />} iconPosition="right">
  Continue
</PrimaryButton>

// Size variants: 'small' | 'medium' | 'large'
<PrimaryButton size="large">Get Started</PrimaryButton>

// Full width
<PrimaryButton fullWidth>Submit</PrimaryButton>

// Disabled state
<PrimaryButton disabled>Coming Soon</PrimaryButton>
```

**Props**: `children`, `onClick`, `href`, `disabled`, `size`, `fullWidth`, `icon`, `iconPosition`, `type`, `className`

### SecondaryButton
Alternate/secondary action button (transparent, cyan border).

```tsx
// Cancel/close action
<SecondaryButton onClick={handleCancel}>Cancel</SecondaryButton>

// Documentation link
<SecondaryButton href="/docs" icon={<DocsIcon />}>
  View Docs
</SecondaryButton>
```

**Props**: Same as PrimaryButton

---

## Card Components

### FeatureCard
Compact icon-focused card for feature grids.

```tsx
<FeatureCard
  icon={<RobotIcon />}
  title="Advanced Kinematics"
  description="Master forward and inverse kinematics algorithms"
/>
```

Use in 3-4 column grids for feature showcases.

### ContentCard
Generic content card with optional icon.

```tsx
<ContentCard
  icon={<CheckIcon />}
  title="Learning Milestone"
  description="You've completed this chapter"
  hoverable
>
  Additional content here...
</ContentCard>

// As link
<ContentCard href="/next-chapter" variant="feature">
  Proceed to Next Chapter
</ContentCard>
```

**Variants**: `default`, `feature`, `elevated`

### ModuleCard
Comprehensive module overview card with progress.

```tsx
<ModuleCard
  title="ROS 2 Fundamentals"
  description="Master the core concepts of Robot Operating System 2"
  number={1}
  chapters={4}
  duration="8 hours"
  difficulty="beginner"
  progress={45}
  tags={['ROS 2', 'Python', 'Linux']}
  status="in-progress"
  href="/module-1"
/>
```

**Status values**: `locked`, `available`, `in-progress`, `completed`
**Difficulty**: `beginner`, `intermediate`, `advanced`

---

## Feedback & Status Components

### Alert
Display system messages and notifications.

```tsx
// Info alert
<Alert type="info" title="Getting Started">
  Complete the prerequisites before starting this module.
</Alert>

// Success alert
<Alert type="success" dismissible>
  Module completed successfully!
</Alert>

// Warning alert
<Alert type="warning" title="Deprecation">
  This API will be removed in v2.0
</Alert>

// Error alert
<Alert type="error" icon={<ErrorIcon />}>
  Failed to submit assignment. Please check your connection.
</Alert>
```

**Types**: `info`, `success`, `warning`, `error`

### Badge
Small label or tag component.

```tsx
// Default (cyan)
<Badge>Python</Badge>

// Success (green)
<Badge variant="success">Completed</Badge>

// Warning (orange)
<Badge variant="warning">In Review</Badge>

// Custom (purple)
<Badge variant="custom">Advanced</Badge>

// With icon
<Badge icon={<CheckIcon />} variant="success">Done</Badge>
```

**Variants**: `default`, `success`, `warning`, `error`, `custom`

---

## Data Visualization

### ProgressBar
Display progress or completion status.

```tsx
// Linear progress
<ProgressBar value={65} showPercentage />

// Circular progress
<ProgressBar value={80} variant="circular" showPercentage />

// Segment progress (steps)
<ProgressBar
  value={37.5}
  variant="segments"
  label="3 of 8 chapters"
/>

// Animated progress
<ProgressBar value={45} animated />
```

**Variants**: `linear`, `circular`, `segments`

---

## Navigation Components

### Breadcrumb
Navigation path indicator.

```tsx
<Breadcrumb
  items={[
    { label: 'Home', href: '/' },
    { label: 'Modules', href: '/modules' },
    { label: 'Module 1', href: '/module-1' },
    { label: 'Chapter 2 - Topics' },  // Current page, no href
  ]}
/>
```

Output: `Home / Modules / Module 1 / Chapter 2 - Topics`

---

## Layout Components

### Container
Consistent content width wrapper.

```tsx
// Default size (xl: 1280px)
<Container>
  <h1>Page Title</h1>
</Container>

// Documentation size (lg: 1024px)
<Container size="lg">
  <article>Article content...</article>
</Container>

// Full width
<Container size="full">
  <div>Full viewport width</div>
</Container>
```

**Sizes**: `sm` (640px), `md` (768px), `lg` (1024px), `xl` (1280px), `full` (100%)

### Grid
Responsive multi-column layout.

```tsx
// Default: 1 col mobile, 2 col tablet, 3 col desktop
<Grid gap="lg">
  <Card />
  <Card />
  <Card />
  <Card />
</Grid>

// Custom columns per breakpoint
<Grid
  cols={{ mobile: 1, tablet: 2, desktop: 4 }}
  gap="md"
>
  {/* items */}
</Grid>
```

**Gap options**: `sm` (16px), `md` (24px), `lg` (32px)

### Section
Full-width section container with vertical padding.

```tsx
// Default dark section with border
<Section variant="dark" withBorder>
  <Container>
    <h2>Advanced Topics</h2>
    {/* content */}
  </Container>
</Section>

// Elevated section (contrast variant)
<Section variant="elevated">
  {/* content */}
</Section>
```

**Variants**: `default`, `dark`, `elevated`

---

## Common Patterns

### Feature Grid
```tsx
<Section variant="dark" withBorder>
  <Container>
    <h2>Why Choose Our Platform</h2>
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
  </Container>
</Section>
```

### Module Curriculum
```tsx
<Container>
  <h1>Curriculum</h1>
  <Grid cols={{ mobile: 1, tablet: 2, desktop: 3 }} gap="lg">
    {modules.map(module => (
      <ModuleCard
        key={module.id}
        {...module}
        href={`/module-${module.number}`}
      />
    ))}
  </Grid>
</Container>
```

### Learning Path with Breadcrumb
```tsx
<>
  <Breadcrumb
    items={[
      { label: 'Home', href: '/' },
      { label: 'Module 1', href: '/module-1' },
      { label: 'Chapter 2: Topics' },
    ]}
  />
  <main>
    {/* Chapter content */}
  </main>
</>
```

### Status Indicators
```tsx
<div>
  {task.completed && (
    <Badge variant="success" icon={<CheckIcon />}>Completed</Badge>
  )}
  {task.inProgress && (
    <Badge variant="warning">In Progress</Badge>
  )}
  {task.locked && (
    <Badge icon={<LockIcon />}>Locked</Badge>
  )}
</div>
```

---

## Responsive Behavior

All components are **mobile-first** and adapt automatically:

| Breakpoint | Size | Usage |
|------------|------|-------|
| Mobile | 375px–767px | Single column, touch-friendly (48px targets) |
| Tablet | 768px–1023px | 2-column grids, standard spacing |
| Desktop | 1024px–1535px | 3-4 column grids, full design system |
| Wide | 1536px+ | Extra breathing room, generous spacing |

**Note**: You don't need to do anything—components handle responsiveness automatically.

---

## Styling & Customization

### Using Design Tokens
All colors, spacing, and typography use CSS custom properties from `src/css/custom.css`:

```css
.customElement {
  color: var(--color-text-primary);
  padding: var(--spacing-lg);
  border-radius: var(--radius-md);
  transition: all var(--transition-base);
  box-shadow: var(--shadow-glow-cyan);
}
```

### Overriding Component Styles
All components accept a `className` prop for additional styles:

```tsx
<PrimaryButton className="custom-button">
  Custom Styled
</PrimaryButton>
```

---

## Accessibility Features

All components include:
- ✓ Keyboard navigation (Tab, Enter, Space)
- ✓ Visible focus indicators (2px cyan outline)
- ✓ ARIA labels and roles
- ✓ Screen reader optimization
- ✓ High contrast support
- ✓ Reduced motion support

**For users with motion sensitivity**, animations are automatically disabled when `prefers-reduced-motion: reduce` is set.

---

## TypeScript Support

All components are fully typed with TypeScript:

```typescript
import { ModuleCard, type ModuleCardProps } from '@site/src/components';

const moduleProps: ModuleCardProps = {
  title: 'String',
  description: 'String',
  number: 1,
  chapters: 4,
  duration: '8 hours',
  difficulty: 'beginner',
  progress: 45,
  tags: ['tag1', 'tag2'],
  status: 'in-progress',
  href: '/module-1',
};

<ModuleCard {...moduleProps} />
```

---

## Performance Tips

1. **Use Container for consistent width** instead of custom max-width rules
2. **Use Grid for responsive layouts** instead of custom media queries
3. **Leverage design tokens** (CSS variables) for theming consistency
4. **Icon sizing**: 24px is standard, 48px is large, 16px is inline
5. **Button sizes**: Use size prop instead of custom padding
6. **Spacing**: Use spacing tokens (var(--spacing-*)) instead of hardcoded values

---

## Troubleshooting

### Component not showing?
1. Check import statement
2. Ensure component is exported from `src/components/index.tsx`
3. Verify TypeScript compilation (`npm run typecheck`)

### Styling issues?
1. Check CSS module import (should be `styles` from `.module.css`)
2. Ensure custom.css is loaded globally
3. Verify no conflicting global styles

### Accessibility warnings?
1. All components use semantic HTML and ARIA
2. Ensure you're providing `alt` text for images
3. Use `aria-label` for icon-only buttons

---

## Resources

- **Design System Docs**: `IMPLEMENTATION_COMPLETE.md`
- **Design Tokens**: `src/css/custom.css`
- **Component Files**: `src/components/`
- **Docusaurus Docs**: https://docusaurus.io

---

**Last Updated**: December 13, 2025
**Status**: Production Ready
