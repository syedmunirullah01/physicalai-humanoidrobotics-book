# Physical AI & Humanoid Robotics - Component Library

## Overview

This document describes the professional UI component library for the Physical AI & Humanoid Robotics educational platform. Each component is built with accessibility, performance, and scientific credibility as primary concerns.

---

## Component Index

1. [PrimaryButton](#primarybutton)
2. [SecondaryButton](#secondarybutton)
3. [ContentCard](#contentcard)
4. [FeatureCard](#featurecard)
5. [ModuleCard](#modulecard)
6. [Alert](#alert)
7. [Badge](#badge)
8. [CodeBlock](#codeblock)
9. [ProgressBar](#progressbar)
10. [Breadcrumb](#breadcrumb)
11. [Container](#container)
12. [Section](#section)
13. [Grid](#grid)

---

## Components

### PrimaryButton

**Purpose**: Main call-to-action button for primary actions (start learning, continue, submit)

**File**: `src/components/PrimaryButton.tsx`

**Visual Design**:
- Gradient cyan background (#00d9ff to #22d3ee)
- Glow shadow effect
- Smooth lift on hover
- No text decoration

**Props**:

```typescript
interface PrimaryButtonProps {
  children: ReactNode;           // Button text/content
  onClick?: () => void;          // Click handler
  href?: string;                 // Link destination (renders as <a> if provided)
  disabled?: boolean;            // Disabled state
  size?: 'small' | 'medium' | 'large'; // Size variant
  fullWidth?: boolean;           // Stretch to 100% width
  icon?: ReactNode;              // Optional icon
  iconPosition?: 'left' | 'right'; // Icon position
  type?: 'button' | 'submit' | 'reset'; // Button type
  className?: string;            // Additional CSS classes
}
```

**Usage**:

```tsx
import { PrimaryButton } from '@site/src/components/PrimaryButton';

// Simple button
<PrimaryButton onClick={handleStart}>Start Learning</PrimaryButton>

// As link
<PrimaryButton href="/docs" size="large">Documentation</PrimaryButton>

// With icon
<PrimaryButton icon={<ArrowIcon />} iconPosition="right">
  Continue to Module 2
</PrimaryButton>

// Form submission
<PrimaryButton type="submit" fullWidth>
  Submit Assignment
</PrimaryButton>
```

**Accessibility**:
- Keyboard accessible (Tab, Enter/Space)
- Visible focus indicator
- `aria-disabled` attribute for disabled state
- Sufficient color contrast (7:1 WCAG AAA)

**Responsive Behavior**:
- Mobile: Reduced padding (10px 18px)
- Desktop: Full padding (12px 24px)
- Large variant: 16px 32px

**Animation**:
- Hover: Lift (-2px translateY) + enhanced glow
- Focus: Outline glow
- Duration: 200ms cubic-bezier(0.4, 0, 0.2, 1)
- Respects `prefers-reduced-motion`

---

### SecondaryButton

**Purpose**: Secondary actions (cancel, close, alternate options)

**File**: `src/components/SecondaryButton.tsx` (to be created)

**Visual Design**:
- Transparent background with cyan border
- Cyan text
- Light glow on hover
- Professional but less prominent than Primary

**Props**:

```typescript
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
}
```

**Usage**:

```tsx
import { SecondaryButton } from '@site/src/components/SecondaryButton';

<SecondaryButton onClick={handleCancel}>Cancel</SecondaryButton>

<SecondaryButton href="/docs" icon={<DocsIcon />}>
  View Documentation
</SecondaryButton>
```

**Styling**:
- Background: rgba(0, 217, 255, 0.1)
- Border: 1px rgba(0, 217, 255, 0.3)
- Color: #00d9ff
- Hover: Background rgba(0, 217, 255, 0.2), border rgba(0, 217, 255, 0.5)

---

### ContentCard

**Purpose**: Display content blocks with optional icon, title, and description

**File**: `src/components/ContentCard.tsx`

**Visual Design**:
- Dark background with cyan border accent
- 1.5rem padding (feature variant: 2rem)
- Smooth hover lift and glow
- Flexible layout (icon + text)

**Props**:

```typescript
interface ContentCardProps {
  children: ReactNode;         // Card content
  title?: ReactNode;           // Card title (h3)
  description?: ReactNode;     // Short description
  icon?: ReactNode;            // Optional icon (48x48)
  href?: string;               // Link destination
  onClick?: () => void;        // Click handler
  variant?: 'default' | 'feature' | 'elevated'; // Style variant
  hoverable?: boolean;         // Enable hover effects
  className?: string;          // Additional classes
}
```

**Usage**:

```tsx
import { ContentCard } from '@site/src/components/ContentCard';

<ContentCard
  icon={<RobotIcon />}
  title="Advanced Kinematics"
  description="Master forward and inverse kinematics"
  hoverable
>
  Learn the mathematical foundations of robot motion...
</ContentCard>

<ContentCard
  href="/module-1"
  variant="feature"
  title="ROS 2 Fundamentals"
>
  Build your foundation with ROS 2 messaging and services
</ContentCard>
```

**Variants**:
- **default**: Standard card, subtle border
- **feature**: Stronger border, more padding
- **elevated**: Enhanced shadow at baseline

**Responsive Behavior**:
- Mobile: Reduced padding (1.25rem)
- Tablet/Desktop: Full padding (1.5rem or 2rem)
- Icon size: Adjusts based on screen

---

### FeatureCard

**Purpose**: Showcase features with icon, title, and description (typically in grids)

**File**: `src/components/FeatureCard.tsx` (to be created)

**Visual Design**:
- Compact, icon-focused design
- Icon in accent color (cyan)
- Short title and description
- No border, subtle shadow

**Props**:

```typescript
interface FeatureCardProps {
  icon: ReactNode;           // Required icon
  title: ReactNode;          // Feature title
  description: ReactNode;    // Feature description
  className?: string;        // Additional classes
}
```

**Usage**:

```tsx
<FeatureCard
  icon={<SimulationIcon />}
  title="Gazebo Integration"
  description="Physics-accurate simulation with real-time visualization"
/>
```

**Grid Recommendations**:
- Mobile: 1 column
- Tablet: 2 columns, 24px gap
- Desktop: 3–4 columns, 32px gap

---

### ModuleCard

**Purpose**: Display module overview with progress, metadata, and call-to-action

**File**: `src/components/ModuleCard.tsx` (to be created)

**Visual Design**:
- Gradient header (cyan to purple)
- Progress bar at bottom
- Clear action button
- Comprehensive metadata

**Props**:

```typescript
interface ModuleCardProps {
  title: string;               // Module title
  description: string;         // Module description
  number: number;              // Module number
  chapters: number;            // Total chapters
  duration: string;            // Estimated duration (e.g., "8 hours")
  difficulty: 'beginner' | 'intermediate' | 'advanced'; // Difficulty level
  progress: number;            // 0-100 progress percentage
  tags: string[];              // Technology tags
  status?: 'locked' | 'available' | 'in-progress' | 'completed'; // Module status
  href: string;                // Link to module
  className?: string;          // Additional classes
}
```

**Usage**:

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

**Layout**:
- Header: Gradient background, module number, title
- Description: 2-3 lines max
- Meta: 2-column grid (chapters + duration, difficulty + tags)
- Progress: Visual bar + percentage
- Footer: CTA button + status indicator

---

### Alert

**Purpose**: Display system messages (info, success, warning, error)

**File**: `src/components/Alert.tsx`

**Visual Design**:
- Type-based color (info: cyan, success: green, warning: orange, error: red)
- 4px left border accent
- Subtle background
- Optional dismiss button

**Props**:

```typescript
type AlertType = 'info' | 'success' | 'warning' | 'error';

interface AlertProps {
  children: ReactNode;         // Alert message
  type?: AlertType;            // Alert type (default: 'info')
  title?: string;              // Optional title
  icon?: ReactNode;            // Optional icon
  dismissible?: boolean;       // Show dismiss button
  onDismiss?: () => void;      // Dismiss callback
  className?: string;          // Additional classes
}
```

**Usage**:

```tsx
import { Alert } from '@site/src/components/Alert';

<Alert type="info" title="Getting Started">
  Complete the prerequisites before starting this module.
</Alert>

<Alert type="success" icon={<CheckIcon />} dismissible>
  Module completed successfully! Share your achievement.
</Alert>

<Alert type="warning" title="Deprecation Notice">
  This API will be removed in v2.0. Migrate to the new service.
</Alert>

<Alert type="error" title="Error">
  Failed to submit assignment. Please check your network connection.
</Alert>
```

**Types & Colors**:
- **info**: Cyan (#3b82f6) - informational messages
- **success**: Green (#10b981) - successful operations
- **warning**: Orange (#f59e0b) - warnings
- **error**: Red (#ef4444) - errors

**Accessibility**:
- `role="alert"` for screen readers
- `aria-live="polite"` for dynamic updates
- `aria-atomic="true"` for full alert reading
- Dismissible button: `aria-label="Dismiss alert"`

---

### Badge

**Purpose**: Tag, label, or small status indicator

**File**: `src/components/Badge.tsx` (to be created)

**Visual Design**:
- Small, pill-shaped (border-radius: 9999px)
- Cyan background or border
- 12px font
- Compact: 4px 12px padding

**Props**:

```typescript
type BadgeVariant = 'default' | 'success' | 'warning' | 'error' | 'custom';

interface BadgeProps {
  children: ReactNode;
  variant?: BadgeVariant;
  icon?: ReactNode;
  className?: string;
}
```

**Usage**:

```tsx
<Badge variant="default">Python</Badge>
<Badge variant="success">Completed</Badge>
<Badge variant="warning">In Review</Badge>
<Badge variant="custom">Advanced</Badge>
```

**Styling**:
- Default: Cyan background rgba(0, 217, 255, 0.1), cyan text
- Success: Green background/text
- Warning: Orange background/text
- Error: Red background/text

---

### CodeBlock

**Purpose**: Display syntax-highlighted code snippets with metadata

**File**: `src/components/CodeBlock.tsx` (to be created)

**Visual Design**:
- Dark background with subtle border
- Language badge (top-right)
- Line numbers (optional)
- Copy button
- Proper syntax highlighting

**Props**:

```typescript
interface CodeBlockProps {
  code: string;                  // Code content
  language: string;              // Language (python, cpp, bash, etc.)
  showLineNumbers?: boolean;     // Show line numbers
  showCopyButton?: boolean;      // Show copy button (default: true)
  title?: string;                // Optional code title/filename
  highlight?: number[];          // Lines to highlight
  className?: string;            // Additional classes
}
```

**Usage**:

```tsx
import { CodeBlock } from '@site/src/components/CodeBlock';

<CodeBlock
  language="python"
  title="ros2_node.py"
  code={`
import rclpy
from rclpy.node import Node

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, 10)
`}
  highlight={[2, 3, 5]}
/>
```

**Features**:
- Syntax highlighting via Prism.js
- Language-specific color themes
- Copy-to-clipboard functionality
- Line highlighting
- Responsive overflow handling

**Supported Languages**:
- Python, C++, JavaScript, Bash, ROS launch files, XML, JSON, YAML, etc.

---

### ProgressBar

**Purpose**: Show progress or status (e.g., module completion)

**File**: `src/components/ProgressBar.tsx` (to be created)

**Visual Design**:
- Cyan gradient bar
- Subtle background track
- Optional percentage label
- Smooth animation on change

**Props**:

```typescript
type ProgressVariant = 'linear' | 'circular' | 'segments';

interface ProgressBarProps {
  value: number;                 // 0-100 percentage
  variant?: ProgressVariant;     // Bar style
  label?: ReactNode;             // Optional label (e.g., "45%")
  showPercentage?: boolean;      // Show numeric percentage
  animated?: boolean;            // Animate bar fill
  className?: string;            // Additional classes
}
```

**Usage**:

```tsx
import { ProgressBar } from '@site/src/components/ProgressBar';

// Linear progress
<ProgressBar value={65} showPercentage />

// Circular progress
<ProgressBar value={80} variant="circular" />

// Segment progress (e.g., 3/8 chapters)
<ProgressBar value={37.5} variant="segments" label="3 of 8 chapters" />
```

**Visual Variants**:
- **linear**: Horizontal bar, full-width
- **circular**: Circular radial progress, centered
- **segments**: Step indicators (e.g., 4 steps, 2 completed)

---

### Breadcrumb

**Purpose**: Navigation path indicator (Home > Module 1 > Chapter 2)

**File**: `src/components/Breadcrumb.tsx` (to be created)

**Visual Design**:
- Gray text with cyan current page
- "/" separator
- Links styled as buttons on hover
- Semantic structure

**Props**:

```typescript
interface BreadcrumbItem {
  label: string;
  href?: string;
}

interface BreadcrumbProps {
  items: BreadcrumbItem[];
  className?: string;
}
```

**Usage**:

```tsx
import { Breadcrumb } from '@site/src/components/Breadcrumb';

<Breadcrumb
  items={[
    { label: 'Home', href: '/' },
    { label: 'Module 1', href: '/module-1' },
    { label: 'Chapter 2: Topics', href: '/module-1/chapter-2' },
  ]}
/>
```

**Output**: `Home / Module 1 / Chapter 2: Topics`

**Accessibility**:
- `<nav>` wrapper with `aria-label="Breadcrumb"`
- Links properly marked
- Last item not a link (current page)

---

### Container

**Purpose**: Consistent content width and centering

**File**: `src/components/Container.tsx` (to be created)

**Visual Design**:
- Max-width: 1280px
- Horizontal centering
- Responsive padding

**Props**:

```typescript
type ContainerSize = 'sm' | 'md' | 'lg' | 'xl' | 'full';

interface ContainerProps {
  children: ReactNode;
  size?: ContainerSize;        // Max-width variant
  className?: string;          // Additional classes
}
```

**Usage**:

```tsx
<Container size="lg">
  <h1>Learning Module</h1>
  <p>Content here...</p>
</Container>
```

**Max-Widths**:
- sm: 640px
- md: 768px
- lg: 1024px (docs content)
- xl: 1280px (pages)
- full: 100% (hero sections)

**Responsive Padding**:
- Mobile: 24px
- Tablet: 32px
- Desktop: 40px

---

### Section

**Purpose**: Full-width section container with vertical padding and optional border

**File**: `src/components/Section.tsx` (to be created)

**Visual Design**:
- Full viewport width
- Generous vertical padding (64px–96px)
- Optional top border accent
- Background color support

**Props**:

```typescript
type SectionVariant = 'default' | 'dark' | 'elevated';

interface SectionProps {
  children: ReactNode;
  variant?: SectionVariant;     // Background/styling variant
  withBorder?: boolean;         // Top border accent
  className?: string;           // Additional classes
}
```

**Usage**:

```tsx
<Section variant="dark" withBorder>
  <Container>
    <h2>Advanced Topics</h2>
    {/* Content */}
  </Container>
</Section>
```

**Variants**:
- **default**: Dark background (#000000)
- **dark**: Darker, elevated appearance
- **elevated**: Contrast against default

**Padding**:
- Mobile: 64px vertical
- Desktop: 96px vertical

---

### Grid

**Purpose**: Responsive multi-column layout

**File**: `src/components/Grid.tsx` (to be created)

**Visual Design**:
- Responsive columns (1/2/3/4)
- Consistent gap spacing
- No enforced aspect ratio (content-driven)

**Props**:

```typescript
type GridCols = 1 | 2 | 3 | 4;

interface GridProps {
  children: ReactNode;
  cols?: {
    mobile?: GridCols;
    tablet?: GridCols;
    desktop?: GridCols;
  };
  gap?: 'sm' | 'md' | 'lg';      // Gap size
  className?: string;            // Additional classes
}
```

**Usage**:

```tsx
import { Grid } from '@site/src/components/Grid';

<Grid cols={{ mobile: 1, tablet: 2, desktop: 3 }} gap="lg">
  <ContentCard title="Feature 1" />
  <ContentCard title="Feature 2" />
  <ContentCard title="Feature 3" />
</Grid>
```

**Responsive Defaults**:
- Mobile: 1 column
- Tablet: 2 columns
- Desktop: 3–4 columns (configurable)

**Gap Values**:
- sm: 16px
- md: 24px
- lg: 32px

---

## Design Tokens Reference

### Colors

```
Primary: #00d9ff (Cyan)
Primary Light: #22d3ee
Primary Dark: #0ea5e9

Accent: #a855f7 (Purple)
Accent Pink: #ec4899

Success: #10b981
Warning: #f59e0b
Error: #ef4444
Info: #3b82f6

Dark BG: #000000
Surface Dark: #0f0f0f
Text Primary: #f1f5f9
Text Secondary: #cbd5e1
Text Tertiary: #64748b
```

### Typography

```
Font Family: IBM Plex Sans (UI), JetBrains Mono (code)
Base Size: 16px mobile, 17px desktop

h1: 28px → 48px (weight: 800)
h2: 24px → 40px (weight: 700)
h3: 20px → 32px (weight: 700)
body: 16px → 17px (weight: 400)
code: 13px → 14px (weight: 500, monospace)
```

### Spacing

```
--spacing-xs: 4px
--spacing-sm: 8px
--spacing-md: 16px
--spacing-lg: 24px
--spacing-xl: 32px
--spacing-2xl: 48px
--spacing-3xl: 64px
--spacing-4xl: 96px
```

### Shadows

```
--shadow-sm: 0 1px 2px rgba(0,0,0,0.05)
--shadow-md: 0 4px 6px -1px rgba(0,0,0,0.1)
--shadow-lg: 0 10px 15px -3px rgba(0,0,0,0.1)
--shadow-xl: 0 20px 25px -5px rgba(0,0,0,0.1)
--shadow-glow-cyan: 0 0 20px rgba(0,217,255,0.3)
```

### Border Radius

```
--radius-sm: 6px
--radius-md: 12px
--radius-lg: 16px
--radius-xl: 24px
--radius-full: 9999px
```

---

## Implementation Checklist

- [ ] PrimaryButton (created)
- [ ] SecondaryButton
- [ ] ContentCard (created)
- [ ] FeatureCard
- [ ] ModuleCard
- [ ] Alert (created)
- [ ] Badge
- [ ] CodeBlock
- [ ] ProgressBar
- [ ] Breadcrumb
- [ ] Container
- [ ] Section
- [ ] Grid
- [ ] Navbar (enhance existing)
- [ ] Sidebar (enhance existing)
- [ ] Footer (enhance existing)

---

## Testing & QA

### Unit Testing

```bash
npm run test -- src/components
```

### Visual Regression

```bash
npm run test:visual
```

### Accessibility Audit

```bash
npm run test:a11y
```

### Performance Profiling

```bash
npm run test:performance
```

---

## Contributing

When adding new components:

1. Create component file: `src/components/ComponentName.tsx`
2. Create styles: `src/components/ComponentName.module.css`
3. Export from barrel file: `src/components/index.tsx`
4. Add documentation to this file
5. Add tests: `src/components/__tests__/ComponentName.test.tsx`
6. Run: `npm run test && npm run typecheck`

---

## Versioning

- **Version**: 1.0.0
- **Docusaurus**: 3.9.2
- **React**: 19.0
- **TypeScript**: 5.6

---
