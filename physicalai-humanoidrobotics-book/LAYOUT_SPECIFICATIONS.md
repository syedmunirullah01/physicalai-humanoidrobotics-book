# Physical AI & Humanoid Robotics - Layout Specifications

## Page Hierarchy & Layout Patterns

This document details the specific layout requirements for each page type in the platform.

---

## 1. Homepage (`src/pages/index.tsx`)

### Current Implementation Status
- Hero section: ✓ Implemented
- Metrics section: ✓ Implemented
- Features section: ✓ Implemented
- Curriculum section: ✓ Implemented
- Enterprise section: ✓ Implemented

### Hero Section Enhancements

**Target Layout:**
- Full viewport height (100vh minimum)
- Centered content container (max-width: 1000px)
- Robot illustration positioned absolutely (top-right, mobile: hidden)
- Animated background (geometric patterns, low opacity)

**Content Hierarchy:**
```
Label ("Industry-Leading Robotics Education")
  ↓
H1 (Title: "Physical AI & Humanoid Robotics")
  ↓
Tagline ("Master ROS 2, Simulation, and Vision-Language-Action Models")
  ↓
Description (2–3 sentences)
  ↓
CTA Buttons (Primary: "Start Learning", Secondary: "View Docs")
```

**Responsive Adjustments:**
- Mobile (375px): Full padding, single column, centered alignment
- Tablet (768px): Content centered, illustration hidden
- Desktop (1024px+): Illustration visible right, two-column visual balance

**Animations:**
- Hero title: Fade-in + scale (0.95→1.0) over 500ms
- Subtitle: Fade-in with 200ms stagger
- CTA buttons: Fade-in with 400ms stagger
- Robot: Levitate animation (±25px Y offset) over 6s
- Background: Subtle circuit animation, hexagon float

---

### Metrics Section

**Grid Layout:**
- Desktop: 4 columns (1fr each), 32px gap
- Tablet: 2 columns, 24px gap
- Mobile: 1 column, stacked

**Metric Card Structure:**
```
┌─────────────────┐
│     12,000      │ (large number, cyan, h2 style)
│  ────────────   │ (decorative line)
│   Active Users  │ (label, slate-300)
└─────────────────┘
```

**Styling:**
- Background: rgba(0, 217, 255, 0.05)
- Border: 1px rgba(0, 217, 255, 0.2)
- Padding: 32px 24px (desktop), 24px 16px (mobile)
- Border-radius: 12px

**Animation:**
- Numbers count up on scroll using Intersection Observer
- Duration: 2s cubic-bezier(0.25, 0.46, 0.45, 0.94)
- Staggered animation between cards (200ms delay each)

---

### Features Section

**Grid Layout:**
- Desktop: 3 columns, 40px gap
- Tablet: 2 columns, 32px gap
- Mobile: 1 column, 24px gap

**Feature Card:**
```
┌──────────────────────────────────────┐
│  [⚙️] (48×48px icon, cyan glow)     │
│                                      │
│  Advanced Architecture               │ (h3)
│  Designed for production robotics... │ (body text)
└──────────────────────────────────────┘
```

**Card Styling:**
- No border (unlike ContentCard)
- Background: transparent or light rgba
- Icon background: rgba(0, 217, 255, 0.1)
- Icon glow on hover: 0 0 20px rgba(0, 217, 255, 0.3)
- Text: Slate-300 body, Slate-100 title

---

### Curriculum Section

**Section Title:** "Curriculum: Learn Advanced Robotics"

**Grid Layout:**
- Desktop: 2–3 columns, 40px gap
- Tablet: 1 column, 32px gap (consider 2 if horizontal space)
- Mobile: 1 column, 24px gap

**Module Card Detailed Layout:**

```
┌─────────────────────────────────────────────────┐
│ ╔═══════════════════════════════════════════════╗ │
│ ║ [Module Header - Gradient Bg]                 ║ │
│ ║ Module 1: ROS 2 Fundamentals                  ║ │
│ ╚═══════════════════════════════════════════════╝ │
│                                                   │
│ Master the core concepts of Robot Operating...   │ (description)
│                                                   │
│ ┌───────────────────┬───────────────────┐        │
│ │ 4 Chapters        │ 8 Hours           │ (meta) │
│ │ Beginner          │ Python, ROS 2     │        │
│ └───────────────────┴───────────────────┘        │
│                                                   │
│ ████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ 20%    │ (progress bar)
│                                                   │
│ [Start Module →]                                  │ (CTA button)
└─────────────────────────────────────────────────┘
```

**Card Dimensions:**
- Min-height: 480px
- Padding: 24px
- Header: 200px (gradient, cyan to purple)

**Status Indicators:**
- Locked: Overlay with lock icon
- Available: Standard appearance
- In-progress: Progress bar visible, "Continue" button
- Completed: Checkmark icon, "Review" button

---

### Enterprise Section

**Layout:** Two-column side-by-side

**Left Column (Text):**
```
[Enterprise-Grade Badge]
Production-Ready Architecture

Built on principles used by Fortune 500 companies...

[4 Feature Items in 2×2 grid]
• Test-Driven Development
• Containerized Deployment
• Real-Time Performance
• Security First

[CTA: "Start Building →"]
```

**Right Column (Visual):**
```
Code Window:
┌──────────────────────┐
│ ● ● ● | ros2_workspace/src/ │
├──────────────────────┤
│ # Production-grade ROS 2 node
│ import rclpy
│ from rclpy.node import Node
│ ...
└──────────────────────┘
```

**Responsive:**
- Desktop: 1fr 1fr grid, 64px gap
- Tablet: Stacked (1 column), code window below text
- Mobile: Stacked, code window reduced font

---

## 2. Documentation Hub

### Layout Structure

```
┌──────────────────────────────────────────────────────────────────┐
│ [NAVBAR]                                                         │
├──────────────────────────────────────────────────────────────────┤
│  [Sidebar]           [Main Content]         [Right Rail - ToC]   │
│  280px               800px max-width        220px                │
│  ├─ Search           ├─ H1 Title           ├─ On This Page      │
│  ├─ Module 1         ├─ Metadata           ├─ Heading 2         │
│  │  ├─ Intro         ├─ ──────────         ├─ Heading 3         │
│  │  ├─ Chapter 1     ├─ Markdown Content   ├─ ...               │
│  │  └─ Chapter 2     ├─ Code Blocks        └─                   │
│  ├─ Module 2         ├─ Diagrams           [Below content]       │
│  │  └─ ...           ├─ Related Links      ├─ Previous/Next      │
│  └─ API Ref          └─ Footer             └─ Author Bio        │
└──────────────────────────────────────────────────────────────────┘
```

**Widths:**
- Sidebar: 280px (sticky, top: 80px, max-height: calc(100vh - 80px))
- Main: Max-width 800px, centered in available space
- Right Rail: 220px (hidden on tablet <1200px)

**Gaps:**
- Sidebar ↔ Main: 48px (desktop), 32px (tablet)
- Main ↔ Right Rail: 40px

---

### Sidebar Navigation

**Search Box:**
- Position: Top, sticky
- Placeholder: "Search docs..."
- Icon: Magnifying glass (right side)
- Styling: Input field with accent border on focus

**Navigation Items:**
- Root level: Module names (bold)
- Children: Chapter/section titles
- Active state: Cyan left border (4px), background glow
- Hover: Color shift to cyan
- Smooth scroll to section

**Styling:**
- Font-size: 14px
- Line-height: 1.6
- Color: Slate-300, cyan on hover
- Padding: 8px 12px (items)
- Border-left: 4px solid transparent (active: cyan)

---

### Main Content Area

**Header Metadata:**
```
Home / Docs / Module 1 / Chapter 2: Topics  [breadcrumb]
───────────────────────────────────────────

Chapter 2: ROS 2 Topics and Pub/Sub        [h1]

Last updated: Dec 13, 2025 | Reading time: 12 min | By: Author Name
───────────────────────────────────────────
```

**Content Block Styling:**
- Paragraph: 17px, line-height 1.75, margin-bottom 1.5rem
- Links: Cyan color, underline on hover, no default underline
- Code inline: Monospace, dark background, padding 0.25rem 0.5rem
- Code blocks: Full width or max 100% with scroll (see CodeBlock component)

**Callout Boxes:**

```
┌─ 💡 Tip ─────────────────────────────────────┐
│ This is useful information to enhance your   │
│ understanding of the topic.                  │
└───────────────────────────────────────────────┘

┌─ ⚠️ Warning ──────────────────────────────────┐
│ Be careful with this pattern; it can cause   │
│ unexpected behavior in some scenarios.       │
└───────────────────────────────────────────────┘

┌─ ℹ️ Note ─────────────────────────────────────┐
│ Additional context that doesn't fit the     │
│ main narrative.                              │
└───────────────────────────────────────────────┘
```

---

### Right Rail (Table of Contents)

**Sticky Position:**
- Top: 100px (below navbar)
- Max-height: calc(100vh - 120px)
- Overflow: auto

**Structure:**
```
On This Page

├─ Introduction (h2)
│  ├─ Background (h3) → highlights while scrolling
│  └─ Prerequisites (h3)
├─ Core Concepts (h2)
│  ├─ Understanding Topics (h3)
│  ├─ Pub/Sub Pattern (h3)
│  └─ Message Types (h3)
├─ Practical Examples (h2)
└─ Best Practices (h2)
```

**Styling:**
- Font-size: 13px
- Line-height: 1.5
- Color: Slate-400, cyan active
- Active: Cyan color, left underline
- Smooth scroll: Scroll behavior: smooth

---

## 3. Module Overview Page

### Header Section

**Height:** 400px (desktop), 300px (mobile)

**Content:**
```
┌─────────────────────────────────────────────────────┐
│                                                     │
│  Module 1                                           │ (label, small)
│  ROS 2 Fundamentals                                 │ (h1)
│  Master the core concepts of Robot Operating...   │ (description)
│                                                     │
│  ◼ 4 Chapters  ◼ 8 Hours  ◼ Beginner               │ (key metrics)
│                                                     │
└─────────────────────────────────────────────────────┘
```

**Background:**
- Gradient: Darker gradient (similar to hero)
- No animated elements (clean presentation)
- Full width

---

### Chapter List

**Grid or List:**
- Desktop: 2 columns, 32px gap
- Tablet: 1 column
- Mobile: 1 column, 16px gap

**Chapter Card:**
```
┌─────────────────────────────────┐
│ Chapter 1: Installation & Setup │ (title)
│ Get your development...         │ (description, 2 lines)
│                                 │
│ ⏱️ 2 hours | 👤 Beginner       │ (metadata)
│                                 │
│ [Status: Completed ✓]           │ (status badge)
│ [Start Chapter →]               │ (CTA)
└─────────────────────────────────┘
```

**Status Indicators:**
- Completed: ✓ Green badge
- In-progress: ◐ Partial fill, "Continue" button
- Available: Standard appearance, "Start" button
- Locked: Lock icon, grayed out, "Prerequisites" tooltip

---

### Learning Objectives

**Section:**
```
What You'll Learn

✓ Understand ROS 2 architecture and middleware
✓ Create your first ROS 2 package
✓ Implement publishers and subscribers
✓ Debug ROS 2 applications with rqt and command-line tools
```

**Styling:**
- Grid: 2 columns (desktop), 1 column (mobile)
- Icon: Cyan checkmark, 20px
- Gap: 16px vertical, 8px horizontal
- Font: 16px, 1.6 line-height

---

### Resources Section

**Layout:**
```
Resources

Downloadable Files:
• Code repository (GitHub)
• Datasets (CSV, 2.3 MB)
• ROS workspaces (tarball)

External Links:
• ROS 2 Official Documentation
• Paper: "ROS 2 Architecture" (IEEE)
• Community Forum

Tools & Requirements:
• Ubuntu 22.04+
• ROS 2 Humble
• Python 3.10+
```

**Card Styling:**
- Icon left side (24px)
- Title (h4)
- Link with arrow icon
- Hover: Color shift to cyan

---

## 4. Chapter/Article Page

### Header

```
Home / Module 1 / Chapter 2                [breadcrumb]

Chapter 2: ROS 2 Topics                    [h1]

Comprehensive guide to pub/sub communication patterns.

Dec 13, 2025 | 12 min read | Intermediate | By: Author Name
──────────────────────────────────────────
```

---

### Content Columns

**Desktop Layout:**
```
┌────────────────┬──────────────────────────────┬──────────────────┐
│ Left Sidebar   │ Main Content (800px)         │ Right ToC (220px)│
│ (Sticky Nav)   │ ├─ Content sections          │ On This Page     │
│ ├─ Chapter 1   │ ├─ Embedded diagrams         │ • H2 1           │
│ ├─ Chapter 2   │ ├─ Code blocks               │ • H3 1           │
│ │  └─ Active   │ ├─ Interactive elements      │ • H2 2           │
│ └─ Chapter 3   │ └─ Related links             │ • H3 2           │
│                │                              │                  │
└────────────────┴──────────────────────────────┴──────────────────┘
```

**Tablet Layout:**
```
Main content full width with top breadcrumb, no sidebars.
Table of Contents: Collapsible drawer (icon at top).
```

**Mobile Layout:**
```
Full width content, single column.
Navigation: Hamburger menu.
ToC: Collapsed drawer.
```

---

### Content Typography

**Markdown Elements:**

```markdown
# Heading 1 (page title, 48px)
## Heading 2 (section, 32px, cyan, margin-top: 2rem)
### Heading 3 (subsection, 24px, margin-top: 1.5rem)

Paragraph text with proper line-height and color.

> Block quotes are styled with left cyan accent (4px),
> slight background tint, and italic style.

- List items with proper spacing
- Indentation for nested lists
  - Nested item

1. Numbered list
2. With proper counter
3. And styling
```

---

### Embedded Diagrams & Media

**Diagram Container:**
```
┌─────────────────────────────────────────────────┐
│                                                 │
│  [SVG/Canvas Diagram]                           │
│  (Responsive, aspect-ratio: auto)               │
│                                                 │
│  Figure 1: ROS 2 Node Communication Pattern    │ (caption)
│                                                 │
└─────────────────────────────────────────────────┘
```

**Styling:**
- Background: rgba(0, 217, 255, 0.05)
- Border: 1px rgba(0, 217, 255, 0.2)
- Padding: 32px
- Border-radius: 12px
- Caption: 12px, slate-400, margin-top: 8px
- Max-width: 100%, aspect-ratio: auto

**Interaction:**
- Click to enlarge (lightbox/modal)
- Keyboard navigation (Escape to close)
- Proper alt text for accessibility

---

### Code Blocks

**Inline Code:**
- Background: rgba(0, 217, 255, 0.1)
- Padding: 0.25rem 0.5rem
- Border-radius: 4px
- Font: JetBrains Mono, 13px
- Color: #22d3ee

**Code Block:**
- Full width or max 100% with horizontal scroll
- Header: Language badge (top-right), copy button, filename (optional)
- Body: Line numbers (optional), syntax highlighting
- Max-height: 400px (with scroll), or expand on click
- Line highlighting: Subtle background, 2px left border accent

---

## 5. Search Results Page

### Layout

```
┌─────────────────────────────────────────────────────────────┐
│ [NAVBAR]                                                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Search Results for "ros2 topics"                           │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │ [Search Input] [Filters ▼]                              │ │
│  └─────────────────────────────────────────────────────────┘ │
│                                                             │
│  ┌──────────────────┐  ┌──────────────────────────────────┐ │
│  │ Filters:         │  │ Results: 24 found                 │ │
│  │ ☑ Documentation  │  ├──────────────────────────────────┤ │
│  │ ☐ Blog Posts     │  │                                  │ │
│  │ ☐ Examples       │  │ ROS 2 Topics - Pub/Sub Pattern   │ │
│  │                  │  │ docs > module-1 > chapter-2     │ │
│  │ Module:          │  │ Comprehensive guide...           │ │
│  │ ☐ Module 1       │  │ Reading time: 12 min | Interme...│ │
│  │ ☐ Module 2       │  │                                  │ │
│  │ ☑ All            │  │ [Prev] 1 2 3 4 [Next]           │ │
│  │                  │  │                                  │ │
│  │ Difficulty:      │  │ ROS 2 Topics Overview (Excerpt)  │ │
│  │ ☐ Beginner       │  │ docs > preface                  │ │
│  │ ☐ Intermediate   │  │ Overview section...              │ │
│  │ ☑ Advanced       │  │ Reading time: 8 min | Beginner...│ │
│  │ ☑ All            │  │                                  │ │
│  │                  │  │ [More Results...]                │ │
│  └──────────────────┘  └──────────────────────────────────┘ │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**Sidebar (Left):**
- Width: 240px (desktop), hidden mobile
- Sticky: Yes
- Filters: Document type, module, difficulty
- Collapsible sections

**Results (Main):**
- Width: calc(100% - 240px - 32px) desktop, 100% mobile
- Result card: Title link, breadcrumb path, description, metadata
- Pagination: Bottom of results, or infinite scroll

---

## 6. Not Found (404) Page

### Design

```
┌─────────────────────────────────────────────────────┐
│                                                     │
│                  404                                │
│            Page Not Found                           │
│                                                     │
│     The page you're looking for doesn't exist.     │
│   It might have been moved or the link is broken.  │
│                                                     │
│         [Return to Home]  [View Documentation]     │
│                                                     │
│                                                     │
│   ┌────────────────────────────────────────────┐   │
│   │  Suggested Pages:                          │   │
│   │  • Getting Started                         │   │
│   │  • Module 1: ROS 2 Fundamentals            │   │
│   │  • API Reference                           │   │
│   │  • Community Forum                         │   │
│   └────────────────────────────────────────────┘   │
│                                                     │
└─────────────────────────────────────────────────────┘
```

**Design Elements:**
- Centered layout
- Large "404" text (120px, gray)
- Message: h2, neutral color
- CTA buttons: Primary + Secondary
- Suggested links: Card below
- Illustration: Robot or error icon (optional)

---

## Responsive Breakpoints Summary

### Mobile (375px–767px)
- Full-width sections with padding (24px)
- Single-column layouts
- Hamburger menu navigation
- No sidebars; drawer-based
- Smaller typography (2–3px reduction)
- Touch targets: 48px minimum
- Bottom sheet for modals

### Tablet (768px–1023px)
- 2-column layouts where applicable
- Navbar remains fixed
- Sidebar appears but narrower (240px)
- Padding: 32px
- 2-column feature/module grids
- Right rail hidden (ToC collapsed drawer)

### Desktop (1024px–1535px)
- Full design system
- Multi-column layouts (3–4)
- All sidebars visible
- Padding: 40px
- Standard spacing (40px+ gaps)

### Wide (1536px+)
- Same as desktop but with additional breathing room
- Possibly wider content containers
- Larger illustrations

---

## Performance Considerations

### Layout Stability
- Reserve space for images: Use aspect-ratio CSS property
- Avoid layout shifts on image load
- Fixed navbar height: No jumping on scroll
- Skeleton screens for deferred content

### Responsive Images
- `srcset` for 1x/2x resolution
- WebP format with PNG fallback
- Lazy-loading by default (loading="lazy")
- Proper alt text

### CSS Optimization
- CSS custom properties for theming
- Minimal media queries (mobile-first)
- No redundant selectors
- Leverage Cascade for inheritance

### JavaScript
- No layout-triggering DOM changes
- Use Intersection Observer for lazy-loading
- Debounce scroll events
- Defer non-critical scripts

---

## Accessibility in Layouts

### Keyboard Navigation
- Tab order follows visual order
- Skip links present (Skip to main content)
- Visible focus indicators
- Modal focus management

### Screen Readers
- Proper heading hierarchy (no skipped levels)
- Semantic HTML: `<nav>`, `<main>`, `<section>`, `<article>`
- ARIA landmarks: role="navigation", role="main", etc.
- List structure preserved (`<ul>`, `<ol>`, `<li>`)

### Color & Contrast
- Text: 7:1 contrast ratio (WCAG AAA)
- Interactive elements: 4.5:1 minimum
- No color alone to convey info (text + icon)

---

## Document Control

- **Version**: 1.0
- **Last Updated**: December 2025
- **Status**: Active
- **Next Review**: January 2026

---
