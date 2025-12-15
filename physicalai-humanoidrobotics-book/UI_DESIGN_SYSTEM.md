# Physical AI & Humanoid Robotics - Professional UI Design System

## Executive Summary

This document defines a comprehensive, research-grade UI design system for the Physical AI & Humanoid Robotics educational textbook. The system prioritizes scientific credibility, visual sophistication, and professional accessibility while maintaining a futuristic dark theme aesthetic.

---

## 1. Design Philosophy

### Core Principles

1. **Scientific Credibility First**: Every design choice must withstand scrutiny from robotics engineers and researchers. Visual decisions are justified by information architecture, not trends.

2. **Hierarchical Information Architecture**: Content flows from hero → credibility → technical depth → engagement. Readers immediately understand value and can navigate to their level of interest.

3. **Performance-Optimized**: GPU-accelerated animations, will-change properties, and lazy-loaded assets. The site feels snappy and responsive—befitting a technical audience.

4. **Accessibility as Foundation**: WCAG 2.1 AAA compliance, keyboard navigation, screen reader optimization, and motion reduction support are non-negotiable.

5. **Responsive & Device-Agnostic**: Works flawlessly on mobile, tablet, desktop, and ultra-wide displays. Content reflows intelligently; functionality never degrades.

6. **Precision in Details**: Consistent spacing grids, deliberate type scales, carefully orchestrated transitions. No cosmetic elements without purpose.

---

## 2. Design System Architecture

### 2.1 Typography

#### Font Stack
- **UI & Body**: IBM Plex Sans, Segoe UI, Roboto (modern, readable, technical credibility)
- **Code**: JetBrains Mono, SF Mono, Menlo (consistent with industry standards)
- **Display/Headings**: IBM Plex Sans (weight: 700-800, letter-spacing: -0.02em)

#### Type Scale (Mobile → Desktop)
```
h1: 28px → 48px (font-weight: 800)
h2: 24px → 40px (font-weight: 700)
h3: 20px → 32px (font-weight: 700)
h4: 18px → 24px (font-weight: 600)
body: 16px → 17px (font-weight: 400)
caption: 12px → 13px (font-weight: 500)
code: 13px → 14px (font-weight: 500, monospace)
```

#### Line Heights
- Headings: 1.2 (tight, authoritative)
- Body text: 1.6–1.75 (optimal reading comfort for technical content)
- Code: 1.5 (prevents cramping)

#### Letter Spacing
- Headings: -0.02em (tightness conveys control)
- Body: 0 (default)
- UPPERCASE labels: 0.1em (clarity)

### 2.2 Color Palette

#### Primary Colors
- **Cyan/Electric Blue**: #00d9ff (primary accent, energy, advanced tech)
- **Cyan Secondary**: #22d3ee (lighter variant, readability)
- **Sky Blue**: #0ea5e9 (supporting accent)

#### Accent Colors
- **Purple**: #a855f7 (secondary accent, visual variety)
- **Pink**: #ec4899 (tertiary accent, emphasis)

#### Neutral/Grayscale (Slate)
- **Slate-50**: #f8fafc (lightest)
- **Slate-100**: #f1f5f9
- **Slate-200**: #e2e8f0
- **Slate-300**: #cbd5e1
- **Slate-400**: #94a3b8
- **Slate-500**: #64748b (mid-tone)
- **Slate-600**: #475569
- **Slate-700**: #334155
- **Slate-800**: #1e293b
- **Slate-900**: #0f172a (darkest)

#### Background
- **Primary Dark**: #000000 (pure black for contrast)
- **Secondary Dark**: #0a0a0a (page backgrounds)
- **Surface Dark**: #0f0f0f (cards, modals)

#### Text Colors
- **Dark Mode (Primary)**: Slate-100 (#f1f5f9) on dark backgrounds
- **Dark Mode (Secondary)**: Slate-300 (#cbd5e1)
- **Dark Mode (Tertiary)**: Slate-500 (#64748b)
- **Dark Mode (Disabled)**: Slate-600 (#475569)

### 2.3 Spacing System

**8px base unit** (also supports 4px for fine adjustments)

```
--spacing-xs: 4px (0.25rem)
--spacing-sm: 8px (0.5rem)
--spacing-md: 16px (1rem)
--spacing-lg: 24px (1.5rem)
--spacing-xl: 32px (2rem)
--spacing-2xl: 48px (3rem)
--spacing-3xl: 64px (4rem)
--spacing-4xl: 96px (6rem)
```

**Responsive Scale Factor**: Multiply by 1.5x on tablet+ for breathing room.

### 2.4 Shadows & Depth

```
--shadow-sm: 0 1px 2px 0 rgba(0,0,0,0.05)
--shadow-md: 0 4px 6px -1px rgba(0,0,0,0.1), 0 2px 4px -2px rgba(0,0,0,0.1)
--shadow-lg: 0 10px 15px -3px rgba(0,0,0,0.1), 0 4px 6px -4px rgba(0,0,0,0.1)
--shadow-xl: 0 20px 25px -5px rgba(0,0,0,0.1), 0 8px 10px -6px rgba(0,0,0,0.1)
--shadow-2xl: 0 25px 50px -12px rgba(0,0,0,0.25)

/* Glowing shadows for accent elements */
--shadow-glow-cyan: 0 0 20px rgba(0,217,255,0.3), 0 0 40px rgba(0,217,255,0.1)
--shadow-glow-purple: 0 0 20px rgba(168,85,247,0.3), 0 0 40px rgba(168,85,247,0.1)
```

### 2.5 Border Radius

```
--radius-sm: 6px (subtle, UI elements)
--radius-md: 12px (cards, buttons)
--radius-lg: 16px (larger containers)
--radius-xl: 24px (hero sections, modals)
--radius-full: 9999px (pills, badges)
```

### 2.6 Transitions & Animations

#### Base Timing Functions
```
--transition-fast: 150ms cubic-bezier(0.4, 0, 0.2, 1) (easing in)
--transition-base: 200ms cubic-bezier(0.4, 0, 0.2, 1) (standard)
--transition-smooth: 300ms cubic-bezier(0.4, 0, 0.2, 1) (content)
--transition-slow: 500ms cubic-bezier(0.4, 0, 0.2, 1) (large sections)
--transition-bounce: 500ms cubic-bezier(0.68, -0.55, 0.265, 1.55) (playful)
```

#### Animation Principles
- **Entrance**: Fade-in + slight scale (0.95 → 1.0) over 300–500ms
- **Hover**: Color shift + subtle lift (transform: translateY(-2px)) over 200ms
- **Focus**: 2px cyan outline with slight glow, no animation needed (accessibility)
- **Active/Selected**: Solid color + underline or background shift
- **Disabled**: Reduced opacity (0.5), cursor: not-allowed, no hover effects

#### Reduced Motion Support
```css
@media (prefers-reduced-motion: reduce) {
  * {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
  }
}
```

### 2.7 Breakpoints

```
mobile: 375px–767px
tablet: 768px–1023px
desktop: 1024px–1535px
wide: 1536px+
```

---

## 3. Component Architecture

### 3.1 Layout Components

#### Container
- Max-width: 1280px (content area, centered)
- Padding: 24px mobile, 40px tablet+
- Purpose: Consistent content width across pages

#### Grid System
- Mobile: 1 column (full bleed with padding)
- Tablet: 2–3 columns
- Desktop: 3–4 columns
- Gap: 24px (mobile), 32px (tablet), 40px (desktop)

#### Section
- Padding: 64px (mobile), 96px (desktop) vertical
- Margin: 0
- Border-top: Optional 1px cyan accent (0.2 opacity)

### 3.2 Navigation Components

#### Primary Navigation (Navbar)
- Height: 70px (desktop), 60px (mobile)
- Position: Fixed, top, z-index: 1000
- Background: rgba(10,10,10,0.95) with backdrop-filter: blur(12px)
- Border-bottom: 1px cyan with 0.3 opacity
- Shadow: Glow effect with cyan

**Items**:
- Logo: 36px height, gradient cyan text (left side)
- Title: "Physical AI & Humanoid Robotics", gradient cyan
- Links: 4–5 primary (Modules, Docs, Research, Sign In, Sign Up)
- Dropdown: Topics (expands to show modules)

#### Secondary Navigation (Breadcrumbs/Path)
- Font-size: 12px
- Color: Slate-400
- Separator: " / "
- Last item: Bold, Slate-200

#### Sidebar (Docs)
- Width: 280px (desktop), hidden mobile
- Position: Sticky, top: 100px
- Scrollable: Within viewport
- Links: Styled with active indicator (left border, background)

#### Footer Navigation
- 3–4 columns (stacked mobile)
- Categories: Product, Learning, Community, Legal
- Sub-links: 8–10 per category
- Text color: Slate-300

### 3.3 Button Components

#### Primary Button
```
Padding: 12px 24px
Background: #00d9ff (cyan)
Color: #000000
Font-weight: 600
Border-radius: 12px
Box-shadow: 0 4px 15px rgba(0,217,255,0.3)
Transition: All 200ms

Hover:
  - Background: #22d3ee (lighter)
  - Transform: translateY(-2px)
  - Box-shadow: 0 8px 25px rgba(0,217,255,0.4)

Focus:
  - Outline: 2px cyan, 4px offset

Active:
  - Background: #0ea5e9
  - Transform: translateY(0)

Disabled:
  - Background: Slate-300
  - Color: Slate-500
  - Cursor: not-allowed
  - Opacity: 0.6
```

#### Secondary Button
```
Padding: 12px 24px
Background: rgba(0,217,255,0.1)
Color: #00d9ff
Border: 1px solid rgba(0,217,255,0.3)
Font-weight: 600
Border-radius: 12px
Transition: All 200ms

Hover:
  - Background: rgba(0,217,255,0.2)
  - Border-color: rgba(0,217,255,0.5)
  - Color: #22d3ee
```

#### Ghost Button (Navigation, Links)
```
Background: transparent
Color: Slate-300
Border: none
Font-weight: 500
Padding: 8px 12px
Transition: All 200ms

Hover:
  - Color: #00d9ff
  - Background: rgba(0,217,255,0.05)
```

#### Size Variants
- Small: 8px 16px (inline, secondary actions)
- Medium: 12px 24px (primary, standard)
- Large: 16px 32px (hero, call-to-action)

### 3.4 Card Components

#### Content Card
```
Background: rgba(15,15,15,0.6)
Border: 1px solid rgba(0,217,255,0.2)
Border-radius: 12px
Padding: 24px
Box-shadow: 0 4px 6px -1px rgba(0,0,0,0.1)
Transition: All 300ms

Hover:
  - Border-color: rgba(0,217,255,0.4)
  - Box-shadow: 0 10px 25px -5px rgba(0,217,255,0.2)
  - Transform: translateY(-4px)
```

#### Feature Card (with icon)
```
Icon area: 48px × 48px, cyan glow
Title: h4, Slate-100
Description: body, Slate-300
Background: Gradient (dark to darker) or solid dark
Padding: 32px
Border-radius: 16px
```

#### Module Card
```
Header: Gradient background (cyan to purple)
Title: h3
Description: Slate-300
Meta: Grid (duration, difficulty, topics)
Footer: Button + progress indicator
Height: Auto or 480px
Padding: 24px
```

### 3.5 Form Components

#### Input Field
```
Padding: 12px 16px
Background: rgba(0,0,0,0.3)
Border: 1px solid rgba(0,217,255,0.2)
Border-radius: 8px
Color: Slate-100
Font-size: 16px
Transition: All 200ms

Focus:
  - Border-color: #00d9ff
  - Box-shadow: 0 0 0 3px rgba(0,217,255,0.15)
  - Background: rgba(0,217,255,0.05)

Error:
  - Border-color: #ef4444
  - Box-shadow: 0 0 0 3px rgba(239,68,68,0.15)
```

#### Label
```
Font-size: 14px
Font-weight: 600
Color: Slate-200
Margin-bottom: 8px
```

#### Select/Dropdown
```
Same as input field
Icon: Chevron down, right side, pointer-events: none
```

### 3.6 Data Visualization Components

#### Code Block
```
Background: rgba(0,0,0,0.5)
Border: 1px solid rgba(0,217,255,0.2)
Border-radius: 8px
Padding: 16px
Font: JetBrains Mono, 14px
Line-height: 1.5
Overflow: Auto (max-height: 400px)

Header (optional):
  - Language badge (top-right)
  - Copy button (top-right)
  - File name (top-left)

Syntax highlighting:
  - Keywords: Purple (#a855f7)
  - Strings: Green (#10b981)
  - Numbers: Orange (#f97316)
  - Comments: Slate-500
```

#### Progress Bar
```
Track: Slate-700 background, 12px height
Bar: Cyan gradient, 12px height, rounded caps
Animation: Subtle shimmer on active/loading

Variants:
  - Linear (percentage shown)
  - Circular (radial progress)
  - Segments (step progress, e.g., 4/12 chapters)
```

#### Diagram/Schematic Container
```
Background: rgba(10,10,10,0.6)
Border: 1px cyan (0.3 opacity)
Padding: 32px
Border-radius: 16px
Max-width: 100%, aspect-ratio aware
SVG/Canvas: Lazy-loaded
```

#### Tab Component
```
Tabs bar: Flex row, gap 8px
Tab: Padding 12px 16px, background transparent
  Hover: Background rgba(0,217,255,0.05)
  Active: Bottom border 2px cyan
  Transition: All 200ms

Content: Fade in/out, 200ms
```

### 3.7 Feedback & Status Components

#### Alert Box
```
Padding: 16px 20px
Border-left: 4px solid (color by type)
Border-radius: 8px
Background: rgba(color, 0.1)
Icon: 24px, left side
Text: Slate-200

Types:
  - Info: Cyan border, cyan icon
  - Success: Green border, green icon
  - Warning: Orange border, orange icon
  - Error: Red border, red icon
```

#### Badge/Tag
```
Padding: 4px 12px
Background: rgba(0,217,255,0.1)
Border: 1px solid rgba(0,217,255,0.3)
Border-radius: 6px
Font-size: 12px
Font-weight: 600
Color: #00d9ff

Variants by type:
  - Primary (cyan)
  - Success (green)
  - Warning (orange)
  - Custom (purple for topics)
```

#### Tooltip
```
Background: Slate-900
Color: Slate-100
Padding: 8px 12px
Border-radius: 6px
Font-size: 12px
Border: 1px solid Slate-700
Box-shadow: 0 4px 12px rgba(0,0,0,0.4)
Delay: 200ms hover
Arrow: CSS triangle to source
```

---

## 4. Page Layouts & Specifications

### 4.1 Homepage (index)

#### Hero Section
- Min-height: 100vh
- Background: Gradient (dark to darker)
- Content: Centered, max-width 1000px
- Elements:
  - Label: "Industry-Leading Robotics Education"
  - H1: Main title with gradient text
  - Subtitle: Tagline
  - Description: 2–3 sentences
  - CTA buttons: Primary (Start Learning) + Secondary (Documentation)
  - Animated robot illustration (right side, desktop only)
  - Background: Geometric patterns (hexagons, circuit lines) with low opacity

#### Metrics Section
- 4 metrics in a row (responsive grid)
- Metrics: Users, Modules, Research Hours, Industry Partners
- Layout: Cards with large numbers, small labels
- Animation: Numbers count up on scroll

#### Features/Highlights Section
- 3 columns (mobile: 1, tablet: 2)
- Feature cards with icons
- Icon: 48×48px, cyan glow
- Title: h3
- Description: Slate-300
- Spacing: 32px gap

#### Curriculum Section
- Title: "Master Advanced Robotics"
- Module cards: 2–3 per row
- Card elements:
  - Gradient header (cyan to purple)
  - Title, description
  - Tech tags
  - Progress indicator
  - CTA button (Start/Continue)
  - Duration + difficulty

#### Enterprise/Advanced Section
- 2-column layout (image + text)
- Title: "Production-Ready Architecture"
- 4 feature items with icons
- Code window visualization (syntax-highlighted)
- CTA: "Start Building"

#### Footer
- 4-column link grid (mobile: 1)
- Newsletter signup form
- Social links
- Copyright

### 4.2 Documentation Index

#### Layout
- Sidebar (left, sticky): 280px width
- Main content (center): 800px max-width
- Right rail (optional): Table of contents / On This Page

#### Sidebar
- Search box at top (input field)
- Collapsible sections
- Active link highlighting (left border, background)
- Smooth scrolling

#### Content Area
- H1 at top (title)
- Metadata: Last updated, author, reading time
- Horizontal rule separator
- Markdown/MDX content with styled elements
- Code blocks with language badges
- Callout boxes (tip, warning, note)
- Related links at bottom

#### Right Rail (Table of Contents)
- Position: Sticky, top: 100px
- Width: 220px
- List: H2/H3 headings from page
- Active: Color highlight, smooth scroll
- Hide on tablet

### 4.3 Module Overview Page

#### Hero Section (Smaller)
- Height: 400px
- Title: Module name + number
- Description: 2–3 sentences
- Key stats: Duration, chapters, topics

#### Module Navigation
- Breadcrumb: Home / Modules / Module 1
- Chapter list: Vertical cards
- Chapter card:
  - Chapter number + title
  - Description
  - Estimated time
  - Status indicator (completed, in progress, locked)
  - CTA: "Start Chapter"

#### Learning Objectives
- Title: "What You'll Learn"
- Unordered list with checkmarks (cyan)
- 3–4 objectives
- Large font, easy to scan

#### Prerequisites
- Title: "Prerequisites"
- List or tags
- Links to prerequisite modules

#### Resources Section
- Downloadables: Code, datasets
- Links: External resources, papers
- Tools: Software requirements

### 4.4 Chapter/Article Page

#### Header
- Breadcrumb navigation
- H1: Chapter title
- Metadata: Author, date, reading time, difficulty
- Featured image or diagram (if applicable)

#### Table of Contents Sidebar
- Sticky, left side
- Headings from page (h2, h3)
- Scroll highlighting
- Mobile: Collapsible menu icon

#### Main Content
- Wide margins (responsive)
- Consistent typography
- Embedded diagrams/schematics with captions
- Code blocks with line numbers
- Callout boxes (info, warning, tip)
- Images: Lazy-loaded, responsive, with alt text

#### Interactive Elements
- Collapsible sections (details/summary)
- Tabs for comparing concepts
- Lightbox for large images/diagrams

#### Footer
- "Previous Chapter" / "Next Chapter" navigation
- "Was this helpful?" widget
- Author bio (small avatar, name, bio)
- Related articles

### 4.5 Search Results Page

#### Layout
- Sidebar (filters) + Results (main)
- Search box at top (with query)
- Filter options: Type (docs/blog), Module, Difficulty

#### Results
- List view or grid view toggle
- Result card:
  - Title (link)
  - Breadcrumb path
  - 2-line description
  - Metadata: Date, reading time, module
  - Highlight of matching text

#### Pagination
- Bottom of results
- Previous / Next buttons
- Page numbers (1, 2, 3...)

### 4.6 Not Found (404) Page

#### Design
- Centered, large text
- Message: "Page not found" + description
- Illustration: Abstract robot or error icon
- Suggested links: Home, Docs, Contact

---

## 5. Interaction Patterns

### 5.1 Hover States

- **Links**: Color shift (Slate-300 → Cyan), no underline unless explicitly styled
- **Buttons**: Color shift + lift (translate -2px Y) + shadow increase
- **Cards**: Border brightening + shadow increase + slight lift
- **Code blocks**: Background brightens, copy button appears

### 5.2 Focus States

- **All focusable elements**: 2px cyan outline, 4px offset
- **No blur on focus**: Outline remains visible on click
- **Accessibility**: Outline fits design language, not removed

### 5.3 Loading States

- **Skeleton screens**: Gradient shimmer on gray boxes
- **Progress bars**: Subtle animation (optional shimmer)
- **Spinners**: Cyan rotating circle, 24px diameter
- **State text**: "Loading..." with trailing dots animation

### 5.4 Success States

- **Form submission**: Toast notification (top-right), green accent
- **Action completion**: Checkmark icon, brief message
- **Duration**: Display for 3–4 seconds, auto-dismiss

### 5.5 Error States

- **Form field errors**: Red border, error message below field
- **Inline errors**: Red text, red icon
- **Toast notification**: Red background, error icon, dismiss button
- **Network errors**: Retry button offered

### 5.6 Empty States

- **No results**: Centered illustration, message, suggested action
- **No data**: Gray text, call-to-action to populate

---

## 6. Responsive Design Strategy

### 6.1 Mobile (375px–767px)

- Single-column layouts
- Full-bleed sections (padding: 24px)
- Hamburger menu (navbar collapse)
- Large touch targets (48px minimum)
- No sidebars; drawer-based navigation
- Stacked cards
- Simplified animations (reduced motion by default)

### 6.2 Tablet (768px–1023px)

- 2-column layouts where applicable
- Smaller padding (32px)
- Navbar remains fixed, no collapse
- Sidebar appears but may be narrower (240px)
- Cards in 2-column grid
- Touch-friendly spacing maintained

### 6.3 Desktop (1024px+)

- Full design system implemented
- Multi-column layouts (3–4 columns)
- Standard padding/margins (40px+)
- All sidebars visible
- Large illustrations and diagrams
- Hover effects fully enabled

### 6.4 Responsive Typography

- Base scale: 16px mobile → 17px desktop
- Headings: 28px → 48px (h1), proportional scaling for h2–h4
- Line-height: Maintained at mobile, increases on desktop for comfort

### 6.5 Responsive Images

- `<img>` with `srcset` for 1x/2x resolution
- `<picture>` element for format fallbacks (WebP)
- Aspect-ratio CSS property for consistent sizing
- Lazy-loading by default
- Intrinsic sizing where possible

---

## 7. Dark Mode Implementation

### 7.1 Color Adjustments

All colors already defined for dark mode in CSS custom properties.

### 7.2 Image Handling

- Transparent PNGs: Work in both modes
- Photographic images: Add slight overlay on dark mode for contrast
- Diagrams: SVG with CSS color properties (prefer SVG for vector graphics)

### 7.3 Code Blocks

- Dark mode: Already dark; maintain syntax highlighting
- Light mode: Swap background (light) and text colors

### 7.4 Transitions

- Color mode toggle: Fade (200ms) for smooth transition
- No jarring changes; all colors transition smoothly

---

## 8. Accessibility Standards

### 8.1 WCAG 2.1 Level AAA Compliance

- **Color Contrast**: All text meets 7:1 contrast ratio (AAA standard)
- **Font Sizes**: Minimum 14px for body, 16px for interactive elements
- **Link Identification**: Underlines or color alone; color + underline preferred
- **Focus Indicators**: Always visible, not removed

### 8.2 Keyboard Navigation

- **Tab order**: Logical, left-to-right, top-to-bottom
- **Skip links**: "Skip to main content" at top of page
- **Visible focus**: All elements show clear focus state
- **Modal management**: Focus traps in modals, returns to trigger on close

### 8.3 Screen Reader Support

- **Semantic HTML**: `<nav>`, `<main>`, `<section>`, `<article>` used correctly
- **ARIA labels**: Added to icon-only buttons, empty links
- **List structure**: Proper `<ul>`, `<ol>`, `<li>` for lists
- **Headings**: Hierarchical, no skipped levels
- **Image alt text**: Descriptive, not just "image"

### 8.4 Motion Reduction

- **`prefers-reduced-motion` query**: All animations respect this
- **Default**: Animations disabled if user prefers reduced motion
- **Fallback**: Animated elements work without animation

---

## 9. Performance Optimization

### 9.1 CSS Strategies

- **Custom Properties**: Reduce duplication, simplify theming
- **Critical CSS**: Inline hero section styles, defer rest
- **Minimize repaints**: Use `transform` and `opacity` for animations
- **Will-change**: Applied sparingly to animated elements
- **Media queries**: Mobile-first approach, progressive enhancement

### 9.2 Asset Optimization

- **Images**: WebP with PNG fallback, lazy-loading, responsive srcset
- **SVGs**: Inline for small icons, external for large diagrams
- **Fonts**: Subset fonts, font-display: swap for fast rendering
- **JavaScript**: Defer non-critical scripts, use async where safe

### 9.3 Layout Stability

- **CLS prevention**: Reserve space for images, no shifting layouts
- **Aspect-ratio**: Used for media containers
- **Font-size-adjust**: Mitigates flash of unstyled text

---

## 10. Brand & Visual Identity

### 10.1 Visual Style

- **Geometric**: Subtle hexagons, circuit lines, data streams (background elements)
- **Futuristic**: Cyan and purple neon accents, glow effects
- **Professional**: Flat design with subtle depth, no skeuomorphism
- **Minimal**: Only necessary visual elements; no ornamentation

### 10.2 Iconography

- **Library**: Use a consistent set (e.g., Heroicons, Feather)
- **Size**: 24px default, 16px inline, 48px large
- **Color**: Inherit text color or use accent color
- **Stroke**: 2px width for consistency

### 10.3 Illustrations

- **Robot character**: Simple, geometric, friendly
- **Diagrams**: Technical, detailed, research-grade
- **Patterns**: Hexagons, circuits, data flows (low opacity, background)
- **Consistency**: Same stroke weight, line style, color palette

### 10.4 Data Visualization

- **Charts**: Use libraries (e.g., Recharts, D3.js)
- **Colors**: Follow accent color palette; cyan primary
- **Legends**: Always included, positioned clearly
- **Responsiveness**: Resize gracefully on smaller screens

---

## 11. Testing & Quality Assurance

### 11.1 Visual Testing

- **Responsive checks**: All breakpoints tested
- **Browser support**: Latest 2 versions of Chrome, Firefox, Safari, Edge
- **Dark mode**: Both modes tested thoroughly
- **Motion reduction**: Verified on macOS, Windows, iOS

### 11.2 Accessibility Testing

- **Keyboard nav**: Tab through every page
- **Screen reader**: Test with NVDA, JAWS, VoiceOver
- **Color contrast**: Check with WebAIM tools
- **Lighthouse**: Run accessibility audit (target: 95+)

### 11.3 Performance Testing

- **Lighthouse**: Target 90+ for performance
- **Web Vitals**: LCP <2.5s, FID <100ms, CLS <0.1
- **Network throttling**: Test on 3G (slow) connection
- **Bundle size**: Monitor CSS/JS; aim for <50KB gzipped

---

## 12. File Structure & Implementation

### 12.1 CSS Organization

```
src/css/
├── custom.css (global styles, animations, theme)
├── components/ (optional, component-specific styles)
│   ├── navbar.css
│   ├── buttons.css
│   ├── cards.css
│   └── forms.css
└── layouts/ (optional, page layout styles)
    ├── homepage.css
    ├── docs.css
    └── page.css
```

### 12.2 Component Structure

```
src/components/
├── Navigation/
│   ├── Navbar.tsx
│   ├── Sidebar.tsx
│   ├── Breadcrumb.tsx
│   └── Footer.tsx
├── Buttons/
│   ├── PrimaryButton.tsx
│   └── SecondaryButton.tsx
├── Cards/
│   ├── ContentCard.tsx
│   ├── FeatureCard.tsx
│   └── ModuleCard.tsx
├── Forms/
│   ├── Input.tsx
│   ├── Select.tsx
│   └── FormGroup.tsx
├── Feedback/
│   ├── Alert.tsx
│   ├── Badge.tsx
│   └── Tooltip.tsx
├── DataViz/
│   ├── CodeBlock.tsx
│   ├── ProgressBar.tsx
│   └── DiagramContainer.tsx
└── Layout/
    ├── Container.tsx
    ├── Section.tsx
    └── Grid.tsx
```

### 12.3 Page Structure

```
src/pages/
├── index.tsx (homepage)
├── index.module.css
└── 404.tsx (not found)

docs/
├── preface.mdx
├── module1/ (ROS 2 Fundamentals)
│   ├── _category_.json
│   ├── overview.md
│   ├── chapter-1-installation.mdx
│   └── chapter-2-topics.mdx
├── module2/ (Digital Twin)
│   ├── _category_.json
│   ├── overview.md
│   ├── chapter-1-gazebo.mdx
│   └── chapter-2-unity.mdx
└── api/ (API Reference)
    └── ...
```

---

## 13. Future Enhancements

- Interactive schematics (WebGL, Three.js)
- Code playground (live execution, output)
- Video tutorials (embedded, lazy-loaded)
- Community forum integration
- User accounts & progress tracking
- Searchable API reference
- Mobile app (React Native)

---

## Document Control

- **Version**: 1.0
- **Date**: December 2025
- **Status**: Active
- **Maintainer**: Design & Engineering Team

---
