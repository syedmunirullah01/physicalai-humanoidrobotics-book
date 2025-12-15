# Physical AI & Humanoid Robotics - Visual Style Guide

## Brand Identity & Visual Language

This guide ensures consistent visual representation across all platform touchpoints while maintaining professional credibility and scientific authority.

---

## 1. Color Psychology & Application

### Primary Color: Cyan (#00d9ff)

**Symbolic Meaning**:
- Advanced technology and innovation
- Digital/electronic systems
- Precision and accuracy
- Future-forward thinking

**Application**:
- Primary CTAs (buttons)
- Links and focus indicators
- Active states
- Accent borders
- Glow effects
- Logo/brand mark

**When NOT to use**:
- Don't use as background for large text (contrast issues)
- Avoid overuse (max 20% of design)
- Not suitable for error states

**Variants**:
- Light: #22d3ee (hover states, lighter accents)
- Dark: #0ea5e9 (pressed states)
- Darkest: #0284c7 (active/deepest state)

---

### Secondary Color: Purple (#a855f7)

**Symbolic Meaning**:
- Sophisticated engineering
- Advanced learning/mastery
- Premium/enterprise features
- Thoughtful design

**Application**:
- Gradient overlays (with cyan)
- Alternative accent elements
- Module headers
- Highlight text
- Secondary CTAs

**Pairing with Cyan**:
```css
/* Gradient pattern used across site */
background: linear-gradient(135deg, #00d9ff 0%, #a855f7 100%);
```

---

### Accent Color: Pink (#ec4899)

**Symbolic Meaning**:
- Energy and attention
- Engagement and community
- Modern aesthetics

**Application**:
- Tertiary accents (use sparingly)
- Hover effects on secondary elements
- Emphasis text
- Special announcements

**Overuse Warning**: Pink is powerful—limit to <5% of design to avoid distraction.

---

### Semantic Colors

#### Success (#10b981 - Green)
- Completion indicators
- Successful operations
- Passed tests/validation
- Achievement badges

#### Warning (#f59e0b - Orange)
- Deprecation notices
- Caution messages
- Action required
- Attention-needed alerts

#### Error (#ef4444 - Red)
- Critical errors
- Failed operations
- System issues
- Validation errors

#### Info (#3b82f6 - Blue)
- Information messages
- Tips and hints
- Documentation references
- Neutral notifications

---

### Neutral/Grayscale Application

**Slate-100 (#f1f5f9)**: Primary text on dark backgrounds
- Headings
- Body text
- Maximum contrast

**Slate-300 (#cbd5e1)**: Secondary text
- Descriptions
- Supporting text
- Lower importance

**Slate-500 (#64748b)**: Tertiary text
- Metadata
- Captions
- Disabled text
- Subtle elements

**Slate-700 (#334155)**: Borders and dividers
- Subtle separation
- Form field borders
- Dividing lines

**Slate-900 (#0f172a)**: Darkest backgrounds
- Footer backgrounds
- Dark card backgrounds
- Deepest shadows

---

## 2. Typography System

### Font Stack Hierarchy

```css
/* UI & Body Text (Highly Readable) */
--font-family-base: -apple-system, BlinkMacSystemFont, 'Segoe UI',
                    'Roboto', 'Oxygen', 'Ubuntu', 'Cantarell',
                    'Fira Sans', 'Droid Sans', 'Helvetica Neue',
                    sans-serif;

/* Code & Technical Content */
--font-family-mono: 'JetBrains Mono', 'SF Mono', 'Menlo',
                    'Consolas', monospace;
```

**Why This Stack**:
- IBM Plex Sans: Professional, readable, trusted
- JetBrains Mono: Purpose-built for code, excellent ligatures
- Fallbacks ensure rendering across platforms

---

### Type Scale & Weights

#### Headings

| Level | Mobile | Desktop | Weight | Usage |
|-------|--------|---------|--------|-------|
| H1 | 28px | 48px | 800 | Page titles, hero content |
| H2 | 24px | 40px | 700 | Section headers |
| H3 | 20px | 32px | 700 | Subsection headers |
| H4 | 18px | 24px | 600 | Card titles, labels |
| H5 | 16px | 20px | 600 | Small headers |
| H6 | 14px | 18px | 600 | Micro headers |

**Heading Characteristics**:
- Line-height: 1.2 (tight, authoritative)
- Letter-spacing: -0.02em (brings weight forward)
- Color: Slate-100 (#f1f5f9)
- Margin-bottom: 1rem

#### Body Text

```css
.body-text {
  font-size: 16px;            /* Mobile */
  line-height: 1.75;
  letter-spacing: 0;
  color: var(--color-text-primary);
  font-weight: 400;
}

@media (min-width: 1024px) {
  .body-text {
    font-size: 17px;
    line-height: 1.75;
  }
}
```

**Paragraph Spacing**:
- Margin-bottom: 1.5rem (between paragraphs)
- Max-width: 65–75 characters per line (readability)

#### Code Text

```css
.code {
  font-family: var(--font-family-mono);
  font-size: 13px;            /* Mobile */
  line-height: 1.5;
  letter-spacing: 0.02em;
  font-weight: 500;
}

@media (min-width: 1024px) {
  .code {
    font-size: 14px;
  }
}
```

---

### Weight Usage

| Weight | Font File | Usage |
|--------|-----------|-------|
| 400 | Regular | Body text, paragraphs |
| 500 | Medium | Labels, captions, code |
| 600 | Semibold | Small headers, emphasis |
| 700 | Bold | Headings, strong emphasis |
| 800 | Extrabold | H1, hero titles |

---

## 3. Spacing & Rhythm

### The 8px Grid System

All spacing derives from 8px base unit (or 4px for fine adjustments).

```
4px:   xs (8 × 0.5)
8px:   sm (8 × 1)
16px:  md (8 × 2)
24px:  lg (8 × 3)
32px:  xl (8 × 4)
48px:  2xl (8 × 6)
64px:  3xl (8 × 8)
96px:  4xl (8 × 12)
```

### Application Rules

**Padding**:
- Buttons: 12px 24px (md vertical, md horizontal)
- Cards: 24px (lg)
- Containers: 24px mobile, 40px desktop

**Margin**:
- Section vertical: 64px mobile, 96px desktop
- Element vertical: 1.5rem
- Element horizontal: Auto-centered or grid-based

**Gap** (flexbox/grid):
- Small elements: 8px
- Medium elements: 16px
- Large elements: 24px–32px

---

## 4. Visual Consistency Patterns

### Border & Divider Style

**Accent Border** (Cyan):
```css
border: 1px solid rgba(0, 217, 255, 0.2);  /* Subtle */
border: 1px solid rgba(0, 217, 255, 0.4);  /* Medium */
border: 1px solid #00d9ff;                  /* Strong */
```

**Divider Line**:
```css
border-top: 1px solid var(--color-slate-700);
```

**Left Accent Border** (for callouts, alerts):
```css
border-left: 4px solid var(--color-primary);
```

---

### Shadow Depth

**No-Shadow (Flat)**:
```css
box-shadow: none;
```

**Depth Level 1 (Subtle)**:
```css
box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1);
```

**Depth Level 2 (Medium)**:
```css
box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.1);
```

**Depth Level 3 (Strong)**:
```css
box-shadow: 0 20px 25px -5px rgba(0, 0, 0, 0.1);
```

**Glow Effect** (For emphasis):
```css
box-shadow: 0 0 20px rgba(0, 217, 255, 0.3),
            0 0 40px rgba(0, 217, 255, 0.1);
```

---

### Border Radius Progression

| Value | Use Case |
|-------|----------|
| 0px (none) | Sharp edges (rare) |
| 6px | Small elements (badges, inputs) |
| 12px | Cards, buttons, small containers |
| 16px | Larger containers, modals |
| 24px | Hero sections, featured areas |
| 9999px | Pills, circular badges |

---

## 5. Animation & Motion

### Timing & Easing

**Fast** (150ms): Small interactions
```css
transition: all 150ms cubic-bezier(0.4, 0, 0.2, 1);
```

**Base** (200ms): Standard interactions
```css
transition: all 200ms cubic-bezier(0.4, 0, 0.2, 1);
```

**Smooth** (300ms): Larger elements
```css
transition: all 300ms cubic-bezier(0.4, 0, 0.2, 1);
```

**Slow** (500ms): Full-page transitions
```css
transition: all 500ms cubic-bezier(0.4, 0, 0.2, 1);
```

### Common Animation Patterns

#### Hover Lift
```css
@keyframes hover-lift {
  0% { transform: translateY(0); }
  100% { transform: translateY(-4px); }
}

button:hover {
  animation: hover-lift 200ms ease-out forwards;
  box-shadow: 0 8px 25px rgba(0, 217, 255, 0.4);
}
```

#### Fade In
```css
@keyframes fade-in {
  0% { opacity: 0; }
  100% { opacity: 1; }
}

.content {
  animation: fade-in 300ms ease-out;
}
```

#### Scale In
```css
@keyframes scale-in {
  0% {
    opacity: 0;
    transform: scale(0.95);
  }
  100% {
    opacity: 1;
    transform: scale(1);
  }
}

.element {
  animation: scale-in 300ms cubic-bezier(0.4, 0, 0.2, 1) forwards;
}
```

---

## 6. Icon System

### Icon Specifications

**Sizes**:
- 16px: Inline with text
- 20px: Navigation, UI elements
- 24px: Standard buttons, large UI
- 32px: Feature cards
- 48px: Hero sections, featured
- 64px: Extra large, display

**Stroke Weight**: 2px (consistent)

**Color**: Inherit from text or use accent

**Margin**: 8px gap from adjacent text

### Icon Library Recommendation

Use **Heroicons** (heroicons.com):
- Professional, technical aesthetic
- Comprehensive icon set
- MIT licensed
- Two variants: Solid and Outline

```tsx
import { ArrowRightIcon } from '@heroicons/react/24/outline';

<button>
  Continue
  <ArrowRightIcon className="w-5 h-5 ml-2" />
</button>
```

---

## 7. Component Visual Specifications

### Button Styles

#### Primary Button
```
┌─────────────────────────────────┐
│ Cyan Gradient Background        │
│ Black Text (800 weight)         │
│ Cyan Glow Shadow                │
│ Rounded: 12px                   │
│ Padding: 12px 24px              │
│ Hover: Lift + Enhanced Glow     │
└─────────────────────────────────┘
```

#### Secondary Button
```
┌─────────────────────────────────┐
│ Transparent Background          │
│ Cyan Border (1px)               │
│ Cyan Text                       │
│ Rounded: 12px                   │
│ Padding: 12px 24px              │
│ Hover: Background Tint          │
└─────────────────────────────────┘
```

---

### Card Styles

#### Content Card
```
┌─────────────────────────────────┐
│ Dark Background (rgba surface)  │
│ Cyan Accent Border (0.2 opacity)│
│ Rounded: 12px                   │
│ Padding: 24px                   │
│ Hover:                          │
│   - Border brightens (0.4)      │
│   - Shadow glows cyan           │
│   - Lifts -4px                  │
└─────────────────────────────────┘
```

#### Feature Card
```
┌─────────────────────────────────┐
│ Icon (48×48, cyan glow)         │
│ Title (h4)                      │
│ Description (slate-300)         │
│ No border                       │
│ Hover: Icon glow intensifies    │
└─────────────────────────────────┘
```

---

### Alert Box Styles

| Type | Left Border | Background | Icon Color |
|------|-------------|------------|-----------|
| Info | Cyan | rgba(59, 130, 246, 0.1) | Cyan |
| Success | Green | rgba(16, 185, 129, 0.1) | Green |
| Warning | Orange | rgba(245, 158, 11, 0.1) | Orange |
| Error | Red | rgba(239, 68, 68, 0.1) | Red |

---

## 8. Data Visualization

### Chart Color Palette

**Primary Series** (use in order):
1. Cyan (#00d9ff)
2. Purple (#a855f7)
3. Pink (#ec4899)
4. Green (#10b981)
5. Orange (#f59e0b)

**Avoid**: Red/Green together (colorblind-unfriendly)

### Code Syntax Highlighting

```css
.token-keyword { color: #a855f7; }      /* Purple */
.token-string { color: #10b981; }       /* Green */
.token-number { color: #f59e0b; }       /* Orange */
.token-comment { color: #64748b; }      /* Gray */
.token-function { color: #00d9ff; }     /* Cyan */
.token-class { color: #a855f7; }        /* Purple */
```

---

## 9. Dark Mode Implementation

### Color Adjustments

All colors remain consistent between modes. Dark mode is **always on** by default.

**Text on dark**:
- Primary: Slate-100 (#f1f5f9)
- Secondary: Slate-300 (#cbd5e1)
- Tertiary: Slate-500 (#64748b)

**Backgrounds**:
- Primary: #000000 (pure black)
- Secondary: #0a0a0a (subtle variation)
- Surface: #0f0f0f (cards, modals)

---

## 10. Accessibility in Visual Design

### Color Contrast Ratios

**WCAG AAA Compliance** (7:1 minimum):
- Cyan (#00d9ff) on black: 7.85:1 ✓
- Slate-100 (#f1f5f9) on black: 13.9:1 ✓
- Slate-300 (#cbd5e1) on #0f0f0f: 9.3:1 ✓

### Focus Indicators

**Visible Focus Ring**:
```css
outline: 2px solid var(--color-primary);  /* Cyan */
outline-offset: 4px;                       /* Space from element */
```

**Never remove focus with `outline: none`** without providing visual feedback.

---

## 11. Typography for Different Reading Contexts

### Code (Technical Documentation)
- Monospace font (JetBrains Mono)
- Line-height: 1.5 (tighter, prevents blur)
- Size: 13–14px
- Syntax highlighting for clarity

### Body (Learning Content)
- Sans-serif (IBM Plex Sans)
- Line-height: 1.75 (comfortable for sustained reading)
- Size: 16–17px
- Ample margins

### UI Labels
- Sans-serif (IBM Plex Sans)
- Font-weight: 600 (clarity)
- Size: 14–16px
- All-caps for important labels

---

## 12. Component State Matrix

### Button States

```
Normal    → Hover    → Active   → Disabled
─────────────────────────────────────────
Cyan      Lighter    Darker    Gray
          Lifted     Pressed   Dimmed
          Glow+      Glow-     No shadow
```

### Card States

```
Rest      → Hover    → Active   → Loading
─────────────────────────────────────────
Subtle    Bright     Highlighted Shimmer
Border    Border     Border     Skeleton
Shadow    Shadow++   Shadow+++  Gray tone
```

---

## 13. Responsive Visual Hierarchy

### Mobile (375px)
- Larger touch targets (48px minimum)
- Simplified shadows (max 1 level)
- Bolder type weights
- Generous spacing (24px padding)

### Tablet (768px)
- Balanced visual hierarchy
- Standard shadows (2 levels)
- Normal weights
- Standard spacing (32px padding)

### Desktop (1024px+)
- Complex layouts possible
- Full shadow hierarchy
- Refined spacing (40px+)
- All visual effects enabled

---

## 14. Quality Assurance Checklist

- [ ] All text meets 7:1 contrast ratio (WCAG AAA)
- [ ] Focus indicators visible on all interactive elements
- [ ] Animations respect `prefers-reduced-motion`
- [ ] Colors used purposefully (not decorative)
- [ ] Typography scale is consistent
- [ ] Spacing follows 8px grid
- [ ] Icons are 24px default size
- [ ] Buttons have sufficient padding (touch-friendly)
- [ ] Hover states provide visual feedback
- [ ] Loading states are clear (spinner, skeleton, etc.)
- [ ] Error states are distinguishable
- [ ] Responsive design tested on 3+ breakpoints
- [ ] Dark mode text is readable
- [ ] SVGs are crisp at 2x scale
- [ ] Shadows enhance depth, don't distract

---

## 15. Common Design Mistakes to Avoid

1. **Too Much Cyan**: Limit to 20% of design area
2. **Removing Focus Rings**: Never—users rely on them
3. **Contrast Fails**: Test every text color combination
4. **Inconsistent Spacing**: Use the 8px grid, always
5. **Unintended Motion**: Respect `prefers-reduced-motion`
6. **Oversized Icons**: 24px is standard, 48px is maximum
7. **Weak Hierarchy**: Use weight + size + color + spacing
8. **Ignoring Mobile**: Mobile-first design is non-negotiable
9. **Non-Semantic Colors**: Use semantic colors (error = red)
10. **Animations on Load**: Debounce, lazy-load, don't bog down initial render

---

## Resources

### Color Tools
- WebAIM Contrast Checker: webaim.org/resources/contrastchecker/
- Color-Blind Simulator: colorblindsimulator.com
- Coolors.co: For palette exploration

### Typography
- Typewolf: Font pairing inspiration
- Google Fonts: IBM Plex Sans download
- Font File Optimization: subfont, pyftsubset

### Icon Resources
- Heroicons: heroicons.com
- Feather Icons: feathericons.com
- Material Icons: fonts.google.com/icons

### Accessibility
- WebAIM: webaim.org
- A11y Project: a11yproject.com
- Axe DevTools: Browser extension for testing

---

## Document Control

- **Version**: 1.0
- **Last Updated**: December 2025
- **Status**: Active
- **Maintainer**: Design Team

---
