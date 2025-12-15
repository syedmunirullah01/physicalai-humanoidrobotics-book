# Homepage Implementation Guide: Professional UI Components

## Quick Start

### Build & Test
```bash
cd E:\AIDD-HACKATHON\physicalai-humanoidrobotics-book
npm run build          # Production build
npm run start          # Development server
npm run serve          # Serve production build locally
```

## Component Integration Patterns

### 1. Section + Container Pattern
All major sections use this layout structure:

```tsx
<Section variant="default">  {/* or "dark" or "elevated" */}
  <Container size="lg">       {/* or "sm", "md", "xl", "full" */}
    <div className={styles.sectionHeader}>
      <div className={styles.sectionLabel}>Section Label</div>
      <Heading as="h2" className={styles.sectionHeading}>
        Section Title
      </Heading>
      <p className={styles.sectionText}>
        Section description...
      </p>
    </div>

    {/* Content goes here */}
  </Container>
</Section>
```

### 2. Feature Grid Pattern
For displaying feature cards in responsive grids:

```tsx
<Grid cols={{mobile: 1, tablet: 2, desktop: 4}} gap="lg">
  {features.map((feature) => (
    <FeatureCard
      key={feature.title}
      title={feature.title}
      description={feature.description}
      icon={<span className={styles.featureIcon}>{feature.icon}</span>}
    />
  ))}
</Grid>
```

### 3. Module Card Pattern
For displaying learning modules with full metadata:

```tsx
<ModuleCard
  number={module.number}
  title={module.title}
  description={module.description}
  chapters={module.chapters}
  duration={module.duration}
  difficulty={module.difficulty}  // 'beginner' | 'intermediate' | 'advanced'
  progress={module.progress}       // 0-100
  tags={module.tags}               // ['ROS 2', 'Python', ...]
  status={module.status}           // 'locked' | 'available' | 'in-progress' | 'completed'
  href={module.href}               // Navigation link
/>
```

### 4. Button Pattern
Two complementary button styles:

```tsx
{/* Primary Action (Cyan) */}
<PrimaryButton href="/path" size="large">
  Begin Your Journey
</PrimaryButton>

{/* Secondary Action (Outlined) */}
<SecondaryButton href="/path" size="large">
  View Modules
</SecondaryButton>

{/* Button Sizes: "small" | "medium" | "large" */}
{/* href optional - can use onClick instead */}
```

### 5. Badge Pattern
For technology tags and status indicators:

```tsx
{technologies.map((tech) => (
  <Badge key={tech} variant="default">
    {tech}
  </Badge>
))}

{/* Variants: 'default' | 'success' | 'warning' | 'error' */}
```

## Layout Hierarchy

### Hero Section
The hero is full-viewport and uses absolute positioning for background elements:

```tsx
<header className={styles.heroBanner}>  {/* min-height: 100vh */}
  {/* Background layers */}
  <div className={styles.geometricBackground}>
    <div className={styles.hexGrid}>{/* animated hexagons */}</div>
    <div className={styles.circuitLayer}>{/* SVG circuit paths */}</div>
  </div>

  {/* Robot illustration (absolute positioned) */}
  <div className={styles.robotContainer}>{/* SVG robot */}</div>

  {/* Content (relative positioned) */}
  <Container size="full">
    <div className={styles.heroContent}>
      {/* headline, cta, tech stack */}
    </div>
  </Container>

  {/* Scroll indicator (bottom) */}
  <div className={styles.scrollIndicator}>{/* animation */}</div>
</header>
```

### Section Variants

#### Default (Primary)
- Background: `#0a0a0a`
- For content sections
- Example: Features Section, Modules Section

```tsx
<Section variant="default">
  {/* content */}
</Section>
```

#### Dark (Secondary)
- Background: Darker than default
- For alternating visual rhythm
- Example: Metrics Section, Enterprise Section

```tsx
<Section variant="dark">
  {/* content */}
</Section>
```

#### Elevated (Tertiary)
- Maximum contrast
- For important CTAs or highlights
- Example: Final CTA Section

```tsx
<Section variant="elevated">
  {/* content */}
</Section>
```

## CSS Classes Reference

### Page-Level Classes

| Class | Purpose | Used In |
|-------|---------|---------|
| `.heroBanner` | Hero section container | HomepageHeader |
| `.geometricBackground` | Animated hex grid + circuits | Background layer |
| `.robotContainer` | Robot SVG wrapper | Background layer |
| `.heroContent` | Hero text content | Hero section |
| `.heroLabel` | "Industry-Leading..." badge | Hero label |
| `.heroTitle` | Main h1 title | Hero |
| `.heroCtaGroup` | Button container | Hero |
| `.heroDescription` | Subtitle text | Hero |
| `.techStack` | "Powered by" section | Hero footer |
| `.scrollIndicator` | Scroll hint animation | Hero bottom |

### Section Headers

| Class | Purpose | Value |
|-------|---------|-------|
| `.sectionHeader` | Container for header | text-align: center |
| `.sectionLabel` | Inline badge | gradient background |
| `.sectionHeading` | h2 title | clamp(2rem, 5vw, 3rem) |
| `.sectionText` | Description | 1.125rem, 1.75 line-height |

### Feature Cards

| Class | Purpose |
|-------|---------|
| `.featureIcon` | Icon styling (2.5rem, glow filter) |

### Metrics

| Class | Purpose |
|-------|---------|
| `.metricCard` | Card container with backdrop blur |
| `.metricValue` | Large number (3.5rem, cyan gradient) |
| `.metricLabel` | Stat label (uppercase) |
| `.metricSublabel` | Sublabel (secondary text) |

### Enterprise Section

| Class | Purpose |
|-------|---------|
| `.enterpriseGrid` | 2-col grid (1-col on tablet) |
| `.enterpriseContent` | Left column content |
| `.enterpriseDescription` | Lead paragraph |
| `.enterpriseFeatures` | Feature list container |
| `.enterpriseFeature` | Individual feature item |
| `.enterpriseVisual` | Right column (code window) |
| `.featureDot` | Accent dot (8px cyan) |
| `.featureTitle` | Feature heading |
| `.featureDescription` | Feature detail text |

### CTA Section

| Class | Purpose |
|-------|---------|
| `.ctaContainer` | Center alignment container |
| `.ctaHeading` | h2 title (clamp) |
| `.ctaText` | Description (max-width: 600px) |
| `.ctaButtons` | Button flex group |

## Responsive Design Strategy

### Breakpoints
- **1024px**: Tablet/Small Desktop
- **768px**: Mobile
- **480px**: Extra Small

### Column Adjustments

#### Features Grid
```
Mobile: 1 col → Tablet: 2 cols → Desktop: 4 cols
```

#### Metrics Grid
```
Mobile: 1 col → Tablet: 2 cols → Desktop: 4 cols
```

#### Modules Grid
```
All: 1 col (full width cards)
```

#### Enterprise Grid
```
Mobile/Tablet: 1 col (stacked) → Desktop: 2 cols (side-by-side)
```

### Common Responsive Adjustments

```css
@media (max-width: 768px) {
  .robotContainer {
    display: none;  /* Hide on mobile */
  }

  .heroCtaGroup {
    flex-direction: column;  /* Stack buttons */
    width: 100%;
  }

  .stackBadges {
    justify-content: flex-start;  /* Align left */
  }
}
```

## Animation & Transition Details

### Hero Animations
- **Robot Levitation**: 6s ease-in-out infinite (±25px y-axis)
- **Robot Eyes**: 3s pulse effect (blink animation)
- **Robot Arms**: 4s wave left/right alternating
- **Antenna Glow**: 2s brightness oscillation
- **Particles**: 6s drift with opacity change
- **Circuit Paths**: 4s dash animation (drawing effect)
- **Hexagons**: 8s float with rotation
- **Label Dot**: 2s pulse shadow glow
- **Scroll Indicator**: 2s bounce animation

### Transition Timings
- `--transition-base`: 200ms (default hover effects)
- `--transition-smooth`: 400ms (more deliberate transitions)

### Button States
```tsx
// PrimaryButton
:hover { transform: translateY(-2px) scale(1.02); }
:active { transform: translateY(0) scale(1); }

// Card Hover
:hover { transform: translateY(-6px) scale(1.02); }
```

## Color System

### Primary Accent
```css
--color-blue-500: #22d3ee  /* Cyan */
--color-blue-600: #22d3ee  /* Darker cyan (same in current setup) */
```

### Backgrounds
```css
#000000    /* Pure black (hero, enterprise) */
#0a0a0a    /* Almost black (default sections) */
```

### Text
```css
--color-slate-100: #f1f5f9  /* White text */
--color-slate-300: #cbd5e1  /* Light gray */
--color-slate-400: #94a3b8  /* Medium gray */
--color-slate-500: #64748b  /* Darker gray */
--color-slate-600: #475569  /* Even darker (not commonly used) */
```

### Accent Overlays
```css
rgba(59, 130, 246, 0.1)   /* Very subtle */
rgba(59, 130, 246, 0.2)   /* Subtle (borders) */
rgba(59, 130, 246, 0.3)   /* Medium (hovers) */
```

## Typography Scale

| Size | Usage | Example |
|------|-------|---------|
| clamp(2.5rem, 6vw, 4rem) | h1 (hero title) | "Physical AI & Humanoid Robotics" |
| clamp(2rem, 5vw, 3rem) | h2 (section) | "What You Will Master" |
| 1.75rem | h3 (module) | Module titles |
| 1.125rem | h4 (feature/metric) | Feature/metric labels |
| 1rem | body | Standard text |
| 0.875rem | small | Metadata, badges |
| 0.75rem | extra small | Labels, tags |

## Code Window Styling

The enterprise section includes a code window with syntax highlighting:

```tsx
<div className={styles.codeWindow}>
  <div className={styles.codeHeader}>
    {/* Window chrome */}
  </div>
  <div className={styles.codeBody}>
    <pre className={styles.codeContent}>
      {codeString}
    </pre>
  </div>
</div>
```

### Syntax Highlighting Classes
- `.token-comment` → `#6a9955` (green)
- `.token-keyword` → `#c586c0` (purple, bold)
- `.token-class` → `#4ec9b0` (teal)
- `.token-function` → `#dcdcaa` (yellow)
- `.token-string` → `#ce9178` (orange)
- `.token-number` → `#b5cea8` (light green)

## Adding New Sections

To add a new section following the established pattern:

```tsx
function NewSection() {
  const items = [/* data */];

  return (
    <Section variant="default">  {/* Choose variant */}
      <Container size="lg">
        <div className={styles.sectionHeader}>
          <div className={styles.sectionLabel}>Category</div>
          <Heading as="h2" className={styles.sectionHeading}>
            Section Title
          </Heading>
          <p className={styles.sectionText}>Description...</p>
        </div>

        <Grid cols={{mobile: 1, tablet: 2, desktop: 3}} gap="lg">
          {items.map((item) => (
            <YourCard key={item.id} {...item} />
          ))}
        </Grid>
      </Container>
    </Section>
  );
}
```

Then add to main export:

```tsx
export default function Home() {
  return (
    <Layout title="..." description="...">
      <HomepageHeader />
      <FeaturesSection />
      <MetricsSection />
      <ModulesSection />
      <EnterpriseSection />
      <NewSection />      {/* Add here */}
      <CtaSection />
    </Layout>
  );
}
```

## Browser Support

The design uses modern CSS features supported in:
- Chrome 90+
- Firefox 88+
- Safari 14+
- Edge 90+

### Fallbacks Provided
- Backdrop filter with fallback opacity
- CSS Grid with auto-fit fallback
- Gradient text with text-fill fallback
- CSS variables supported

## Performance Tips

1. **Images**: Lazy-load robot SVG if it becomes large
2. **Animations**: Hardware-accelerate with `transform` and `opacity`
3. **Grid**: Use `auto-fit`/`auto-fill` for flexibility
4. **Backdrop Blur**: Monitor performance on older devices
5. **Shadows**: CSS shadows are performant, avoid drop-shadow overuse

## Common Customization Tasks

### Change Accent Color
Replace all instances of `#22d3ee` (cyan) with desired color:

```css
--color-blue-500: #your-color;
--color-blue-600: #your-color-dark;
```

### Add New Feature
Add to `features` array in `FeaturesSection`:

```tsx
{
  title: 'Your Feature',
  description: 'Description...',
  icon: '🎯',
}
```

### Update Tech Stack
Modify array in hero section:

```tsx
{['ROS 2', 'Your Tech', ...].map((tech) => (
  <Badge key={tech}>{tech}</Badge>
))}
```

### Adjust Section Spacing
Modify `sectionHeader` margin in CSS:

```css
.sectionHeader {
  margin: 0 auto calc(var(--spacing-unit) * 24);  /* Increase spacing */
}
```

## Troubleshooting

### Button Not Responding
- Check if `href` is valid or `onClick` is defined
- Verify `disabled` prop is not set
- Ensure Button is not inside a form without proper type

### Cards Not Responsive
- Verify Grid `cols` prop has all three breakpoints
- Check Container `size` is appropriate
- Ensure parent Section has proper width handling

### Animation Not Playing
- Check `prefers-reduced-motion` media query
- Verify animation-duration is not 0
- Check z-index layering if animation hidden

### Colors Looking Different
- Verify dark theme is active (Docusaurus theme toggle)
- Check `--color-*` CSS variables are defined
- Look for media query `[data-theme='light']` overrides

## Testing Checklist

- [ ] Desktop view (1440px)
- [ ] Tablet view (768px)
- [ ] Mobile view (375px)
- [ ] Dark mode
- [ ] Light mode (if applicable)
- [ ] Button clicks navigate correctly
- [ ] Animations play smoothly
- [ ] No console errors
- [ ] Keyboard navigation works
- [ ] Screen reader tested

---

**Last Updated**: December 13, 2025
**Version**: 1.0
**Status**: Production Ready
