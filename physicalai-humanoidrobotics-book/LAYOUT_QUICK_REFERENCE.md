# Professional Layouts - Quick Reference Card

**For**: Developers implementing module and chapter pages
**Updated**: December 13, 2025
**Time to Learn**: 5 minutes

---

## Two Components

### 1. ModuleOverviewPage
**For**: Module landing pages showing chapter structure, outcomes, prerequisites

```mdx
import { ModuleOverviewPage } from '@site/src/components';

<ModuleOverviewPage
  moduleNumber={1}
  moduleTitle="The Robotic Nervous System (ROS 2)"
  description="Master ROS 2 middleware..."
  duration="Weeks 3-5 (3 weeks total)"
  difficulty="beginner"
  totalHours="15-20 hours"
  chapters={[...]}        // Array of Chapter objects
  outcomes={[...]}        // Array of LearningOutcome objects
  prerequisites={[...]}   // Array of Prerequisite objects
  techStack={[...]}       // Array of TechStackItem objects
  ctaText="Start Chapter 1"
  ctaLink="/docs/module1/chapter1"
/>
```

### 2. ChapterPage
**For**: Chapter reading pages with TOC, navigation, progress

```mdx
import { ChapterPage } from '@site/src/components';

<ChapterPage
  moduleNumber={1}
  moduleName="ROS 2 Fundamentals"
  chapterNumber={1}
  chapterTitle="ROS 2 Installation and Workspace Setup"
  description="Install ROS 2 and create workspaces..."
  bloomLevel="Understand"
  estimatedTime="1.5-2 hours"
  tableOfContents={[...]}  // Array of TOCItem objects
  progress={0}             // 0-100 module progress
  previousChapter={undefined}  // Or {title, link}
  nextChapter={{title: "...", link: "..."}}
>
  # Your chapter content goes here

  ## Section 1
  Content...

  ## Section 2
  Content...
</ChapterPage>
```

---

## Data Structures

### Chapter
```typescript
{
  id: "ch1",
  number: 1,
  title: "Chapter Title",
  description: "Short description of chapter content",
  bloomLevel: "Understand",  // or Apply, Analyze, Evaluate, Create
  duration: "1.5-2 hours",
  link: "/docs/module1/chapter1"
}
```

### LearningOutcome
```typescript
{
  level: "Remember & Understand",
  title: "Fundamental Concepts",
  items: [
    "First learning outcome",
    "Second learning outcome"
  ]
}
```

### Prerequisite
```typescript
{
  category: "Required Knowledge",
  icon: "📝",
  items: [
    "Python 3 fundamentals",
    "Linux CLI basics"
  ]
}
```

### TechStackItem
```typescript
{
  category: "Core Middleware",
  tools: ["ROS 2 Humble", "rclpy", "colcon"]
}
```

### TOCItem (for ChapterPage)
```typescript
{
  id: "section-id",
  title: "Section Title",
  level: 1,              // 1=h2, 2=h3, etc.
  children: [
    {
      id: "subsection-id",
      title: "Subsection Title",
      level: 2
    }
  ]
}
```

### NavChapter (for ChapterPage)
```typescript
{
  title: "Next Chapter Title",
  link: "/docs/module2/chapter3"
}
// or undefined for first/last chapter
```

---

## Styling & Customization

### Colors (Edit `/src/css/custom.css`)
```css
--color-primary: #00d9ff;           /* Cyan accent */
--color-accent-purple: #a855f7;     /* Purple accent */
--color-bg-primary: #000000;        /* Dark background */
--color-text-primary: #f1f5f9;      /* Light text */
```

### Typography
```css
--font-size-5xl: 3rem;              /* Module titles */
--font-size-4xl: 2.5rem;            /* Chapter titles */
--font-size-2xl: 1.5rem;            /* Section headings */
--font-size-base: 1rem;             /* Body text */
```

### Spacing
```css
--spacing-lg: 1.5rem;               /* Section padding */
--spacing-xl: 2rem;                 /* Major sections */
--spacing-2xl: 3rem;                /* Vertical rhythm */
```

---

## Common Patterns

### Full Module Setup
```mdx
---
id: module-overview
title: "Module N: Title"
sidebar_position: 1
---

import { ModuleOverviewPage } from '@site/src/components';

export const ModuleData = {
  moduleNumber: 1,
  moduleTitle: "...",
  description: "...",
  // ... all props
};

<ModuleOverviewPage {...ModuleData} />

---

# Additional Context

Optional additional markdown content...
```

### Full Chapter Setup
```mdx
---
id: chapter1-name
title: "Chapter 1: Name"
sidebar_position: 2
---

import { ChapterPage, type TOCItem } from '@site/src/components';

export const toc: TOCItem[] = [
  {id: "section-1", title: "First Section", level: 1}
];

<ChapterPage
  moduleNumber={1}
  moduleName="Module Name"
  chapterNumber={1}
  chapterTitle="Chapter Title"
  tableOfContents={toc}
  progress={10}
  nextChapter={{title: "Chapter 2", link: "..."}}
>

# Chapter content here

Use markdown as normal. Everything inside ChapterPage
gets the professional layout with sidebar, breadcrumbs, etc.

</ChapterPage>
```

---

## Layout Features

### ModuleOverviewPage Includes
- ✓ Gradient header with module title
- ✓ Chapter progression list
- ✓ Learning outcomes by Bloom's level
- ✓ Prerequisites by category
- ✓ Tech stack by category
- ✓ Sidebar: quick stats, prerequisites, tech, support
- ✓ Call-to-action buttons
- ✓ Responsive design
- ✓ Dark theme with cyan/purple accents

### ChapterPage Includes
- ✓ Breadcrumb navigation
- ✓ Chapter header with metadata
- ✓ Full article content area
- ✓ Sidebar: progress bar, TOC, resources, support
- ✓ Table of contents (auto-generate or provide)
- ✓ Progress indicator (0-100%)
- ✓ Previous/Next chapter navigation
- ✓ Print-friendly styles
- ✓ Responsive design

---

## Responsive Behavior

| Device | Columns | Behavior |
|--------|---------|----------|
| Mobile (375px) | 1 | Sidebar below content, single column |
| Tablet (768px) | 1-2 | TOC becomes accordion or separate |
| Desktop (1024px) | 2-3 | Sidebar sticky, full layout |
| Wide (1536px+) | 3-4 | Generous spacing, max-width applied |

---

## Accessibility

- ✓ Keyboard navigation (Tab, Enter, Space)
- ✓ Focus indicators (cyan 2px outline)
- ✓ Screen reader support (ARIA labels)
- ✓ Color contrast AAA compliant
- ✓ Motion reduction support
- ✓ Semantic HTML structure

**Test with**: Keyboard only, NVDA/VoiceOver, WebAIM contrast

---

## Do's & Don'ts

### Do's ✓
- Use Bloom's levels correctly (Understand, Apply, Analyze, Evaluate, Create)
- Provide realistic time estimates
- Link all chapters
- Use semantic heading hierarchy (h2, h3, h4 in content)
- Test on mobile devices
- Include learning outcomes
- Break content into sections
- Use code examples

### Don'ts ✗
- Don't skip chapter links
- Don't make time estimates unrealistic
- Don't use inconsistent Bloom's levels
- Don't nest headings more than 3 levels
- Don't assume desktop-only users
- Don't omit prerequisites
- Don't write 1000-word sections
- Don't forget accessibility

---

## File Paths

### Component Files
- `/src/components/ModuleOverviewPage.tsx` - Component code
- `/src/components/ModuleOverviewPage.module.css` - Styling
- `/src/components/ChapterPage.tsx` - Component code
- `/src/components/ChapterPage.module.css` - Styling

### Examples
- `/docs/module1/module-overview-new.mdx` - Module 1 example
- `/docs/module2/module-overview-new.mdx` - Module 2 example
- `/src/pages/ChapterLayoutExample.tsx` - Chapter example

### Documentation
- `LAYOUT_IMPLEMENTATION_GUIDE.md` - Full guide
- `PROFESSIONAL_LAYOUTS_SUMMARY.md` - Feature summary
- `IMPLEMENTATION_CHECKLIST.md` - Verification checklist
- `LAYOUT_QUICK_REFERENCE.md` - This file

---

## Troubleshooting

**Component not showing?**
- Check import is correct: `import { ModuleOverviewPage } from '@site/src/components';`
- Run TypeScript check: `npm run typecheck`
- Clear cache: `rm -rf build .docusaurus`

**Styling looks wrong?**
- Verify CSS is loaded: Open DevTools > Elements
- Check CSS variables: `src/css/custom.css` loaded?
- Try rebuild: `npm run build`

**Mobile layout broken?**
- Test with DevTools device emulation
- Check viewport meta tag exists
- Verify breakpoint: `@media (max-width: 1200px)`

**Accessibility issues?**
- Test keyboard navigation (Tab through page)
- Check with screen reader (NVDA/VoiceOver)
- Verify heading hierarchy (h2 before h3)
- Use WebAIM contrast checker for colors

---

## Performance Tips

1. **Use Container for width** instead of custom max-width
2. **Use Grid for layouts** instead of custom flexbox
3. **Leverage design tokens** instead of hardcoded values
4. **Keep sections under 500 words** each
5. **Optimize images** before embedding
6. **Use code syntax highlighting** for readability
7. **Test with Lighthouse** before deploying

---

## Links & Resources

- **Component API**: See `COMPONENT_LIBRARY.md`
- **Design Tokens**: See `src/css/custom.css`
- **Examples**: See example MDX files in `/docs/`
- **Docusaurus**: https://docusaurus.io/
- **Accessibility**: https://www.w3.org/WAI/WCAG21/quickref/

---

**5-minute reference for quick implementation. See full docs for details.**

Last updated: December 13, 2025
