# Implementation Plan: Textbook Preface - Welcome to the AI-Native Era

**Branch**: `001-textbook-preface` | **Date**: 2025-12-10 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-textbook-preface/spec.md`

## Summary

Create the opening preface for the Physical AI & Humanoid Robotics textbook that orients students, motivates instructors, and guides self-paced learners through the 13-week journey. The preface establishes the revolutionary nature of Physical AI, explains the three technological convergences (Foundation Models, Simulation, Edge Computing), and outlines the four-phase course structure.

**Technical Approach**: Content delivered as Docusaurus MDX file with custom components for learning outcomes, readability targets (Flesch-Kincaid 13-15), and bilingual support (English primary, Urdu translation tracked separately).

## Technical Context

**Language/Version**: MDX (Markdown + JSX), English (primary), Urdu (i18n)
**Primary Dependencies**: Docusaurus 3.x, custom React components (`<LearningOutcome>`, `<TierSelector>`), readability analysis tools (textstat)
**Storage**: Git repository (English MDX in `docs/preface.mdx`, Urdu in `i18n/ur/docusaurus-plugin-content-docs/current/preface.mdx`)
**Testing**: Readability analysis (Flesch-Kincaid), comprehension quiz (post-reading assessment), instructor feedback survey, link validation
**Target Platform**: Web (Docusaurus static site), deployed to GitHub Pages/Vercel
**Project Type**: Content/Documentation (not software development)
**Performance Goals**: <20 minute reading time for 90% of students, <1.5s First Contentful Paint
**Constraints**: Flesch-Kincaid grade level 13-15 (university undergraduate), zero technical jargon confusion
**Scale/Scope**: Single 4000-5000 word preface, 12 functional requirements, 10 success criteria, 3 user stories

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Applicable Principles

**Principle III: Bloom's Taxonomy Learning Outcomes**
- ✅ **Status**: COMPLIANT
- **Evidence**: Spec defines learning outcomes implicitly through comprehension quiz (analyze), instructor lecture delivery (apply), self-paced decision-making (evaluate)
- **Action**: Preface must support Bloom's progression by enabling students to analyze Physical AI concepts, apply them to course expectations, and evaluate textbook fit

**Principle VII: Bilingual Support (English/Urdu)**
- ✅ **Status**: COMPLIANT
- **Evidence**: Spec Assumptions document "The preface will be translated to Urdu following the same structure (per Principle VII)"
- **Action**: English MDX created first, Urdu translation via TranslationSync agent using glossary.yml

**Principle X: Code Quality Standards**
- ✅ **Status**: COMPLIANT (adapted for content)
- **Evidence**: Success criteria include readability targets, link validation, zero jargon confusion
- **Action**: Apply content quality gates (grammar check via LanguageTool, readability via textstat, link validation via markdown-link-check)

### Non-Applicable Principles (Content vs Code)

**Principle I: Three-Tier Hardware Accessibility**
- ⚠️ **Status**: INFORMATIONAL ONLY
- **Rationale**: Preface *describes* the three-tier structure but does not implement code for different tiers
- **Action**: Ensure preface clearly explains Tier A (simulation, $0), Tier B (Edge AI, $700), Tier C (physical robot)

**Principle II: Safety-First Physical AI**
- ⚠️ **Status**: INFORMATIONAL ONLY
- **Rationale**: Preface *explains* safety philosophy (FR-007) but does not contain motor control code requiring E-Stop
- **Action**: "The Weight of Responsibility" section must present concrete failure mode examples

**Principle IV: Visual Intelligence (Mermaid-First)**
- ⚠️ **Status**: NOT APPLICABLE
- **Rationale**: Preface is prose-based; no system architectures to diagram
- **Action**: None - Mermaid diagrams introduced in Module 1 chapters

**Principle V: RAG Chatbot Integration**
- ✅ **Status**: COMPLIANT (as indexable content)
- **Evidence**: Preface will be chunked and indexed by RAGIndexer agent for chatbot Q&A
- **Action**: Write content in clear, self-contained paragraphs suitable for semantic chunking (1000 tokens, 200 overlap)

**Principle VI: Personalization Engine**
- ⚠️ **Status**: INFORMATIONAL ONLY
- **Rationale**: Preface describes personalization (user profiles) but single static content version for all readers
- **Action**: Future enhancement could adapt preface based on user experience level (beginner scaffolding, expert condensed)

**Principle VIII: Reusable Intelligence Architecture**
- ✅ **STATUS**: COMPLIANT
- **Evidence**: This planning process creates ADRs for content decisions, PHRs for agent interactions
- **Action**: Document content structure decisions (e.g., "Why historical framing?", "Why Three Laws metaphor?") if architecturally significant

**Principle IX: Module-Aligned Content Structure**
- ✅ **Status**: COMPLIANT
- **Evidence**: FR-010 requires explaining four-phase structure (Nervous System, Digital Twin, AI Brain, Voice of Action) aligned with 13-week schedule
- **Action**: Preface must accurately reference module breakdown from constitution

### Gate Evaluation

**PASSED** - No blocking violations. Content-focused feature uses adapted quality standards (readability, comprehension, translation) instead of code quality gates.

## Project Structure

### Documentation (this feature)

```text
specs/001-textbook-preface/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification (already created)
├── research.md          # Phase 0 output (content research)
├── data-model.md        # Phase 1 output (content structure model)
├── quickstart.md        # Phase 1 output (writing guidelines)
├── contracts/           # Phase 1 output (content contracts)
│   └── preface-structure.yml  # Section headers, word counts, learning outcome mapping
└── checklists/
    └── requirements.md  # Quality validation (already created)
```

### Source Code (Docusaurus repository)

```text
# Content Structure (Docusaurus docs directory)
docs/
└── preface.mdx          # English preface (primary)

i18n/ur/docusaurus-plugin-content-docs/current/
└── preface.mdx          # Urdu translation (via TranslationSync agent)

# Custom Components (if new components needed)
src/components/
└── LearningOutcome.tsx  # Already specified in constitution

# Glossary (for Urdu translation)
i18n/
└── glossary.yml         # English-Urdu term mappings (e.g., "Physical AI" → "فزیکل اے آئی")
```

**Structure Decision**: Single-file content feature within larger Docusaurus documentation site. Preface is standalone MDX file, not a multi-file module. No custom components required beyond those already specified in constitution (LearningOutcome, TierSelector). Focus is on prose quality, not code structure.

## Complexity Tracking

> **No violations** - Constitution Check passed. This section intentionally left blank.

---

## Phase 0: Research - Content Framework

**Goal**: Resolve all content structure unknowns and establish writing guidelines.

### Research Tasks

#### R1: Pedagogical Best Practices for Textbook Prefaces
**Question**: What makes an effective textbook preface for university-level technical courses?

**Research Approach**:
- Review prefaces from high-quality technical textbooks (e.g., "Introduction to Algorithms" by CLRS, "Deep Learning" by Goodfellow et al.)
- Identify common structural patterns (motivation, prerequisites, how to use, acknowledgments)
- Analyze average word count and reading time

**Decision**: Structure preface with clear sections that map to user stories:
- **Orientation**: "The Moment Everything Changes", "From Mind to Body" (US1: Student comprehension)
- **Motivation**: "Why Humanoid? Why Now?", "Your Place in History" (US1: Excitement)
- **Framework**: "Three Laws of Physical AI", "What You Will Build" (US1 & US3: Clear expectations)
- **Guidance**: "How to Use This Course", "The Tools of Creation" (US2 & US3: Practical info)
- **Ethics**: "The Weight of Responsibility" (US1 & US2: Safety culture)

**Rationale**: Follows proven pedagogical patterns while addressing all three user stories (students, instructors, self-paced learners).

**Alternatives Considered**:
- Academic style (abstract, literature review) - Rejected: Too dry, does not motivate
- Narrative-only (storytelling, no structure) - Rejected: Harder to extract practical info for US3

#### R2: Readability Targets for University Undergraduates
**Question**: What Flesch-Kincaid grade level is appropriate for university technical content?

**Research Approach**:
- Constitution specifies "Flesch-Kincaid grade level 13-15"
- Verify this aligns with undergraduate reading level (13 = college freshman, 15 = college junior)
- Check if technical content typically scores higher due to specialized vocabulary

**Decision**: Target Flesch-Kincaid grade level 13-15, Flesch Reading Ease 30-50 (Difficult/College level)

**Rationale**: Aligns with constitution assumption. Technical content naturally scores in this range due to domain terms (kinematics, embodied AI, simulation). Grade 15 is acceptable for upper-level undergraduates and graduate students.

**Alternatives Considered**:
- Lower target (grade 10-12) - Rejected: Would require oversimplification of technical concepts
- No readability target - Rejected: Success criterion SC-009 requires zero jargon confusion, implying controlled complexity

#### R3: Historical Framing Examples for Physical AI
**Question**: Which historical examples (HAL 9000, Terminator, etc.) resonate with modern students?

**Research Approach**:
- Verify relevance of 1968 (HAL 9000), 1984 (Terminator), 2001 (A.I. Artificial Intelligence)
- Check if modern students recognize these references (potential generational gap)
- Consider adding recent examples (Tesla Optimus 2021, Figure 01 2022, Boston Dynamics Atlas updates)

**Decision**: Use both classic sci-fi (HAL, Terminator, David) AND modern industry examples (Tesla Optimus, Boston Dynamics Atlas, Figure AI)

**Rationale**: Classic examples provide historical depth and cultural touchstones; modern examples prove "it's happening now" and link to verifiable reality (FR-009 requirement).

**Alternatives Considered**:
- Only classic sci-fi - Rejected: May feel outdated, skepticism about "56 years" timeline (Edge Case)
- Only modern examples - Rejected: Loses historical context, less inspiring narrative arc

#### R4: Urdu Translation Workflow for Preface
**Question**: How should the Urdu translation be managed given the preface's motivational/narrative style?

**Research Approach**:
- Review Principle VII: Bilingual Support requirements (RTL layout, glossary, human review)
- Identify technical terms needing glossary entries (Physical AI, Foundation Models, VLA, etc.)
- Determine if GPT-4 translation quality is sufficient for motivational prose vs technical definitions

**Decision**: Two-phase translation approach:
1. **Phase 1**: GPT-4 translation of prose with glossary injection (TranslationSync agent)
2. **Phase 2**: Human review by Urdu-speaking educator for cultural/motivational tone

**Rationale**: Motivational prose requires cultural adaptation beyond literal translation. Technical terms use glossary (consistent), but metaphors/quotes may need localization.

**Alternatives Considered**:
- Fully automated GPT-4 translation - Rejected: Risk of losing motivational impact, cultural mismatch
- Fully manual translation - Rejected: Resource-intensive, delays iteration
- Skip Urdu initially - Rejected: Violates Principle VII and constitution commitment

### Research Findings Summary

All unknowns resolved. No NEEDS CLARIFICATION markers remain in Technical Context.

**Key Decisions**:
1. Preface structure: 7 major sections (orientation, motivation, framework, guidance, ethics, path ahead, history)
2. Readability target: FK 13-15, FRE 30-50 (validated by textstat)
3. Historical framing: Hybrid approach (classic sci-fi + modern industry)
4. Urdu translation: Two-phase (GPT-4 + human review) with glossary support

---

## Phase 1: Design - Content Architecture

**Goal**: Define content structure, writing guidelines, and quality contracts.

### 1. Content Structure Model (`data-model.md`)

Since this is a content feature, the "data model" represents the **information architecture** of the preface, not database schemas.

#### Preface Content Entities

**Sect**ion**
- **Attributes**: title (string), purpose (string), word_count_target (int), bloom_level (string), learning_outcomes (array)
- **Relationships**: Has many paragraphs, supports one or more user stories
- **Validation Rules**: Total word count 4000-5000, no section exceeds 1200 words

**Learning Outcome**
- **Attributes**: objective (string), bloom_verb (enum: analyze, apply, create, evaluate), success_criterion_id (string)
- **Relationships**: Belongs to section, maps to success criteria (SC-001 through SC-010)
- **Validation Rules**: Must use Bloom's taxonomy verbs, testable via comprehension quiz or survey

**Technical Term**
- **Attributes**: english_term (string), urdu_term (string), definition (string), context (string)
- **Relationships**: Appears in one or more sections, stored in glossary.yml
- **Validation Rules**: All technical terms must have Urdu translation before final publication

**Historical Reference**
- **Attributes**: name (string), year (int), medium (enum: film, literature, industry), relevance (string)
- **Relationships**: Used in section "Your Place in History" or "A Final Word"
- **Validation Rules**: Year must be verifiable, industry examples must be current (2020+)

#### Content State Transitions

```
[Draft] → [Readability Check] → [Comprehension Test] → [Translation] → [Published]
   ↓             ↓                      ↓                    ↓              ↓
FK 13-15?    SC-001 90%?          glossary.yml      Vercel/GH Pages
   |             |                  complete?              |
   No → Rewrite  No → Revise       No → Block          Yes → Live
```

### 2. Content Contracts (`contracts/preface-structure.yml`)

```yaml
# Content Contract: Textbook Preface Structure
version: 1.0.0
last_updated: 2025-12-10

sections:
  - id: sec-01
    title: "The Moment Everything Changes"
    purpose: "Hook reader with revolutionary framing"
    word_count_target: 150-250
    bloom_level: "attention (pre-cognitive)"
    learning_outcomes: []
    success_criteria: [SC-003]  # Motivation score
    user_stories: [US1]
    required_content:
      - "You are standing at the threshold" (inclusive language, FR-012)
      - Contrast: AI as "ghost in the machine" (disembodied) vs Physical AI
      - Emotional hook (e.g., "never shake hands", "never feel warmth")

  - id: sec-02
    title: "From Mind to Body: The Great Awakening"
    purpose: "Explain what Physical AI is and why now"
    word_count_target: 400-600
    bloom_level: "understand, analyze"
    learning_outcomes:
      - objective: "Distinguish between traditional AI and Physical AI"
        bloom_verb: "analyze"
        success_criterion: "SC-001"
    success_criteria: [SC-001, SC-002]
    user_stories: [US1, US3]
    required_content:
      - Definition of Physical AI (FR-001: minds + bodies)
      - Three technological tsunamis (FR-002):
        * Foundation Models (language, vision, reasoning)
        * Simulation Environments (Isaac Sim, Gazebo)
        * Edge Computing (Jetson, neural networks in palm)
      - "Embodied AI" term introduction

  - id: sec-03
    title: "Why Humanoid? Why Now?"
    purpose: "Answer why humanoids are significant form factor"
    word_count_target: 300-500
    bloom_level: "understand"
    learning_outcomes:
      - objective: "Explain infrastructure compatibility argument for humanoids"
        bloom_verb: "explain"
        success_criterion: "SC-001"
    success_criteria: [SC-001]
    user_stories: [US1]
    required_content:
      - Infrastructure compatibility (FR-003: doors, stairs, tools designed for human form)
      - Learning from human video data (YouTube, TikTok → training dataset)
      - Specific examples: "wheeled robot cannot climb stairs"

  - id: sec-04
    title: "The Three Laws of Physical AI"
    purpose: "Present foundational principles governing course"
    word_count_target: 600-800
    bloom_level: "understand, analyze"
    learning_outcomes:
      - objective: "Recall and explain the Three Laws of Physical AI"
        bloom_verb: "explain"
        success_criterion: "SC-001"
    success_criteria: [SC-001, SC-004]
    user_stories: [US1, US2]
    required_content:
      - Law 1: The Body Shapes the Mind (FR-004)
      - Law 2: Simulation is the New Training Ground (FR-004)
      - Law 3: Language is the Universal Interface (FR-004)
      - Concrete examples for each law (kinematics, digital twins, VLA models)

  - id: sec-05
    title: "What You Will Build"
    purpose: "Outline course structure and capstone project"
    word_count_target: 500-700
    bloom_level: "understand"
    learning_outcomes:
      - objective: "Articulate the four-phase course structure"
        bloom_verb: "articulate"
        success_criterion: "SC-004"
      - objective: "Identify at least 3 technologies used in the course"
        bloom_verb: "identify"
        success_criterion: "SC-005"
    success_criteria: [SC-004, SC-005]
    user_stories: [US1, US2, US3]
    required_content:
      - Four components (FR-005):
        * The Spinal Cord (ROS 2)
        * The Imagination (Gazebo & Isaac Sim)
        * The Cerebellum (Navigation & Control)
        * The Cortex (Vision-Language-Action)
      - Capstone description: "simulated humanoid that listens, understands, plans, avoids obstacles"
      - Technologies list (FR-006): ROS 2 Humble/Jazzy, Isaac Sim, Gazebo, Whisper, VLA models

  - id: sec-06
    title: "The Tools of Creation"
    purpose: "List technologies and industry context"
    word_count_target: 200-400
    bloom_level: "remember"
    learning_outcomes:
      - objective: "Identify specific technologies by name"
        bloom_verb: "identify"
        success_criterion: "SC-005"
    success_criteria: [SC-005, SC-007]
    user_stories: [US3]
    required_content:
      - Technology list (FR-006): ROS 2 Humble/Iron, Isaac Sim, Gazebo, Isaac ROS, Whisper, VLA models
      - Industry validation: "same tools used by Tesla for Optimus, Boston Dynamics for Atlas, Figure AI"
      - Self-assessment cue: "You are learning the production stack"

  - id: sec-07
    title: "The Weight of Responsibility"
    purpose: "Establish safety and ethics culture"
    word_count_target: 300-500
    bloom_level: "understand, evaluate"
    learning_outcomes:
      - objective: "Understand ethical weight of building Physical AI systems"
        bloom_verb: "understand"
        success_criterion: "SC-001"
    success_criteria: [SC-001]
    user_stories: [US1, US2]
    required_content:
      - Safety importance (FR-007): "bugs can cause injury—or worse"
      - Concrete failure mode examples (FR-007): "A bug in a web app causes frustration. A bug in embodied AI can cause injury."
      - Ethics framing: "not just engineering, ethics encoded in motors and sensors"

  - id: sec-08
    title: "The Path Ahead"
    purpose: "Acknowledge difficulty while providing motivation"
    word_count_target: 200-400
    bloom_level: "evaluate (self-assessment)"
    learning_outcomes:
      - objective: "Realistically assess course difficulty and rewards"
        bloom_verb: "evaluate"
        success_criterion: "SC-003"
    success_criteria: [SC-003, SC-008]
    user_stories: [US1]
    required_content:
      - Difficulty acknowledgment (FR-008): "wrestle with coordinate frames", "debug launch files at 2 AM"
      - Motivation (FR-008): "moments of pure wonder" — first voice response, first obstacle avoidance, first gentle grasp
      - Reading time cue for SC-008 validation

  - id: sec-09
    title: "Your Place in History"
    purpose: "Position students as revolution builders"
    word_count_target: 200-400
    bloom_level: "evaluate, create (vision)"
    learning_outcomes:
      - objective: "Connect personal learning to historical significance"
        bloom_verb: "evaluate"
        success_criterion: "SC-003"
    success_criteria: [SC-003, SC-006]
    user_stories: [US1, US2]
    required_content:
      - Positioning as builders (FR-011): "You are the students who will build the Physical AI revolution"
      - Career relevance: "code you write could run on machines that outlive you"
      - Historical parallel: CS students 1975 (digital revolution), ML students 2015 (AI revolution), YOU 2025 (Physical AI)

  - id: sec-10
    title: "A Final Word Before We Begin"
    purpose: "Historical framing and call to action"
    word_count_target: 300-500
    bloom_level: "understand, apply"
    learning_outcomes:
      - objective: "Contextualize Physical AI within sci-fi to reality timeline"
        bloom_verb: "understand"
        success_criterion: "SC-001"
    success_criteria: [SC-001, SC-003]
    user_stories: [US1, US2]
    required_content:
      - Historical framing (FR-009):
        * HAL 9000 (1968, Arthur C. Clarke)
        * Terminator (1984, James Cameron)
        * David (2001, Steven Spielberg - A.I.)
      - Modern validation: "56 years, but we are finally building the reality"
      - Inclusive call to action (FR-012): "Take a breath. Look around at your peers."

  - id: sec-11
    title: "How to Use This Course"
    purpose: "Practical guidance for navigating textbook"
    word_count_target: 300-500
    bloom_level: "understand, apply"
    learning_outcomes:
      - objective: "Navigate course structure and dependencies"
        bloom_verb: "apply"
        success_criterion: "SC-004"
    success_criteria: [SC-004, SC-007]
    user_stories: [US2, US3]
    required_content:
      - Four-phase structure (FR-010):
        * Nervous System: ROS 2 fundamentals
        * Digital Twin: Gazebo, Unity, Isaac Sim
        * AI Brain: Perception, navigation, learning on Isaac
        * Voice of Action: Language models for human-robot interaction
      - Sequential dependencies (FR-010): "Each module builds on the last. Skip nothing."
      - Warning: "The capstone will demand every skill you have developed"

quality_gates:
  readability:
    tool: "textstat (Python)"
    metrics:
      - name: "Flesch-Kincaid Grade Level"
        target: "13-15"
        threshold: "17 max (graduate level acceptable)"
      - name: "Flesch Reading Ease"
        target: "30-50"
        threshold: "25 min (not incomprehensible)"
    validation: "Run after each draft iteration"

  comprehension:
    tool: "Post-reading quiz (Google Forms or Docusaurus Quiz plugin)"
    metrics:
      - name: "Physical AI definition accuracy"
        target: "90% correct (SC-001)"
      - name: "Three convergences recall"
        target: "80% recall (SC-002)"
      - name: "Four-phase structure recall"
        target: "95% recall (SC-004)"
    validation: "Pilot test with n≥20 students before publication"

  motivation:
    tool: "Anonymous survey (Likert scale 1-10)"
    metrics:
      - name: "Willingness to continue after preface"
        target: "7+ average (SC-003)"
      - name: "Instructor effectiveness rating (for US2)"
        target: "4+/5 (SC-006)"
    validation: "Post-preface survey, minimum n=10 students, n=3 instructors"

  translation:
    tool: "TranslationSync agent + Human reviewer"
    metrics:
      - name: "Glossary coverage"
        target: "100% of technical terms have Urdu equivalents"
      - name: "Cultural appropriateness"
        target: "Human reviewer approval (Urdu-speaking educator)"
    validation: "Before publishing Urdu version to i18n/ur/"

  links:
    tool: "markdown-link-check (npm package)"
    metrics:
      - name: "Broken links"
        target: "0 (SC-009 zero jargon confusion implies working references)"
    validation: "CI pipeline on every commit"

total_word_count_target: "4000-5000 words"
reading_time_target: "<20 minutes for 90% of students (SC-008)"
```

### 3. Writing Guidelines (`quickstart.md`)

*This file provides practical guidance for content authors implementing the preface.*

**File**: `specs/001-textbook-preface/quickstart.md`

```markdown
# Quickstart: Writing the Textbook Preface

## Overview

This guide provides step-by-step instructions for writing the Physical AI textbook preface following the approved specification and content contract.

## Prerequisites

- Familiarity with Docusaurus MDX syntax
- Access to readability analysis tools (textstat Python package)
- Understanding of Bloom's Taxonomy learning outcomes
- Spec and plan documents reviewed

## Writing Process

### Step 1: Structure the MDX File

Create `docs/preface.mdx` with this frontmatter:

\`\`\`mdx
---
id: preface
title: "Preface: Welcome to the AI-Native Era"
sidebar_label: "Preface"
sidebar_position: 1
description: "Introduction to Physical AI, the convergence of foundation models, simulation, and edge computing"
keywords: [physical ai, humanoid robotics, ros2, isaac sim, embodied ai]
---

# Preface: Welcome to the AI-Native Era

[Content sections follow...]
\`\`\`

### Step 2: Write Sections in Order

Follow the structure from `contracts/preface-structure.yml`:

1. **sec-01**: The Moment Everything Changes (150-250 words)
2. **sec-02**: From Mind to Body (400-600 words)
3. **sec-03**: Why Humanoid? Why Now? (300-500 words)
4. **sec-04**: The Three Laws of Physical AI (600-800 words)
5. **sec-05**: What You Will Build (500-700 words)
6. **sec-06**: The Tools of Creation (200-400 words)
7. **sec-07**: The Weight of Responsibility (300-500 words)
8. **sec-08**: The Path Ahead (200-400 words)
9. **sec-09**: Your Place in History (200-400 words)
10. **sec-10**: A Final Word Before We Begin (300-500 words)
11. **sec-11**: How to Use This Course (300-500 words)

### Step 3: Validate Readability

After drafting, run readability analysis:

\`\`\`python
import textstat

with open('docs/preface.mdx', 'r') as f:
    text = f.read()

fk_grade = textstat.flesch_kincaid_grade(text)
fre_score = textstat.flesch_reading_ease(text)

print(f"Flesch-Kincaid Grade: {fk_grade} (target: 13-15)")
print(f"Flesch Reading Ease: {fre_score} (target: 30-50)")
\`\`\`

**Iterate** if scores are outside target ranges.

### Step 4: Check Functional Requirements

Go through FR-001 to FR-012 checklist:

- [ ] FR-001: Physical AI defined by contrasting with disembodied AI
- [ ] FR-002: Three technological tsunamis articulated
- [ ] FR-003: Humanoid rationale explained (infrastructure compatibility, video learning)
- [ ] FR-004: Three Laws presented
- [ ] FR-005: Four components outlined (ROS 2, Digital Twins, Navigation, VLA)
- [ ] FR-006: Technologies listed (ROS 2 Humble/Jazzy, Isaac Sim, Gazebo, Whisper, VLAs)
- [ ] FR-007: Safety/ethics addressed with failure mode examples
- [ ] FR-008: Difficulty acknowledged, motivation provided
- [ ] FR-009: Historical framing (HAL, Terminator, David + Tesla, Boston Dynamics)
- [ ] FR-010: Four-phase structure explained with dependencies
- [ ] FR-011: Students positioned as revolution builders
- [ ] FR-012: Inclusive language used ("you", "your peers")

### Step 5: Prepare for Urdu Translation

Extract technical terms and add to `i18n/glossary.yml`:

\`\`\`yaml
terms:
  - en: "Physical AI"
    ur: "فزیکل اے آئی"
    context: "AI with physical embodiment"

  - en: "Foundation Models"
    ur: "بنیادی ماڈلز"
    context: "Large pre-trained AI models"

  - en: "Embodied AI"
    ur: "مجسم اے آئی"
    context: "AI in physical form"

  - en: "Digital Twin"
    ur: "ڈیجیٹل جڑواں"
    context: "Simulated replica"

  - en: "Vision-Language-Action (VLA)"
    ur: "بصارت-زبان-عمل"
    context: "Multimodal robot control"
\`\`\`

### Step 6: Validate Links

Run link checker:

\`\`\`bash
markdown-link-check docs/preface.mdx
\`\`\`

Fix any broken links before commit.

### Step 7: Create Comprehension Quiz

Draft 5-question quiz for SC-001 validation:

1. What is the primary difference between traditional AI and Physical AI?
   - A) Processing speed
   - B) Physical embodiment ✓
   - C) Programming language
   - D) Cost

2. Which three technological convergences enable Physical AI? (Select 3)
   - A) Foundation Models ✓
   - B) Blockchain
   - C) Simulation Environments ✓
   - D) Edge Computing ✓
   - E) Quantum Computing

3. Why are humanoid robots significant for inheriting our world?
   - A) They look like humans
   - B) Infrastructure compatibility ✓
   - C) They are cheaper
   - D) They are faster

4. What are the four phases of the course? (Match order)
   - Nervous System (ROS 2) → Digital Twin → AI Brain → Voice of Action ✓

5. Which Law of Physical AI states "Simulation is the New Training Ground"?
   - A) Law 1
   - B) Law 2 ✓
   - C) Law 3
   - D) Law 4

### Step 8: Pilot Test (Before Publication)

- Share with 3-5 students (mix of beginner/intermediate/expert)
- Administer comprehension quiz
- Survey motivation score (1-10)
- Collect feedback on jargon confusion (SC-009)

## Writing Style Guidelines

### Voice and Tone

- **Inspirational but realistic**: Balance "moments of pure wonder" with "debug launch files at 2 AM"
- **Inclusive second-person**: Use "you", "your", not "one" or "students"
- **Active voice**: "You will build" not "will be built by students"
- **Concrete over abstract**: "A wheeled robot cannot climb stairs" not "mobility limitations exist"

### Terminology

- Introduce technical terms with context on first use
- Use metaphors sparingly (Three Laws, Spinal Cord/Cortex) - explain them
- Link to glossary for Urdu translation support
- Avoid acronyms without expansion (e.g., "Vision-Language-Action (VLA)" not just "VLA")

### Paragraph Structure

- **Topic sentence first**: Clear main idea
- **Supporting details**: Examples, evidence, quotes
- **Transition to next**: Logical flow between paragraphs
- **Length**: 3-5 sentences average (not too dense for readability)

### Sentence Variety

- **Mix sentence lengths**: Short punchy sentences for emphasis, longer complex sentences for nuance
- **Avoid passive voice**: "The robot learns" not "learning is performed by the robot"
- **Parallel structure for lists**: "You will master X, create Y, and implement Z"

## Success Criteria Checklist

Before declaring preface complete, validate against SC-001 through SC-010:

- [ ] **SC-001**: 90%+ students pass comprehension quiz (pilot test n≥20)
- [ ] **SC-002**: 80%+ recall three convergences (quiz question 2)
- [ ] **SC-003**: 7+/10 motivation score (pilot survey)
- [ ] **SC-004**: 95%+ articulate four-phase structure (quiz question 4)
- [ ] **SC-005**: 3/5 technologies identified (quiz + survey)
- [ ] **SC-006**: 4+/5 instructor effectiveness (instructor pilot test n≥3)
- [ ] **SC-007**: 70%+ self-paced learners assess prerequisites correctly (survey)
- [ ] **SC-008**: <20 min reading time for 90% (measure via analytics or self-report)
- [ ] **SC-009**: Zero jargon confusion (pilot feedback)
- [ ] **SC-010**: 85%+ retention Week 1→2 (measure post-publication)

## Troubleshooting

### Readability Too High (FK > 17)

- Break long sentences (20+ words) into shorter ones
- Replace complex words with simpler synonyms where possible without losing meaning
- Use more active voice
- Add transition words for flow ("However", "Therefore", "For example")

### Readability Too Low (FK < 13)

- This is acceptable if content is clear
- Consider adding technical depth where appropriate
- Ensure jargon is explained, not removed

### Motivation Score Low (< 7/10)

- Strengthen "moments of pure wonder" examples
- Add more industry validation (Tesla, Boston Dynamics)
- Emphasize career relevance ("run on machines that outlive you")

### Comprehension Quiz Fails (< 90%)

- Revise unclear explanations
- Add concrete examples where abstract
- Check if question tests content actually in preface

## Next Steps

After preface passes all quality gates:

1. **Commit to Git**: `git add docs/preface.mdx && git commit -m "Add textbook preface (FR-001 through FR-012)"`
2. **Trigger Translation**: Use TranslationSync agent with glossary.yml
3. **Deploy to Staging**: Vercel preview for final review
4. **Launch Pilot Test**: Recruit students/instructors for validation
5. **Iterate**: Incorporate feedback, re-validate
6. **Publish**: Merge to main, deploy to GitHub Pages
\`\`\`

---

## Phase 1 Complete

**Artifacts Created**:
- ✅ `data-model.md`: Content structure with entities (Section, Learning Outcome, Technical Term, Historical Reference) and state transitions
- ✅ `contracts/preface-structure.yml`: 11 sections with word counts, Bloom's levels, success criteria mapping, required content, quality gates
- ✅ `quickstart.md`: Step-by-step writing guidelines, validation procedures, troubleshooting

**Next Command**: `/sp.tasks` (not executed by /sp.plan - separate command)

**Ready for Implementation**: Yes - content authors have complete blueprint to write preface following specification.
