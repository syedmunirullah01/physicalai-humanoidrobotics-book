---
id: 002
title: Generate Textbook Preface Tasks
stage: tasks
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: textbook-preface
branch: 001-textbook-preface
user: system
command: /sp.tasks
labels: ["tasks", "textbook", "preface", "task-breakdown", "user-stories"]
links:
  spec: specs/001-textbook-preface/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-textbook-preface/tasks.md
tests:
  - null
---

## Prompt

```text
/sp.tasks
```

Context: Continue from `/sp.plan` command completion. The implementation plan is ready at `specs/001-textbook-preface/plan.md` with:
- 11-section content structure (sec-01 through sec-11)
- Contracts defining word counts, Bloom's levels, success criteria mapping
- Writing guidelines with readability targets (FK 13-15)
- Quality gates (readability, comprehension, motivation, translation, links)

User stories from spec.md:
- US1 (P1): Student Orientation & Motivation - comprehension quiz, motivation score
- US2 (P2): Instructor Course Framing - 20-minute lecture delivery
- US3 (P3): Self-Paced Learner Onboarding - hardware/software self-assessment

## Response snapshot

Generated comprehensive task breakdown at `specs/001-textbook-preface/tasks.md` with 49 total tasks organized by user story:

**Phase 1: Setup (4 tasks)**
- Content infrastructure: Docusaurus directory, readability tools, link checker, Urdu glossary

**Phase 2: Foundational (4 tasks)**
- Content framework: MDX file structure, 11 section headings, validation scripts, quiz template
- BLOCKS all user story work until complete

**Phase 3: User Story 1 - Student Orientation (13 tasks, P1 - MVP)**
- T009-T017: Write 9 sections in parallel (sec-01 through sec-10, excluding sec-06 and sec-11)
  - sec-01: The Moment Everything Changes (150-250 words, emotional hook)
  - sec-02: From Mind to Body (400-600 words, FR-001 Physical AI, FR-002 three tsunamis)
  - sec-03: Why Humanoid? (300-500 words, FR-003 infrastructure compatibility)
  - sec-04: Three Laws (600-800 words, FR-004 with concrete examples)
  - sec-05: What You Will Build (500-700 words, FR-005 four components, FR-006 technologies)
  - sec-07: Weight of Responsibility (300-500 words, FR-007 safety/ethics)
  - sec-08: The Path Ahead (200-400 words, FR-008 difficulty + motivation)
  - sec-09: Your Place in History (200-400 words, FR-011 builders, FR-009 historical)
  - sec-10: A Final Word (300-500 words, FR-009 HAL/Terminator/David, FR-012 inclusive)
- T018: Validate readability (FK 13-15, FRE 30-50)
- T019: Create 5-question comprehension quiz
- T020: Pilot test with n≥20 students (SC-001 90%+ quiz, SC-003 7+/10 motivation)
- T021: Revise based on feedback

**Phase 4: User Story 2 - Instructor Framing (8 tasks, P2)**
- T022: Write sec-11 "How to Use This Course" (FR-010 four-phase structure)
- T023-T025: Enhance sections with lecture-ready examples (Three Laws, safety, career)
- T026: Validate word count 4000-5000, reading time <20 min (SC-008)
- T027: Create instructor lecture guide
- T028: Pilot test with n≥3 instructors (SC-006 4+/5 effectiveness)
- T029: Revise based on instructor feedback

**Phase 5: User Story 3 - Self-Paced Onboarding (8 tasks, P3)**
- T030: Write sec-06 "The Tools of Creation" (200-400 words, FR-006, three-tier hardware)
- T031-T034: Enhance sections with self-paced guidance (skill levels, dependencies, prerequisites, encouragement)
- T035: Create self-paced learner survey
- T036: Pilot test with n≥10 self-paced learners (SC-007 70%+ correct self-assessment)
- T037: Revise based on feedback

**Phase 6: Polish & Cross-Cutting (12 tasks)**
- T038-T040: Final quality validation (readability, FR checklist, link check)
- T041: Extract terms to glossary.yml
- T042-T043: Urdu translation (TranslationSync agent + human review)
- T044-T045: RAG indexing (English + Urdu, Qdrant)
- T046: Deploy to staging
- T047-T049: Final validation (SC-009 jargon, SC-010 retention plan, merge)

**Key Adaptations for Content Feature**:
- Tasks organized by user story for independent testing (students, instructors, self-paced learners each get pilot tests)
- Parallel opportunities: 9 sections in US1 can be written simultaneously (T009-T017 marked [P])
- Validation tasks = readability analysis + pilot tests (not automated unit tests)
- Success criteria validated via comprehension quiz (SC-001 90%+), motivation survey (SC-003 7+/10), instructor rating (SC-006 4+/5)
- MVP = US1 only (student orientation), incremental delivery adds US2 (instructors), then US3 (self-paced)

**Dependencies**:
- Setup → Foundational → User Stories (parallel or sequential by priority) → Polish
- US1, US2, US3 independently testable after Foundational phase completes
- Urdu translation sequential (English content must be validated first)
- RAG indexing after translation approval

## Outcome

- ✅ Impact: Task breakdown enables immediate implementation start with clear checkpoints and pilot testing at each user story phase
- 🧪 Tests: Pilot tests defined for each user story (n≥20 students for US1, n≥3 instructors for US2, n≥10 self-paced for US3) with success criteria thresholds
- 📁 Files: specs/001-textbook-preface/tasks.md (49 tasks), will create docs/preface.mdx, scripts/validate-readability.py, assessment files, lecture guide
- 🔁 Next prompts: Begin implementation with Phase 1 Setup tasks (T001-T004), or jump to MVP by completing Setup + Foundational + US1
- 🧠 Reflection: Content feature tasks adapt software workflow to pedagogical validation - "tests" become comprehension quizzes and pilot studies with measurable thresholds (90%, 7+/10, 4+/5)

## Evaluation notes (flywheel)

- Failure modes observed: None - task generation followed user story organization correctly, adapted "tests" to content validation appropriately
- Graders run and results (PASS/FAIL): Format validation PASSED (all tasks follow `- [ ] TXXX [P?] [Story] Description with file path` format)
- Prompt variant (if applicable): Standard `/sp.tasks` with content feature adaptation (no code tests, pilot test validation instead)
- Next experiment (smallest change to try): For future content features, experiment with automated readability validation in CI pipeline vs manual validation between drafts to reduce iteration cycle time
