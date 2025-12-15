# Tasks: Textbook Preface - Welcome to the AI-Native Era

**Input**: Design documents from `specs/001-textbook-preface/`
**Prerequisites**: plan.md, spec.md, contracts/preface-structure.yml (embedded in plan.md)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Content Infrastructure)

**Purpose**: Initialize content structure and validation tools

- [X] T001 Create Docusaurus content directory structure for preface at docs/preface.mdx
- [X] T002 [P] Install readability validation tools (textstat Python package)
- [X] T003 [P] Configure markdown-link-check for CI pipeline validation
- [X] T004 [P] Create Urdu glossary file at i18n/glossary.yml with initial structure

---

## Phase 2: Foundational (Content Framework)

**Purpose**: Core content structure that ALL user stories depend on

**⚠️ CRITICAL**: No user story content can be written until this phase is complete

- [X] T005 Create MDX file with frontmatter (id: preface, title, sidebar_label, description, keywords) at docs/preface.mdx
- [X] T006 Add heading structure for all 11 sections (sec-01 through sec-11) per contracts/preface-structure.yml
- [X] T007 Create readability validation script at scripts/validate-readability.py using textstat
- [X] T008 Create comprehension quiz template at specs/001-textbook-preface/assessment/comprehension-quiz.md

**Checkpoint**: Framework ready - section content can now be written in parallel

---

## Phase 3: User Story 1 - Student Orientation & Motivation (Priority: P1) 🎯 MVP

**Goal**: Enable students to understand Physical AI, three convergences, course structure, and feel motivated

**Independent Test**: Students read preface and complete 5-question comprehension quiz with 90%+ accuracy (SC-001), report 7+/10 motivation score (SC-003)

### Implementation for User Story 1

- [ ] T009 [P] [US1] Write sec-01 "The Moment Everything Changes" (150-250 words, emotional hook, FR-012 inclusive language) in docs/preface.mdx
- [ ] T010 [P] [US1] Write sec-02 "From Mind to Body: The Great Awakening" (400-600 words, FR-001 Physical AI definition, FR-002 three tsunamis) in docs/preface.mdx
- [ ] T011 [P] [US1] Write sec-03 "Why Humanoid? Why Now?" (300-500 words, FR-003 infrastructure compatibility) in docs/preface.mdx
- [ ] T012 [P] [US1] Write sec-04 "The Three Laws of Physical AI" (600-800 words, FR-004 three laws with concrete examples) in docs/preface.mdx
- [ ] T013 [US1] Write sec-05 "What You Will Build" (500-700 words, FR-005 four components, FR-006 technologies list) in docs/preface.mdx
- [ ] T014 [P] [US1] Write sec-07 "The Weight of Responsibility" (300-500 words, FR-007 safety/ethics with failure mode examples) in docs/preface.mdx
- [ ] T015 [P] [US1] Write sec-08 "The Path Ahead" (200-400 words, FR-008 difficulty acknowledgment + motivation) in docs/preface.mdx
- [ ] T016 [P] [US1] Write sec-09 "Your Place in History" (200-400 words, FR-011 position as builders, FR-009 historical framing) in docs/preface.mdx
- [ ] T017 [P] [US1] Write sec-10 "A Final Word Before We Begin" (300-500 words, FR-009 HAL 9000/Terminator/David references, FR-012 inclusive call to action) in docs/preface.mdx
- [ ] T018 [US1] Validate readability (FK 13-15, FRE 30-50) using scripts/validate-readability.py on docs/preface.mdx
- [ ] T019 [US1] Create 5-question comprehension quiz testing FR-001 (Physical AI), FR-002 (three convergences), FR-004 (three laws), SC-004 (four-phase structure) in specs/001-textbook-preface/assessment/comprehension-quiz.md
- [ ] T020 [US1] Pilot test with n≥20 students, validate SC-001 (90%+ quiz accuracy), SC-002 (80%+ recall convergences), SC-003 (7+/10 motivation), SC-004 (95%+ recall phases)
- [ ] T021 [US1] Revise content based on pilot feedback to meet SC-001 through SC-004 targets in docs/preface.mdx

**Checkpoint**: At this point, User Story 1 (student orientation) should be fully functional - students can read, comprehend, and feel motivated

---

## Phase 4: User Story 2 - Instructor Course Framing (Priority: P2)

**Goal**: Enable instructors to deliver compelling first-day lecture using preface content

**Independent Test**: Instructors deliver 20-minute opening lecture using only preface content, students report comprehension/engagement, instructors rate effectiveness 4+/5 (SC-006)

### Implementation for User Story 2

- [ ] T022 [US2] Write sec-11 "How to Use This Course" (300-500 words, FR-010 four-phase structure with dependencies) in docs/preface.mdx
- [ ] T023 [US2] Enhance sec-04 "The Three Laws" with ready-to-use examples for lecture (kinematics for Law 1, digital twins for Law 2, VLA for Law 3) in docs/preface.mdx
- [ ] T024 [US2] Enhance sec-07 "Weight of Responsibility" with concrete lecture-ready failure mode examples ("bug in web app = frustration, bug in embodied AI = injury") in docs/preface.mdx
- [ ] T025 [US2] Enhance sec-09 "Your Place in History" with career prospects framing ("code you write could run on machines that outlive you") in docs/preface.mdx
- [ ] T026 [US2] Validate total word count 4000-5000 and reading time <20 minutes for 90% (SC-008) using scripts/validate-readability.py on docs/preface.mdx
- [ ] T027 [US2] Create instructor lecture guide at specs/001-textbook-preface/instructor-resources/lecture-guide.md mapping preface sections to 20-minute lecture flow
- [ ] T028 [US2] Pilot test with n≥3 instructors delivering first-day lecture using preface, validate SC-006 (4+/5 effectiveness rating)
- [ ] T029 [US2] Revise based on instructor feedback to improve lecture-readiness in docs/preface.mdx

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - students comprehend AND instructors can lecture effectively

---

## Phase 5: User Story 3 - Self-Paced Learner Onboarding (Priority: P3)

**Goal**: Enable self-paced learners to assess textbook fit, understand prerequisites, and plan learning path

**Independent Test**: Self-paced learners survey whether they (1) know hardware/software requirements, (2) understand time commitment, (3) feel confident following path - 70%+ correctly assess prerequisites (SC-007)

### Implementation for User Story 3

- [ ] T030 [US3] Write sec-06 "The Tools of Creation" (200-400 words, FR-006 technologies, three-tier hardware structure, self-assessment cues) in docs/preface.mdx
- [ ] T031 [US3] Enhance sec-05 "What You Will Build" with skill level indicators (beginner/intermediate/advanced markers for capstone project) in docs/preface.mdx
- [ ] T032 [US3] Enhance sec-11 "How to Use This Course" with self-paced learner guidance ("Each module builds on the last. Skip nothing.") in docs/preface.mdx
- [ ] T033 [US3] Add prerequisite acknowledgment to sec-08 "The Path Ahead" ("If you lack linear algebra background, see Prerequisites chapter") addressing edge case in docs/preface.mdx
- [ ] T034 [US3] Add intimidation acknowledgment to sec-08 with encouragement balancing difficulty with rewards in docs/preface.mdx
- [ ] T035 [US3] Create self-paced learner assessment survey at specs/001-textbook-preface/assessment/self-paced-survey.md testing hardware/software awareness, time commitment, confidence
- [ ] T036 [US3] Pilot test with n≥10 self-paced learners (professionals/hobbyists), validate SC-007 (70%+ correct prerequisite self-assessment)
- [ ] T037 [US3] Revise content based on self-paced learner feedback in docs/preface.mdx

**Checkpoint**: All user stories should now be independently functional - students, instructors, and self-paced learners all served

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Quality gates, translation, and deployment

- [ ] T038 [P] Run final readability validation (FK 13-15, FRE 30-50, total words 4000-5000, reading time <20 min) using scripts/validate-readability.py on docs/preface.mdx
- [ ] T039 [P] Validate all functional requirements FR-001 through FR-012 checklist per specs/001-textbook-preface/plan.md quickstart.md guidelines
- [ ] T040 [P] Run markdown-link-check to ensure zero broken links (SC-009 zero jargon confusion implies working references)
- [ ] T041 Extract all technical terms to i18n/glossary.yml (Physical AI, Foundation Models, Embodied AI, Digital Twin, VLA, etc.)
- [ ] T042 Trigger TranslationSync agent to generate Urdu translation at i18n/ur/docusaurus-plugin-content-docs/current/preface.mdx using glossary.yml
- [ ] T043 Human review of Urdu translation by Urdu-speaking educator for cultural/motivational tone appropriateness
- [ ] T044 [P] Index English preface into Qdrant using RAGIndexer agent (chunks: 1000 tokens, overlap: 200) for chatbot Q&A
- [ ] T045 [P] Index Urdu preface into Qdrant using RAGIndexer agent after translation approval
- [ ] T046 Deploy to staging (Vercel preview) for final review before production
- [ ] T047 Validate SC-009 (zero jargon confusion instances) via pilot test feedback analysis
- [ ] T048 Create launch plan for SC-010 (85%+ Week 1 to Week 2 retention) measurement post-publication
- [ ] T049 Final commit and merge to main branch with message "Add textbook preface (FR-001 through FR-012, US1-US3 complete)"

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Enhances US1 sections but independently testable via instructor pilot
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Adds sec-06, enhances US1 sections, independently testable via self-paced survey

### Within Each User Story

- **US1**: Sections sec-01, sec-02, sec-03, sec-04, sec-07, sec-08, sec-09, sec-10 can be written in parallel (different sections, marked [P])
  - sec-05 and sec-13 depend on understanding overall course structure (sequential after other sections)
  - Readability validation (T018) depends on all sections being written
  - Pilot test (T020) depends on readability validation passing
- **US2**: All enhancement tasks can run in parallel with each other (marked [P] where applicable)
  - Instructor lecture guide (T027) depends on all enhancements complete
  - Pilot test (T028) depends on lecture guide being ready
- **US3**: sec-06 can be written in parallel with enhancements to other sections
  - Self-paced survey (T035) depends on all US3 content being written
  - Pilot test (T036) depends on survey being ready

### Parallel Opportunities

- **Phase 1 Setup**: All tasks (T001-T004) can run in parallel
- **Phase 2 Foundational**: T005-T008 should run sequentially (T005 creates file, T006 adds structure, T007-T008 parallel after)
- **Phase 3 US1**: T009, T010, T011, T012, T014, T015, T016, T017 can all run in parallel (different sections)
- **Phase 4 US2**: T023, T024, T025 can run in parallel (enhancing different sections)
- **Phase 5 US3**: T031, T032, T033, T034 can run in parallel after T030 creates sec-06
- **Phase 6 Polish**: T038, T039, T040 can run in parallel (different validation types), T044 and T045 sequential (English first, then Urdu)

---

## Parallel Example: User Story 1

```bash
# Launch all section writing tasks for User Story 1 together:
Task: "Write sec-01 'The Moment Everything Changes' in docs/preface.mdx"
Task: "Write sec-02 'From Mind to Body' in docs/preface.mdx"
Task: "Write sec-03 'Why Humanoid? Why Now?' in docs/preface.mdx"
Task: "Write sec-04 'The Three Laws of Physical AI' in docs/preface.mdx"
Task: "Write sec-07 'The Weight of Responsibility' in docs/preface.mdx"
Task: "Write sec-08 'The Path Ahead' in docs/preface.mdx"
Task: "Write sec-09 'Your Place in History' in docs/preface.mdx"
Task: "Write sec-10 'A Final Word Before We Begin' in docs/preface.mdx"

# After all sections complete, validate readability:
Task: "Validate readability (FK 13-15) using scripts/validate-readability.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Student Orientation & Motivation)
4. **STOP and VALIDATE**: Pilot test with n≥20 students, validate SC-001 through SC-004
5. Deploy to staging if ready (instructors and self-paced learners get partial value from US1 alone)

### Incremental Delivery

1. Complete Setup + Foundational → Framework ready
2. Add User Story 1 → Pilot test students → Deploy/Demo (MVP! Students can read and comprehend)
3. Add User Story 2 → Pilot test instructors → Deploy/Demo (Instructors can now lecture effectively)
4. Add User Story 3 → Pilot test self-paced learners → Deploy/Demo (Self-paced learners can assess fit)
5. Complete Polish → Translation + RAG indexing → Production deployment

### Parallel Team Strategy

With multiple content authors:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Author A: User Story 1 sections (sec-01, sec-02, sec-03, sec-04)
   - Author B: User Story 1 sections (sec-07, sec-08, sec-09, sec-10)
   - Author C: User Story 2 enhancements (after US1 sections exist)
3. Stories complete and integrate via shared docs/preface.mdx file (merge coordination required)

---

## Success Criteria Validation Checklist

Before declaring preface complete, validate all success criteria:

- [ ] **SC-001**: 90%+ students pass comprehension quiz (T020 pilot test n≥20)
- [ ] **SC-002**: 80%+ recall three convergences (T020 quiz question 2)
- [ ] **SC-003**: 7+/10 motivation score (T020 pilot survey)
- [ ] **SC-004**: 95%+ articulate four-phase structure (T020 quiz question 4)
- [ ] **SC-005**: 3/5 technologies identified (T020 quiz + survey)
- [ ] **SC-006**: 4+/5 instructor effectiveness (T028 instructor pilot n≥3)
- [ ] **SC-007**: 70%+ self-paced learners assess prerequisites correctly (T036 survey n≥10)
- [ ] **SC-008**: <20 min reading time for 90% (T026, T038 readability validation)
- [ ] **SC-009**: Zero jargon confusion (T047 pilot feedback analysis)
- [ ] **SC-010**: 85%+ retention Week 1→2 (T048 post-publication measurement plan created)

---

## Notes

- [P] tasks = different sections of docs/preface.mdx or different files entirely, can run in parallel
- [Story] label maps task to specific user story (US1, US2, US3) for traceability
- Each user story should be independently testable via pilot tests (students, instructors, self-paced learners)
- Pilot tests validate success criteria before moving to next phase
- Commit after each section or logical group of enhancements
- Stop at any checkpoint to validate story independently
- **Content feature adaptation**: Tests are validation activities (readability analysis, pilot tests, surveys) rather than automated unit tests
- All writing follows guidelines in specs/001-textbook-preface/plan.md quickstart.md section
- Urdu translation (T042-T043) uses two-phase approach: GPT-4 via TranslationSync agent + human review
- RAG indexing (T044-T045) enables chatbot Q&A support after publication
