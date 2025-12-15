# Tasks: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)

**Input**: Design documents from `/specs/004-module3-isaac/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/, quickstart.md

**Tests**: Validation scripts are included for educational content verification. No unit tests required for this educational module.

**Organization**: Tasks are grouped by user story (P1: Simulation, P2: VSLAM, P3: Navigation) to enable independent chapter implementation.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Educational content module following structure:
- **Module root**: `module-3-isaac/` at repository root
- **Chapters**: `chapter-1-isaac-sim/`, `chapter-2-isaac-ros/`, `chapter-3-nav2/`
- **Shared utilities**: `shared/utils/`, `shared/datasets/`
- **Tests**: `tests/` at module root

---

## Phase 1: Setup (Module Infrastructure)

**Purpose**: Initialize Module 3 structure and shared resources

- [x] T001 Create module directory structure per plan.md project structure
- [x] T002 [P] Create module-3-isaac/README.md with overview, prerequisites, and setup instructions
- [x] T003 [P] Create module-3-isaac/requirements.txt with Python dependencies (pytest, rospkg, pyyaml, numpy, opencv-python)
- [x] T004 [P] Create module-3-isaac/.gitignore for Python cache, ROS artifacts, and dataset files
- [x] T005 [P] Create shared/utils/tier_detection.py for automatic Tier A/B/C hardware detection
- [x] T006 [P] Create shared/utils/metrics.py for performance measurement utilities
- [x] T007 [P] Create shared/utils/validation.py for success criteria validation functions
- [x] T008 Create module-3-isaac/package.xml for ROS 2 package metadata

---

## Phase 2: Foundational (Shared Assets & Sample Data)

**Purpose**: Core assets and datasets that ALL chapters depend on

**⚠️ CRITICAL**: No chapter work can begin until this phase is complete

- [x] T009 Download and organize shared/datasets/sample-warehouse/ with pre-generated synthetic dataset (1000 RGB images, depth maps, labels.json)
- [x] T010 [P] Create shared/datasets/recorded-trajectories/ with sample ROS bag files for VSLAM testing
- [x] T011 [P] Create tests/test_tier_detection.py to validate hardware tier detection utility
- [x] T012 [P] Create tests/test_metrics.py to validate performance measurement utilities
- [x] T013 Create shared/assets/warehouse-scene.usd as base Isaac Sim environment (or document download instructions)

**Checkpoint**: Foundation ready - chapter implementation can now begin in parallel

---

## Phase 3: User Story 1 - Photorealistic Simulation & Synthetic Data (Priority: P1) 🎯 MVP

**Goal**: Enable learners to create Isaac Sim environments, configure sensors, and generate synthetic datasets with domain randomization

**Independent Test**: Learner can launch Isaac Sim, load a scene, configure a camera, generate 100 labeled images, and export dataset in COCO format

### Chapter 1 Structure & Documentation

- [x] T014 [P] [US1] Create chapter-1-isaac-sim/01-intro.mdx with introduction to Isaac Sim, installation verification, and learning objectives
- [x] T015 [P] [US1] Create chapter-1-isaac-sim/02-environment-creation.mdx with USD scene creation tutorial and Mermaid diagram of simulation pipeline
- [x] T016 [P] [US1] Create chapter-1-isaac-sim/03-sensor-configuration.mdx documenting RGB, depth, stereo cameras, and LiDAR setup
- [x] T017 [P] [US1] Create chapter-1-isaac-sim/04-data-generation.mdx with synthetic dataset generation workflow and ground truth annotation
- [x] T018 [P] [US1] Create chapter-1-isaac-sim/05-domain-randomization.mdx explaining Tier 1/2/3 randomization strategies per research.md
- [x] T019 [P] [US1] Create chapter-1-isaac-sim/06-sim-to-real.mdx covering sim-to-real gap mitigation techniques

### Chapter 1 Assets & Configuration

- [x] T020 [P] [US1] Create chapter-1-isaac-sim/assets/sensor-configs.yaml with example camera/LiDAR configurations
- [x] T021 [P] [US1] Document chapter-1-isaac-sim/assets/warehouse-scene.usd setup instructions (link to shared asset)
- [x] T022 [US1] Create chapter-1-isaac-sim/config/randomization-tier1.yaml for lighting and texture randomization
- [x] T023 [US1] Create chapter-1-isaac-sim/config/randomization-tier2.yaml adding object pose jitter
- [x] T024 [US1] Create chapter-1-isaac-sim/config/randomization-tier3.yaml with full camera noise randomization

### Chapter 1 Exercises & Validation

- [x] T025 [US1] Create chapter-1-isaac-sim/exercises/ex1-create-scene.py guiding scene creation and sensor placement (FR-001, FR-002)
- [x] T026 [US1] Create chapter-1-isaac-sim/exercises/ex2-generate-dataset.py for synthetic data generation with 1000+ images (FR-003, SC-001)
- [x] T027 [US1] Create chapter-1-isaac-sim/exercises/ex3-domain-randomization.py implementing Tier 1-3 randomization (FR-004)
- [x] T028 [US1] Create chapter-1-isaac-sim/exercises/ex4-export-dataset.py for COCO/YOLO format export (FR-005)
- [x] T029 [US1] Create chapter-1-isaac-sim/exercises/validation.py to verify SC-001 (1000+ images/hour) and SC-007 (>85% perception accuracy)
- [x] T030 [US1] Create tests/test_chapter1_simulation.py validating all Chapter 1 functional requirements (FR-001 to FR-006)

**Checkpoint**: Chapter 1 complete - learners can generate synthetic datasets independently

---

## Phase 4: User Story 2 - Hardware-Accelerated Visual SLAM (Priority: P2)

**Goal**: Implement real-time VSLAM using Isaac ROS cuVSLAM, demonstrating map building, loop closure, and relocalization

**Independent Test**: Learner can launch VSLAM on recorded bag file, build 3D map, save map, reload map, and relocalize successfully

### Chapter 2 Structure & Documentation

- [x] T031 [P] [US2] Create chapter-2-isaac-ros/01-intro.mdx with Isaac ROS overview, GPU requirements, and cuVSLAM introduction
- [x] T032 [P] [US2] Create chapter-2-isaac-ros/02-visual-odometry.mdx explaining visual odometry pipeline with Mermaid computation graph
- [x] T033 [P] [US2] Create chapter-2-isaac-ros/03-stereo-vslam.mdx covering stereo camera VSLAM setup and calibration
- [x] T034 [P] [US2] Create chapter-2-isaac-ros/04-loop-closure.mdx documenting loop closure detection and pose graph optimization
- [x] T035 [P] [US2] Create chapter-2-isaac-ros/05-map-management.mdx for saving, loading, and relocalization workflows (FR-011)
- [x] T036 [P] [US2] Create chapter-2-isaac-ros/06-performance-tuning.mdx with GPU acceleration tuning and performance benchmarks (FR-013, SC-002)
- [x] T037 [P] [US2] Create chapter-2-isaac-ros/07-debugging.mdx covering VSLAM failure modes and recovery strategies (edge cases)

### Chapter 2 Launch Files & Configuration

- [x] T038 [P] [US2] Create chapter-2-isaac-ros/launch/vslam-sim.launch.py for Tier A simulation VSLAM (integrates with Chapter 1 scenes)
- [x] T039 [P] [US2] Create chapter-2-isaac-ros/launch/vslam-jetson.launch.py for Tier B Jetson Orin deployment with power mode settings
- [x] T040 [US2] Create chapter-2-isaac-ros/config/vslam-params.yaml with Isaac ROS cuVSLAM parameters per contracts/vslam-topics-services.yaml
- [x] T041 [US2] Create chapter-2-isaac-ros/config/camera-calibration.yaml with sample stereo calibration data
- [x] T042 [US2] Create chapter-2-isaac-ros/config/rviz-vslam.rviz for visualization of landmarks, pose graph, and trajectory

### Chapter 2 Exercises & Validation

- [x] T043 [US2] Create chapter-2-isaac-ros/exercises/ex1-visual-odometry.py demonstrating monocular VO on recorded data (FR-007)
- [x] T044 [US2] Create chapter-2-isaac-ros/exercises/ex2-build-map.py for stereo VSLAM map building with loop closure (FR-008, FR-010)
- [x] T045 [US2] Create chapter-2-isaac-ros/exercises/ex3-relocalization.py testing save/load/relocalize workflow (FR-011)
- [x] T046 [US2] Create chapter-2-isaac-ros/exercises/ex4-performance-benchmark.py measuring frame rate, latency, accuracy (FR-012, SC-002, SC-003)
- [x] T047 [US2] Create chapter-2-isaac-ros/exercises/validation.py verifying SC-002 (>30 Hz) and SC-003 (<2% drift over 100m)
- [x] T048 [US2] Create tests/test_chapter2_vslam.py validating all Chapter 2 functional requirements (FR-007 to FR-013)

**Checkpoint**: ✅ Chapter 2 complete - learners can run real-time VSLAM independently

---

## Phase 5: User Story 3 - Bipedal Path Planning & Navigation (Priority: P3)

**Goal**: Configure Nav2 with custom footstep planner for bipedal navigation with stability constraints

**Independent Test**: Learner can set navigation goal in RViz, planner generates valid footstep sequence respecting ZMP constraints, robot reaches goal in simulation

### Chapter 3 Structure & Documentation

- [ ] T049 [P] [US3] Create chapter-3-nav2/01-intro.mdx with Nav2 overview and bipedal planning challenges
- [ ] T050 [P] [US3] Create chapter-3-nav2/02-global-planning.mdx covering grid-based global planner configuration (FR-014)
- [ ] T051 [P] [US3] Create chapter-3-nav2/03-local-planning.mdx explaining DWB local planner and bipedal adaptations (FR-015)
- [ ] T052 [P] [US3] Create chapter-3-nav2/04-footstep-planning.mdx documenting custom footstep planner plugin with ZMP stability (FR-016, FR-020)
- [ ] T053 [P] [US3] Create chapter-3-nav2/05-obstacle-avoidance.mdx for dynamic obstacle handling with costmap updates (FR-017)
- [ ] T054 [P] [US3] Create chapter-3-nav2/06-recovery-behaviors.mdx covering spin, backup, and stuck detection (FR-018)
- [ ] T055 [P] [US3] Create chapter-3-nav2/07-integration.mdx for end-to-end VSLAM + Nav2 integration (FR-019)

### Chapter 3 Launch Files & Configuration

- [ ] T056 [P] [US3] Create chapter-3-nav2/launch/nav2-sim.launch.py for Tier A navigation in Isaac Sim (integrates Chapter 1 & 2)
- [ ] T057 [P] [US3] Create chapter-3-nav2/launch/nav2-hardware.launch.py for Tier C physical robot deployment
- [ ] T058 [US3] Create chapter-3-nav2/config/nav2-params.yaml with planner/controller configuration per contracts/nav2-action-interfaces.yaml
- [ ] T059 [US3] Create chapter-3-nav2/config/footstep-planner.yaml with bipedal constraints (max step length/width, stability margin)
- [ ] T060 [US3] Create chapter-3-nav2/config/bipedal-constraints.yaml defining footstep kinematics and ZMP limits per research.md R3
- [ ] T061 [US3] Create chapter-3-nav2/config/costmap-params.yaml for global and local costmap configuration

### Chapter 3 URDF & Robot Model

- [ ] T062 [US3] Create chapter-3-nav2/urdf/humanoid-robot.urdf with simplified bipedal model (base_link, left_foot, right_foot frames)
- [ ] T063 [US3] Create chapter-3-nav2/urdf/humanoid-robot.urdf.xacro with parametric bipedal model and TF tree
- [ ] T064 [US3] Document chapter-3-nav2/urdf/README.md explaining URDF structure and how to adapt for different robots

### Chapter 3 Custom Planner Plugin (Educational Implementation)

- [ ] T065 [US3] Create chapter-3-nav2/src/footstep_planner_plugin.py with Nav2 planner plugin interface skeleton
- [ ] T066 [US3] Implement discrete footstep selection logic in footstep_planner_plugin.py (2D grid search)
- [ ] T067 [US3] Implement ZMP stability margin checks in footstep_planner_plugin.py per data-model.md Footstep validation
- [ ] T068 [US3] Implement collision checking against costmap in footstep_planner_plugin.py (FR-017)
- [ ] T069 [US3] Add footstep plan visualization publisher in footstep_planner_plugin.py (MarkerArray for RViz)
- [ ] T070 [US3] Create chapter-3-nav2/src/setup.py for Python plugin installation

### Chapter 3 Exercises & Validation

- [ ] T071 [US3] Create chapter-3-nav2/exercises/ex1-global-planning.py demonstrating grid-based path planning (FR-014)
- [ ] T072 [US3] Create chapter-3-nav2/exercises/ex2-footstep-plan.py generating and visualizing footstep sequences (FR-016, SC-004)
- [ ] T073 [US3] Create chapter-3-nav2/exercises/ex3-autonomous-nav.py for full navigation with VSLAM localization (FR-019, SC-008)
- [ ] T074 [US3] Create chapter-3-nav2/exercises/ex4-recovery-test.py testing obstacle avoidance and recovery (FR-018, SC-009)
- [ ] T075 [US3] Create chapter-3-nav2/exercises/validation.py verifying SC-004 (<5s planning), SC-008 (>95% success), SC-009 (>70% recovery)
- [ ] T076 [US3] Create tests/test_chapter3_navigation.py validating all Chapter 3 functional requirements (FR-014 to FR-020)

**Checkpoint**: All user stories complete - learners can demonstrate end-to-end autonomous bipedal navigation

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Documentation, optimization, and final validation across all chapters

- [ ] T077 [P] Create module-3-isaac/docs/faq.md with common issues and troubleshooting (consolidate from quickstart.md)
- [ ] T078 [P] Create module-3-isaac/docs/hardware-tiers.md documenting Tier A/B/C setup variations
- [ ] T079 [P] Create module-3-isaac/docs/glossary.md with VSLAM, domain randomization, ZMP, and other technical terms
- [ ] T080 [P] Update module-3-isaac/README.md with final chapter links, learning outcomes, and success criteria
- [ ] T081 [P] Create module-3-isaac/CHANGELOG.md tracking Isaac Sim/ROS versions and breaking changes
- [ ] T082 Add comprehensive code comments and docstrings to all Python utilities and exercises
- [ ] T083 Create module-3-isaac/scripts/validate-all-chapters.sh running all validation scripts (T030, T048, T076)
- [ ] T084 Test complete learning path: Setup → Ch1 (simulation) → Ch2 (VSLAM) → Ch3 (navigation)
- [ ] T085 Verify all Mermaid diagrams render correctly in Docusaurus
- [ ] T086 Run quickstart.md validation on clean Ubuntu 22.04 system with RTX 3060 GPU
- [ ] T087 [P] Add Docusaurus admonitions (:::tip, :::danger, :::note) per constitution safety requirements
- [ ] T088 Verify all success criteria SC-001 through SC-009 are testable with provided scripts
- [ ] T089 Create module-3-isaac/examples/ directory with reference solutions for all exercises
- [ ] T090 Final review: Ensure all file paths match plan.md project structure

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - **US1 (Ch1)**: Can start after Foundational - No dependencies on other stories
  - **US2 (Ch2)**: Can start after Foundational - Uses datasets from US1 but independently testable
  - **US3 (Ch3)**: Can start after Foundational - Integrates US1 & US2 but can use pre-recorded data
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Independent - only needs shared assets from Foundational
- **User Story 2 (P2)**: Can use datasets from US1 OR shared/datasets (independent testing)
- **User Story 3 (P3)**: Can use VSLAM from US2 OR recorded bags (independent testing)

**Key Insight**: Each chapter is independently completable using pre-recorded data from shared/datasets

### Within Each User Story

- **Documentation tasks** ([P] marked) can run in parallel
- **Configuration files** ([P] marked) can run in parallel
- **Exercises** should be created sequentially (ex1 → ex2 → ex3) for pedagogical flow
- **Validation** comes after all exercises are complete

### Parallel Opportunities

- **Phase 1**: All tasks T002-T008 can run in parallel
- **Phase 2**: Tasks T010, T011, T012 can run in parallel
- **Phase 3 (US1)**: All documentation (T014-T019), assets (T020-T024) can run in parallel
- **Phase 4 (US2)**: All documentation (T031-T037), launch files (T038-T042) can run in parallel
- **Phase 5 (US3)**: All documentation (T049-T055), launch files (T056-T064) can run in parallel
- **Phase 6**: Tasks T077-T081, T087 can run in parallel

**Team Strategy**: After Foundational phase, 3 developers can work on US1, US2, US3 simultaneously

---

## Parallel Example: User Story 1 (Chapter 1)

```bash
# Documentation tasks (all parallel):
Task T014: "Create 01-intro.mdx"
Task T015: "Create 02-environment-creation.mdx"
Task T016: "Create 03-sensor-configuration.mdx"
Task T017: "Create 04-data-generation.mdx"
Task T018: "Create 05-domain-randomization.mdx"
Task T019: "Create 06-sim-to-real.mdx"

# Configuration tasks (all parallel):
Task T020: "Create sensor-configs.yaml"
Task T022: "Create randomization-tier1.yaml"
Task T023: "Create randomization-tier2.yaml"
Task T024: "Create randomization-tier3.yaml"
```

---

## Parallel Example: User Story 2 (Chapter 2)

```bash
# Documentation tasks (all parallel):
Task T031: "Create 01-intro.mdx"
Task T032: "Create 02-visual-odometry.mdx"
Task T033: "Create 03-stereo-vslam.mdx"
Task T034: "Create 04-loop-closure.mdx"
Task T035: "Create 05-map-management.mdx"
Task T036: "Create 06-performance-tuning.mdx"
Task T037: "Create 07-debugging.mdx"

# Launch files (all parallel):
Task T038: "Create vslam-sim.launch.py"
Task T039: "Create vslam-jetson.launch.py"
```

---

## Parallel Example: User Story 3 (Chapter 3)

```bash
# Documentation tasks (all parallel):
Task T049: "Create 01-intro.mdx"
Task T050: "Create 02-global-planning.mdx"
Task T051: "Create 03-local-planning.mdx"
Task T052: "Create 04-footstep-planning.mdx"
Task T053: "Create 05-obstacle-avoidance.mdx"
Task T054: "Create 06-recovery-behaviors.mdx"
Task T055: "Create 07-integration.mdx"

# Configuration files (all parallel):
Task T058: "Create nav2-params.yaml"
Task T059: "Create footstep-planner.yaml"
Task T060: "Create bipedal-constraints.yaml"
Task T061: "Create costmap-params.yaml"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only - Chapter 1)

1. Complete Phase 1: Setup (T001-T008)
2. Complete Phase 2: Foundational (T009-T013) - CRITICAL
3. Complete Phase 3: User Story 1 / Chapter 1 (T014-T030)
4. **STOP and VALIDATE**: Run tests/test_chapter1_simulation.py
5. Deploy/demo Chapter 1 independently

**MVP Deliverable**: Learners can create Isaac Sim scenes and generate 1000+ synthetic images (SC-001)

### Incremental Delivery

1. **Foundation**: Setup + Foundational → Shared assets ready
2. **Chapter 1 (US1)**: T014-T030 → Test independently → Demo simulation workflows
3. **Chapter 2 (US2)**: T031-T048 → Test independently → Demo VSLAM on recorded data
4. **Chapter 3 (US3)**: T049-T076 → Test independently → Demo full autonomous navigation
5. **Polish**: T077-T090 → Final validation and documentation

Each chapter adds value without breaking previous chapters.

### Parallel Team Strategy

With 3 developers after Foundational phase:

1. **Team completes Setup + Foundational together** (T001-T013)
2. **Once Foundational done**:
   - Developer A: Chapter 1 (T014-T030) - Simulation expert
   - Developer B: Chapter 2 (T031-T048) - Perception/VSLAM expert
   - Developer C: Chapter 3 (T049-T076) - Planning/navigation expert
3. **Integration**: Developer D or rotating: Phase 6 Polish (T077-T090)

---

## Task Mapping to Requirements

### User Story 1 (Chapter 1) - Functional Requirements Coverage

- **FR-001** (Photorealistic environments): T015, T025
- **FR-002** (Virtual sensors): T016, T020, T025
- **FR-003** (Synthetic datasets): T017, T026
- **FR-004** (Domain randomization): T018, T022-T024, T027
- **FR-005** (Export formats): T028
- **FR-006** (Sim-to-real): T019

### User Story 2 (Chapter 2) - Functional Requirements Coverage

- **FR-007** (Visual odometry): T032, T043
- **FR-008** (VSLAM mapping): T033, T044
- **FR-009** (Camera inputs): T033, T041
- **FR-010** (Loop closure): T034, T044
- **FR-011** (Map management): T035, T045
- **FR-012** (Performance metrics): T036, T046, T047
- **FR-013** (GPU acceleration): T036, T046

### User Story 3 (Chapter 3) - Functional Requirements Coverage

- **FR-014** (Global planning): T050, T071
- **FR-015** (Local planning): T051, T066
- **FR-016** (Footstep planning): T052, T065-T069, T072
- **FR-017** (Obstacle avoidance): T053, T068, T074
- **FR-018** (Recovery behaviors): T054, T074
- **FR-019** (VSLAM integration): T055, T056, T073
- **FR-020** (Stability constraints): T052, T060, T067

---

## Success Criteria Validation

### Measurable Outcomes (SC-001 to SC-009)

- **SC-001** (1000+ images/hour): Verified by T029 (Chapter 1 validation)
- **SC-002** (>30 Hz VSLAM): Verified by T047 (Chapter 2 validation)
- **SC-003** (<2% drift): Verified by T047 (Chapter 2 validation)
- **SC-004** (<5s planning): Verified by T075 (Chapter 3 validation)
- **SC-005** (90% completion): Verified by T084 (full learning path test)
- **SC-006** (Sim-to-real understanding): Assessed in Chapter 1 (T019, quiz/reflection)
- **SC-007** (>85% perception accuracy): Verified by T029 (dataset quality check)
- **SC-008** (>95% nav success): Verified by T075 (Chapter 3 validation)
- **SC-009** (>70% recovery): Verified by T075 (Chapter 3 validation)

### Learning Outcomes (LO-001 to LO-006)

- **LO-001** (Simulation value): Chapter 1 learning objectives (T014)
- **LO-002** (VSLAM principles): Chapter 2 content (T031-T037)
- **LO-003** (Nav2 tuning): Chapter 3 exercises (T071-T074)
- **LO-004** (GPU acceleration): Chapter 2 performance tuning (T036, T046)
- **LO-005** (VSLAM debugging): Chapter 2 debugging guide (T037)
- **LO-006** (Integration): Chapter 3 integration (T055, T073)

---

## Notes

- **[P] tasks** = Different files, can run in parallel
- **[Story] labels**: [US1] = Chapter 1, [US2] = Chapter 2, [US3] = Chapter 3
- **Each chapter is independently completable** using shared datasets/bags
- **No unit tests**: Educational content uses validation scripts instead
- **Mermaid diagrams required**: T015 (simulation pipeline), T032 (VSLAM graph), T050+ (Nav2 architecture)
- **Safety admonitions**: T087 ensures :::danger blocks before Tier C hardware code
- **Commit strategy**: Commit after each task or after completing all parallel tasks in a group
- **Checkpoints**: Stop at end of each phase to validate chapter independently
- **Avoid**: Skipping Foundational phase, cross-chapter dependencies that break independence
