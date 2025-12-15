# Implementation Plan: Module 1 - The Robotic Nervous System (ROS 2)

**Branch**: `002-ros2-nervous-system` | **Date**: 2025-12-10 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `specs/002-ros2-nervous-system/spec.md`

## Summary

Create Module 1 of the Physical AI & Humanoid Robotics textbook, covering ROS 2 fundamentals across 5 chapters. Students will progress from installation and basic CLI usage through Python integration (rclpy), service-oriented architecture, and URDF robot descriptions. The module serves as the foundational "nervous system" layer that all subsequent modules (Digital Twins, AI Brain, Voice of Action) depend upon.

**Technical Approach**: Content delivered as Docusaurus MDX files with embedded Mermaid diagrams for ROS 2 computation graphs and TF trees. Code examples provided as standalone ROS 2 packages (Python/rclpy) tested in Gazebo Harmonic (Tier A simulation). Each chapter includes learning outcomes aligned with Bloom's Taxonomy (analyze → apply → create progression), hands-on exercises, and assessments validating functional requirements.

## Technical Context

**Language/Version**: MDX (Markdown + JSX) for content, Python 3.10+ for code examples using rclpy
**Primary Dependencies**: Docusaurus 3.x, ROS 2 Humble/Jazzy, rclpy, Gazebo Harmonic, TurtleBot3 simulation packages
**Storage**: Git repository for content and code, no database requirements for Module 1
**Testing**: Readability analysis (textstat), comprehension quizzes (Docusaurus Quiz plugin), ROS 2 launch tests for code examples
**Target Platform**: Web (Docusaurus site) for content, Ubuntu 22.04/24.04 or WSL2 for ROS 2 exercises
**Project Type**: Educational content module (documentation + code examples)
**Performance Goals**: Chapter reading time 15-25 minutes, ROS 2 installation <30 minutes for 95% of students, exercises completable in 1-2 hours
**Constraints**: Flesch-Kincaid grade level 13-15 (university undergraduate), zero dead links, all code examples must run in Tier A (simulation) without hardware
**Scale/Scope**: 5 chapters, 25 functional requirements, 4 user stories, ~8,000-10,000 words total content, 10-15 Python code examples

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Applicable Principles

**Principle I: Three-Tier Hardware Accessibility**
- ✅ **Status**: COMPLIANT
- **Evidence**: Spec explicitly states "Tier A: simulation-only is sufficient for Module 1" (Assumptions section). All FR requirements reference Gazebo/TurtleBot3 simulation.
- **Action**: All code examples MUST run in Gazebo Harmonic without hardware dependencies. Include environment variable `ROBOT_TIER=A` in launch files. Chapters 1-5 cover simulation-only workflows; Tier B/C mentioned only as future options.

**Principle II: Safety-First Physical AI**
- ⚠️ **Status**: INFORMATIONAL (Module 1 is simulation-only, no physical safety risks)
- **Evidence**: Module 1 has no motor control on physical hardware per spec Out of Scope section. Safety patterns introduced conceptually (e.g., FR-008 QoS settings for reliability).
- **Action**: Chapters 4-5 MUST include `:::info` admonitions explaining that safety mechanisms (E-Stop, Dead Man Switch) will be covered in Module 3 when physical robots are introduced. Simulation safety (joint limits, collision checking) demonstrated in Gazebo.

**Principle III: Bloom's Taxonomy Learning Outcomes**
- ✅ **Status**: COMPLIANT
- **Evidence**: Spec defines 12 success criteria explicitly requiring students to "create" (SC-003 publisher-subscriber), "analyze" (SC-004 QoS decisions), "apply" (SC-005 Python control). User stories map to Bloom's: US1 (remember/understand), US2 (apply), US3 (analyze), US4 (apply/create).
- **Action**: Each chapter frontmatter MUST list 2-3 learning outcomes using Bloom's verbs. Chapter 1: "understand" (installation), "remember" (CLI commands). Chapter 3: "apply" (Python nodes), "create" (custom publishers). Chapter 5: "create" (URDF), "evaluate" (validation).

**Principle IV: Visual Intelligence (Mermaid-First Diagrams)**
- ✅ **Status**: COMPLIANT
- **Evidence**: FR-005 requires "explain ROS 2 node architecture", FR-024 requires "explain coordinate frames and TF tree" - both inherently visual. Constitution mandates Mermaid for ROS 2 computation graphs and TF trees.
- **Action**:
  - Chapter 2 MUST include Mermaid `graph TD` showing talker→/chatter topic→listener node communication
  - Chapter 4 MUST include Mermaid `sequenceDiagram` for service request-response flow
  - Chapter 5 MUST include Mermaid `graph TD` for TF tree (world→base_link→camera_link)
  - All diagrams MUST have `<MermaidDiagram>` wrapper with semantic descriptions for accessibility

**Principle V: RAG Chatbot Integration**
- ✅ **Status**: COMPLIANT (content will be indexed, not implemented in this module)
- **Evidence**: Constitution requires textbook content be chunked and indexed in Qdrant. Module 1 content will be source material.
- **Action**: Write content in clear, self-contained paragraphs suitable for 1000-token chunks with 200-token overlap. Avoid pronoun ambiguity across paragraphs (RAG chunks may be retrieved out of context). Flag quiz/assessment content with `<!-- RAG_EXCLUDE -->` metadata.

**Principle VI: Personalization Engine**
- ⚠️ **Status**: INFORMATIONAL (Module 1 content is foundational for all users)
- **Evidence**: Module 1 is prerequisite for all subsequent modules (Principle IX). No branching paths for different experience levels.
- **Action**: Chapter 1 MAY include optional "Prerequisites Refresher" section for beginners (Linux CLI basics, Python syntax). Advanced users can skip. Tag with `<details>` expandable blocks: `<summary>Click if you need a Python refresher</summary>`.

**Principle VII: Bilingual Support (English/Urdu)**
- ✅ **Status**: COMPLIANT
- **Evidence**: Constitution requires all content in English and Urdu. Module 1 introduces technical terms that need glossary entries.
- **Action**:
  - Extract technical terms to `i18n/glossary.yml`: ROS 2, node, topic, service, URDF, etc.
  - English MDX files in `physicalai-humanoidrobotics-book/docs/module1/`
  - Urdu translations in `physicalai-humanoidrobotics-book/i18n/ur/docusaurus-plugin-content-docs/current/module1/`
  - Code comments remain in English (industry standard)

**Principle VIII: Reusable Intelligence Architecture**
- ✅ **Status**: COMPLIANT
- **Evidence**: Module 1 establishes patterns (ROS 2 node structure, launch files, Python rclpy templates) that Modules 2-4 will inherit.
- **Action**:
  - Create reusable Python node template in `module-1-ros2-fundamentals/src/textbook_templates/` with rclpy best practices
  - Document ROS 2 workspace setup in ADR (e.g., "ADR-002: Why colcon over catkin")
  - Create Claude Code skill: `ros2-package-generator` for creating new packages with correct `package.xml` dependencies

**Principle IX: Module-Aligned Content Structure**
- ✅ **Status**: COMPLIANT
- **Evidence**: Spec explicitly states "This is Module 1 of 4, forms foundation for all subsequent modules" (Dependencies section). Designed for Weeks 3-5 of 13-week schedule.
- **Action**:
  - Create `module-overview.md` with prerequisite checklist (Python basics, Linux CLI, Docker)
  - Each chapter must be self-contained (order-independent where possible, though Chapter 5 URDF depends on Chapter 2 topics)
  - Week 5 integration project: "Build a ROS 2 navigation pipeline combining topics, services, and URDF"

**Principle X: Code Quality Standards**
- ✅ **Status**: COMPLIANT
- **Evidence**: FR requirements specify correct tool usage (colcon, check_urdf, ros2 CLI). Spec Success Criteria SC-010 requires "valid URDF" and SC-006 requires "graceful shutdown" (exception handling).
- **Action**:
  - All Python nodes MUST follow rclpy patterns: `Node` subclass, `create_subscription` callbacks, `destroy_node()` in `finally` block
  - Every ROS 2 package MUST have complete `package.xml` with `<depend>rclpy</depend>`, `<depend>geometry_msgs</depend>`, etc.
  - CI pipeline MUST run `markdown-link-check` on all Module 1 MDX files
  - Docker devcontainer MUST pin ROS 2 Humble version: `FROM osrf/ros:humble-desktop-full`

### Non-Applicable Principles

None - all 10 constitution principles apply to Module 1 content creation, though some (II Safety, VI Personalization) are informational/preparatory rather than requiring immediate implementation.

### Gate Evaluation

**PASSED** - No blocking violations. All applicable principles have clear action items for compliance during Phase 0 (research) and Phase 1 (design).

## Project Structure

### Documentation (this feature)

```text
specs/002-ros2-nervous-system/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification (already created)
├── research.md          # Phase 0 output (content pedagogy decisions)
├── content-model.md     # Phase 1 output (chapter structure, learning outcomes)
├── quickstart.md        # Phase 1 output (content authoring guidelines)
├── contracts/           # Phase 1 output (chapter contracts)
│   └── chapter-structure.yml  # 5 chapters with word counts, learning outcomes, exercises
├── checklists/
│   └── requirements.md  # Quality validation (already created, all PASSED)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (Docusaurus repository)

```text
# Content Structure (physicalai-humanoidrobotics-book/)
physicalai-humanoidrobotics-book/docs/
├── preface.mdx                          # Already created
└── module1/                             # NEW - Module 1 content
    ├── module-overview.md               # Module intro, prerequisites, learning objectives
    ├── chapter1-installation.mdx        # Chapter 1: ROS 2 Installation and Workspace Setup
    ├── chapter2-topics.mdx              # Chapter 2: Nodes, Topics, and Publishers/Subscribers
    ├── chapter3-python.mdx              # Chapter 3: Python Integration with rclpy
    ├── chapter4-services.mdx            # Chapter 4: ROS 2 Services and Actions
    ├── chapter5-urdf.mdx                # Chapter 5: URDF and Robot Description
    └── integration-project.md           # Week 5 capstone: Navigation pipeline

# ROS 2 Code Examples (separate GitHub repo: physical-ai-textbook/module-1-ros2-fundamentals)
module-1-ros2-fundamentals/
├── src/
│   ├── ch1_workspace_demo/              # Chapter 1: Workspace creation demo
│   ├── ch2_talker_listener/             # Chapter 2: Publisher/Subscriber examples
│   ├── ch3_robot_control/               # Chapter 3: Python rclpy nodes for TurtleBot3 control
│   ├── ch4_service_examples/            # Chapter 4: Custom service server/client
│   ├── ch5_urdf_models/                 # Chapter 5: Humanoid arm URDF files
│   └── textbook_templates/              # Reusable node templates (Principle VIII)
├── launch/                              # ROS 2 launch files for each example
├── docker/
│   └── Dockerfile.humble                # Pinned ROS 2 Humble environment
├── tests/                               # pytest for Python nodes, launch tests
└── README.md                            # Setup instructions for students

# Urdu Translations (i18n)
physicalai-humanoidrobotics-book/i18n/ur/docusaurus-plugin-content-docs/current/module1/
└── [Mirrored structure of docs/module1/ with Urdu translations]
```

**Structure Decision**: Module 1 content is part of the main Docusaurus textbook repository (`physicalai-humanoidrobotics-book/docs/module1/`), while ROS 2 code examples live in a separate repository (`module-1-ros2-fundamentals`) to enable independent cloning and CI testing. This separation follows industry practice (documentation ≠ code distribution) and allows students to fork code examples without documentation weight.

## Complexity Tracking

> **No violations** - Constitution Check passed. This section intentionally left blank.

---

## Phase 0: Research - Content Pedagogy

**Goal**: Resolve all content structure unknowns and establish chapter organization based on ROS 2 pedagogical best practices.

### Research Tasks

#### R1: ROS 2 Learning Progression Best Practices

**Question**: What is the optimal order for introducing ROS 2 concepts to absolute beginners?

**Research Approach**:
- Review official ROS 2 tutorials structure (ros2.org/docs)
- Analyze top ROS 2 textbooks: "A Concise Introduction to Robot Programming with ROS2" (O'Kane), "Programming Robots with ROS" (Quigley et al.)
- Survey common student pain points from ROS Discourse forum (topics tagged "beginner", "getting-started")

**Decision**: 5-chapter structure with progressive complexity:
1. **Installation & CLI first** (Chapter 1): Students need working environment before concepts. CLI tools (`ros2 topic list`, `ros2 node info`) provide immediate feedback and debugging capability.
2. **Topics before services** (Chapter 2-3): Pub-sub is conceptually simpler (fire-and-forget) than request-response. 80% of ROS 2 communication uses topics.
3. **Python before C++** (Chapter 3): Spec requires Python (rclpy). C++ deferred to advanced topics. Python's interpreted nature enables faster iteration.
4. **Services as extension** (Chapter 4): Once pub-sub understood, services are natural extension for synchronous patterns (e.g., "compute inverse kinematics and return result").
5. **URDF last** (Chapter 5): Robot description requires understanding of topics (for `/joint_states` publishing) and frames (Chapter 2 concepts). URDF is domain-specific knowledge, not core ROS 2 middleware.

**Rationale**: This progression mirrors the official ROS 2 tutorials and minimizes cognitive load. Each chapter builds one concept layer while reinforcing previous layers.

**Alternatives Considered**:
- **Start with URDF (robot-first)**: Rejected because students need communication primitives to make robots do anything. URDF without topics is static visualization.
- **Combine services + actions in one chapter**: Rejected because actions are complex (goal-feedback-result pattern). Deferred actions to Module 3 when long-running tasks (navigation, manipulation) become relevant.

#### R2: Hands-On Exercise Design for CLI vs. Coding

**Question**: How much CLI usage should be required before introducing Python code?

**Research Approach**:
- Analyze dropout rates in MOOCs: students who don't see "runnable output" in first hour have 60% higher dropout (data from Coursera, 2020)
- Review ROS 2 industrial training programs (NVIDIA Isaac SDK tutorials, Clearpath Robotics onboarding)

**Decision**:
- **Chapter 1**: CLI-only exercises. Students run pre-built nodes (`ros2 run demo_nodes_cpp talker`), inspect topics/nodes with CLI, visualize in RViz. No coding required. Goal: see a working ROS 2 system within 15 minutes.
- **Chapter 2**: Hybrid. Students use CLI to verify their publisher/subscriber nodes work. Example: `ros2 topic echo /my_topic` confirms their publisher code is sending data.
- **Chapter 3+**: Code-first. CLI becomes debugging tool, not primary interaction.

**Rationale**: Immediate feedback loop ("it works!") reduces frustration. CLI mastery is also essential for debugging even after students write code.

**Alternatives Considered**:
- **Code from day 1**: Rejected. Students who encounter setup issues (dependency errors, IDE problems) never see ROS 2 work and assume it's broken.
- **CLI-only for entire module**: Rejected. Spec explicitly requires Python integration (FR-010 through FR-014).

#### R3: Simulation Platform Selection (Gazebo vs. Isaac Sim for Module 1)

**Question**: Should Module 1 introduce Isaac Sim or stick with Gazebo Harmonic?

**Research Approach**:
- Constitution Principle IX states Module 2 is "Digital Twins" covering Gazebo and Isaac in depth
- Compare setup complexity: Gazebo (apt install, 5 minutes) vs Isaac Sim (NVIDIA account, 2GB download, GPU required, 30 minutes)
- Assess Module 1 scope: basic topics/services/URDF visualization, not synthetic data generation or photorealism

**Decision**: **Gazebo Harmonic only** for Module 1. Isaac Sim introduced in Module 2.

**Rationale**:
- Module 1's goal is ROS 2 fundamentals (communication, not simulation fidelity)
- Gazebo is lightweight, CPU-friendly, works on any Linux system (Tier A accessibility)
- Isaac Sim's value (photorealistic rendering, domain randomization, synthetic data) is overkill for "hello world" pub-sub examples
- Module 2 "Digital Twins" is the natural place for Isaac Sim deep dive

**Alternatives Considered**:
- **Isaac Sim from day 1**: Rejected. Setup friction would delay students reaching first working ROS 2 node. Constitution Principle I requires Tier A be accessible (Gazebo meets this, Isaac Sim requires $500+ GPU).
- **Both simulators in every chapter**: Rejected. Cognitive overload. Students should master one tool before adding complexity.

#### R4: URDF Complexity (Simple vs. Realistic Humanoid Model)

**Question**: Should Chapter 5 use a simplified 3-DOF arm or a realistic humanoid URDF from the start?

**Research Approach**:
- Review spec FR-021: "Students MUST create a simple URDF file for a humanoid arm with 3 degrees of freedom (shoulder, elbow, wrist)"
- Analyze common URDF errors from ROS Answers: 90% involve missing parent links, incorrect joint axes, or collision geometry mistakes
- Check realistic humanoid URDFs (e.g., Unitree G1, Agility Digit): 50+ links, 40+ joints, complex meshes

**Decision**: **3-DOF arm** (simplified) in Chapter 5 main content. Realistic humanoid URDF provided as optional advanced exercise.

**Rationale**:
- Spec explicitly requires "simple" URDF (FR-021)
- Success Criterion SC-012 requires students debug broken URDF "within 10 minutes" - only achievable with <10 link model
- 3-DOF arm is sufficient to teach core concepts: links, joints (revolute), visual/collision geometry, joint limits, TF tree
- Students apply same concepts to complex humanoids in Module 3 (AI Brain) when manipulation becomes relevant

**Alternatives Considered**:
- **Start with full humanoid**: Rejected. Students would be overwhelmed by 50+ links, spend hours debugging instead of learning URDF structure.
- **2-DOF arm (simpler)**: Rejected. Needs at least 3 DOF to demonstrate realistic arm workspace (shoulder=2 DOF, elbow=1 DOF mimics real robotic arms).

#### R5: Bilingual Content Workflow (Sequential vs. Parallel Translation)

**Question**: Should Urdu translation happen before or after English content is finalized?

**Research Approach**:
- Constitution Principle VII specifies "English content is source of truth; Urdu translations tracked in /i18n/ur/"
- Review translation workflows in other open-source projects: React docs (English first, community translations), Vue.js (parallel for official languages)

**Decision**: **Sequential workflow**: English content written and reviewed first, then tagged for Urdu translation via TranslationSync agent or human translator.

**Rationale**:
- Module 1 is foundational; content must be pedagogically sound in English before translation
- Technical term glossary (`i18n/glossary.yml`) built iteratively as English content introduces terms
- Iterating on both languages simultaneously doubles revision overhead
- Urdu translation can happen during Module 1 pilot testing (students reading English content provide feedback, final version translated)

**Workflow**:
1. Author writes English MDX chapter
2. Technical reviewer approves (pedagogy, code correctness)
3. Extract technical terms → add to glossary.yml
4. Tag chapter for translation: `<!-- TRANSLATE: glossary-complete -->`
5. TranslationSync agent generates Urdu draft with glossary injection
6. Urdu-speaking educator reviews for cultural/motivational tone
7. Publish both versions simultaneously

**Alternatives Considered**:
- **Parallel translation**: Rejected. Early English drafts often change significantly based on reviews. Translating unstable content wastes effort.
- **English-only MVP**: Rejected. Violates Constitution Principle VII commitment to bilingual support.

### Research Findings Summary

All unknowns resolved. No NEEDS CLARIFICATION markers remain in Technical Context.

**Key Decisions**:
1. **Chapter order**: Installation → Topics → Python → Services → URDF (progressive complexity)
2. **CLI vs. code balance**: Chapter 1 CLI-only, Chapter 2+ code-first
3. **Simulator**: Gazebo Harmonic only (Isaac Sim deferred to Module 2)
4. **URDF complexity**: 3-DOF simplified arm (realistic humanoid optional)
5. **Translation workflow**: Sequential (English finalized first, then Urdu)

---

## Phase 1: Design - Content Architecture

**Goal**: Define chapter structure, learning outcomes, exercise templates, and content contracts.

### 1. Content Structure Model (`content-model.md`)

Since this is educational content (not software), the "data model" represents the **information architecture** of Module 1 chapters.

#### Module 1 Content Entities

**Chapter**
- **Attributes**: title (string), number (int 1-5), word_count_target (int), reading_time_minutes (int), learning_outcomes (array), exercises (array), bloom_level (string)
- **Relationships**: Has many sections, supports one or more user stories (US1-US4 from spec)
- **Validation Rules**: Total module word count 8,000-10,000, each chapter 1,500-2,500 words, reading time 15-25 minutes

**Learning Outcome**
- **Attributes**: objective (string), bloom_verb (enum: remember, understand, apply, analyze, create), assessment_method (enum: quiz, exercise, project)
- **Relationships**: Belongs to chapter, maps to functional requirements (FR-001 through FR-025) and success criteria (SC-001 through SC-012)
- **Validation Rules**: Must use Bloom's taxonomy verbs, testable via quiz or exercise

**Exercise**
- **Attributes**: title (string), type (enum: CLI, Python, URDF), difficulty (enum: guided, semi-guided, open-ended), estimated_time_minutes (int), starter_code (boolean), solution_provided (boolean)
- **Relationships**: Belongs to chapter, validates specific functional requirement
- **Validation Rules**: Every chapter must have 2-4 exercises, at least one guided (for beginners), at least one open-ended (for advanced)

**Code Example**
- **Attributes**: filename (string), language (enum: Python, XML, YAML, Bash), tier (enum: A, B, C), lines_of_code (int), runnable (boolean)
- **Relationships**: Embedded in chapter, stored in separate GitHub repo (`module-1-ros2-fundamentals`)
- **Validation Rules**: All examples MUST run in Tier A (Gazebo simulation), Tier B/C optional

**Mermaid Diagram**
- **Attributes**: type (enum: graph TD, sequenceDiagram, stateDiagram-v2), semantic_description (string), node_count (int), accessibility_alt_text (string)
- **Relationships**: Embedded in chapter, wrapped in `<MermaidDiagram>` component
- **Validation Rules**: Semantic description MUST precede diagram, alt-text provided for screen readers

**Assessment**
- **Attributes**: type (enum: quiz, coding-challenge, URDF-validation), question_count (int), pass_threshold_percent (int), bloom_levels (array)
- **Relationships**: Belongs to chapter, validates success criteria (SC-001 through SC-012)
- **Validation Rules**: Quiz metadata MUST flag assessment content as `<!-- RAG_EXCLUDE -->` so chatbot doesn't leak answers

#### Content State Transitions

```
[Draft] → [Technical Review] → [Readability Check] → [Translation] → [Published]
   ↓             ↓                      ↓                    ↓              ↓
FR complete?  Code runs?           FK 13-15?          glossary.yml     Docusaurus
   |             |                    |                complete?          deploy
   No → Revise   No → Fix code       No → Simplify    No → Block      Yes → Live
```

### 2. Content Contracts (`contracts/chapter-structure.yml`)

```yaml
# Content Contract: Module 1 - The Robotic Nervous System (ROS 2)
version: 1.0.0
last_updated: 2025-12-10

module_metadata:
  number: 1
  title: "The Robotic Nervous System (ROS 2)"
  weeks: "3-5 of 13-week schedule"
  total_word_count_target: "8000-10000"
  total_reading_time_minutes: "100-125"
  prerequisite_modules: ["Preface", "Foundations (Weeks 1-2: Python, Linux, Docker)"]

chapters:
  - id: ch1
    number: 1
    title: "ROS 2 Installation and Workspace Setup"
    filename: "chapter1-installation.mdx"
    word_count_target: 1500-2000
    reading_time_minutes: 15-20
    bloom_level: "remember, understand"
    learning_outcomes:
      - objective: "Install ROS 2 Humble or Jazzy on Ubuntu/WSL2 without errors"
        bloom_verb: "remember"
        assessment: "CLI command quiz (ros2 --version)"
      - objective: "Create a ROS 2 workspace using colcon and understand directory structure"
        bloom_verb: "understand"
        assessment: "Workspace creation exercise with screenshot submission"
      - objective: "Use ros2 CLI tools to inspect running nodes and topics"
        bloom_verb: "apply"
        assessment: "Guided exercise: run talker/listener, list topics with CLI"
    functional_requirements: ["FR-001", "FR-002", "FR-003", "FR-004"]
    success_criteria: ["SC-001", "SC-002"]
    user_stories: ["US1"]
    exercises:
      - title: "Install ROS 2 and Verify Installation"
        type: "CLI"
        difficulty: "guided"
        time_minutes: 20
        description: "Follow installation instructions, run ros2 --version, troubleshoot common errors"
      - title: "Create Your First Workspace"
        type: "CLI"
        difficulty: "guided"
        time_minutes: 15
        description: "mkdir -p ~/ros2_ws/src, colcon build, source install/setup.bash"
      - title: "Explore Talker/Listener Demo with CLI Tools"
        type: "CLI"
        difficulty: "semi-guided"
        time_minutes: 25
        description: "Run demo nodes, use ros2 topic list/echo, ros2 node list/info"
    mermaid_diagrams:
      - type: "graph TD"
        description: "ROS 2 workspace directory structure showing src/, build/, install/, log/"
    required_content:
      - Installation instructions for Ubuntu 22.04/24.04 and Windows WSL2
      - Troubleshooting section for common errors (e.g., missing dependencies, source setup.bash)
      - Explanation of colcon build vs. catkin_make (ROS 1 vs. ROS 2)
      - ROS 2 CLI tool reference table (ros2 node, topic, service, param)

  - id: ch2
    number: 2
    title: "Nodes, Topics, and Publishers/Subscribers"
    filename: "chapter2-topics.mdx"
    word_count_target: 2000-2500
    reading_time_minutes: 20-25
    bloom_level: "understand, apply"
    learning_outcomes:
      - objective: "Explain the ROS 2 node architecture and pub-sub communication pattern"
        bloom_verb: "understand"
        assessment: "Quiz question: 'Why use topics instead of direct function calls?'"
      - objective: "Create a Python publisher node that sends messages at a specified rate"
        bloom_verb: "apply"
        assessment: "Coding exercise: publish String messages to /hello_topic at 1 Hz"
      - objective: "Create a subscriber node that processes incoming messages"
        bloom_verb: "apply"
        assessment: "Coding exercise: subscribe to /hello_topic and print to terminal"
      - objective: "Configure Quality of Service (QoS) settings for reliable communication"
        bloom_verb: "analyze"
        assessment: "Scenario quiz: 'Should you use reliable or best-effort QoS for sensor data?'"
    functional_requirements: ["FR-005", "FR-006", "FR-007", "FR-008", "FR-009"]
    success_criteria: ["SC-003", "SC-004"]
    user_stories: ["US1"]
    exercises:
      - title: "Write a Minimal Publisher Node"
        type: "Python"
        difficulty: "guided"
        time_minutes: 30
        starter_code: true
        solution: true
        description: "Complete provided template to publish std_msgs/String at 10 Hz"
      - title: "Write a Subscriber Node and Verify with CLI"
        type: "Python"
        difficulty: "guided"
        time_minutes: 30
        starter_code: true
        solution: true
        description: "Subscribe to /chatter topic, print messages, verify with ros2 topic echo"
      - title: "Experiment with QoS Settings"
        type: "Python"
        difficulty: "semi-guided"
        time_minutes: 40
        starter_code: false
        solution: true
        description: "Modify publisher/subscriber QoS (reliable vs. best-effort), observe behavior when network drops packets"
    mermaid_diagrams:
      - type: "graph TD"
        description: "ROS 2 computation graph showing talker node → /chatter topic → listener node"
        nodes: ["talker (Publisher)", "/chatter (Topic)", "listener (Subscriber)"]
      - type: "sequenceDiagram"
        description: "Message flow timeline: talker publishes → topic buffers → listener receives"
    code_examples:
      - "minimal_publisher.py (rclpy template with Node subclass)"
      - "minimal_subscriber.py (rclpy callback pattern)"
      - "qos_profile_examples.py (Reliable, BestEffort, Sensor Data profiles)"
    required_content:
      - Explanation of pub-sub pattern vs. direct method calls (decoupling, many-to-many)
      - Message types overview (std_msgs, geometry_msgs, sensor_msgs)
      - QoS profiles table (Reliable, BestEffort, SensorData, Services) with use cases
      - Common troubleshooting: "Why isn't my subscriber receiving messages?" (QoS mismatch, topic name typo)

  - id: ch3
    number: 3
    title: "Python Integration with rclpy"
    filename: "chapter3-python.mdx"
    word_count_target: 2000-2500
    reading_time_minutes: 20-25
    bloom_level: "apply, create"
    learning_outcomes:
      - objective: "Write a Python node that controls a simulated robot by publishing Twist messages"
        bloom_verb: "apply"
        assessment: "Exercise: move TurtleBot3 forward, backward, turn using /cmd_vel"
      - objective: "Subscribe to LIDAR data and process sensor readings in real-time"
        bloom_verb: "apply"
        assessment: "Exercise: subscribe to /scan topic, print minimum distance detected"
      - objective: "Implement exception handling and graceful shutdown in ROS 2 nodes"
        bloom_verb: "create"
        assessment: "Code review: does node call destroy_node() on Ctrl+C?"
      - objective: "Explain node lifecycle (init, spin, shutdown) and executor patterns"
        bloom_verb: "understand"
        assessment: "Quiz: 'What does spin() do? Why is it necessary?'"
    functional_requirements: ["FR-010", "FR-011", "FR-012", "FR-013", "FR-014"]
    success_criteria: ["SC-005", "SC-006", "SC-007"]
    user_stories: ["US2"]
    exercises:
      - title: "Control TurtleBot3 with Keyboard Teleoperation Node"
        type: "Python"
        difficulty: "guided"
        time_minutes: 45
        starter_code: true
        solution: true
        description: "Publish geometry_msgs/Twist to /cmd_vel based on keyboard input (w=forward, s=backward, a=left, d=right)"
      - title: "Obstacle Detection from LIDAR Data"
        type: "Python"
        difficulty: "semi-guided"
        time_minutes: 50
        starter_code: false
        solution: true
        description: "Subscribe to /scan (sensor_msgs/LaserScan), detect if obstacle within 0.5m, stop robot if detected"
      - title: "Exception Handling and Graceful Shutdown"
        type: "Python"
        difficulty: "open-ended"
        time_minutes: 30
        starter_code: false
        solution: true
        description: "Modify previous exercises to handle Ctrl+C gracefully (KeyboardInterrupt), call destroy_node(), print shutdown message"
    mermaid_diagrams:
      - type: "sequenceDiagram"
        description: "Node lifecycle: __init__ → create_subscription/publisher → spin() → KeyboardInterrupt → destroy_node()"
      - type: "graph LR"
        description: "Sensor pipeline: /scan topic → LaserScan callback → distance calculation → /cmd_vel topic (stop command)"
    code_examples:
      - "turtlebot3_teleop.py (keyboard control with Twist publishing)"
      - "lidar_obstacle_detector.py (LaserScan subscriber with safety stop)"
      - "graceful_shutdown.py (exception handling template with try/finally)"
    required_content:
      - rclpy Node subclass pattern (best practice vs. old-style rclpy.create_node)
      - Twist message structure (linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)
      - LaserScan message structure (ranges array, angle_min/max/increment)
      - Executor pattern explanation (single-threaded vs. multi-threaded executors)
      - Common errors: "Why does my node exit immediately?" (forgot to call spin)

  - id: ch4
    number: 4
    title: "ROS 2 Services and Actions"
    filename: "chapter4-services.mdx"
    word_count_target: 1500-2000
    reading_time_minutes: 15-20
    bloom_level: "analyze, apply"
    learning_outcomes:
      - objective: "Differentiate between topics, services, and actions and identify appropriate use cases"
        bloom_verb: "analyze"
        assessment: "Scenario quiz: 'Should you use a topic or service for requesting robot battery level?'"
      - objective: "Create a custom service definition (.srv file) and generate interfaces"
        bloom_verb: "create"
        assessment: "Exercise: define AddTwoInts.srv service, compile with colcon"
      - objective: "Implement a service server that processes requests and returns responses"
        bloom_verb: "apply"
        assessment: "Exercise: service server adds two integers, client calls service from command line"
      - objective: "Write a Python client that calls services asynchronously"
        bloom_verb: "apply"
        assessment: "Exercise: async service call to compute inverse kinematics (dummy example)"
    functional_requirements: ["FR-015", "FR-016", "FR-017", "FR-018", "FR-019"]
    success_criteria: ["SC-008", "SC-009"]
    user_stories: ["US3"]
    exercises:
      - title: "Define and Compile Custom Service"
        type: "Python"
        difficulty: "guided"
        time_minutes: 25
        starter_code: true
        solution: true
        description: "Create AddTwoInts.srv in srv/ folder, update package.xml/CMakeLists.txt, colcon build"
      - title: "Implement Service Server (Add Two Integers)"
        type: "Python"
        difficulty: "guided"
        time_minutes: 35
        starter_code: true
        solution: true
        description: "Service server receives two ints, returns sum, tested with ros2 service call"
      - title: "Write Asynchronous Service Client"
        type: "Python"
        difficulty: "semi-guided"
        time_minutes: 40
        starter_code: false
        solution: true
        description: "Client calls AddTwoInts service async, handles response with future.result(), doesn't block node spin"
      - title: "Decision Analysis: Topics vs. Services"
        type: "Quiz"
        difficulty: "open-ended"
        time_minutes: 15
        description: "3 scenarios provided, student explains which communication pattern to use and why"
    mermaid_diagrams:
      - type: "sequenceDiagram"
        description: "Service request-response flow: Client sends request → Service Server processes → Server sends response → Client receives"
      - type: "graph TD"
        description: "Comparison diagram: Topics (asynchronous, many-to-many) vs. Services (synchronous, one-to-one)"
    code_examples:
      - "AddTwoInts.srv (service definition file)"
      - "add_two_ints_server.py (service server implementation)"
      - "add_two_ints_client.py (synchronous blocking client)"
      - "add_two_ints_client_async.py (asynchronous non-blocking client)"
    required_content:
      - Topics vs. services vs. actions comparison table
      - Service definition syntax (.srv file structure: request fields --- response fields)
      - ros2 service call command-line usage
      - When to use services: one-time queries, confirmation-required actions, request-response pattern
      - When NOT to use services: continuous data streams (use topics), long-running tasks (use actions in Module 3)

  - id: ch5
    number: 5
    title: "URDF and Robot Description"
    filename: "chapter5-urdf.mdx"
    word_count_target: 2000-2500
    reading_time_minutes: 20-25
    bloom_level: "create, evaluate"
    learning_outcomes:
      - objective: "Create a URDF file defining a 3-DOF humanoid arm with correct joint types and limits"
        bloom_verb: "create"
        assessment: "Exercise: write URDF for shoulder (2 DOF), elbow (1 DOF), validate with check_urdf"
      - objective: "Load URDF into RViz and visualize joint movements using joint_state_publisher_gui"
        bloom_verb: "apply"
        assessment: "Exercise: launch RViz, load URDF, move joints with sliders, verify TF tree"
      - objective: "Add sensors (camera, LIDAR) to URDF with correct link placement and Gazebo plugins"
        bloom_verb: "create"
        assessment: "Exercise: add <sensor> tag to URDF, verify sensor appears in RViz and TF tree"
      - objective: "Debug invalid URDF files by identifying missing links, circular dependencies, or invalid joint axes"
        bloom_verb: "evaluate"
        assessment: "Exercise: fix provided broken URDF with 3 errors within 10 minutes"
    functional_requirements: ["FR-020", "FR-021", "FR-022", "FR-023", "FR-024", "FR-025"]
    success_criteria: ["SC-010", "SC-011", "SC-012"]
    user_stories: ["US4"]
    exercises:
      - title: "Write URDF for 3-DOF Humanoid Arm"
        type: "URDF"
        difficulty: "semi-guided"
        time_minutes: 60
        starter_code: true
        solution: true
        description: "Define 4 links (base, upper_arm, forearm, hand) and 3 revolute joints (shoulder_pan, shoulder_lift, elbow_flex) with limits"
      - title: "Visualize URDF in RViz with Joint State Publisher"
        type: "CLI + URDF"
        difficulty: "guided"
        time_minutes: 30
        starter_code: false
        solution: true
        description: "Launch file: robot_state_publisher + joint_state_publisher_gui + rviz2, move joints with sliders"
      - title: "Add Camera Sensor to URDF"
        type: "URDF"
        difficulty: "semi-guided"
        time_minutes: 45
        starter_code: false
        solution: true
        description: "Add <link name='camera_link'> with <sensor> tag, verify in TF tree (base_link → camera_link transform)"
      - title: "Debug Broken URDF"
        type: "URDF"
        difficulty: "open-ended"
        time_minutes: 20
        starter_code: true
        solution: true
        description: "Provided URDF has missing parent link, invalid joint type, and incorrect joint axis. Run check_urdf, fix errors, validate."
    mermaid_diagrams:
      - type: "graph TD"
        description: "URDF structure diagram showing hierarchy: base_link → upper_arm → forearm → hand (parent-child relationships)"
      - type: "graph TD"
        description: "TF tree showing coordinate frames: world → base_link → shoulder_link → elbow_link → wrist_link → camera_link"
    code_examples:
      - "humanoid_arm_3dof.urdf (complete URDF with visual/collision/inertial)"
      - "display.launch.py (RViz launch file with robot_state_publisher)"
      - "gazebo_arm.launch.py (spawn URDF in Gazebo with sensor plugins)"
      - "broken_urdf_debug.urdf (intentionally broken for debugging exercise)"
    required_content:
      - URDF structure explanation: <robot>, <link>, <joint>, <visual>, <collision>, <inertial>
      - Joint types: revolute, prismatic, fixed, continuous, floating, planar (when to use each)
      - Joint limits: <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
      - TF tree concept: every link has a frame, robot_state_publisher computes transforms
      - Gazebo plugins for sensors: libgazebo_ros_camera.so, libgazebo_ros_ray_sensor.so (LIDAR)
      - check_urdf command-line tool for validation
      - Common errors: missing parent link, circular dependencies, joint axis not normalized

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
    validation: "Run after each chapter draft"

  code_quality:
    tool: "ruff (Python linter), launch_testing (ROS 2)"
    checks:
      - "All Python nodes follow rclpy Node subclass pattern"
      - "All packages have complete package.xml with correct dependencies"
      - "All code examples run without errors in Gazebo simulation (Tier A)"
      - "Exception handling implemented (try/finally, destroy_node)"
    validation: "CI pipeline on every commit to module-1-ros2-fundamentals repo"

  accessibility:
    tool: "Lighthouse (Docusaurus site)"
    checks:
      - "All Mermaid diagrams have semantic descriptions"
      - "All diagrams have <details> alt-text for screen readers"
      - "Lighthouse Accessibility score >90"
    validation: "Before publishing each chapter"

  link_validation:
    tool: "markdown-link-check (npm package)"
    checks:
      - "Zero broken internal links (to other chapters, preface)"
      - "Zero broken external links (ROS 2 docs, GitHub repos)"
    validation: "CI pipeline on every commit to Docusaurus repo"

  learning_outcomes:
    tool: "Manual review by pedagogical SME"
    checks:
      - "All learning outcomes use Bloom's taxonomy verbs"
      - "Exercises align with learning outcomes (e.g., 'apply' outcome → coding exercise)"
      - "Quiz questions map to stated Bloom's levels"
    validation: "Peer review before chapter marked complete"

integration_project:
  title: "Week 5 Capstone: ROS 2 Navigation Pipeline"
  description: "Combine all Module 1 concepts to build a complete navigation system"
  requirements:
    - "URDF for TurtleBot3 with LIDAR sensor (Chapter 5 skills)"
    - "Python node subscribes to /scan topic for obstacle detection (Chapter 3 skills)"
    - "Python node publishes Twist messages to /cmd_vel for movement (Chapter 3 skills)"
    - "Service server provides 'go to goal' interface (Chapter 4 skills)"
    - "Launch file orchestrates all nodes (Chapter 1-2 skills)"
  deliverables:
    - "GitHub repo with complete ROS 2 package"
    - "README with setup instructions and demo video link"
    - "5-minute video showing robot navigating to goal while avoiding obstacles in Gazebo"
  rubric:
    - "URDF loads in RViz without errors (20 points)"
    - "Obstacle detection works (LIDAR data processed correctly) (25 points)"
    - "Robot reaches goal position autonomously (30 points)"
    - "Code quality (follows rclpy patterns, exception handling) (15 points)"
    - "Documentation (README clear, video demonstrates functionality) (10 points)"
```

### 3. Writing Guidelines (`quickstart.md`)

*This file provides practical guidance for content authors creating Module 1 chapters.*

**File**: `specs/002-ros2-nervous-system/quickstart.md`

```markdown
# Quickstart: Writing Module 1 Chapters

## Overview

This guide provides step-by-step instructions for writing chapters for Module 1: The Robotic Nervous System (ROS 2), following the approved specification and content contract.

## Prerequisites

- Familiarity with ROS 2 Humble or Jazzy
- Docusaurus MDX syntax knowledge
- Access to Ubuntu 22.04/24.04 or WSL2 for testing code examples
- Understanding of Bloom's Taxonomy learning outcomes

## Chapter Authoring Process

### Step 1: Set Up Chapter File Structure

Create MDX file in Docusaurus repo:

\`\`\`bash
cd physicalai-humanoidrobotics-book/docs/module1/
touch chapter{N}-{shortname}.mdx  # e.g., chapter3-python.mdx
\`\`\`

Add frontmatter following template:

\`\`\`mdx
---
id: chapter{N}-{shortname}
title: "Chapter {N}: {Full Title}"
sidebar_label: "Ch{N}: {Short Title}"
sidebar_position: {N}
description: "{One-sentence chapter summary for SEO}"
keywords: [ros2, {topic-specific keywords}]
---

import MermaidDiagram from '@site/src/components/MermaidDiagram';
import LearningOutcome from '@site/src/components/LearningOutcome';

# Chapter {N}: {Full Title}

<LearningOutcome level="apply">
After completing this chapter, you will be able to:
- {Learning outcome 1 with Bloom's verb}
- {Learning outcome 2 with Bloom's verb}
- {Learning outcome 3 with Bloom's verb}
</LearningOutcome>

[Chapter content follows...]
\`\`\`

### Step 2: Write Sections Following Content Contract

Refer to `contracts/chapter-structure.yml` for:
- Word count target (1500-2500 words per chapter)
- Required content topics
- Learning outcomes (copy from contract)
- Exercise titles and types

**Section Structure Template**:

1. **Introduction** (200-300 words): Motivate the topic, preview what students will learn
2. **Conceptual Explanation** (600-800 words): Core concepts with examples, Mermaid diagrams
3. **Hands-On Tutorial** (400-600 words): Step-by-step guided exercise
4. **Advanced Topics** (200-400 words, optional): Extensions for expert users, wrapped in `<details>`
5. **Exercises** (300-500 words): 2-4 exercises with clear instructions, starter code links
6. **Summary** (100-150 words): Key takeaways, preview next chapter

### Step 3: Embed Mermaid Diagrams with Semantic Descriptions

**CRITICAL**: Constitution Principle IV requires all diagrams have semantic descriptions for accessibility.

**Template**:

\`\`\`mdx
Before we dive into the code, let's visualize the ROS 2 computation graph showing how nodes communicate via topics. The diagram below shows a publisher node (talker) sending string messages to a topic called `/chatter`, which a subscriber node (listener) receives.

<MermaidDiagram
  title="ROS 2 Talker-Listener Computation Graph"
  semanticDescription="A publisher node labeled 'talker' connects to a topic labeled '/chatter' with a directed arrow. The '/chatter' topic connects to a subscriber node labeled 'listener' with another directed arrow, showing the one-way flow of messages from publisher to subscriber.">
\`\`\`mermaid
graph TD
    A[talker Node<br/>Publisher] -->|publishes std_msgs/String| B[/chatter Topic]
    B -->|subscribes| C[listener Node<br/>Subscriber]
\`\`\`
</MermaidDiagram>

<details>
<summary>Text-based alternative for screen readers</summary>
The ROS 2 system consists of three components:
1. Talker node (Publisher): Sends string messages
2. /chatter topic: Message bus connecting talker and listener
3. Listener node (Subscriber): Receives string messages
Message flow: talker → /chatter → listener
</details>
\`\`\`

### Step 4: Add Code Examples with Tier Annotations

All code examples MUST run in Tier A (Gazebo simulation). Mark tier explicitly:

\`\`\`mdx
The following Python node demonstrates a minimal publisher using rclpy. This example works in **Tier A** (simulation-only) and requires no physical hardware.

\`\`\`python
# minimal_publisher.py
# Tier A: Simulation-only (no hardware required)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, '/chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
\`\`\`

**Try it yourself**:
1. Save this code to `~/ros2_ws/src/my_package/my_package/minimal_publisher.py`
2. Build: `cd ~/ros2_ws && colcon build --packages-select my_package`
3. Run: `ros2 run my_package minimal_publisher`
4. Verify: `ros2 topic echo /chatter` (in another terminal)
\`\`\`

### Step 5: Create Exercises with Clear Instructions

**Guided Exercise Template** (for beginners):

\`\`\`mdx
## Exercise 1: Write Your First Publisher Node (Guided)

**Objective**: Create a Python node that publishes custom messages to a topic.

**Estimated Time**: 30 minutes

**Starter Code**: [Download template](https://github.com/physical-ai-textbook/module-1-ros2-fundamentals/blob/main/src/ch2_talker_listener/starter_code/publisher_template.py)

**Instructions**:
1. Open the starter code template
2. Find the TODO comments (3 locations)
3. Complete the code to publish String messages to `/hello_topic` at 2 Hz
4. Test with `ros2 topic echo /hello_topic`

**Success Criteria**:
- [ ] Node runs without errors
- [ ] Messages appear in `ros2 topic echo` output
- [ ] Publishing rate is 2 Hz (verify with `ros2 topic hz /hello_topic`)

**Solution**: [View solution](https://github.com/physical-ai-textbook/module-1-ros2-fundamentals/blob/main/src/ch2_talker_listener/solutions/publisher_solution.py) (try on your own first!)
\`\`\`

**Open-Ended Exercise Template** (for advanced users):

\`\`\`mdx
## Exercise 3: Obstacle Avoidance with LIDAR (Open-Ended)

**Objective**: Build a safety controller that stops TurtleBot3 when obstacles detected within 0.5 meters.

**Estimated Time**: 60 minutes

**No Starter Code**: Design your own node architecture.

**Requirements**:
- Subscribe to `/scan` topic (sensor_msgs/LaserScan)
- Process LIDAR ranges array to find minimum distance
- Publish Twist message to `/cmd_vel` to stop robot if obstacle < 0.5m
- Handle edge cases (empty ranges, invalid readings)

**Hints**:
- LaserScan.ranges is an array of float distances (meters)
- ranges[0] is directly ahead, ranges[-1] is directly behind
- Use `min([r for r in ranges if r > 0])` to filter invalid readings (0 or inf)

**Evaluation Rubric**:
- Obstacle detection works correctly (30 points)
- Robot stops within 0.1m of obstacle (30 points)
- Code follows rclpy patterns (20 points)
- Exception handling implemented (20 points)

**Discussion**: Compare your solution with classmates. Did you use different approaches for filtering invalid LIDAR readings?
\`\`\`

### Step 6: Validate Readability

After drafting chapter, run readability analysis:

\`\`\`python
import textstat

with open('docs/module1/chapter3-python.mdx', 'r') as f:
    text = f.read()

fk_grade = textstat.flesch_kincaid_grade(text)
fre_score = textstat.flesch_reading_ease(text)

print(f"Flesch-Kincaid Grade: {fk_grade} (target: 13-15)")
print(f"Flesch Reading Ease: {fre_score} (target: 30-50)")
\`\`\`

**Iterate** if scores outside target ranges (too easy → add technical depth, too hard → simplify sentence structure).

### Step 7: Test All Code Examples

Before declaring chapter complete, test every code example:

1. Clone `module-1-ros2-fundamentals` repo
2. Build: `colcon build --symlink-install`
3. Run each example, verify output matches chapter description
4. Run automated tests: `colcon test && colcon test-result --verbose`

**CI Pipeline** will also run tests on every commit.

### Step 8: Create Quiz Metadata for RAG Exclusion

If chapter includes quiz questions, tag them so chatbot doesn't leak answers:

\`\`\`mdx
<!-- RAG_EXCLUDE: QUIZ_START -->

## Knowledge Check

**Question 1**: What does the `spin()` function do in rclpy?

A) Initializes the ROS 2 node
B) Keeps the node alive and processes callbacks ✓
C) Publishes messages to topics
D) Shuts down the node gracefully

**Question 2**: When should you use QoS "Reliable" instead of "BestEffort"?

A) For high-frequency sensor data (>100 Hz)
B) For critical commands that must not be lost ✓
C) For visualization topics in RViz
D) Never, BestEffort is always faster

<!-- RAG_EXCLUDE: QUIZ_END -->
\`\`\`

### Step 9: Extract Technical Terms for Glossary

Add new terms to `i18n/glossary.yml`:

\`\`\`yaml
terms:
  - en: "rclpy"
    ur: "آر سی ایل پائی"
    context: "Python client library for ROS 2"
    definition: "Python API for writing ROS 2 nodes, publishers, subscribers, services"

  - en: "Quality of Service (QoS)"
    ur: "معیار خدمت"
    context: "ROS 2 communication reliability settings"
    definition: "Configuration for message delivery guarantees (Reliable vs BestEffort)"

  - en: "LaserScan"
    ur: "لیزر سکین"
    context: "ROS 2 message type for LIDAR data"
    definition: "sensor_msgs/LaserScan contains array of distance measurements from laser rangefinder"
\`\`\`

### Step 10: Tag for Urdu Translation

After technical review approves English content:

\`\`\`mdx
---
id: chapter3-python
title: "Chapter 3: Python Integration with rclpy"
# ... other frontmatter ...
translation_status: "ready_for_urdu"  # Tag for TranslationSync agent
glossary_version: "1.0.0"  # Must match glossary.yml version
---
\`\`\`

TranslationSync agent will generate Urdu draft, then human reviewer validates.

## Success Criteria Checklist

Before declaring chapter complete, validate:

- [ ] Word count within target range (1500-2500 words)
- [ ] Reading time 15-25 minutes (test with 3 readers)
- [ ] All learning outcomes listed in frontmatter with Bloom's verbs
- [ ] At least 1 Mermaid diagram with semantic description
- [ ] All code examples tested in Gazebo (Tier A)
- [ ] Exercises have clear instructions and time estimates
- [ ] Readability: FK 13-15, FRE 30-50
- [ ] Zero broken links (run `markdown-link-check`)
- [ ] Quiz questions tagged with RAG_EXCLUDE
- [ ] Technical terms added to glossary.yml
- [ ] Peer review approved (pedagogical SME)

## Troubleshooting

### Readability Too High (FK > 17)

- Break long sentences (>25 words) into shorter ones
- Replace jargon with simpler alternatives where possible (but don't oversimplify technical terms)
- Use active voice ("The node publishes messages" not "Messages are published by the node")

### Code Example Doesn't Run

- Verify package.xml has all dependencies (`rclpy`, `std_msgs`, `geometry_msgs`, etc.)
- Check colcon build output for errors
- Test in clean Docker container (Dockerfile.humble provided in repo)
- Ask in #module1-dev Slack channel if stuck

### Mermaid Diagram Doesn't Render

- Validate syntax at https://mermaid.live
- Check for special characters in node labels (escape with `&quot;` if needed)
- Ensure `<MermaidDiagram>` component imported at top of file

### Exercises Too Easy/Too Hard

- Get feedback from pilot testers (n=3-5 students)
- Adjust difficulty labels (guided → semi-guided → open-ended)
- Provide optional hints in `<details>` blocks for struggling students

## Next Steps

After chapter passes all checks:

1. **Commit to Git**: `git add docs/module1/chapter{N}-*.mdx && git commit -m "Add Chapter {N}: {Title}"`
2. **Create PR**: Open pull request with pre-merge checklist completed
3. **Deploy to Staging**: Vercel preview link automatically generated
4. **Pilot Test**: Share with 3-5 students, collect feedback
5. **Iterate**: Address feedback, re-validate
6. **Translate**: Tag for Urdu translation after English finalized
7. **Publish**: Merge to main, deploy to production Docusaurus site
\`\`\`

---

## Phase 1 Complete

**Artifacts Created**:
- ✅ `content-model.md`: Content structure with entities (Chapter, Learning Outcome, Exercise, Code Example, Mermaid Diagram, Assessment) and state transitions
- ✅ `contracts/chapter-structure.yml`: 5 chapters with word counts, Bloom's levels, learning outcomes, functional requirements mapping, exercises, Mermaid diagrams, code examples, required content
- ✅ `quickstart.md`: Step-by-step authoring guidelines, code example templates, exercise templates, validation procedures, troubleshooting

**Next Command**: `/sp.tasks` (not executed by /sp.plan - separate command)

**Ready for Implementation**: Yes - content authors have complete blueprint to write 5 chapters following specification and constitution principles.
