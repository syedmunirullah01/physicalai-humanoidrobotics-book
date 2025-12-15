<!--
==================================================================================
SYNC IMPACT REPORT
==================================================================================
Version: 1.0.0 (Initial constitution)
Type: INITIAL
Date: 2025-12-10

Sections Created:
- Core Principles (10 principles covering hardware accessibility, safety, pedagogy, visualization, RAG, personalization, i18n, architecture, content structure, code quality)
- Technical Stack Specifications (Frontend, Backend, Auth, Robotics)
- Development Workflow (Content creation, RAG integration, testing protocols)
- Quality Gates (Safety, accessibility, learning outcomes, reproducibility)
- Governance (Amendment procedures, compliance, versioning)

Template Compatibility:
✅ plan-template.md - Constitution Check gates defined
✅ spec-template.md - Learning outcomes and user scenarios aligned
✅ tasks-template.md - Safety-first and simulation testing tasks supported
✅ phr-template.prompt.md - Compatible with all stages

Follow-up TODOs:
- Create ADR for Vector DB selection (Qdrant vs alternatives) when RAG implementation begins
- Document emergency stop patterns in shared library when motor control code is first introduced
- Establish translation workflow and glossary management process before Urdu content creation
==================================================================================
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Three-Tier Hardware Accessibility

**Principle**: Every learning module MUST provide complete, functional implementations for three hardware tiers to eliminate economic barriers to learning.

- **Tier A (Simulation Only)**: Zero-cost access via NVIDIA Isaac Sim, Gazebo Harmonic, or Unity. All code MUST run in simulation without hardware dependencies.
- **Tier B (Edge AI)**: $700 budget using Jetson Orin Nano/NX + Intel RealSense D435i. Code MUST gracefully detect hardware availability and provide fallback paths.
- **Tier C (Physical Robot)**: Advanced learners with Unitree Go2/G1 or equivalent platforms. Code MUST follow same interfaces as Tier B for portability.

**Rationale**: Educational equity demands that financial resources never block access to knowledge. Simulation-first design also enforces better abstractions and testability.

**Non-negotiable Rules**:
- Chapter code examples MUST include tier detection logic (environment variable or config-based)
- Tier A MUST be fully functional standalone; Tiers B/C are enhancements, not requirements
- Performance benchmarks MUST be provided for each tier where applicable
- Hardware setup instructions MUST include cost breakdowns and vendor-neutral alternatives

### II. Safety-First Physical AI

**Principle**: All motor control, robot actuation, and physical interaction code MUST implement multiple safety layers to prevent harm to humans, robots, and environment.

**Mandatory Safety Patterns**:
- **Dead Man Switch**: Any code controlling motors MUST implement a heartbeat mechanism that stops motion if not actively renewed
- **Emergency Stop (E-Stop)**: Every robotics chapter MUST document E-Stop procedures for both simulation and hardware
- **Danger Admonitions**: Motor control code blocks MUST be preceded by `:::danger` Docusaurus admonitions warning about physical risks
- **Simulation-First Testing**: Physical robot code (Tier C) MUST NOT be introduced until equivalent simulation tests pass in Tier A

**Rationale**: Physical AI systems can cause injury, property damage, or equipment destruction. Educational content has heightened responsibility to instill safety culture from day one.

**Validation Gates**:
- Code review checklist MUST verify E-Stop implementation before merge
- CI/CD pipeline MUST run simulation safety tests (collision detection, joint limits, velocity caps)
- Tier C code MUST include liability disclaimer and safety equipment recommendations

### III. Bloom's Taxonomy Learning Outcomes

**Principle**: Every chapter MUST define explicit learning outcomes following Bloom's Taxonomy cognitive levels, progressing from lower-order to higher-order thinking.

**Required Cognitive Progression**:
1. **Analyze**: Deconstruct existing ROS 2 systems, TF trees, or AI architectures to understand components
2. **Apply**: Use provided tools/libraries to solve guided problems (e.g., implement a navigation stack)
3. **Create**: Design and build novel solutions to open-ended challenges (e.g., custom manipulation pipeline)

**Implementation Requirements**:
- Chapter frontmatter MUST list 2-4 learning outcomes with explicit Bloom's verbs (analyze, apply, create, evaluate)
- Assessments (quizzes, projects) MUST align with stated outcomes using corresponding action verbs
- Capstone project (Module 4) MUST require "Create" and "Evaluate" levels exclusively

**Rationale**: Pedagogical research shows that explicit alignment between outcomes, instruction, and assessment dramatically improves retention and transfer. Bloom's provides testable, measurable framework.

### IV. Visual Intelligence (Mermaid-First Diagrams)

**Principle**: Complex system architectures MUST be represented visually using Mermaid diagrams with semantic descriptions for accessibility.

**Required Diagram Types**:
- **ROS 2 Computation Graphs**: `graph TD` showing nodes, topics, services with data flow directions
- **TF Trees**: `graph TD` showing coordinate frame hierarchies for robot kinematics
- **Control Loop Architectures**: `sequenceDiagram` or `graph LR` for sensor→decision→actuation pipelines
- **State Machines**: `stateDiagram-v2` for behavior trees, navigation modes, manipulation states

**Accessibility Requirements**:
- Every Mermaid diagram MUST have a preceding paragraph describing its purpose and key relationships
- Node labels MUST use semantic naming (e.g., `/camera/depth` not `node_3`)
- Diagrams MUST include a text-based alternative in an expandable `<details>` block for screen readers

**Rationale**: ROS 2 and robotics systems are inherently graph-structured. Visual representations reduce cognitive load and improve debugging. Accessibility ensures compliance with WCAG 2.1 AA standards.

### V. RAG Chatbot Integration

**Principle**: The textbook platform MUST provide an intelligent Q&A assistant using retrieval-augmented generation to support self-paced learning.

**Technical Requirements**:
- **SDK**: OpenAI Agents SDK or ChatKit SDK for structured agent workflows
- **Vector Search**: Qdrant Cloud (free tier) for semantic search over textbook content, code examples, and documentation
- **Relational Storage**: Neon Serverless Postgres for user conversations, feedback, and analytics
- **Selected Text Q&A**: Users MUST be able to highlight any passage and ask questions with that context injected into the prompt

**Behavioral Constraints**:
- Chatbot MUST NOT provide answers to assessment questions (quiz/exam content flagged in metadata)
- Responses MUST cite source chapters with direct links (e.g., "See Chapter 5.3: TF Listeners")
- Chatbot MUST gracefully handle out-of-scope questions (e.g., "I can help with Physical AI topics covered in this textbook. For general ROS 2 questions, see [official docs].")

**Rationale**: Self-paced learners need on-demand support. RAG ensures answers are grounded in textbook content, reducing hallucinations while maintaining pedagogical coherence.

### VI. Personalization Engine

**Principle**: Learning content MUST adapt to individual user backgrounds, hardware access, and pacing preferences to optimize engagement and outcomes.

**User Profile Capture (Signup Flow)**:
- **Software Experience**: Beginner (no ROS 2), Intermediate (ROS 1/basic Python), Expert (production robotics)
- **Hardware Availability**: Tier A only, Tier B available, Tier C available
- **Learning Pace**: Self-paced (async), Cohort-based (13-week schedule), Accelerated (6-week intensive)
- **Goals**: Academic (student), Professional (engineer), Hobbyist (maker)

**Content Adaptations**:
- **Chapter-Level Filtering**: Beginners see additional "Prerequisites" sections; Experts see "Advanced Extensions"
- **Code Examples**: Tier B users see hardware-specific examples highlighted; Tier A users see simulation equivalents
- **Navigation Suggestions**: System recommends next chapter based on quiz performance and stated pace
- **Difficulty Scaling**: Assessments adjust (more scaffolding for beginners, open-ended challenges for experts)

**Rationale**: One-size-fits-all pedagogy has high dropout rates. Adaptive learning systems can improve completion by 30-40% (research: Knewton, 2019).

### VII. Bilingual Support (English/Urdu)

**Principle**: All textbook content MUST be available in both English and Urdu to serve the Pakistani robotics community and demonstrate i18n best practices.

**Implementation Requirements**:
- **Translation Toggle**: Chapter-level language switcher in Docusaurus navbar (persisted to localStorage)
- **RTL Layout**: Urdu content MUST use `dir="rtl"` with appropriate CSS adjustments for text flow
- **Technical Term Glossary**: Maintain `glossary.yml` with English-Urdu mappings for domain terms (e.g., "kinematics" → "حرکیات")
- **Code Comments**: Remain in English (industry standard), but explanatory prose MUST be fully translated

**Translation Workflow**:
- English content is source of truth; Urdu translations tracked in `/i18n/ur/` following Docusaurus i18n conventions
- Machine translation (GPT-4 with custom prompt) as first pass, human review required before publication
- Community contributions via GitHub PRs with translation validation checklist

**Rationale**: Pakistan has 230M people with growing robotics programs (NUST, FAST, PIEAS). Urdu access removes language barriers. i18n architecture demonstrates production web practices.

### VIII. Reusable Intelligence Architecture

**Principle**: Development workflows MUST capture reusable knowledge in structured formats (ADRs, Agent Skills, Subagent Definitions) to accelerate future work and enable cross-chapter learning transfer.

**Knowledge Capture Mechanisms**:
- **Architecture Decision Records (ADRs)**: Documented in `history/adr/` for significant choices (e.g., "ADR-001: Why Qdrant over Pinecone for Vector DB")
- **Claude Code Subagents**: Define specialized agents in `.specify/agents/` for repetitive tasks (e.g., `ros2-node-generator`, `mermaid-validator`)
- **Agent Skills Library**: Reusable tool chains in `.specify/skills/` (e.g., `check-ros2-dependencies`, `validate-tf-tree`)
- **Cross-Chapter Inheritance**: Chapter N can reference/import code from Chapter N-1 with semantic versioning

**Documentation Requirements**:
- ADRs MUST follow template: Context, Decision, Consequences, Alternatives Considered
- Subagents MUST have `README.md` with usage examples and input/output schemas
- Skills MUST include unit tests and version compatibility matrix (ROS 2 Humble vs Jazzy)

**Rationale**: Educational projects often reinvent the wheel. Systematic knowledge capture creates compounding returns, reduces cognitive load, and models professional engineering practices.

### IX. Module-Aligned Content Structure

**Principle**: Textbook chapters MUST align with the 13-week academic schedule, with clear prerequisite chains and modular boundaries for flexible delivery.

**Module Breakdown**:
- **Weeks 1-2 (Foundations)**: Python for robotics, Linux CLI, Docker, Git workflows
- **Module 1 (Weeks 3-5: ROS 2 Fundamentals)**: Nodes, topics, services, actions, parameter server, TF2
- **Module 2 (Weeks 6-7: Digital Twins)**: Gazebo Harmonic, Unity integration, URDF/SDF modeling
- **Module 3 (Weeks 8-10: NVIDIA Isaac)**: Isaac Sim, synthetic data generation, GPU-accelerated simulation
- **Module 4 (Weeks 11-13: VLA & Capstone)**: Vision-language-action models, end-to-end project

**Structural Requirements**:
- Each module MUST have `module-overview.md` with learning objectives and prerequisite checklist
- Chapters within a module MUST be order-independent where possible (parallelizable learning)
- Final week of each module MUST include integration project combining all chapter topics

**Rationale**: Academic calendars demand predictable pacing. Modular structure supports both semester-long courses and self-paced learning. Clear prerequisites reduce frustration.

### X. Code Quality Standards

**Principle**: All code examples, starter templates, and solution repositories MUST meet production-grade quality standards to instill professional habits.

**Mandatory Practices**:
- **Dependency Declaration**: Every ROS 2 package MUST have complete `package.xml` with correct `<depend>` tags and version constraints
- **rclpy Patterns**: Python nodes MUST follow official rclpy patterns (e.g., `Node` subclass, `create_subscription` callbacks, lifecycle management)
- **Link Validation**: CI pipeline MUST check for dead links in Markdown files (tools: `markdown-link-check`)
- **Reproducible Environments**: Every chapter with code MUST provide Docker/devcontainer configuration with pinned dependency versions

**Forbidden Practices**:
- Global variables for ROS 2 nodes (breaks testability)
- Hardcoded file paths (breaks portability across Tiers A/B/C)
- `sudo` commands in setup instructions (security risk, breaks automation)
- Commented-out code in final examples (pollutes learning signal)

**Rationale**: Students learn by imitation. Low-quality examples teach bad habits that are expensive to unlearn in industry. Code quality is inseparable from pedagogical quality.

## Technical Stack Specifications

### Frontend (Docusaurus Platform)

**Framework**: Docusaurus 3.x (React-based static site generator)
**Language**: TypeScript for custom components, MDX for content
**Styling**: Tailwind CSS for custom components, CSS Modules for legacy overrides
**Deployment**: GitHub Pages (primary), Vercel (staging/preview)
**i18n**: Built-in Docusaurus i18n with English (default) and Urdu locales
**Search**: Algolia DocSearch (free tier for open-source projects)

**Custom Components Required**:
- `<TierSelector>`: Filters code examples by hardware tier
- `<MermaidDiagram>`: Wrapper for mermaid diagrams with semantic descriptions
- `<ROSPlayground>`: Embedded ROS 2 node simulator (Tier A)
- `<LearningOutcome>`: Styled callout for chapter objectives with Bloom's level badges

**Performance Budgets**:
- First Contentful Paint: <1.5s (3G connection)
- Time to Interactive: <3.0s
- Lighthouse Score: >90 (Performance, Accessibility, Best Practices, SEO)

### Backend (RAG & API Services)

**Language**: Python 3.12+ (type hints mandatory)
**Framework**: FastAPI with Pydantic v2 for request/response validation
**Vector Database**: Qdrant Cloud (Free tier: 1GB storage, 1M vectors)
**Relational Database**: Neon Serverless Postgres (Free tier: 512MB, 10GB storage)
**AI SDK**: OpenAI Agents SDK (structured workflows) or ChatKit SDK (if lighter footprint needed)
**Embeddings**: OpenAI `text-embedding-3-small` (1536 dimensions, $0.02/1M tokens)

**API Endpoints** (contract examples):
```
POST /api/v1/chat/query
  Request: { "question": str, "context": str (optional selected text), "user_id": str }
  Response: { "answer": str, "sources": [{ "chapter": str, "url": str }], "confidence": float }

GET /api/v1/user/profile
  Response: { "experience": enum, "tier": enum, "pace": enum, "progress": { "chapters_completed": int[] } }

POST /api/v1/analytics/feedback
  Request: { "chapter_id": str, "rating": int (1-5), "comment": str (optional) }
```

**Infrastructure**:
- **Hosting**: Railway.app or Fly.io (free tier sufficient for MVP)
- **Caching**: Redis Cloud (free 30MB) for rate limiting and session management
- **Monitoring**: Sentry (error tracking), Grafana Cloud (metrics)

### Authentication (Better-Auth)

**Framework**: Better-Auth (modern alternative to NextAuth/Passport)
**Strategy**: Email/password with JWT tokens (httpOnly cookies)
**Session Storage**: Neon Postgres (shares DB with main backend)

**User Signup Flow**:
1. Email verification (via Resend.com or SendGrid free tier)
2. Background questionnaire (modal after email confirmed):
   - Software experience (radio: Beginner / Intermediate / Expert)
   - Hardware access (checkboxes: Tier A / Tier B / Tier C)
   - Learning pace (radio: Self-paced / Cohort / Accelerated)
   - Goals (checkboxes: Academic / Professional / Hobbyist)
3. Profile stored in `users` table with JSONB column for questionnaire data

**Security Requirements**:
- Passwords hashed with Argon2id (Better-Auth default)
- Rate limiting: 5 failed login attempts → 15min lockout
- HTTPS-only in production (enforced via middleware)

### Robotics Code Stack

**ROS Version**: ROS 2 Humble (Ubuntu 22.04 LTS) and Jazzy (Ubuntu 24.04 LTS)
**Python Interface**: `rclpy` (Python 3.10+ for Humble, 3.12+ for Jazzy)
**Message Types**: Standard `std_msgs`, `geometry_msgs`, `sensor_msgs`, custom messages in `textbook_msgs` package
**Build System**: `colcon` with `ament_cmake` (C++) and `ament_python` (Python)

**Simulation Platforms**:
- **NVIDIA Isaac Sim** (Primary): GPU-accelerated, photorealistic rendering, synthetic data generation
  - License: Free for educational use
  - System Requirements: RTX 2060+ or cloud (AWS G4dn instances)
- **Gazebo Harmonic** (Alternative): Open-source, lighter weight, CPU-friendly
  - License: Apache 2.0
  - System Requirements: Any Linux system with OpenGL 3.3+
- **Unity with ROS 2** (Visualization): For students with Unity experience
  - License: Unity Personal (free, revenue <$100k/year)
  - Integration: `unity-robotics-hub` package

**Hardware Platforms** (Tier B/C):
- **Jetson Orin Nano** (8GB): $499, runs ROS 2 Humble, supports Isaac ROS
- **Jetson Orin NX** (16GB): $699, higher compute for VLA models
- **Intel RealSense D435i**: $329, depth camera + IMU, official ROS 2 drivers
- **Unitree Go2** (Tier C): $1,600, quadruped with ROS 2 SDK
- **Unitree G1** (Tier C): $16,000, humanoid with open API (aspirational, not required)

**Code Distribution**:
- GitHub organization: `physical-ai-textbook`
- Repositories per module: `module-1-ros2-fundamentals`, `module-2-digital-twin`, etc.
- Each repo includes: `src/` (ROS 2 packages), `docker/` (Dockerfiles), `docs/` (API docs), `tests/` (pytest + gtest)

## Development Workflow

### Content Creation Process

1. **Spec Phase**: Author drafts chapter outline with learning outcomes, user scenarios (student personas), assessments
2. **Plan Phase**: Technical review by robotics SME, architecture decisions documented (e.g., which Isaac Sim features to cover)
3. **Tasks Phase**: Break chapter into:
   - Tier A implementation (simulation code)
   - Tier B/C extensions (hardware code, if applicable)
   - Mermaid diagrams (ROS graph, TF tree, etc.)
   - Bilingual content (English written first, Urdu translation tagged)
   - Assessments (quiz in Docusaurus Quiz plugin, project rubric)
4. **Implementation**: Agent-assisted or manual coding, following code quality standards (Principle X)
5. **Review**: Peer review checklist (safety, accessibility, learning outcomes alignment)
6. **Merge**: ADR created for significant decisions, PHR logged for agent interactions

### RAG Integration Process

**Initial Setup**:
1. **Content Ingestion**: CI pipeline on `main` branch runs `qdrant-ingest.py` to:
   - Parse all MDX files into chunks (1000 tokens, 200 overlap)
   - Generate embeddings via OpenAI API
   - Upsert to Qdrant collection `textbook-content-v1` with metadata: `chapter_id`, `tier`, `language`, `content_type` (prose/code/diagram)
2. **Schema Sync**: Postgres migrations create tables for `conversations`, `user_feedback`, `chat_messages`

**Query Flow**:
1. User submits question → Frontend calls `/api/v1/chat/query`
2. Backend embeds question → Qdrant vector search (top 5 chunks, filtered by user's tier and language)
3. OpenAI Agents SDK call with:
   - System prompt: "You are a teaching assistant for a Physical AI textbook..."
   - Context: Retrieved chunks + selected text (if any)
   - User query
4. Response streamed to frontend, sources rendered as citations
5. User feedback (thumbs up/down) stored in Postgres for eval

**Update Workflow**:
- When chapter updated → CI re-ingests changed files only (based on git diff)
- Version control via `content_version` field in Qdrant (allows rollback if ingestion corrupted)

### Testing Protocols

**Safety Testing** (Principle II):
- Tier A: Gazebo/Isaac collision tests (MUST pass before Tier B/C code introduced)
- Tier B: Dry-run on Jetson (motors disabled) to validate sensor pipelines
- Tier C: Supervised testing with E-Stop in reach, video recording required

**Learning Outcomes Testing** (Principle III):
- Quiz questions mapped to Bloom's levels in `quiz-meta.json`
- Automated grading for MC/fill-in-blank, rubric for projects
- Item response theory analysis (if >100 students) to validate question difficulty

**Accessibility Testing** (Principle IV):
- `axe-core` Lighthouse audits in CI (WCAG 2.1 AA compliance)
- Manual screen reader testing (NVDA on Windows, Orca on Linux)
- Mermaid diagram alt-text validated against semantic description

**Code Quality Testing** (Principle X):
- `ruff` linting (Python), `clang-tidy` (C++)
- Unit tests: `pytest` (Python), `gtest` (C++)
- Integration tests: ROS 2 launch tests with `launch_testing`
- Link validation: `markdown-link-check` on all `.md` and `.mdx` files

## Quality Gates

### Pre-Merge Checklist (All Chapters)

- [ ] Learning outcomes defined with Bloom's taxonomy levels
- [ ] All three tiers (A/B/C) have functional code paths (or N/A documented)
- [ ] Safety mechanisms implemented if code controls motors/actuators
- [ ] Mermaid diagrams include semantic descriptions
- [ ] English content complete, Urdu translation tagged for follow-up (or completed)
- [ ] Code examples pass linting, unit tests, and link validation
- [ ] Accessibility audit passes (Lighthouse score >90)
- [ ] ADR created if new technology introduced or architecture changed

### Pre-Release Checklist (Per Module)

- [ ] Integration project combines all chapters in module
- [ ] RAG chatbot tested with module-specific questions (accuracy >80% on test set)
- [ ] Docker images built and pushed to GitHub Container Registry
- [ ] Personalization logic tested with all user profile combinations
- [ ] Student focus group feedback collected (n≥5) and incorporated

### Continuous Validation

- **Daily**: CI runs all tests on `main` branch, deploys staging site to Vercel
- **Weekly**: Community office hours to gather feedback, triage GitHub issues
- **Per-Module**: External robotics educator review (checklist: pedagogical soundness, technical accuracy)
- **Semester**: Learning analytics review (completion rates, quiz performance, chatbot usage)

## Governance

### Amendment Procedure

**Proposing Changes**:
1. Open GitHub issue with label `constitution-amendment` describing proposed change and rationale
2. Tag relevant stakeholders (content authors, technical leads, accessibility SMEs)
3. Discussion period: 7 days minimum for MINOR/PATCH changes, 14 days for MAJOR changes

**Approval Process**:
- **PATCH** (typos, clarifications): Single maintainer approval
- **MINOR** (new principle, expanded guidance): Two maintainer approvals + pedagogical SME review
- **MAJOR** (breaking change, principle removal): Unanimous maintainer approval + community RFC (request for comments)

**Propagation**:
- Author of amendment MUST update dependent templates (plan, spec, tasks) and file list in Sync Impact Report
- ADR MUST be created documenting decision rationale (e.g., "ADR-005: Why We Removed Principle XI")

### Versioning Policy

**Semantic Versioning**: `MAJOR.MINOR.PATCH`
**Examples**:
- `1.0.0` → `1.0.1`: Fixed typo in Principle III rationale (PATCH)
- `1.0.1` → `1.1.0`: Added Principle XI: "Collaborative Learning Features" (MINOR)
- `1.1.0` → `2.0.0`: Removed Tier C (physical robots) due to safety concerns (MAJOR)

**Backward Compatibility**:
- MAJOR bumps MAY require existing chapters to be updated (migration guide required)
- MINOR bumps MUST be backward compatible (new principles additive, not replacing)
- PATCH bumps MUST NOT change meaning of existing principles

### Compliance Review

**Enforcement**:
- PR template includes constitution compliance checklist (subset based on PR scope)
- Automated checks in CI: code quality (Principle X), link validation (Principle X), safety patterns (Principle II)
- Human review required for: learning outcomes (Principle III), safety mechanisms (Principle II), accessibility (Principle IV)

**Exceptions**:
- If constitution compliance is impossible for valid reason, author MUST:
  1. Document exception in PR description with justification
  2. Add entry to `docs/constitution-exceptions.md` with expiration date (if temporary)
  3. Get explicit approval from two maintainers

**Audit Trail**:
- PHRs (Prompt History Records) capture all agent-assisted work with constitution compliance notes
- ADRs document significant deviations or interpretations of principles
- Quarterly constitution review: check if principles still serve project goals, propose amendments if misaligned

---

**Version**: 1.0.0 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-10
