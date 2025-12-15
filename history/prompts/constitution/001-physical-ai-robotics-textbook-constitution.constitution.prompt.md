---
id: 001
title: Physical AI Robotics Textbook Constitution
stage: constitution
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: system
command: /sp.constitution
labels: ["constitution", "initial", "robotics", "education", "physical-ai"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

Physical AI & Humanoid Robotics Textbook

## 🎯 OBJECTIVE
Generate a comprehensive `Constitution.md` file for the "Physical AI & Humanoid Robotics"
textbook project that will be used with Spec-Kit Plus and Claude Code. This constitution
must govern all content creation, RAG chatbot development, and bonus features implementation.

### Technical Stack
- **Frontend**: Docusaurus (React/MDX)
- **Deployment**: GitHub Pages / Vercel
- **RAG Backend**: FastAPI + OpenAI Agents SDK
- **Vector DB**: Qdrant Cloud (Free Tier)
- **Relational DB**: Neon Serverless Postgres
- **Auth**: Better-Auth
- **Robotics**: ROS 2 (Humble/Jazzy), NVIDIA Isaac Sim, Gazebo, Unity
- **AI Agent Framework**: Claude Code with Spec-Kit Plus

---

## 📝 CONSTITUTION STRUCTURE TO GENERATE

### Section 1: SYNC IMPACT REPORT (Header Comment)
Generate an HTML comment block containing:
- Version: 1.0.0
- Type: INITIAL
- Sections created list
- Template compatibility status
- Follow-up TODOs

### Section 2: CORE PRINCIPLES (Minimum 10 Principles)

Generate these mandatory principles with full rationale:

#### Principle I: Three-Tier Hardware Accessibility
- **Tier A**: Simulation Only (Isaac Sim/Gazebo) - Zero Cost
- **Tier B**: Edge AI (Jetson Orin Nano/NX) - $700 budget
- **Tier C**: Physical Robot (Unitree Go2/G1) - Advanced

#### Principle II: Safety-First Physical AI
- Dead Man Switch patterns
- Emergency stop mechanisms
- :::danger admonitions for motor control code
- Simulation-first testing mandate

#### Principle III: Bloom's Taxonomy Learning Outcomes
- Every chapter: Analyze → Apply → Create
- Clear cognitive progression
- Assessment alignment

#### Principle IV: Visual Intelligence (Mermaid-First)
- ROS 2 Computation Graphs
- TF Trees
- Control Loop Architectures
- State Machines
- Semantic descriptions for accessibility

#### Principle V: RAG Chatbot Integration
- OpenAI Agents SDK patterns
- Selected text Q&A feature
- Context-aware responses
- Qdrant vector search optimization

#### Principle VI: Personalization Engine
- User background capture at signup
- Software experience level (Beginner/Intermediate/Expert)
- Hardware availability (Tier A/B/C)
- Learning pace preference
- Chapter-level content adaptation

#### Principle VII: Bilingual Support (English/Urdu)
- Translation toggle per chapter
- RTL layout support for Urdu
- Technical term glossary (English-Urdu)
- Code comments remain in English

#### Principle VIII: Reusable Intelligence Architecture
- Claude Code Subagents definition
- Agent Skills library
- ADR documentation in history/adr/
- Cross-chapter knowledge inheritance

#### Principle IX: Module-Aligned Content Structure
- Module 1: ROS 2 (Weeks 3-5)
- Module 2: Digital Twin - Gazebo/Unity (Weeks 6-7)
- Module 3: NVIDIA Isaac (Weeks 8-10)
- Module 4: VLA & Capstone (Weeks 11-13)
- Weeks 1-2: Foundations

#### Principle X: Code Quality Standards
- package.xml dependencies mandatory
- rclpy patterns documented
- No dead links (validated)
- Reproducible environments (Docker/devcontainer)

### Section 3: TECHNICAL STACK SPECIFICATIONS

Generate detailed specifications for:

```yaml
Frontend:
  Framework: Docusaurus 3.x
  Language: TypeScript/MDX
  Styling: Tailwind CSS / CSS Modules
  Deployment: GitHub Pages (primary), Vercel (backup)

Backend_RAG:
  Language: Python 3.12+
  Framework: FastAPI
  Vector_DB: Qdrant Cloud Free Tier
  Relational_DB: Neon Serverless Postgres
  AI_SDK: OpenAI Agents SDK / ChatKit SDK

Authentication:
  Framework: Better-Auth
  Features:
    - Email/Password signup
    - Background questionnaire
    - Session management
    - User profile storage

Robotics_Code:
  ROS_Version: ROS 2 Humble/Jazzy
  Python_Interface: rclpy
  Simulation:
    - NVIDIA Isaac Sim (primary)
    - Gazebo Harmonic (alternative)
    - Unity (visualization)
  Hardware:
    - Jetson Orin Nano/NX
    - Intel RealSense D435i
    - Unitree Go2/G1
```

## Response snapshot

Created comprehensive constitution (v1.0.0) for Physical AI & Humanoid Robotics textbook project with:

**10 Core Principles**:
1. Three-Tier Hardware Accessibility (Sim/Edge/Physical)
2. Safety-First Physical AI (E-Stop, Dead Man Switch, simulation testing)
3. Bloom's Taxonomy Learning Outcomes (Analyze→Apply→Create)
4. Visual Intelligence (Mermaid diagrams with accessibility)
5. RAG Chatbot Integration (Qdrant + OpenAI Agents SDK)
6. Personalization Engine (experience/tier/pace adaptive content)
7. Bilingual Support (English/Urdu with RTL)
8. Reusable Intelligence Architecture (ADRs, Subagents, Skills)
9. Module-Aligned Content Structure (13-week academic schedule)
10. Code Quality Standards (rclpy patterns, Docker, link validation)

**Technical Stack Specifications**:
- Frontend: Docusaurus 3.x with TypeScript/MDX, custom components (<TierSelector>, <MermaidDiagram>, <ROSPlayground>)
- Backend: FastAPI + Qdrant Cloud + Neon Postgres + OpenAI Agents SDK
- Auth: Better-Auth with profile questionnaire flow
- Robotics: ROS 2 Humble/Jazzy, Isaac Sim/Gazebo/Unity, Jetson Orin, RealSense D435i

**Development Workflow**: Spec→Plan→Tasks→Implementation→Review→Merge with ADR/PHR capture

**Quality Gates**: Pre-merge checklist (safety, accessibility, learning outcomes), pre-release validation, continuous CI/CD

**Governance**: Amendment procedures (PATCH/MINOR/MAJOR), semantic versioning, compliance enforcement

## Outcome

- ✅ Impact: Initial constitution established for entire textbook project governance
- 🧪 Tests: Template compatibility validated (plan/spec/tasks templates aligned)
- 📁 Files: `.specify/memory/constitution.md` (445 lines)
- 🔁 Next prompts: `/sp.specify` for first chapter spec, `/sp.plan` for chapter architecture, `/sp.adr` for Vector DB decision
- 🧠 Reflection: Constitution balances educational equity (Tier A accessibility), safety (Physical AI risks), pedagogical rigor (Bloom's taxonomy), and production-grade practices (code quality, i18n, RAG)

## Evaluation notes (flywheel)

- Failure modes observed: None - initial constitution creation
- Graders run and results (PASS/FAIL): Template compatibility checks PASS, no placeholder tokens remaining PASS
- Prompt variant (if applicable): Standard /sp.constitution command execution
- Next experiment (smallest change to try): Create first feature spec for "Module 1 Chapter 1: ROS 2 Nodes & Topics" to validate spec template alignment with Principle III (Bloom's taxonomy)
