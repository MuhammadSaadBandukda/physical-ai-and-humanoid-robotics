<!--
Sync Impact Report:
- Version Change: 0.0.0 -> 1.0.0 (Initial Ratification)
- Added Principles: Embodied Intelligence, Specification-Driven Development (SDD-RI), "Process as Product", Accessibility & Adaptability
- Added Sections: Tech Stack Standards, Constraints, Governance
- Templates requiring updates:
  - .specify/templates/plan-template.md (Constitution Check section aligns with new principles) ✅
  - .specify/templates/spec-template.md (Success criteria align) ✅
  - .specify/templates/tasks-template.md (Task categorization aligns) ✅
-->

# Textbook for Teaching Physical AI & Humanoid Robotics Course Constitution

## Core Principles

### I. Embodied Intelligence
**Bridging the digital brain and physical body.**
All content and examples must demonstrate the connection between software logic and physical actuation. Code is not abstract; it moves mass. We emphasize the feedback loop between perception, reasoning, and action in the physical world.

### II. Specification-Driven Development (SDD-RI)
**Rigorous adherence to the Specify -> Plan -> Implement workflow.**
Zero "vibe coding". Every chapter, code example, and infrastructure component must be specified and planned before implementation. This ensures pedagogical rigor and technical correctness. We follow the Spec-Kit Plus workflow.

### III. "Process as Product"
**Treating content creation as engineering.**
The textbook is not just written; it is engineered. The creation process itself—research, design decisions, trade-offs—is valuable educational content. We transparently document how we build the course materials.

### IV. Accessibility & Adaptability
**Multi-language and context-aware content.**
The content must be accessible to a global audience (English/Urdu) and adaptable to the user's hardware reality (Digital Twin/Cloud vs. Physical AI Edge/Jetson). We do not assume a single hardware configuration; we explicitly handle the diversity of physical computing environments.

## Tech Stack Standards

### Content & Framework
- **Framework:** Docusaurus (Markdown/MDX format) for static site generation.
- **Deployment:** GitHub Pages.
- **Structure:** 4 Modules covering ROS 2, Gazebo, NVIDIA Isaac, and VLA.

### Engineering Stack
- **Development:** Spec-Kit Plus, Claude Code, OpenAI Agents SDK.
- **Backend:** FastAPI for API services, Neon Serverless Postgres for data, Qdrant Cloud for vector search (RAG).
- **Authentication:** Better-Auth for capturing user hardware context.

## Constraints

### Hardware Context Distinction
Content must strictly distinguish between:
1.  **"Digital Twin" Workstations:** High-compute (RTX), simulation-heavy (Isaac Sim/Gazebo).
2.  **"Physical AI" Edge Kits:** Low-power, real-time (Jetson Orin), hardware-interface focused.

### Interactive Elements
- Embedded RAG chatbot for Q&A.
- Personalization toggles for hardware context.
- Urdu translation buttons/toggles.

### Tone
Academic, rigorous, and instruction-focused. Avoid casual slang; prioritize clarity and technical precision.

## Governance

### Amendment Process
Amendments to this constitution require a documented Spec-Kit workflow (Specify -> Plan -> Update). Changes to Core Principles require a Major version bump.

### Compliance
All Pull Requests must verify compliance with these principles. The "Constitution Check" in plans must explicitly validate against Embodied Intelligence and Accessibility requirements.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07