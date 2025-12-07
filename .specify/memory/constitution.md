<!--
Sync Impact Report:
- Version Change: 1.0.0 -> 1.1.0 (Refinement of Standards)
- Modified Principles: Accessibility & Adaptability (Clarified Urdu translation scope)
- Added Sections: Pedagogical Standards, Quality Assurance
- Removed Sections: Tone (Merged into Pedagogical Standards)
- Templates requiring updates:
  - .specify/templates/plan-template.md (Constitution Check must include "Hardware Tagging" and "Bloom's Check") ⚠
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
The content must be accessible to a global audience (English/AI-Assisted Urdu) and adaptable to the user's hardware reality (Digital Twin/Cloud vs. Physical AI Edge/Jetson). We do not assume a single hardware configuration; we explicitly handle the diversity of physical computing environments.

## Tech Stack Standards

### Content & Framework
- **Framework:** Docusaurus (Markdown/MDX format) for static site generation.
- **Deployment:** GitHub Pages.
- **Structure:** 4 Modules covering ROS 2, Gazebo, NVIDIA Isaac, and VLA. Must map 1:1 to the 13-Week Syllabus.

### Engineering Stack
- **Development:** Spec-Kit Plus, Claude Code, OpenAI Agents SDK.
- **Backend:** FastAPI for API services, Neon Serverless Postgres for data, Qdrant Cloud for vector search (RAG).
- **Authentication:** Better-Auth for capturing user hardware context.

## Pedagogical & Content Standards

### Educational Rigor (Testable)
- **Learning Outcomes:** Every chapter MUST begin with 3-5 distinct Learning Outcomes mapped to Bloom's Taxonomy (e.g., "Analyze", "Create", not just "Understand").
- **Reading Level:** Content target is Undergraduate level (Flesch-Kincaid Grade 13-15).
- **Code Style:** All Python code MUST follow PEP 8 standards. All C++ code MUST follow ROS 2 C++ Style Guide.

### Quality Assurance
- **Code Integrity:** All ROS 2 nodes and URDF/Xacro examples must be syntactically valid and capable of `colcon build` without errors in their target environment.
- **RAG Accuracy:** The embedded chatbot MUST provide a citation/link to the specific textbook chapter source for every generated answer.

## Constraints

### Hardware Context Distinction
Content must strictly distinguish between environments using explicit tagging:
1.  **"Digital Twin" Workstations (Cloud/Local RTX):** High-compute, simulation-heavy (Isaac Sim/Gazebo).
    - *Requirement:* Use explicit admonitions: `:::note[RTX-Required]`.
2.  **"Physical AI" Edge Kits (Jetson):** Low-power, real-time (Jetson Orin), hardware-interface focused.
    - *Requirement:* Use explicit admonitions: `:::note[Jetson-Compatible]`.

### Interactive Elements
- **RAG Chatbot:** Must be context-aware of the current chapter.
- **Personalization:** Toggles must switch code snippets between "Simulation Mode" (Gazebo/Isaac) and "Real World Mode" (Hardware interfaces).
- **Urdu Translation:** AI-Assisted translation is acceptable but must be verified for technical term consistency (e.g., "Node" should remain "Node", not translated literally).

## Governance

### Amendment Process
Amendments to this constitution require a documented Spec-Kit workflow (Specify -> Plan -> Update). Changes to Core Principles require a Major version bump.

### Compliance
All Pull Requests must verify compliance with these principles. The "Constitution Check" in plans must explicitly validate against Embodied Intelligence, Accessibility, and Tagging requirements.

**Version**: 1.1.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
