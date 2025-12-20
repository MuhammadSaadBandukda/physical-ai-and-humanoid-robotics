---
id: 0006
title: document-architectural-decisions
stage: plan
date: 2025-12-07
surface: agent
model: gemini
feature: 001-physical-textbook
branch: 001-physical-textbook
user: user
command: Generate full ADR documentation for these four critical choices...
labels: ["adr", "architecture", "decisions"]
links:
  spec: specs/001-physical-textbook/spec.md
  ticket: null
  adr: history/adr/
  pr: null
files:
 - history/adr/0001-ros2-distribution-selection.md
 - history/adr/0002-python-dependency-management.md
 - history/adr/0003-frontend-backend-data-integration.md
 - history/adr/0004-sim-to-real-personalization-strategy.md
tests:
 - null
---

## Prompt

Based on the approved Project Plan and our earlier Specification Clarification, please identify and document the Architectural Decision Records (ADRs)...

## Response snapshot

Created 4 ADRs:
- **ADR-001 (ROS 2 Jazzy)**: Chosen for LTS longevity (2029) and Nav2 features.
- **ADR-002 (Poetry)**: Chosen for strict locking/reproducibility across Sim/Real.
- **ADR-003 (Client-side Fetch)**: Chosen for dynamic RAG/Auth integration in static Docusaurus.
- **ADR-004 (Dual-Reality Toggle)**: Chosen for unified content maintenance and Sim-to-Real pedagogy.

## Outcome

- ✅ Impact: Formalized critical architectural decisions, providing a clear record of "why" for future contributors and ensuring alignment with project goals.
- 🧪 Tests: N/A
- 📁 Files: history/adr/000{1..4}-*.md
- 🔁 Next prompts: /sp.tasks
- 🧠 Reflection: Documenting the "Sim-to-Real" strategy as an ADR (ADR-004) is particularly valuable as it's the core pedagogical differentiator of this project.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A