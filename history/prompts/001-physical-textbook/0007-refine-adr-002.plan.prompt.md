---
id: 0007
title: refine-adr-002
stage: plan
date: 2025-12-07
surface: agent
model: gemini
feature: 001-physical-textbook
branch: 001-physical-textbook
user: user
command: Please Update ADR-002 to explicitly document the "Negative Consequence"...
labels: ["adr", "refinement", "poetry", "ros2"]
links:
  spec: specs/001-physical-textbook/spec.md
  ticket: null
  adr: history/adr/0002-python-dependency-management.md
  pr: null
files:
 - history/adr/0002-python-dependency-management.md
tests:
 - null
---

## Prompt

Please **Update ADR-002** to explicitly document the "Negative Consequence" of using Poetry with ROS 2...

## Response snapshot

Updated ADR-002 to include the risk that `colcon build` does not natively support Poetry's virtual environment encapsulation, requiring mitigation strategies like build scripts or shims.

## Outcome

- ✅ Impact: Accurately reflected the technical complexity of integrating Poetry with ROS 2, preventing future build pipeline surprises.
- 🧪 Tests: N/A
- 📁 Files: history/adr/0002-python-dependency-management.md
- 🔁 Next prompts: /sp.tasks
- 🧠 Reflection: This detail is crucial because "Python in ROS" is notoriously tricky regarding path isolation.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A