---
id: 0002
title: refine-spec-non-goals
stage: spec
date: 2025-12-07
surface: agent
model: gemini
feature: 001-physical-textbook
branch: 001-physical-textbook
user: user
command: Review the generated specification ... and perform a final audit...
labels: ["spec", "audit", "non-goals"]
links:
  spec: specs/001-physical-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-physical-textbook/spec.md
tests:
 - null
---

## Prompt

Review the generated specification at specs/001-physical-textbook/spec.md and perform a final audit based on the checklist below:

**ACTION REQUIRED:**
The specification is currently missing an explicit **Non-Goals** section. Add a comprehensive Non-Goals section based on the following checklist items to prevent scope creep:

**Non-Goals Checklist (Do NOT Specify):**
- [ ] Live Hardware Deployment (The capstone is simulated or uses Jetson Edge, not full deployment to an expensive commercial humanoid).
- [ ] Advanced Control Theory (Focus remains on bipedal locomotion basics and kinematics, not research-level control algorithms).
- [ ] Financial Costs/Business Models (The book is purely technical and educational, not a market analysis).
- [ ] Ethics and Bias of LLMs (Scope is limited to VLA functional integration, not philosophical discussion).

**Final Verification Check:**
After adding Non-Goals, re-confirm that the entire specification adheres to the rule: "No 'how' leaked in (describes what, not how to build)."

## Response snapshot

Added "Non-Goals" section to `specs/001-physical-textbook/spec.md` covering Live Hardware, Advanced Control Theory, Financials, and Ethics. Verified the rest of the spec focuses on "what" rather than "how".

## Outcome

- ✅ Impact: Clarified project boundaries to prevent scope creep.
- 🧪 Tests: N/A
- 📁 Files: Updated specs/001-physical-textbook/spec.md
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: The prompt correctly identified a missing standard section (Non-Goals) which is critical for educational projects where scope can easily explode.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A