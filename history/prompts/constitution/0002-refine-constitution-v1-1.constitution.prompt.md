---
id: 0002
title: refine-constitution-v1-1
stage: constitution
date: 2025-12-07
surface: agent
model: gemini
feature: none
branch: master
user: user
command: Review my Constitution at .specify/memory/constitution.md and improve it...
labels: ["constitution", "refinement", "standards"]
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

Review my Constitution at `.specify/memory/constitution.md` and improve it:

1. **Are all standards testable (not vague)?**
   - ❌ Vague: "Maintains an academic, rigorous style"
   - ✅ Testable: "All chapters must start with 3-5 Bloom's Taxonomy Learning Outcomes; All Python code snippets must follow PEP 8 standards; Flesch-Kincaid grade level 13-15 (Undergraduate)."

2. **Did I cover essential categories?**
   - **Hardware Segmentation:** Explicit rules for tagging exercises as "[RTX-Required]" vs "[Jetson-Compatible]"?
   - **Code Integrity:** Requirement that all ROS 2 nodes and URDF examples be syntactically valid?
   - **RAG Accuracy:** Metric for the chatbot (e.g., "Must cite the specific chapter source in every answer")?
   - **Personalization Scope:** Clear definition of the two toggle states (Cloud vs. Local)?
   - **Structure Compliance:** 1:1 mapping enforcement between book chapters and the 13-Week Syllabus?

3. **Are any standards unrealistic?**
   - (e.g., Does the "Translate to Urdu" requirement imply manual verification, or is automated translation acceptable?)

Suggest 2-3 concrete improvements.

## Response snapshot

Updated constitution to v1.1.0.
- **Standards**: Added Bloom's Taxonomy, PEP 8, Flesch-Kincaid requirements.
- **Constraints**: Added explicit tagging `:::note[RTX-Required]`, RAG citation requirements, and 1:1 syllabus mapping.
- **Clarification**: Specified "AI-Assisted" for Urdu translation.

## Outcome

- ✅ Impact: Replaced vague "tone" requirements with testable pedagogical and quality standards.
- 🧪 Tests: N/A
- 📁 Files: Updated .specify/memory/constitution.md
- 🔁 Next prompts: /sp.feature to start defining content.
- 🧠 Reflection: The prompt provided excellent specific examples of what "testable" means in this context, which made the refinement process straightforward.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A