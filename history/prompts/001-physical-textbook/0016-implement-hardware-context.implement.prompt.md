---
id: 0016
title: "Implement Hardware Context"
stage: implement
date: 2025-12-07
surface: agent
model: gemini-2.0-flash-exp
feature: 001-physical-textbook
branch: 001-physical-textbook
user: User
command: none
labels: [frontend, react, context, state-management]
links:
  spec: specs/001-physical-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - frontend/src/context/HardwareContext.tsx
 - specs/001-physical-textbook/tasks.md
tests:
 - none (code generation only)
---

## Prompt

**Approval to Proceed: Task T012**

Yes, proceed to **Task T012 (Hardware Context)**.
This is a critical architectural component.

**Action:**
1.  Create `frontend/src/context/HardwareContext.tsx`.
2.  Implement the `HardwareProvider` with the following logic:
    * **State:** `mode` ('sim' | 'physical').
    * **Default:** 'sim' (Safer for beginners).
    * **Persistence:** Use `localStorage` to remember the user's choice.
3.  **Output:** Show me the full code of `HardwareContext.tsx` once created.

**Constraint:**
Do NOT attempt to wrap the application in `docusaurus.config.ts` yet. We will do that in the next atomic task (T013) to ensure we don't break the build.

## Response snapshot

Implemented HardwareContext in frontend/src/context/HardwareContext.tsx using React Context API and localStorage.
Updated tasks.md to mark T012 as complete.
Committed changes to git.

## Outcome

- ✅ Impact: Core state management for Dual-Reality feature established.
- 🧪 Tests: None (unit tests deferred).
- 📁 Files: frontend/src/context/HardwareContext.tsx
- 🔁 Next prompts: T013 - Hardware Toggle UI.
- 🧠 Reflection: Keeping the task atomic (just the context, no wiring) reduces risk of build breakage.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
