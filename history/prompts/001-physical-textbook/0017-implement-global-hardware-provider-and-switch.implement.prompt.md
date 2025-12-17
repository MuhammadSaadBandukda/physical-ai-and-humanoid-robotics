---
id: 0017
title: "Implement Global HardwareProvider and Switch"
stage: implement
date: 2025-12-07
surface: agent
model: gemini-2.0-flash-exp
feature: 001-physical-textbook
branch: 001-physical-textbook
user: User
command: none
labels: [frontend, docusaurus, react, context, ui, state-management]
links:
  spec: specs/001-physical-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - frontend/src/theme/Root.tsx
 - frontend/src/components/HardwareSwitch/index.tsx
 - frontend/src/components/HardwareSwitch/styles.module.css
 - frontend/src/pages/index.tsx
 - specs/001-physical-textbook/tasks.md
tests:
 - manual UI verification
---

## Prompt

**Execute Task T013: Global Integration & UI**

We will now connect the "Brain" (Context) to the "Body" (Docusaurus).

**Action:**
1.  **Global Wrapper:**
    * Create `frontend/src/theme/Root.tsx`.
    * *Logic:* Import `HardwareProvider` and wrap the `{children}`. This ensures the Context is available on *every* page (Docs, Blog, Home).

2.  **Toggle Component:**
    * Create `frontend/src/components/HardwareSwitch/index.tsx`.
    * *UI:* A simple Button that displays "Current Mode: [Sim/Physical]".
    * *Interaction:* Clicking it toggles the mode using the `useHardware()` hook.
    * *Visuals:* Use Green for 'Sim' (Safe) and Orange/Red for 'Physical' (Warning).

3.  **Verification (The Checkpoint):**
    * Import and place `<HardwareSwitch />` inside `frontend/src/pages/index.tsx` (The Homepage header or main body).
    * **Goal:** Prove that clicking the button persists the state across page reloads (F5).

**Output:**
Show me the code for `Root.tsx` and `HardwareSwitch/index.tsx`.

## Response snapshot

Implemented the global `HardwareProvider` by creating `frontend/src/theme/Root.tsx`.
Created the `HardwareSwitch` UI component in `frontend/src/components/HardwareSwitch/index.tsx` along with its CSS module.
Integrated the `HardwareSwitch` component into `frontend/src/pages/index.tsx` for verification.
Updated tasks.md to mark T013 as complete and committed changes.

## Outcome

- ✅ Impact: Hardware context is now globally available and a UI toggle exists.
- 🧪 Tests: Manual UI verification is required to confirm persistence.
- 📁 Files: New files created and existing one modified.
- 🔁 Next prompts: T014a - Base Admonition Wrapper Component.
- 🧠 Reflection: This atomic task covers the full loop of context, component, and initial integration.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
