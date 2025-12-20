---
id: 0013
title: "Docker Setup Failed"
stage: implement
date: 2025-12-07
surface: agent
model: gemini-2.0-flash-exp
feature: 001-physical-textbook
branch: 001-physical-textbook
user: User
command: none
labels: [docker, infrastructure, failure]
links:
  spec: specs/001-physical-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - infrastructure/Dockerfile.sim
 - infrastructure/Dockerfile.physical
tests:
 - docker build (FAILED - Not Found)
---

## Prompt

**Approval to Proceed (With Caution)**

I acknowledge the T001 local verification failure (Python 3.14/Numpy).
You are correct: **Task T002 (Docker)** is the fix for this, as it forces a standard Python 3.10 environment.

**Action:**
Execute **Task T002 (Docker Infrastructure)**.
1.  Create `infrastructure/Dockerfile.sim` (amd64/RTX).
2.  Create `infrastructure/Dockerfile.physical` (arm64/Jetson).
3.  **CRITICAL FIX:** Ensure the Dockerfiles copy the `pyproject.toml` but do NOT rely on the host's broken `poetry.lock`. They must generate a fresh lock inside the container if the host one is invalid.

**Checkpoint:**
Build the **Simulated (amd64)** container only.
**STOP** and show me the successful build logs.

## Response snapshot

Created Dockerfiles (Sim/Physical) as requested, ensuring fresh Poetry lock generation.
Attempted to build, but `docker` command is not found on the host system.
Committed Dockerfiles to git.

## Outcome

- ✅ Impact: Dockerfiles created.
- ❌ Tests: Build failed (Docker missing).
- 📁 Files: infrastructure/Dockerfile.sim, infrastructure/Dockerfile.physical
- 🔁 Next prompts: Ask user to install Docker or provide path.
- 🧠 Reflection: Dependency on host tools (Docker) is a hard blocker.

## Evaluation notes (flywheel)

- Failure modes observed: Missing tool (Docker).
- Graders run and results (FAIL): Verification failed.
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
