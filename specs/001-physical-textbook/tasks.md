---

description: "5-Day Sprint Task List for Physical AI Textbook"
---

# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Implementation Plan (`specs/001-physical-textbook/plan.md`)
**Prerequisites**: ADRs 0001-0004 approved.

## Phase 1: Day 1 - Infrastructure & Docker (Foundational)

**Goal**: Establish the "Dual-Reality" build environment (Simulated x86 & Physical ARM64).
**Risks**: ROS 2 Jazzy on ARM64 build times; Poetry+Colcon integration.

- [ ] T001 [P] Create `infrastructure/docker/dev.amd64.Dockerfile` for ROS 2 Jazzy (Simulated)
- [ ] T002 [P] Create `infrastructure/docker/dev.arm64.Dockerfile` for ROS 2 Jazzy (Jetson/Physical)
- [ ] T003 Setup `backend/pyproject.toml` with Poetry dependencies (FastAPI, LangChain, Qdrant)
- [ ] T004 **[CRITICAL]** Implement `infrastructure/scripts/colcon_poetry_build.sh` shim to allow Colcon to build inside Poetry env (ADR-002)
- [ ] T005 Create `docker-compose.yml` orchestrating Backend, Qdrant, Neon (mock), and Docusaurus dev server

**Checkpoint**: `docker-compose up` launches a working ROS 2 environment and Python backend on both x86 and ARM64.

---

## Phase 2: Day 2 - Backend Core & RAG (Functional)

**Goal**: Functional API for Auth and RAG.

- [ ] T006 Initialize FastAPI app structure in `backend/src/api`
- [ ] T007 Implement Better-Auth integration with Postgres (Neon) for User/HardwareProfile entities
- [ ] T008 Implement Qdrant client connection and collection initialization
- [ ] T009 Create RAG Ingestion Service (`backend/src/rag/ingest.py`) to parse MDX files
- [ ] T010 Implement `/api/chat` endpoint with LangChain retrieval logic (citations required)

**Checkpoint**: `curl` request to `/api/chat` returns a cited answer from a sample MDX file.

---

## Phase 3: Day 3 - Frontend & "Dual-Reality" Toggle (UX)

**Goal**: Docusaurus site with functioning Hardware Context.

- [ ] T011 Initialize Docusaurus project in `frontend/`
- [ ] T012 Implement `HardwareContext` React provider (Context API)
- [ ] T013 Create `HardwareToggle` UI component (Navbar item)
- [ ] T014 Implement custom Admonition components `:::note[RTX-Required]` and `:::note[Jetson-Compatible]` that respect Context
- [ ] T015 Implement `useEffect` hook to fetch User Profile from FastAPI on load

**Checkpoint**: Toggling the Navbar switch instantly hides/shows specific admonitions on a test page.

---

## Phase 4: Day 4 - Content Structure & Integration

**Goal**: Populate the 4-Module structure and verify RAG.

- [ ] T016 Scaffold Docusaurus sidebar with all 13 Weeks (Modules 1-4)
- [ ] T017 Create "Module 1: ROS 2" placeholder content (Week 1-5)
- [ ] T018 Create "Module 2: Gazebo" placeholder content (Week 6-7)
- [ ] T019 Create "Module 3: Isaac" placeholder content (Week 8-10)
- [ ] T020 Create "Module 4: VLA" placeholder content (Week 11-13 + Capstone)
- [ ] T021 Run RAG Ingestion script on the scaffolded content

**Checkpoint**: All 13 weeks visible in sidebar; Chatbot can answer "What is covered in Week 1?".

---

## Phase 5: Day 5 - CI/CD & Polish

**Goal**: Production-ready deployment.

- [ ] T022 Setup GitHub Actions workflow for Docusaurus Build & Deploy (GH Pages)
- [ ] T023 Setup GitHub Actions workflow for Python Linting (Ruff) (SC-005)
- [ ] T024 Implement "Toast Error" fallback for Urdu Translation button (ADR-003)
- [ ] T025 Write `quickstart.md` with "Poetry Crash Course" (ADR-002)

**Checkpoint**: PR merge deploys site to GitHub Pages; Ruff enforces PEP 8.
