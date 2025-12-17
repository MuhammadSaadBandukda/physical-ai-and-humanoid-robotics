---

description: "5-Day Sprint Task List for Physical AI Textbook"
---

# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Implementation Plan (`specs/001-physical-textbook/plan.md`)
**Prerequisites**: ADRs 0001-0004 approved.

## Phase 1: Day 1 - Infrastructure & Docker (Foundational)

**Goal**: Establish the "Dual-Reality" build environment (Simulated x86 & Physical ARM64).
**Risks**: ROS 2 Jazzy on ARM64 build times; Poetry+Colcon integration.

- [-] T001 [P] Create `infrastructure/docker/dev.amd64.Dockerfile` for ROS 2 Jazzy (Simulated) - Deferred to Phase 2 per Strategic Pivot.
  - Output: Docker image `ros2-jazzy-amd64` that runs `ros2 topic list`.
- [-] T002 [P] Create `infrastructure/docker/dev.arm64.Dockerfile` for ROS 2 Jazzy (Jetson/Physical) - Deferred to Phase 2 per Strategic Pivot.
  - Output: Docker image `ros2-jazzy-arm64` (cross-compiled) that runs `ros2 topic list`.
- [-] T003 Setup `backend/pyproject.toml` with Poetry dependencies (FastAPI, LangChain, Qdrant) - Deferred to Phase 2 per Strategic Pivot.
  - Output: `poetry lock` file generated without conflicts.
- [-] T004a Create `infrastructure/scripts/detect_ros_env.sh` to check for active Poetry env - Deferred to Phase 2 per Strategic Pivot.
  - Output: Script returns 0 if Poetry is active, 1 otherwise.
- [-] T004b **[CRITICAL]** Implement `infrastructure/scripts/colcon_poetry_build.sh` build shim (ADR-002) - Deferred to Phase 2 per Strategic Pivot.
  - Output: Script successfully runs `colcon build` inside `poetry run`.
- [-] T005 Create `docker-compose.yml` orchestrating Backend, Qdrant, Neon (mock), and Docusaurus dev server - Deferred to Phase 2 per Strategic Pivot.
  - Output: `docker-compose up` services are healthy (green).

**Checkpoint**: `docker-compose up` launches a working ROS 2 environment and Python backend on both x86 and ARM64.

---

## Phase 2: Day 2 - Backend Core & RAG (Functional)

**Goal**: Functional API for Auth and RAG.

- [-] T006 Initialize FastAPI app structure in `backend/src/api` - Deferred to Phase 2 per Strategic Pivot.
  - Output: `GET /health` returns 200 OK.
- [-] T007 Implement Better-Auth integration with Postgres (Neon) for User/HardwareProfile entities - Deferred to Phase 2 per Strategic Pivot.
  - Output: User can sign up via API and row appears in DB.
- [-] T008 Implement Qdrant client connection and collection initialization - Deferred to Phase 2 per Strategic Pivot.
  - Output: Script creates `textbook_chunks` collection in Qdrant.
- [-] T009 Create RAG Ingestion Service (`backend/src/rag/ingest.py`) to parse MDX files - Deferred to Phase 2 per Strategic Pivot.
  - Output: Service extracts text and metadata (Module/Week) from sample MDX.
- [-] T010 Implement `/api/chat` endpoint with LangChain retrieval logic (citations required) - Deferred to Phase 2 per Strategic Pivot.
  - Output: Endpoint returns JSON with `answer` and `sources` list.

**Checkpoint**: `curl` request to `/api/chat` returns a cited answer from a sample MDX file.

---

## Phase 3: Day 3 - Frontend & "Dual-Reality" Toggle (UX)

**Goal**: Docusaurus site with functioning Hardware Context.

- [x] T011 Initialize Docusaurus project in `frontend/`
  - Output: Docusaurus default site runs on localhost:3000.
- [x] T012 Implement `HardwareContext` React provider (Context API)
  - Output: Context enables switching `hardwareProfile` state.
- [x] T013 Create `HardwareToggle` UI component (Navbar item)
  - Output: Navbar toggle updates Context state.
- [x] T014a Create Base Admonition Wrapper Component
  - Output: Component renders standard Docusaurus admonition.
- [x] T014b Implement Logic for `:::note[RTX-Required]` and `:::note[Jetson-Compatible]`
  - Output: Admonition hides/shows based on Context state.
- [x] T015 Implement `useEffect` hook to fetch User Profile from FastAPI on load
  - Output: Console logs user profile data on page load.

**Checkpoint**: Toggling the Navbar switch instantly hides/shows specific admonitions on a test page.

---

## Phase 4: Day 4 - Content Structure & Integration

**Goal**: Populate the 4-Module structure and verify RAG.

- [x] T016 Scaffold Docusaurus sidebar with all 13 Weeks (Modules 1-4)
  - Output: Sidebar matches Spec structure exactly.
- [x] T017 Create "Module 1: ROS 2" placeholder content (Week 1-5)
  - Output: MDX files exist for all Week 1-5 topics.
- [x] T018 Create "Module 2: Gazebo" placeholder content (Week 6-7)
  - Output: MDX files exist for all Week 6-7 topics.
- [x] T019 Create "Module 3: Isaac" placeholder content (Week 8-10)
  - Output: MDX files exist for all Week 8-10 topics.
- [x] T020 Create "Module 4: VLA" placeholder content (Week 11-13 + Capstone)
  - Output: MDX files exist for all Week 11-13 topics.
- [-] T021 Run RAG Ingestion script on the scaffolded content - Deferred to Phase 2 per Strategic Pivot.
  - Output: Qdrant contains vectors for all new placeholder files.

**Checkpoint**: All 13 weeks visible in sidebar; Chatbot can answer "What is covered in Week 1?".

---

## Phase 5: Day 5 - CI/CD & Polish

**Goal**: Production-ready deployment.

- [x] T022 Setup GitHub Actions workflow for Docusaurus Build & Deploy (GH Pages)
  - Output: Push to main triggers successful deploy.
- [-] T023 Setup GitHub Actions workflow for Python Linting (Ruff) (SC-005) - Deferred to Phase 2 per Strategic Pivot.
  - Output: CI fails if `backend/` code violates PEP 8.
- [x] T024 Implement "Toast Error" fallback for Urdu Translation button (ADR-003)
  - Output: Clicking translate without API key shows UI Toast error.
- [x] T025 Write `quickstart.md` with "Poetry Crash Course" (ADR-002)
  - Output: Markdown file explaining `poetry install` and `poetry run`.

**Checkpoint**: PR merge deploys site to GitHub Pages; Ruff enforces PEP 8.

## Phase 2: Content Authoring

**Goal**: Populate Module 1 with high-fidelity, dual-reality content.

- [x] T026 Write `docs/01-Module-1-ROS2/01-Week-1-Foundations.md` (Dual-Reality Intro)
- [ ] T027 Write `docs/01-Module-1-ROS2/02-Week-2-Nodes.md` (Placeholder)
- [ ] T028 Write `docs/01-Module-1-ROS2/03-Week-3-Topics.md` (Placeholder)
