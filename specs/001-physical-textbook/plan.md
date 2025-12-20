# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `001-physical-textbook` | **Date**: 2025-12-07 | **Spec**: [specs/001-physical-textbook/spec.md](./spec.md)
**Input**: Feature specification for "Physical AI & Humanoid Robotics Textbook"

## Summary

The "Physical AI & Humanoid Robotics Textbook" is an interactive, "Dual-Reality" educational platform deployed as a static Docusaurus site backed by a FastAPI/RAG service. It bridges the gap between digital simulation and physical hardware by enabling students to toggle content between "Simulated (RTX)" and "Physical (Jetson)" contexts. The system integrates an AI teaching assistant (RAG chatbot) grounded in the textbook content and uses Better-Auth for user context persistence.

## Technical Context

**Language/Version**: Python 3.10+ (Backend), Node.js 18+ (Frontend), ROS 2 Jazzy Jalisco (LTS)
**Primary Dependencies**:
- **Frontend**: Docusaurus 3.x, React 18, Tailwind CSS, Axios
- **Backend**: FastAPI, Poetry, LangChain/LlamaIndex (for RAG), Better-Auth
- **Infrastructure**: Docker (Multi-arch: amd64/arm64), Qdrant (Vector DB), Neon (Postgres)
**Storage**: Neon Serverless Postgres (User Data), Qdrant Cloud (Embeddings)
**Testing**: Pytest (Backend), Jest (Frontend), Ruff (Linting)
**Target Platform**: GitHub Pages (Static Host), Render/Railroad (Backend API), Jetson Orin Nano/AGX (Client Hardware)
**Project Type**: Web application (Frontend + Backend + RAG Pipeline)
**Performance Goals**: RAG Latency < 2s, Toggles < 100ms (Client-side)
**Constraints**:
- **Hardware Distinction**: Strict separation of RTX vs. Jetson content.
- **Accessibility**: Urdu translation with technical term preservation.
- **Code Hygiene**: Strict PEP 8 enforcement via Ruff.
**Scale/Scope**: 13-Week Syllabus, ~50 Chapters, 100+ Code Snippets

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Embodied Intelligence**: ✅ Plan includes explicit "Physical (Jetson)" context toggle and hardware-specific deployment instructions.
- **SDD-RI**: ✅ Plan follows Specify -> Plan -> Implement. No "vibe coding".
- **"Process as Product"**: ✅ Infrastructure setup (Docker/Poetry) is documented as part of the student learning path.
- **Accessibility**: ✅ Urdu translation toggle and hardware adaptation are core features.
- **Tagging**: ✅ Plan enforces `:::note[RTX-Required]` and `:::note[Jetson-Compatible]` admonitions.
- **Code Integrity**: ✅ CI pipeline includes `ruff` check.

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-textbook/
├── plan.md              # This file
├── research.md          # Technology selection rationale
├── data-model.md        # Database schema (User, Chat, Chunks)
├── quickstart.md        # Setup guide for contributors
├── contracts/           # API OpenApi Spec (FastAPI -> Frontend)
└── tasks.md             # 5-Day Sprint Plan
```

### Source Code (repository root)

```text
# Web Application Structure
backend/
├── pyproject.toml       # Poetry dependencies
├── src/
│   ├── api/             # FastAPI routes (Auth, Chat)
│   ├── core/            # Config, Security
│   ├── rag/             # Ingestion, Retrieval, Generation logic
│   └── models/          # Pydantic/SQLAlchemy models
└── tests/

frontend/
├── package.json
├── docusaurus.config.js
├── src/
│   ├── components/      # React components (Chatbot, Toggle)
│   ├── context/         # HardwareContext, AuthContext
│   ├── css/             # Tailwind directives
│   └── pages/
├── docs/                # The MDX Content (Modules 1-4)
│   ├── 01-Module-1-ROS2/
│   ├── 02-Module-2-Gazebo/
│   ├── 03-Module-3-Isaac/
│   └── 04-Module-4-VLA/
└── static/              # Images, URDFs

infrastructure/
├── docker/              # Multi-arch Dockerfiles
│   ├── dev.amd64.Dockerfile
│   └── dev.arm64.Dockerfile
└── scripts/             # CI/CD, Ingestion scripts
```

**Structure Decision**: A standard Monorepo structure (`backend/` + `frontend/`) is chosen to keep the "Textbook" content close to the "AI Brain" that powers it. This simplifies the RAG ingestion pipeline (backend can directly read `frontend/docs`).

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| **Multi-Arch Docker** | Supporting both x86 (RTX) and ARM64 (Jetson) is a core requirement. | **Single Arch**: Would alienate 50% of the hardware user base (either Sim or Real). |
| **Custom Backend** | Docusaurus is static; we need dynamic RAG and Auth. | **Serverless Functions**: Too complex to manage RAG state/connections; FastAPI is cleaner for Python-centric RAG. |
| **Client-Side Fetch** | Docusaurus SSG cannot handle real-time robot state/chat. | **Build-Time Fetch**: Chat is dynamic; Robot state is real-time. SSG is insufficient. |