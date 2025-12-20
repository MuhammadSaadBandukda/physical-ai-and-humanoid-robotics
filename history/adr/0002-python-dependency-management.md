# ADR 0002: Python Dependency Management

**Status**: Accepted  
**Date**: 2025-12-07  
**Authors**: AI Agent  
**Reviewers**: User  

## Context

The backend (FastAPI, RAG) involves complex dependency chains (LangChain, Qdrant client, Torch for embeddings). We need a way to manage these reproducibly across development, simulated docker containers, and physical Jetson environments.

## Options Considered

*   **Poetry:** Advanced dependency manager with strict `poetry.lock` and simplified virtualenv handling.
*   **Pip + requirements.txt:** Standard, simple, but prone to "it works on my machine" issues due to lack of transitive dependency locking.
*   **Conda:** Good for data science, but heavy and harder to integrate cleanly with standard Docker/ROS workflows.

## Decision

We have chosen **Poetry**.

## Rationale

1.  **Reproducibility:** The `poetry.lock` file guarantees that the exact same versions of libraries are installed on the developer's laptop, the CI runner, and the Jetson robot. This is critical for avoiding subtle "Sim-to-Real" bugs caused by library drift.
2.  **Dependency Resolution:** Poetry's solver is robust at handling conflicting version requirements, which is common in the fast-moving AI/LLM ecosystem (LangChain updates frequently).
3.  **Dev Experience:** Simplifies virtualenv management and command execution (`poetry run`).

## Consequences

*   **Benefit:** Zero "dependency drift" issues. Consistent environments.
*   **Cost:** Steep learning curve for students unfamiliar with it.
*   **Risk:** `colcon build` does not natively support Poetry's virtual environment encapsulation.
*   **Mitigation:** We must configure `colcon` to build into the Poetry shell or use a `setup.py` shim. This adds complexity to the build pipeline compared to standard `pip`. We will include a "Poetry Crash Course" in the Quickstart guide and provide pre-configured `colcon` build scripts.