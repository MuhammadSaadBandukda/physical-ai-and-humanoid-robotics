# ADR 0001: ROS 2 Distribution Selection

**Status**: Accepted  
**Date**: 2025-12-07  
**Authors**: AI Agent  
**Reviewers**: User  

## Context

The project requires a ROS 2 distribution for both the "Digital Twin" (Simulated) and "Physical AI" (Jetson) environments. The choice impacts software compatibility, long-term support (LTS), and the ability to use modern navigation/perception stacks.

## Options Considered

*   **ROS 2 Jazzy Jalisco (LTS):** Released May 2024, supported until 2029. Targets Ubuntu 24.04.
*   **ROS 2 Humble Hawksbill (LTS):** Released May 2022, supported until 2027. Targets Ubuntu 22.04.
*   **ROS 2 Rolling Ridley:** Continuous release. Bleeding edge but unstable API.

## Decision

We have chosen **ROS 2 Jazzy Jalisco (LTS)**.

## Rationale

1.  **Longevity:** As a textbook project, we want the content to remain relevant for as long as possible. Jazzy provides support until 2029, whereas Humble is already halfway through its lifecycle.
2.  **Navigation Stack:** Jazzy includes the latest improvements in Nav2 (MPPI Controller, etc.) which are beneficial for the "Humanoid Control" module.
3.  **Simulation Parity:** Newer Gazebo (Harmonic) versions are better integrated with Jazzy, improving the "Digital Twin" fidelity.

## Consequences

*   **Benefit:** Content remains current for 5 years. Access to latest VLA/AI integration libraries.
*   **Risk:** Some legacy packages may not yet be ported to Jazzy.
*   **Mitigation:** We will verify all dependencies (Nav2, moveit2) are Jazzy-compatible during the Docker build phase.
*   **Constraint:** Requires users to use Ubuntu 24.04 containers (handled via Docker).