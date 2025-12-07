# ADR 0004: Sim-to-Real Personalization Strategy

**Status**: Accepted  
**Date**: 2025-12-07  
**Authors**: AI Agent  
**Reviewers**: User  

## Context

The curriculum has two distinct tracks: "Simulated" (RTX) and "Physical" (Jetson). The instructions, code parameters, and warnings differ significantly between them. Presenting both simultaneously confuses learners.

## Options Considered

*   **Single Site with Dynamic "Dual-Reality" Context Toggle:** One documentation set; React hides/shows content based on a user-selected profile.
*   **Two Separate Websites:** `sim.textbook.com` and `real.textbook.com`.
*   **Feature Flags:** Build-time flags to generate two static sites from one source.

## Decision

We have chosen **Single Site with Dynamic "Dual-Reality" Context Toggle**.

## Rationale

1.  **Unified Source of Truth:** Having one codebase prevents "content drift" where fixes are applied to the Sim version but forgotten in the Real version.
2.  **Pedagogy:** Students often switch between Sim and Real (Sim-to-Real transfer). A toggle allows them to instantly compare "Here is the code in Sim" vs "Here is the code on Robot" without changing tabs/URLs.
3.  **Maintainability:** Easier to maintain one Docusaurus instance than two.

## Consequences

*   **Benefit:** Seamless student experience. Zero content duplication.
*   **Cost:** Authors must be disciplined about wrapping divergent content in `:::note` blocks with the correct tags.
*   **Risk:** If the toggle fails, users might see incorrect instructions.
*   **Mitigation:** We have defined strict Success Criteria (SC-003) and will write automated tests to verify the toggle logic.