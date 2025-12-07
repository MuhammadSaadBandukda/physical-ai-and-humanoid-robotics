# ADR 0003: Frontend-Backend Data Integration

**Status**: Accepted  
**Date**: 2025-12-07  
**Authors**: AI Agent  
**Reviewers**: User  

## Context

The Docusaurus frontend is a Static Site Generator (SSG). However, we need to display dynamic data:
1.  **RAG Chatbot:** Real-time conversation history.
2.  **Auth State:** User login status and hardware profile.
3.  **Robot State (Future):** Potential live telemetry from the robot.

## Options Considered

*   **Client-side Runtime Fetching (`useEffect`):** React components fetch data from APIs after the page loads.
*   **Build-time Fetching:** Docusaurus fetches data during the build process and bakes it into the HTML.
*   **Server-Side Rendering (Next.js):** Moving to a full SSR framework.

## Decision

We have chosen **Client-side Runtime Fetching (`useEffect`)**.

## Rationale

1.  **Dynamic Nature:** Chat conversations and user authentication are inherently dynamic and specific to the current user/session. They *cannot* be pre-baked at build time.
2.  **Architecture Fit:** Docusaurus is optimized for documentation (our primary goal). We don't want to abandon it for Next.js just for these features. React's `useEffect` pattern is the standard way to "hydrate" static sites with dynamic data.
3.  **Separation of Concerns:** Decouples the "Content" (Static Markdown) from the "App" (Dynamic API). The API can go down, and the textbook remains readable.

## Consequences

*   **Benefit:** Keeps the textbook performant (static CDN delivery) while enabling rich interactive features.
*   **Cost:** "Loading spinners" will be visible while dynamic data fetches. SEO won't see the dynamic content (acceptable for chat/auth).