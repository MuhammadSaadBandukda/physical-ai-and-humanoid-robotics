---
id: 0001
title: establish-constitution
stage: constitution
date: 2025-12-07
surface: agent
model: gemini
feature: none
branch: master
user: user
command: /sp.constitution Project: Textbook for Teaching Physical AI & Humanoid Robotics Course...
labels: ["constitution", "setup", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - null
---

## Prompt

/sp.constitution Project: Textbook for Teaching Physical AI & Humanoid Robotics Course

Core principles:
- Embodied Intelligence (Bridging the digital brain and physical body)
- Specification-Driven Development (SDD-RI)
- "Process as Product" (Treating content creation as engineering)
- Accessibility (Multi-language and adaptive content)

Key standards:
- Framework: Docusaurus (Markdown/MDX format)
- Tech Stack: Spec-Kit Plus, Claude Code, OpenAI Agents SDK
- Data & RAG: FastAPI, Neon Serverless Postgres, Qdrant Cloud
- Content Structure: 4 Modules covering ROS 2, Gazebo, NVIDIA Isaac, and VLA
- Authentication: Better-Auth for capturing user hardware context

Constraints:
- Output: Static site deployed to GitHub Pages
- Hardware Context: Content must distinguish between "Digital Twin" Workstations (RTX) and "Physical AI" Edge Kits (Jetson)
- Interactive Elements: Embedded RAG chatbot, personalization toggles, and Urdu translation buttons
- Tone: Academic, rigorous, and instruction-focused

Success criteria:
- All chapters map to the specified weekly breakdown
- RAG chatbot successfully answers questions based on book content
- Personalization logic adapts content based on user hardware (Cloud vs. Local)
- Zero "vibe coding" (all steps must follow the Specify -> Plan -> Implement workflow)

## Response snapshot

Established project constitution v1.0.0.
Principles: Embodied Intelligence, SDD-RI, "Process as Product", Accessibility.
Standards: Docusaurus, FastAPI, Neon, Qdrant.
Constraints: Digital Twin vs. Physical AI, Urdu support.

## Outcome

- ✅ Impact: Defined core governance and technical standards for the project.
- 🧪 Tests: N/A
- 📁 Files: Updated .specify/memory/constitution.md
- 🔁 Next prompts: /sp.feature to start defining modules.
- 🧠 Reflection: Constitution successfully established from user input, replacing all placeholders.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A