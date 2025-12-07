# Feature Specification: Physical AI & Humanoid Robotics Textbook

**Feature Branch**: `001-physical-textbook`  
**Created**: 2025-12-07  
**Status**: Draft  
**Input**: Syllabus-to-Modules Mapping, Hardware Constraints, Functional Deliverables (RAG, Auth, Personalization, Translation)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - The Hardware-Aware Learner (Priority: P1)

A student visits the textbook site. They are prompted to Sign Up/In. Upon registration, they select their hardware profile: "Digital Twin (RTX Workstation)" or "Physical AI (Jetson Edge Kit)". The site content automatically adapts: code snippets switch to the relevant platform, and unrelated admonitions are hidden or de-emphasized.

**Why this priority**: Core value proposition. The course is unteachable if students are confused by instructions for hardware they don't possess.
**Independent Test**: Register two accounts with different profiles. Verify Chapter 1 displays different setup instructions for each.

**Acceptance Scenarios**:
1. **Given** a new user, **When** they register via Better-Auth, **Then** they MUST select a hardware profile (RTX/Jetson).
2. **Given** an "RTX" user viewing Week 1 content, **When** the page loads, **Then** `:::note[Jetson-Compatible]` sections are collapsed/hidden and `:::note[RTX-Required]` sections are highlighted.
3. **Given** a user reading a code block, **When** they toggle the "Platform" switch manually, **Then** the content updates dynamically without page reload.

### User Story 2 - The AI-Assisted Researcher (Priority: P2)

A student has a question about a complex ROS 2 concept in Module 1. Instead of searching Google, they open the embedded "Course TA" chatbot. They ask, "How do nodes communicate?" The bot answers using *only* the textbook content as ground truth and provides a direct citation link to `01-Module-1-ROS2/03-Week-3/01-Nodes-Topics-Services`.

**Why this priority**: Enhances learning efficiency and ensures answers align with course pedagogy (not generic internet answers).
**Independent Test**: Ask the bot a question covered in Week 3. Verify the answer is correct and the link resolves to the correct page.

**Acceptance Scenarios**:
1. **Given** a logged-in user, **When** they ask a question, **Then** the backend (FastAPI/Qdrant) retrieves relevant chunks.
2. **Given** the retrieved chunks, **When** the LLM generates a response, **Then** it MUST append a markdown link to the source chapter.

### User Story 3 - The Global Learner (Priority: P3)

A student whose primary language is Urdu visits the site. They toggle the "Language" switch. The main instructional text translates to Urdu, but critical technical terms (Node, Topic, Service, URDF) remain in English to ensure they learn industry-standard terminology.

**Why this priority**: Accessibility principle (Constitution Principle IV).
**Independent Test**: Toggle Urdu on Week 1. Verify "Embodied Intelligence" is translated but "Jetson Orin" is not.

**Acceptance Scenarios**:
1. **Given** any page, **When** "Urdu" is toggled, **Then** text content is replaced with AI-translated versions.
2. **Given** a technical term (e.g., "ROS 2"), **When** translated, **Then** it MUST remain "ROS 2" (transliteration or direct English retention).

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001 (Platform)**: System MUST be a Docusaurus static site deployed to GitHub Pages.
- **FR-002 (Auth)**: System MUST use Better-Auth for user management and hardware profile persistence (Postgres).
- **FR-003 (RAG)**: System MUST expose a FastAPI endpoint `/api/chat` that queries a Qdrant vector database populated with the textbook markdown.
- **FR-004 (Personalization)**: System MUST use React Context to manage `hardwareProfile` state and conditionally render Docusaurus Admonitions.
- **FR-005 (Translation)**: System MUST provide a UI toggle that swaps MDX content with pre-generated Urdu translations (or dynamic if feasible/cost-effective, pre-generated preferred for static site).
- **FR-006 (RI Patterns)**: Implementation MUST utilize `Reusable Intelligence` patterns:
    - *Agent Skills*: Encapsulate RAG logic as a reusable skill.
    - *Subagents*: Define a "Content Ingestion Agent" for parsing MDX into Qdrant.

### Content Requirements (Table of Contents Mapping)

The site MUST strictly follow this structure:

#### Module 1: The Robotic Nervous System (ROS 2)
- **Week 1 (Foundations)**: Embodied Intelligence, Hardware Landscape (Jetson vs RTX).
- **Week 2 (Sensors)**: LiDAR, Depth, IMU anatomy.
- **Week 3 (ROS 2 Arch)**: Nodes, Topics, Services.
- **Week 4 (Packages)**: Python Agents, Launch Files.
- **Week 5 (Comms)**: Actions, Interfaces.

#### Module 2: The Digital Twin (Gazebo)
- **Week 6 (Sim Basics)**: URDF/SDF, Physics, Collisions.
- **Week 7 (Adv Sim)**: Simulating Sensors, Unity HRI Intro.

#### Module 3: The AI-Robot Brain (Isaac)
- **Week 8 (Isaac Fundamentals)**: Isaac Sim Photorealism.
- **Week 9 (Perception)**: VSLAM, Nav2.
- **Week 10 (RL)**: Sim-to-Real Transfer.

#### Module 4: VLA & Humanoid Control
- **Week 11 (Kinematics)**: Bipedal Locomotion.
- **Week 12 (Manipulation)**: Interaction Dynamics.
- **Week 13 (Conversational)**: LLMs + Whisper, Cognitive Planning.
- **Capstone**: The Autonomous Humanoid Spec.

### Key Entities

- **User**: ID, Email, HardwareProfile (Enum: RTX, Jetson), Language (Enum: EN, UR).
- **TextbookChunk**: ID, Content, EmbeddingVector, SourceChapterID.
- **ChatMessage**: ID, UserID, Query, Response, Citations.

## Non-Goals *(mandatory)*

- **Live Hardware Deployment**: Content focuses on simulation and development kits (Jetson); deployment to commercial humanoid hardware (e.g., Tesla Optimus, Figure 01) is out of scope.
- **Advanced Control Theory**: Curriculum targets undergraduate foundations; advanced topics like Model Predictive Control (MPC) or Whole-Body Control (WBC) research are excluded.
- **Financial & Market Analysis**: Content is strictly technical; no analysis of robot unit economics, business models, or market trends.
- **AI Ethics & Philosophy**: Integration of VLA models is purely functional; scope excludes philosophical debates on AGI or ethical bias in LLMs.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of the 13-Week Syllabus topics are present in the sidebar.
- **SC-002**: RAG Chatbot answers questions with >90% citation accuracy (link points to correct chapter).
- **SC-003**: "Jetson" users do NOT see "RTX-only" instructions in default view (verified by UI test).
- **SC-004**: Deployment pipeline successfully builds Docusaurus and updates GitHub Pages on `main` merge.
- **SC-005**: All Python code examples adhere to PEP 8 (verified by linter).
