---
title: Week 13 - Conversational
sidebar_label: Week 13 - Conversational
---

# Week 13: Conversational AI and Cognitive Planning

> **Learning Objective:** Our humanoid can walk, perceive, and manipulate. This week, we give it the ultimate human-like capability: understanding and responding to natural language. We'll explore Large Language Models (LLMs) and speech recognition for high-level cognitive control.

## 13.1 The Rise of Vision-Language-Action (VLA) Models

Traditional robotics planning often relies on predefined rules and state machines. **Vision-Language-Action (VLA) models** integrate multimodal inputs (vision, language) to enable more intuitive and flexible robot control.

### How LLMs Transform Robotics:

*   **High-Level Planning:** Convert abstract human commands ("make dinner") into executable robot actions ("go to fridge," "open door," "grasp item").
*   **Common Sense Reasoning:** LLMs inject a vast amount of learned world knowledge.
*   **Human-Robot Collaboration:** Natural language becomes the interface.

---

## 13.2 Speech-to-Text with Whisper

Before an LLM can process a spoken command, it needs to be converted into text. OpenAI's **Whisper** model is a leading solution for robust speech-to-text conversion.

### Why Whisper?

*   **Accuracy:** Highly accurate across various languages and accents.
*   **Robustness:** Handles background noise well.
*   **Open-Source:** Available for fine-tuning and deployment.

:::note[Jetson-Compatible]
### 🤖 Deploying Whisper on Jetson

Whisper models (especially larger ones) can be computationally intensive.
*   **On-Device Inference:** For real-time, privacy-sensitive applications, you can deploy optimized Whisper models (e.g., `tiny.en` or `base.en`) directly on the Jetson using **ONNX Runtime** or **TensorRT**.
*   **Cloud APIs:** For higher accuracy or larger models, you can send audio to a cloud-based Whisper API, but this introduces latency and requires an internet connection.
:::

---

## 13.3 LLMs for Robotic Control

The core idea is to use LLMs as the "brain" that translates human intent into a sequence of robot-executable actions.

### 13.3.1 Prompt Engineering for Robots

Crafting prompts that guide the LLM to generate valid robot commands is an art.

```
"You are a helpful robot assistant. You can call the following functions:
  - move_to(location: str)
  - pick_up(object_name: str)
  - place_on(surface_name: str)

Based on the user's request, choose the appropriate function calls.
User: 'Robot, please go to the kitchen, find the red apple, and bring it to me on the table.'
"
```
The LLM would then output: `move_to("kitchen")`, `pick_up("red apple")`, `place_on("table")`.

### 13.3.2 Function Calling / Tool Use

Many modern LLMs are specifically fine-tuned for **function calling** (or "tool use"). They can parse natural language and respond with a structured JSON object specifying which function to call and with what arguments.

### 13.3.3 Embodied LLMs

The next frontier is LLMs that are not just trained on text/images but on **embodied data** (robot trajectories, sensor readings, and their corresponding actions). This allows them to learn better policies for physical interaction.

---

## 13.4 Cognitive Planning: From Abstract Goals to Executable Steps

LLMs can perform **cognitive planning**, breaking down abstract goals into a sequence of concrete, executable steps.

*   **Hierarchical Planning:**
    *   **High-Level (LLM):** "Make breakfast" -> "Get ingredients," "Cook eggs," "Serve."
    *   **Mid-Level (LLM/Traditional Planner):** "Get ingredients" -> "Go to fridge," "Open door," "Grasp eggs."
    *   **Low-Level (ROS 2 Nav2/MoveIt):** "Go to fridge" -> (path planning, motor commands).

### Integrating with Traditional Planners

LLMs don't replace traditional planners; they augment them. The LLM handles the symbolic reasoning and sequencing, while ROS 2 Nav2 and MoveIt handle the geometric and dynamic planning.

:::note[RTX-Required]
### 🖥️ Prototyping LLM Agents in Simulation

Developing LLM-driven robot behaviors in Isaac Sim (RTX) is highly efficient:
*   **Rapid Iteration:** Test different prompts and LLM integration strategies without risk.
*   **Debugging:** Easily identify why an LLM's generated plan failed in a controlled environment.
*   **Synthetic Data:** Create datasets of human commands and corresponding optimal robot actions to fine-tune smaller, specialized VLA models.
:::

:::note[Jetson-Compatible]
### 🤖 Deploying Conversational Agents on Jetson

Deploying these complex AI pipelines to the Jetson involves:
*   **Model Optimization:** Using smaller, optimized LLMs (e.g., Llama 3 with TensorRT-LLM) for on-device inference.
*   **API Management:** Efficiently handling API calls to larger cloud LLMs to minimize latency.
*   **Robustness:** Designing for network dropouts and ambiguous commands.
*   **Power Efficiency:** Balancing model size and performance with power consumption.
:::

---

## 13.5 Hands-on (Conceptual): Voice Command to Robot Action

1.  **User says:** "Robot, go to the red ball and pick it up."
2.  **Whisper (on Jetson):** Converts audio to text.
3.  **LLM (on Jetson/Cloud):** Receives text, uses function calling to generate:
    ```json
    {
      "function_name": "sequence_actions",
      "arguments": [
        {"action": "detect_object", "object": "red ball"},
        {"action": "navigate_to", "target": "red ball location"},
        {"action": "pick_up", "object": "red ball"}
      ]
    }
    ```
4.  **ROS 2 Agent (on Jetson):** Parses the JSON, executes ROS 2 calls:
    *   Launches object detection node.
    *   Sends goal to Nav2.
    *   Calls MoveIt 2 service for grasping.

**Module 4 Final Stretch!**
We've almost completed the textbook. One final chapter to bring it all together.

**Next:** Our Capstone Project, where we define the spec for a fully autonomous humanoid.