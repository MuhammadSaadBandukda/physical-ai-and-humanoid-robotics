---
title: Week 1 - Foundations
sidebar_label: Week 1 - Foundations
---

# Week 1: Foundations of Embodied Intelligence

> **Learning Objective:** By the end of this week, you will understand the distinction between "Disembodied AI" (LLMs) and "Embodied AI" (Robots), and you will have configured your development workstation—whether it is a Digital Twin (RTX) or a Physical Robot (Jetson).

## 1.1 The Great Divide: Disembodied vs. Embodied AI

Traditional AI (like ChatGPT) is **disembodied**. It lives on a server, processes text or images, and outputs data. It has no physical body, no sensors to touch the world, and no actuators to change it.

**Embodied Intelligence**, or Physical AI, refers to intelligent systems that have:
1.  **A Physical Body:** Defined by kinematics (joints, links) and dynamics (mass, inertia).
2.  **Perception:** The ability to sense the environment (LiDAR, Cameras, IMUs).
3.  **Actuation:** The ability to exert force on the world (Motors, Grippers).
4.  **Loop Closure:** The continuous cycle of *Sense $\rightarrow$ Think $\rightarrow$ Act*.

### Moravec's Paradox
> *It is comparatively easy to make computers exhibit adult level performance on intelligence tests or playing checkers, and difficult or impossible to give them the skills of a one-year-old when it comes to perception and mobility.* - Hans Moravec (1988)

This paradox drives our course. Building a chatbot is "easy" (relatively); building a robot that can reliably pick up a cup is incredibly hard.

### The Agent-Environment Loop

In this course, we model the robot as an **Agent** interacting with an **Environment**.

*   **State ($s_t$):** The current configuration of the robot and world.
*   **Observation ($o_t$):** What the robot's sensors actually see (often noisy or partial).
*   **Action ($a_t$):** The motor command sent to the actuators.
*   **Reward ($r_t$):** (In RL contexts) A signal indicating success or failure.

---

## 1.2 The Hardware Landscape: Two Paths

This textbook supports two distinct hardware paths. You must choose the one that matches your available equipment.

### Path A: The Digital Twin (RTX Workstation)

This path relies on **NVIDIA Isaac Sim** or **Gazebo**. You simulate the physics of the world. This is "sim-to-real" development.

*   **Pros:** Safe, fast, infinite data, no hardware costs (beyond the GPU).
*   **Cons:** "The Reality Gap" (simulations are never perfect).
*   **Requirements:** NVIDIA RTX GPU (3060 or higher recommended).

### Path B: The Physical Edge (Jetson Orin)

This path runs on real embedded hardware, specifically the **NVIDIA Jetson Orin** series (Nano, NX, or AGX).

*   **Pros:** Real-world physics, true latency, real sensor noise.
*   **Cons:** Hardware can break, batteries die, slower compile times on ARM64.
*   **Requirements:** Jetson Orin Nano/AGX Developer Kit.

---

## 1.3 Environment Setup (Dual-Reality)

We use **Docker** to ensure our environments are reproducible across both paths.

:::note[RTX-Required]
### 🖥️ Option A: RTX Workstation Setup

If you are following the **Simulation Path**, you will use the `amd64` Docker container. This container includes the full desktop version of ROS 2 Jazzy and Gazebo Harmonic.

**1. Install NVIDIA Container Toolkit**
Ensure your host machine has the NVIDIA drivers installed.

**2. Pull and Run the Container**
```bash
# Run on your Desktop PC
docker run --gpus all -it --rm \
    -v $(pwd):/workspace \
    physical-ai-sim:latest
```
:::

:::note[Jetson-Compatible]
### 🤖 Option B: Jetson Orin Setup

If you are following the **Physical Path**, you will use the `arm64` Docker container. This is optimized for the Tegra architecture (L4T) and includes only the necessary runtime components to save memory.

**1. Flash JetPack 6.0**
Ensure your Jetson is running the latest JetPack (Ubuntu 22.04/24.04 based).

**2. Pull and Run the Container**
```bash
# Run on your Jetson
docker run --runtime nvidia -it --rm \
    --network host \
    -v $(pwd):/workspace \
    physical-ai-physical:latest
```
:::

---

## 1.4 "Hello World" System Check

Once inside your container (Sim or Physical), verify that ROS 2 is talking to your system.

```python
# check_ros_env.py
import rclpy
from rclpy.node import Node
import platform

def main():
    rclpy.init()
    
    # Detect Architecture
    arch = platform.machine()
    
    print("="*40)
    print("✅ ROS 2 Environment is Active!")
    print(f"   DDS Middleware: {rclpy.get_rmw_implementation_identifier()}")
    print(f"   Architecture:   {arch}")
    
    if arch == 'x86_64':
        print("   Mode:           Simulated (RTX)")
    elif arch == 'aarch64':
        print("   Mode:           Physical (Jetson)")
        
    print("="*40)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

Run this script:
```bash
python3 check_ros_env.py
```

## 1.5 The Project Roadmap

*   **Module 1 (ROS 2):** Learn the nervous system.
*   **Module 2 (Gazebo):** Build the body in simulation.
*   **Module 3 (Isaac):** Train the brain (Reinforcement Learning).
*   **Module 4 (VLA):** Deploy to the real world (Sim-to-Real).

**Next Week:** We will dive into the anatomy of robot sensors—giving eyes and ears to our intelligence.
