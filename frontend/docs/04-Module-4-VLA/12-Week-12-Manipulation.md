---
title: Week 12 - Manipulation
sidebar_label: Week 12 - Manipulation
---

# Week 12: Dexterous Manipulation and Interaction Dynamics

> **Learning Objective:** With our humanoid learning to walk, it's time to teach it to interact with its world. This week delves into the complexities of robotic manipulation, focusing on grippers, grasp planning, and controlling forces during interaction.

## 12.1 The Challenge of Dexterous Manipulation

The human hand is a marvel of engineering, capable of both power grasps and delicate precision. Replicating this in robotics is extremely difficult due to:
*   **High Degrees of Freedom (DOFs):** Many joints to control.
*   **Contact Dynamics:** Friction, slip, deformation of objects.
*   **Uncertainty:** Imperfect perception of object properties (weight, texture).

---

## 12.2 Grippers and End-Effectors

The "hand" of a robot is called an **End-Effector**.

*   **Parallel Jaw Grippers:** Simple, robust for a wide range of objects. Most common in industrial settings.
*   **Underactuated Grippers:** Fewer motors than joints, passively conforms to object shape.
*   **Multi-Fingered Hands:** (e.g., Allegro Hand, Shadow Hand) More dexterous, but complex control.
*   **Suction Cups:** For flat, smooth objects.

:::tip
**Choosing a Gripper:** Start with the simplest gripper that can solve your task. Complexity scales rapidly with dexterity.
:::

---

## 12.3 Grasp Planning

How does a robot decide where and how to grab an object?

*   **Form Closure:** The gripper completely constrains the object, preventing any motion. (Ideal, but rare).
*   **Force Closure:** The gripper can exert forces that prevent the object from moving, even if not fully constrained. (More practical).

### Vision-Based Grasping

*   **Approach:** Use RGB-D cameras (Week 2) and deep learning models to directly predict optimal grasp poses from sensor data.
*   **Example:** **Dex-Net** (Berkeley) uses a large dataset of synthetic grasps to train models that predict robust grasp configurations.

---

## 12.4 Force Control: Beyond Position

For tasks involving contact (e.g., opening a door, screwing in a lightbulb), pure position control is brittle and can damage the robot or environment.

### Position Control vs. Force Control

*   **Position Control:** "Move to this joint angle / Cartesian coordinate." (Robot is rigid).
*   **Force Control:** "Apply this much force in this direction." (Robot is compliant).

### Compliance & Impedance Control

*   **Compliance:** The ability to deform or yield when force is applied.
    *   **Passive:** Achieved through mechanical design (e.g., springs, flexible joints).
    *   **Active:** Achieved through software control, using force/torque sensors.
*   **Impedance Control:** A type of force control that regulates the relationship between force and displacement, making the robot behave like a virtual spring-damper system.

:::note[RTX-Required]
### 🖥️ Force Control in Simulation

Isaac Sim allows you to easily attach force/torque sensors to any joint or link. You can also directly apply forces and torques. This is invaluable for:
*   **Debugging:** Visualize contact forces and robot compliance.
*   **Tuning:** Rapidly tune impedance control parameters for various interaction tasks.
*   **Reinforcement Learning:** Reward for successful force application or penalize excessive force.
:::

:::note[Jetson-Compatible]
### 🤖 Force Control on Physical Hardware

Implementing force control on a Jetson requires:
*   **Force/Torque Sensors:** Expensive, but necessary for accurate force feedback.
*   **Fast Control Loop:** Force control typically runs at very high frequencies (>1000Hz) to maintain stability, pushing the limits of the Jetson's real-time capabilities.
*   **Robust Actuators:** Motors must be able to respond quickly to force commands without instability.
:::

---

## 12.5 Interaction Dynamics

Understanding how the robot interacts with its environment goes beyond simple grasping:

*   **Pushing & Sliding:** How to apply force to move an object without picking it up.
*   **Haptic Feedback:** Providing tactile information to a human operator during teleoperation.
*   **Detecting Unintended Contacts:** Safety mechanisms to stop motion when unexpected forces are detected.

---

## 12.6 Hands-on (Conceptual): Pick-and-Place with MoveIt 2

1.  **Robot Configuration:** Integrate your robot's URDF with MoveIt 2 (ROS 2 package).
2.  **Perception:** Use a simulated camera to detect the object's pose.
3.  **Grasp Planning:** Use MoveIt's built-in grasp generator or a custom one (e.g., from Dex-Net) to find a suitable grasp.
4.  **Motion Planning:** Plan a collision-free path for the arm to reach the object, grasp it, lift it, and place it.
5.  **Execution:** Send the planned trajectory to the robot's joint controllers.

**Next Week:** Our humanoid can walk and grab. Now, let's make it smart and conversational with **LLMs and Cognitive Planning**.