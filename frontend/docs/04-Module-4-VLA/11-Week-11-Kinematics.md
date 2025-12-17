---
title: Week 11 - Kinematics
sidebar_label: Week 11 - Kinematics
---

# Week 11: Humanoid Kinematics and Bipedal Locomotion

> **Learning Objective:** We are now entering the domain of humanoids. Unlike wheeled robots, legged robots must constantly balance against gravity. This week covers the math of movement (Kinematics) and the basics of walking (Bipedal Locomotion).

## 11.1 The Challenge of Bipeds

Humanoids are **under-actuated, unstable systems**.
*   **Under-actuated:** You cannot directly control your position in space; you can only control your joint angles and hope friction moves you.
*   **Unstable:** If you stop controlling it, it falls over (inverted pendulum).

---

## 11.2 Forward vs. Inverse Kinematics

### Forward Kinematics (FK)
*   **Question:** "If I set my hip angle to $30^\circ$ and knee to $45^\circ$, where is my foot?"
*   **Math:** Trigonometry and Transformation Matrices.
*   **Difficulty:** Easy (Unique solution).

### Inverse Kinematics (IK)
*   **Question:** "I want my foot to be at $(x, y, z)$. What should my joint angles be?"
*   **Math:** Jacobian matrices, optimization solvers.
*   **Difficulty:** Hard (Multiple solutions or no solution).

:::note[RTX-Required]
### 🖥️ Solving IK in Simulation

In Isaac Sim / ROS 2, we use solvers like **KDL** (Kinematics and Dynamics Library) or **MoveIt 2**.
Because simulation provides perfect joint states, IK solvers are very stable.
:::

:::note[Jetson-Compatible]
### 🤖 IK on Real Hardware

Real motors have limits (torque, velocity). A mathematical IK solution might require instantaneous infinite acceleration, which will trip your motor drivers (over-current protection).
**Tip:** Always use **smooth trajectory interpolation** (e.g., cubic splines) between IK waypoints.
:::

---

## 11.3 The Inverted Pendulum Model

To make a robot walk, we approximate it as an **Inverted Pendulum**.
*   **CoM:** Center of Mass (the hips/torso).
*   **CoP:** Center of Pressure (the foot on the ground).

**Walking Strategy:**
1.  Shift CoM over the Left Foot.
2.  Lift Right Foot (Swing Phase).
3.  "Fall" forward.
4.  Catch yourself with the Right Foot (Stance Phase).

---

## 11.4 Zero Moment Point (ZMP)

The **Zero Moment Point (ZMP)** is the point on the ground where the tipping moment is zero.
*   **Stable:** If ZMP is inside the support polygon (footprint), the robot doesn't tip.
*   **Unstable:** If ZMP leaves the footprint, the robot begins to rotate (fall).

**Walking Algorithms** (like Linear Inverted Pendulum Mode - LIPM) try to keep the ZMP strictly within the support polygon.

---

## 11.5 Hands-on: Designing a Gait (Conceptual)

1.  **Stance Leg:** Maintain high stiffness (P-gain).
2.  **Swing Leg:** Follow a bezier curve trajectory (lift, move forward, place).
3.  **Torso:** Move sinusoidally left/right to keep CoM over the Stance Leg.

**Next Week:** We will move from legs to hands. How do we interact with the world? We'll explore **Manipulation and Interaction Dynamics**.