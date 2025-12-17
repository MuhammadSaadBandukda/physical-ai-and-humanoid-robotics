---
title: Week 6 - Sim Basics
sidebar_label: Week 6 - Sim Basics
---

# Week 6: Building the Digital Twin

> **Learning Objective:** Before we build a physical robot, we must build it digitally. This week covers the Universal Robot Description Format (URDF) and how to bring it to life in a physics simulator (Gazebo).

## 6.1 The "Digital Twin" Concept

A **Digital Twin** is a virtual replica of a physical system. In robotics, it serves three purposes:
1.  **Design Validation:** Will the parts fit? Will the motors be strong enough?
2.  **Algorithm Testing:** Test navigation and control code safely without breaking expensive hardware.
3.  **Data Generation:** Generate synthetic training data for AI models (Module 3).

---

## 6.2 URDF: The XML of Robotics

**URDF (Unified Robot Description Format)** is an XML file format used to specify the kinematics, dynamics, and visual representation of a robot.

### Core Components:

1.  **Links:** The rigid bodies (e.g., "forearm", "wheel").
2.  **Joints:** The moving connections between links (e.g., "revolute", "prismatic").

```xml
<!-- Simple Robot Arm Link -->
<link name="forearm">
  <visual>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <material name="blue"/>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>
```

### Visual vs. Collision vs. Inertial

*   **Visual:** What you see (High-poly meshes, colors).
*   **Collision:** What the physics engine "feels" (Simple shapes: boxes, cylinders). *Always keep this simple to save CPU!*
*   **Inertial:** Mass and distribution of mass. *Critical for dynamic simulation.*

---

## 6.3 SDF: Simulation Description Format

While URDF is standard for ROS 2 tools (Rviz), Gazebo prefers **SDF**.
*   **URDF:** Describes *one* robot.
*   **SDF:** Describes the *entire world* (robot + lighting + terrain + friction).

*Note: Modern ROS 2 (Gazebo Harmonic) can ingest URDF directly, but understanding SDF is useful for world building.*

---

## 6.4 Hands-on: Your First Robot "Bot-V1"

Let's define a simple 2-wheeled robot.

**1. Create the URDF file:** `my_bot.urdf`

```xml
<robot name="bot_v1">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Add Wheels here (Joints + Links) -->
  <!-- ... -->
</robot>
```

:::note[RTX-Required]
### 🖥️ Spawning in Gazebo

To see your robot in the physics simulator:

1.  **Launch Gazebo:** `ros2 launch ros_gz_sim gz_sim.launch.py`
2.  **Spawn Entity:**
    ```bash
    ros2 run ros_gz_sim create -topic robot_description -entity bot_v1
    ```
You will see your green box drop to the ground. If you didn't define inertia, it might float or explode!
:::

:::note[Jetson-Compatible]
### 🤖 Using URDF on Real Hardware

Why do we need URDF on a physical Jetson?
**The Robot State Publisher.**

The real robot doesn't "know" what it looks like. It only knows encoder values (joint angles). The `robot_state_publisher` node reads the URDF and the encoder values to publish the **TF Tree** (Coordinate Transforms).

Without URDF, your Jetson cannot calculate:
*   Where the camera is relative to the wheels.
*   Where the gripper is relative to the base.

**Command:**
```bash
ros2 launch my_robot_description display.launch.py
```
This typically starts `robot_state_publisher` and `joint_state_publisher`.
:::

---

## 6.5 The Physics Engine

Gazebo simulates Newton's laws.

*   **Gravity:** Things fall.
*   **Friction:** Wheels need friction to move. If `mu1` and `mu2` are 0, your wheels will spin in place.
*   **Contacts:** When two `<collision>` geometries overlap, a restorative force is applied.

**Common Pitfall:** "Exploding Robots".
If two links are initialized overlapping each other, the physics engine calculates infinite force to separate them, launching your robot into orbit. **Solution:** Check your Joint origins!

**Next Week:** We will make our digital twin smarter by adding simulated sensors (Cameras, LiDAR) and bridging them to ROS 2.