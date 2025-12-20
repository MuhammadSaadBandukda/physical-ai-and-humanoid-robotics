---
title: Week 2 - Sensors
sidebar_label: Week 2 - Sensors
---

# Week 2: The Robot's Senses

> **Learning Objective:** A robot without sensors is a statue. This week, we give our agent "eyes" and "ears" to perceive the environment. We will cover the "Holy Trinity" of robotics perception: LiDAR, Depth Cameras, and IMUs.

## 2.1 Proprioception vs. Exteroception

Just like biological organisms, robots have two categories of senses:

1.  **Proprioception (Internal):** Sensing one's own body.
    *   *Examples:* Joint encoders (angles), Battery voltage, Motor temperature.
    *   *Role:* "Am I falling over?" "Is my arm extended?"
2.  **Exteroception (External):** Sensing the world.
    *   *Examples:* LiDAR, Cameras, Microphones.
    *   *Role:* "Is there a wall ahead?" "Where is the user?"

---

## 2.2 LiDAR (Light Detection and Ranging)

LiDAR is the primary sensor for mapping and navigation (SLAM). It shoots laser pulses and measures the time-of-flight to calculate distance.

:::note[RTX-Required]
### 🖥️ Simulating LiDAR in Gazebo

In simulation, we don't need a physical laser. We use a **Ray Sensor Plugin**.

**Key Concept:** "Perfect Data" vs. "Gaussian Noise".
In Gazebo, you can generate *perfect* distance measurements, but this is bad for ML training. Always add noise to your simulation!

```xml
<!-- Example URDF Snippet for Simulated LiDAR -->
<sensor name="lidar" type="ray">
  <plugin filename="libgazebo_ros_ray_sensor.so" name="lidar_plugin">
    <ros>
      <namespace>/sim</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```
:::

:::note[Jetson-Compatible]
### 🤖 Physical LiDAR (RPLidar / YDLidar)

On the Jetson, we use real hardware drivers. Most affordable robots use the **RPLidar A1/A2**.

**1. Permissions (The #1 Issue)**
Real hardware requires USB permissions.
```bash
sudo chmod 666 /dev/ttyUSB0
```

**2. Running the Driver**
We typically use the `sllidar_ros2` package.
```bash
ros2 launch sllidar_ros2 sllidar_launch.py serial_port:=/dev/ttyUSB0
```

**Real World Challenge:** *Sunlight Interference.* Infrared LiDARs often fail outdoors because the sun blinds them. Simulation rarely captures this!
:::

---

## 2.3 RGB-D Cameras (Depth)

For a humanoid to manipulate objects (VLA), LiDAR is not enough. We need **Color (RGB)** + **Depth (D)**.

*   **RGB:** Semantic understanding ("That is a cup").
*   **Depth:** Spatial understanding ("The cup is 0.5m away").

### The Standard: Intel RealSense
The industry standard for research is the Intel RealSense series (D435i, D455).

:::note[RTX-Required]
### 🖥️ Synthetic Cameras
Isaac Sim and Gazebo provide "Synthetic Data Generation" (SDG). We can render semantic segmentation masks directly from the GPU buffer—something impossible in the real world without heavy AI inference.
:::

:::note[Jetson-Compatible]
### 🤖 RealSense on Jetson
The `realsense-ros` wrapper is heavy. On a Jetson Orin Nano, processing 30fps Depth alignment can consume 20-30% of your CPU.

**Optimization Tip:**
Use **hardware-aligned** streams to offload the CPU.
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```
:::

---

## 2.4 IMU (Inertial Measurement Unit)

The IMU is the inner ear of the robot. It measures:
1.  **Accelerometers:** Linear acceleration (gravity).
2.  **Gyroscopes:** Angular velocity (rotation).
3.  **Magnetometers:** Heading (compass).

**Critical for Humanoids:** You cannot balance a bipedal robot without a high-frequency IMU (typically >200Hz).

---

## 2.5 Visualization: Rviz2

How do we know what the robot sees? We use **Rviz2** (ROS Visualization).

*   **LiDAR Data:** Appears as a ring of red dots (`sensor_msgs/LaserScan`).
*   **Camera Data:** Appears as a video feed (`sensor_msgs/Image`).
*   **TF (Transforms):** Shows where the sensors are mounted relative to the robot's center.

**Activity:**
1.  Launch your sensor node (Sim or Real).
2.  Open Rviz2: `ros2 run rviz2 rviz2`.
3.  Set "Fixed Frame" to `laser_frame` or `camera_link`.
4.  Add a "LaserScan" or "Image" display.

**Next Week:** Now that we have data flowing in, we need **Nodes** to process it. We will dive into the ROS 2 Architecture.