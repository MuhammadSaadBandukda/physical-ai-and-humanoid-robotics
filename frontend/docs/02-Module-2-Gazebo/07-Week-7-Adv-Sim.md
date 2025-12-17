---
title: Week 7 - Adv Sim
sidebar_label: Week 7 - Adv Sim
---

# Week 7: Advanced Simulation and HRI Introduction

> **Learning Objective:** Having built a basic robot model, this week we add complexity. We'll learn to integrate simulated sensors into our digital twin and get a brief introduction to using more advanced environments like Unity for Human-Robot Interaction (HRI).

## 7.1 Simulating Sensors in Gazebo

Bringing our Week 2 knowledge of sensors into the simulation. Just like real sensors, simulated sensors publish data to ROS 2 topics.

### 7.1.1 Simulated Cameras (RGB)

The `gazebo_ros_camera` plugin allows you to add a virtual camera to your URDF model. It publishes `sensor_msgs/msg/Image` and `sensor_msgs/msg/CameraInfo`.

```xml
<!-- Example: Simulated RGB Camera -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <alwaysOn>true</alwaysOn>
    <visualize>true</visualize>
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <cameraName>robot/camera</cameraName>
      <frameName>camera_link_optical</frameName>
      <hackBaseline>0.07</hackBaseline>
      <distortionK1>0.0</distortionK1>
      <distortionK2>0.0</distortionK2>
      <distortionK3>0.0</distortionK3>
      <distortionT1>0.0</distortionT1>
      <distortionT2>0.0</distortionT2>
      <ros>
        <namespace>camera</namespace>
        <argument>--ros-args --remap __tf_prefix:=robot</argument>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### 7.1.2 Simulated Depth Cameras (RGB-D)

For depth sensing, similar plugins like `gazebo_ros_openni_kinect` (or direct camera plugins with depth output) can be used. These publish `sensor_msgs/msg/PointCloud2`.

### 7.1.3 Simulated LiDAR

As covered in Week 2, the `gazebo_ros_ray_sensor` plugin is used. When integrating into your full robot URDF, ensure the sensor link is properly attached to a base link via a fixed joint.

### 7.1.4 Simulated IMU

The `gazebo_ros_imu_sensor` plugin provides realistic inertial data.

```xml
<!-- Example: Simulated IMU -->
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <alwaysOn>true</alwaysOn>
    <update_rate>100.0</update_rate>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <namespace>imu</namespace>
        <argument>--ros-args --remap __tf_prefix:=robot</argument>
      </ros>
      <initialOrientationAsReference>false</initialOrientationAsReference>
      <frameName>base_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

---

## 7.2 Physics Control: Making the Robot Move

To make our simulated robot move, we need to apply forces or torques to its joints. ROS 2 provides `ros2_control` as a standardized way to interface with hardware (or simulated hardware).

### 7.2.1 `ros2_control` Integration

`ros2_control` is a set of packages that provide:
*   **Controllers:** PID, position, velocity, effort.
*   **Hardware Interfaces:** How to talk to motors/simulators.
*   **Controller Manager:** Manages multiple controllers.

```xml
<!-- Example: Integrating ros2_control into URDF -->
<ros2_control name="MyRobotHardware" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity">
      <param name="min">...</param>
      <param name="max">...</param>
    </command_interface>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
  <!-- ... other joints -->
</ros2_control>
```

### 7.2.2 PID Control

PID (Proportional-Integral-Derivative) controllers are fundamental for achieving stable motion. You'll typically configure these in a YAML file that `ros2_control` loads.

```yaml
# controller_params.yaml
controller_manager:
  ros__parameters:
    update_rate: 100 # Hz

diff_drive_controller:
  ros__parameters:
    left_wheel_name: 'left_wheel_joint'
    right_wheel_name: 'right_wheel_joint'
    # ... PID gains (p, i, d) for velocity control
```

---

## 7.3 Human-Robot Interaction (HRI) Introduction

While Gazebo is excellent for physics simulation, its visualization capabilities are limited for complex Human-Robot Interaction (HRI) scenarios or high-fidelity rendering.

This is where game engines like **Unity** or **Unreal Engine** come in.

### 7.3.1 Bridging Simulators

Tools like **ROS#** (for Unity) or **Isaac Sim** (which uses Omniverse, a USD-based platform) allow you to stream ROS 2 data (sensor feeds, robot poses) into a high-fidelity rendering environment.

:::tip
**Realism vs. Performance:** High-fidelity graphics are very demanding. For training machine learning models, simplified graphics are often sufficient. For user experience and human-robot collaboration, realism can be crucial.
:::

**Next Module:** Having mastered both the ROS 2 software stack and building/simulating our robot, we are ready to dive into **Module 3: The AI-Robot Brain (Isaac)**, where we'll explore advanced perception and Reinforcement Learning.