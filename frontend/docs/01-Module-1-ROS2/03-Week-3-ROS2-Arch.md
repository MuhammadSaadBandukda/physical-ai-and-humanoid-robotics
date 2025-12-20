---
title: Week 3 - ROS 2 Architecture
sidebar_label: Week 3 - ROS 2 Arch
---

# Week 3: The Robotic Nervous System (ROS 2)

> **Learning Objective:** With our robot sensing its environment, we now need a way for its various "brains" and "organs" to communicate. This week introduces the core architectural concepts of ROS 2: Nodes, Topics, and Services.

## 3.1 What is ROS 2?

**ROS 2 (Robot Operating System 2)** is not an operating system in the traditional sense (like Windows or Linux). It's a **middleware framework** that provides a standardized way for different software components to communicate and interact in a robotics system.

### Why ROS 2?

*   **Interoperability:** Connects hardware drivers, vision algorithms, path planners, and UI with minimal effort.
*   **Distribution:** Components can run on different machines (e.g., a laptop running a joystick, a Jetson running vision, a desktop running path planning).
*   **Tooling:** Rich ecosystem of tools for visualization (Rviz2), debugging (rqt_graph), and logging.
*   **Community:** Large, active community and open-source packages.

---

## 3.2 Nodes: The Worker Bees

A **Node** is an executable process in ROS 2. Each node performs a single, well-defined task.

*   *Analogy:* Think of a humanoid's brain. One node might be "vision processing," another "motor control," another "speech recognition."

### Key Characteristics:

*   **Modular:** Easily swap out one node (e.g., a simple PID controller) for a more complex one (e.g., an ML-based controller).
*   **Independent:** Nodes run as separate processes, improving fault tolerance. If one node crashes, the others can continue.

:::note[RTX-Required]
### 🖥️ Nodes in Simulation

In simulation, you might have many more nodes running, as CPU/GPU resources are abundant. For instance, a separate node for each simulated sensor, each simulated actuator, and multiple debugging tools.
:::

:::note[Jetson-Compatible]
### 🤖 Nodes on Physical Hardware

On the Jetson, resource constraints are critical. You'll want to combine functionalities into fewer, more efficient nodes where possible, or use techniques like **managed nodes** and **component containers** to optimize resource usage and startup times. Every background process consumes valuable CPU/RAM.
:::

---

## 3.3 Topics: The Communication Bus

**Topics** are named buses over which nodes exchange data using a publish/subscribe model.

*   **Publisher:** A node sends messages to a topic.
*   **Subscriber:** A node receives messages from a topic.

*   *Analogy:* Think of a shared whiteboard where anyone can write (publish) or read (subscribe) messages.

### Message Types

Every message transmitted over a topic has a specific **message type** (e.g., `sensor_msgs/msg/LaserScan`, `geometry_msgs/msg/Twist`). This defines the structure of the data.

### Example: Robot Control

*   `cmd_vel` topic (type: `geometry_msgs/msg/Twist`)
    *   **Publisher:** Navigation node (decides where to go).
    *   **Subscriber:** Motor control node (converts `Twist` commands to motor RPMs).

:::tip
**Minimizing Bandwidth on Jetson:** On physical robots, especially over Wi-Fi or limited serial links, excessive topic bandwidth can lead to latency and dropped messages. Only publish data when necessary, and consider compressing image/point cloud data.
:::

---

## 3.4 Services: Request-Response Interactions

While topics are for continuous, asynchronous data streams, **Services** are for request-response interactions. A client makes a request, and a server provides a response.

*   *Analogy:* Calling a function in a distributed system.

### Example: Arm Movement

*   `set_arm_pose` service (request: `Pose`, response: `bool success`)
    *   **Client:** Task planning node (wants to move the arm).
    *   **Server:** Arm control node (moves the arm and reports success/failure).

### Key Differences: Topics vs. Services

| Feature        | Topics                       | Services                          |
| :------------- | :--------------------------- | :-------------------------------- |
| **Model**      | Publish/Subscribe (Async)    | Request/Response (Sync)           |
| **Use Case**   | Continuous data streams      | Infrequent, specific actions      |
| **Guarantees** | Best-effort (usually)        | Guaranteed delivery of response   |
| **Blocking**   | Non-blocking                 | Client-blocking (waits for response) |

---

## 3.5 Hands-on: Building a Talker-Listener

Let's create two simple Python nodes:
1.  **`minimal_publisher` (Talker):** Publishes "Hello ROS!" messages to the `chatter` topic.
2.  **`minimal_subscriber` (Listener):** Subscribes to `chatter` and prints messages.

**Code:** (This code would live in `backend/src/ros_nodes` later)

:::note[Code Block]
```python
# minimal_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# minimal_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
:::

**Running the Nodes (within your Docker container):**

1.  **Terminal 1:** `ros2 run <your_package_name> minimal_publisher`
2.  **Terminal 2:** `ros2 run <your_package_name> minimal_subscriber`

**Verification:** In Terminal 2, you should see "I heard: 'Hello ROS 2! X'" messages.

**Next Week:** We will explore how to organize these nodes into **Packages** and manage their lifecycle with **Launch Files**.