---
title: Week 5 - Comms
sidebar_label: Week 5 - Comms
---

# Week 5: Advanced Communication Patterns (Actions & Interfaces)

> **Learning Objective:** We've mastered the basics of Nodes, Topics, and Services. This week, we elevate our communication skills by learning about ROS 2 Actions for long-running tasks and Custom Interfaces for defining our own message types.

## 5.1 Limitations of Topics & Services

While Topics and Services are powerful, they have limitations for certain use cases:

*   **Topics:** Ideal for continuous streams of data (e.g., sensor readings), but don't provide explicit feedback on whether data was processed or a task completed.
*   **Services:** Great for quick, blocking request/response calls (e.g., "get this data"), but not suitable for tasks that take a long time to complete and require intermediate feedback.

This is where **Actions** come in.

---

## 5.2 Actions: Long-Running Goals with Feedback

**Actions** are a high-level communication type in ROS 2 designed for long-running, interruptible tasks that require continuous feedback. Think of them as a hybrid between Topics and Services.

*   *Analogy:* Imagine ordering a complex coffee (the **Goal**). The barista gives you updates ("Grinding beans," "Steaming milk" - the **Feedback**), and eventually, you get your coffee (the **Result**).

### Action Components:

An Action interaction involves three primary components:

1.  **Goal:** The instruction sent to the Action Server (e.g., "Go to coordinates X, Y").
2.  **Feedback:** Periodic updates from the Action Server about its progress (e.g., "Currently at X-prime, Y-prime, 50% complete").
3.  **Result:** The final outcome of the action, sent once the task is complete (e.g., "Reached destination, distance remaining: 0.1m").

### Use Cases:

*   **Navigation:** "Go to the kitchen."
*   **Manipulation:** "Pick up the red block."
*   **Long-term Planning:** "Execute the inspection route."

### Action Client & Server

*   **Action Client:** Sends goals, receives feedback, and gets the final result.
*   **Action Server:** Receives goals, processes them, sends feedback, and returns a result.

---

## 5.3 Interfaces: Custom Message Types

So far, we've used standard ROS 2 message types (e.g., `std_msgs/msg/String`, `sensor_msgs/msg/LaserScan`). But what if your robot needs to communicate data that doesn't fit a standard type? This is where **Custom Interfaces** come in.

ROS 2 interfaces are defined in `.msg`, `.srv`, and `.action` files.

### Message (`.msg`) Files

Define the structure of data for Topics.

```
# MyCustomMessage.msg
std_msgs/Header header
string name
int32 id
float32[] data
```

### Service (`.srv`) Files

Define the request and response structure for Services. Separated by `---`.

```
# MyCustomService.srv
string request_message
---
bool success
string response_message
```

### Action (`.action`) Files

Define the goal, result, and feedback structure for Actions. Each section separated by `---`.

```
# MyCustomAction.action
# Goal
int32 target_value
---
# Result
int32 final_value
---
# Feedback
float32 current_progress
```

### Why Custom Interfaces?

*   **Specificity:** Precisely define the data your robot needs.
*   **Type Safety:** Ensures all nodes agree on the data structure.
*   **Code Generation:** ROS 2 tools automatically generate code in Python/C++ to handle these types.

:::tip
**Efficient Interfaces for Jetson:** When defining custom messages for a physical robot, be mindful of their size. Large messages (e.g., uncompressed image arrays in a custom type) can quickly saturate network bandwidth on an edge device like the Jetson. Design lean interfaces!
:::

---

## 5.4 Hands-on: Creating a Custom Action

Let's create a simple Action to count from 0 to a target value, providing feedback as it counts.

**1. Define the Action (`Countdown.action`)**

Create this file in `my_robot_package/action/Countdown.action`:

```
# my_robot_package/action/Countdown.action
# Goal
int32 target_count
---
# Result
int32 final_count
---
# Feedback
int32 current_count
```

**2. Update `package.xml`**

Add these dependencies for Action support:

```xml
  <depend>action_msgs</depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

**3. Update `CMakeLists.txt` (if present, for C++ or mixed packages)**

For Python-only packages, this is handled by `ament_python` and `setup.py`.

**4. Action Server (Python)**

```python
# my_action_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from my_robot_package.action import Countdown  # Import your custom action

class CountdownActionServer(Node):
    def __init__(self):
        super().__init__('countdown_action_server')
        self._action_server = ActionServer(
            self,
            Countdown,
            'countdown',
            self.execute_callback)
        self.get_logger().info('Countdown Action Server Ready.')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal: {0}'.format(goal_handle.request.target_count))

        feedback_msg = Countdown.Feedback()
        for i in range(goal_handle.request.target_count + 1):
            feedback_msg.current_count = i
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.current_count))
            goal_handle.publish_feedback(feedback_msg)
            await self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))

        goal_handle.succeed()
        result = Countdown.Result()
        result.final_count = goal_handle.request.target_count
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = CountdownActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**5. Action Client (Python)**

```python
# my_action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from my_robot_package.action import Countdown

class CountdownActionClient(Node):
    def __init__(self):
        super().__init__('countdown_action_client')
        self._action_client = ActionClient(self, Countdown, 'countdown')

    def send_goal(self, target_count):
        goal_msg = Countdown.Goal()
        goal_msg.target_count = target_count

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.final_count))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback.current_count))

def main(args=None):
    rclpy.init(args=args)
    action_client = CountdownActionClient()
    action_client.send_goal(10) # Send a goal to count to 10
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

**Running the Action:**

1.  **Build your package** (if you added the action definition): `colcon build --packages-select my_robot_package`
2.  **Source your workspace:** `. install/setup.bash`
3.  **Terminal 1:** `ros2 run my_robot_package my_action_server`
4.  **Terminal 2:** `ros2 run my_robot_package my_action_client`

You should see the client sending a goal, the server executing it and sending feedback, and finally the client receiving the result.

**Module 1 Complete!**
Congratulations, you've completed Module 1 of the Physical AI and Humanoid Robotics textbook. You now have a foundational understanding of ROS 2.

**Next Module:** We will move on to Module 2: The Digital Twin (Gazebo), where we will learn to build and simulate our robot in a virtual environment.