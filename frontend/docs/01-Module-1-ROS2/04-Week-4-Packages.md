---
title: Week 4 - Packages
sidebar_label: Week 4 - Packages
---

# Week 4: Packaging and Launching

> **Learning Objective:** Now that we understand how ROS 2 components communicate, this week focuses on how to organize our code into reusable packages and automate their startup with launch files.

## 4.1 ROS 2 Packages: Organizing Your Code

A **ROS 2 Package** is the fundamental unit of software organization. It contains source code (Python, C++), build files, message definitions, launch files, and other resources.

### Why Use Packages?

*   **Modularity:** Encapsulate related functionalities.
*   **Reusability:** Easily share and reuse code across projects or with the community.
*   **Dependency Management:** Declare external dependencies, making your code easier to build and run.

### Package Structure (Python Example)

A typical Python ROS 2 package usually contains:

```
my_robot_package/
├── src/
│   └── my_robot_package/
│       ├── __init__.py
│       └── robot_node.py  # Your Python ROS 2 nodes
├── resource/
│   └── my_robot_package   # Marker file for ament to find resources
├── package.xml            # Package metadata and dependencies
├── setup.py               # Python build instructions
├── setup.cfg              # Python build configuration
├── launch/
│   └── my_robot_launch.py # Launch files
├── config/
│   └── params.yaml        # Configuration parameters
└── CMakeLists.txt         # (For C++ packages, or if using ament_python)
```

### `package.xml`: The Manifest

This XML file describes your package: its name, version, description, maintainers, license, and most importantly, its **dependencies**.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Python Entry Points: `setup.py`

For Python packages, `setup.py` defines how your Python nodes are installed and made executable. The `entry_points` section is crucial.

```python
from setuptools import find_packages, setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node = my_robot_package.robot_node:main', # Maps executable name to Python function
            'talker = my_robot_package.talker:main', # From Week 3 example
            'listener = my_robot_package.listener:main', # From Week 3 example
        ],
    },
)
```

---

## 4.2 Launch Files: Automating Startup

**Launch Files** are the orchestrators of a ROS 2 system. They allow you to:

*   Start multiple nodes simultaneously.
*   Set parameters for nodes.
*   Remap topics and services.
*   Include other launch files (for complex systems).

### Python Launch Files (Recommended)

ROS 2 recommends Python-based launch files for their flexibility and readability.

```python
# my_robot_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='robot_node',
            name='my_robot_controller',
            output='screen',
            emulate_tty=True, # Required for colored output
            parameters=[
                {'robot_id': 'humanoid_01'},
                {'debug_mode': False}
            ]
        ),
        Node(
            package='my_robot_package',
            executable='talker',
            name='simple_talker',
            output='screen'
        ),
        Node(
            package='my_robot_package',
            executable='listener',
            name='simple_listener',
            output='screen'
        )
    ])
```

### Running a Launch File

```bash
ros2 launch my_robot_package my_robot_launch.py
```

---

## 4.3 Python Agents: Bridging ROS 2 and Application Logic

The ROS 2 ecosystem primarily focuses on low-level robotics middleware. For higher-level, more complex behaviors (like our RAG chatbot's interaction with the robot), we often use **Python Agents**. These are Python programs that interact with ROS 2 nodes using `rclpy` but might also leverage other Python libraries like `LangChain`, `FastAPI`, etc.

:::note[RTX-Required]
### 🖥️ Sim-side Agents

In a simulated environment, your Python agents might directly access simulation APIs (e.g., Isaac Sim's Python API) to control the robot without going through the full ROS 2 stack, or they might orchestrate complex scenarios.
:::

:::note[Jetson-Compatible]
### 🤖 Edge-side Agents

On the Jetson, Python agents are crucial for running ML inference, managing power states, or interacting with cloud services. They act as a bridge between the real-time ROS 2 system and higher-level AI decision-making. Optimizing their Python dependencies (using Poetry!) is key.
:::

---

## 4.4 Hands-on: Creating Your First Package

1.  **Create a Workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  **Create a New Package:**
    ```bash
    ros2 pkg create --build-type ament_python my_robot_package --dependencies rclpy std_msgs
    ```

3.  **Add `talker` and `listener` from Week 3.**
    Copy your `minimal_publisher.py` and `minimal_subscriber.py` into `my_robot_package/my_robot_package/` and rename them to `talker.py` and `listener.py`.

4.  **Update `setup.py`:**
    Modify the `entry_points` as shown in Section 4.1.

5.  **Build the Package:**
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_robot_package
    ```
    *Note: If building inside your Poetry environment, use the shim script we deferred for Phase 2!*

6.  **Source the Workspace:**
    ```bash
    . install/setup.bash
    ```

7.  **Run the Launch File:**
    Create `my_robot_launch.py` in `my_robot_package/launch/` and run it.
    ```bash
    ros2 launch my_robot_package my_robot_launch.py
    ```

**Next Week:** We will explore more advanced communication patterns like Actions and Interfaces, moving beyond simple topics and services.