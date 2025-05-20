# Robotic Mice DXARTS Exhibit

This repository documents the Robotic Mice project, an interactive art installation for the University of Washington's DXARTS program. The exhibit features 25 autonomous robot mice that navigate their environment, avoid human presence, and seek out darkness.

<p align="center">
  <img src="media/mice_exhibit.png" width="200" />
  <img src="media/mice_lab.jpg" width="200" />
  <img src="media/poster.jpg" width="200" />
</p>

---

## Overview

The Robotic Mice project explores the intersection of robotics, art, and emergent behavior. Each mouse is equipped with sensors and programmed to:

- Move autonomously within the exhibit space
- Avoid humans and other obstacles
- Seek out darker areas, simulating natural rodent behavior

The collective movement of the mice creates a dynamic, unpredictable environment that invites audience interaction and reflection on the boundaries between artificial and natural life.

---

## Team

| Name            | Group           |
|-----------------|----------------|
| Sep Makhsous    | ECE Professor  |
| Eunsun Choi     | DXARTS Student |
| Ethan Widger    | ECE Student    |
| Daniela Berreth | CSE Student    |
| Nathan Cannell  | ECE Student    |
| Martin Li       | ECE Student    |

---

## System Architecture

The project leverages ROS 2 for distributed control and communication between the 25 robot mice. Key components include:

- **Autonomous Navigation:** Each mouse runs its own navigation stack, using sensor input to avoid obstacles and seek darkness.
- **Dynamic Namespace Assignment:** Mice are dynamically assigned unique namespaces for communication using an ID service.
- **Multi-Robot Coordination:** Centralized "mux" nodes aggregate status information (e.g., fuel/battery, presence) from all mice for monitoring and exhibit control.
- **Localization:** Each mouse runs an EKF-based localization node within its namespace to estimate its pose.

---

## Main Code Structure

### 1. Launching the System

The system is launched using ROS 2 launch files that start the ID assignment service, presence and fuel monitoring muxes, and localization nodes for each mouse.

**`mouse.launch.py` (excerpt):**
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
ld = LaunchDescription()
mouse_id_service = Node(
    package='mouse_global',
    executable='id_service',
    name='id_service',
)
ld.add_action(mouse_id_service)

presence_mux = Node(
    package='mouse_global',
    executable='presence_mux',
    name='presence',
)
ld.add_action(presence_mux)

fuel_mux = Node(
    package='mouse_global',
    executable='fuel_monitor',
    name='fuel_monitor',
)
ld.add_action(fuel_mux)

return ld
[6]

### 2. Mouse Namespace Management

Namespaces are assigned dynamically to each mouse using a custom ROS service that ensures each robot receives a unique identifier.

**`id_srv.py` (excerpt):**
class ID_Assigner(Node):
def init(self):
super().init('mouse_global')
self.srv = self.create_service(Id, 'id_assign', self.id_request_callback)
[2]

### 3. Presence and Fuel Monitoring

Centralized mux nodes subscribe to all `/mouse_X/presence` and `/mouse_X/fuel` topics, aggregating this data for exhibit monitoring and safety.

- **PresenceMUX:** Publishes a combined presence signal and logs activity of each mouse.
- **FuelMUX:** Monitors battery levels and warns if any mouse is critically low on power.

**`presence_mux.py` and `fuel_mux.py` (excerpts):**
PresenceMUX subscribes to all /mouse_X/presence topics and publishes /presence_mux
class PresenceMUX(Node):
...
def connect(self):
mouse_topics = get_mouse_topics(self, "presence") [4]
# Subscribe to each topic

FuelMUX subscribes to all /mouse_X/fuel topics and monitors battery levels
class FuelMUX(Node):
...
def connect(self):
mouse_topics = get_mouse_topics(self, "fuel") [1]

### 4. Localization

Each mouse runs an EKF-based localization node within its namespace, as defined in `localization.launch.py`.

**`localization.launch.py` (excerpt):**
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
ld = LaunchDescription()
mouse_namespaces = ['/mouse_1', '/mouse_2']
for namespace in mouse_namespaces:
robot_localization_node = Node(
package='robot_localization',
executable='ekf_node',
name='ekf_filter_node',
namespace=namespace)

ld.add_action(robot_localization_node)

return ld
[5]

---

## How to Run

1. **Install Dependencies:** Ensure ROS 2 and all required Python packages are installed.
2. **Build the Workspace:**
colcon build
source install/setup.bash
3. **Launch the System:**
ros2 launch mouse.launch.py
This will start the ID service, monitoring muxes, and (with additional configuration) the navigation and localization nodes for each mouse.

---

## Customization

- **Number of Mice:** The system supports dynamic scaling. Update the list of namespaces or use the ID service to add/remove mice as needed.
- **Behavior Tuning:** Modify the navigation and sensor processing scripts to adjust how mice respond to light, obstacles, or presence.

---

## Acknowledgments

This project was made possible by the collaborative efforts of students and faculty from UW's DXARTS, ECE, and CSE departments.

---

## Contact

For questions or collaboration inquiries, please contact the project team via the University of Washington DXARTS program.

---

**Enjoy exploring the emergent, collective behavior of our robotic mice!**


