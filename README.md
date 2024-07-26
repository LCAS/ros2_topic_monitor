# ROS2 Topic Monitor

A simple GUI tool designed to check if a desired topic is being published. The interface will periodically update to reflect the current status.

## Installation

To get the package, please clone it into your ROS 2 workspace as follows:

```bash
cd <your_ros2_ws>/src
git clone https://github.com/LCAS/ros2_topic_monitor.git
cd ..
colcon build --packages-select ros2_topic_monitor
source install/setup.bash  
```

## Nodes

This tool consists of two nodes:
- **Monitoring Node**: Responsible for monitoring the topics.
- **Recording Node**: Handles recording of the topics.

## Monitoring node


![image](https://github.com/user-attachments/assets/a1b2d637-f962-49ba-a803-351cba5ac75c)

![image](https://github.com/user-attachments/assets/8e69149b-9cbc-45bd-8668-26686b359e35)

## Recording Node

![image](https://github.com/user-attachments/assets/9309e811-676f-405b-90a0-1efeb3c354b9)

