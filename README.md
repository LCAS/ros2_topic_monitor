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

>This package will be released later 

## Nodes

This tool consists of two nodes:

<details>
<summary>Monitoring Node</summary>

To launch the monitoring node, use the following command:
```bash
ros2 launch ros2_topic_monitor monitor.launch.py
```

The above command will launch the following GUI, where the photo on the left when there are no topics available and on the right with the available topics: 


<p align="center">
    <img src="https://github.com/user-attachments/assets/a1b2d637-f962-49ba-a803-351cba5ac75c" alt="Topics Not Being Published" width="400"/>
    <img src="https://github.com/user-attachments/assets/8e69149b-9cbc-45bd-8668-26686b359e35" alt="Topics Being Published" width="400"/>
</p>

The GUI can dynamically adapt to new topics by adding them to the configuration file [here](https://github.com/LCAS/ros2_topic_monitor/blob/main/src/ros2_topic_monitor/config/cfg_topics_monitor.yaml). All topics need to be added under the 'sensors' section. Here is an example:

```yaml
sensors:
  - name: "GPS odom"
    topic: "/gps_base/odometry"
    message_type: "nav_msgs.msg.Odometry"
```

Where the `name` attribute is used to display it in the GUI, and the `topic` is the topic that needs to be subscribed to. It is important to set the `message_type`.

>There is no need to explicitly set the 'QoS' as the node uses 'qos_profile_sensor_data' when it subscribes to the topic, as shown [here](https://github.com/LCAS/ros2_topic_monitor/blob/246a795e83bf199f854335bc6311876301ba9983/src/ros2_topic_monitor/ros2_topic_monitor/monitor.py#L112).
</details>


<details>
<summary>Recording Node</summary>

To launch the recording node, use the following command:
```bash
ros2 launch ros2_topic_monitor record.launch.py
```

While will give the following GUI where you can define the recording path and the bag name:

<p align="center">
    <img src="https://github.com/user-attachments/assets/9309e811-676f-405b-90a0-1efeb3c354b9" alt="Record GUI" width="400"/>
</p>

The topics to be recorded need to be added into [this](https://github.com/LCAS/ros2_topic_monitor/blob/main/src/ros2_topic_monitor/config/cfg_topics_record.yaml) config file.

</details>




