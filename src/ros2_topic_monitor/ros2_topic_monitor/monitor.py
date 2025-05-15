#!/usr/bin/env python3

"""
Apache 2.0 License

Author: Ibrahim Hroob (ihroob@lincoln.ac.uk)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import os
import re
import yaml
import time
import math
import rclpy
import math
import importlib
import tkinter as tk
from rclpy.node import Node
from rclpy import Parameter
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
import threading
import subprocess

import json
from std_msgs.msg import String

import threading
import subprocess
import time

class MultiTopicHzMonitor:
    def __init__(self, topic_names):
        self.rate_re = re.compile(r"average rate:\s*([\d\.]+)")
        self.rates = {topic: None for topic in topic_names}
        self.last_msg_time = {topic: 0 for topic in topic_names}

        for topic in topic_names:
            t = threading.Thread(target=self._run_hz, args=(topic,))
            t.daemon = True
            t.start()

            # Also start a watchdog thread per topic
            watchdog = threading.Thread(target=self._watch_topic, args=(topic,))
            watchdog.daemon = True
            watchdog.start()

    def _run_hz(self, topic):
        cmd = ["ros2", "topic", "hz", topic]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, text=True)
        for line in proc.stdout:
            m = self.rate_re.search(line)
            if m:
                self.rates[topic] = float(m.group(1))
                self.last_msg_time[topic] = time.time()

    def _watch_topic(self, topic, timeout=2.0):
        while True:
            time.sleep(timeout)
            last = self.last_msg_time.get(topic, 0)
            if time.time() - last > timeout:
                self.rates[topic] = 0.0  # Set to 0 if no messages in `timeout` seconds

    def get_rate(self, topic):
        return self.rates.get(topic, 0.0)

    def get_all_rates(self):
        return dict(self.rates)


class CheckTopicsGui(Node):
    def __init__(self):
        super().__init__('topics_monitor_gui_node')

        # Params
        self.declare_parameter('cfg_pth', Parameter.Type.STRING)
        self.cfg_pth = self.get_parameter('cfg_pth').get_parameter_value().string_value

        package_share_directory = get_package_share_directory('ros2_topic_monitor')
        cfg_pth_default = os.path.join(package_share_directory, 'config', 'cfg_topics_monitor.yaml')
        self.cfg_pth = self.cfg_pth or cfg_pth_default

        self.button_width = 22
        self.font_size = 12

        # Setup flags
        self.robot_status = {}
        self.recording = False
        self.gnss_status = -1
        self.gnss_service = 1

        # Add dictionaries to hold the latest metrics
        self.battery_level = None
        self.traveled_distance = None
        self.robot_speed = None
        
        # Sensor topics list 
        self.topics = []
        self.sensors_hz_monitor = None

        # Load configuration from YAML file
        try:
            with open(self.cfg_pth, 'r') as file:
                self.config = yaml.safe_load(file)
        except FileNotFoundError:
            self.get_logger().error(f"Configuration file not found: {self.cfg_pth}")
            self.config = {}

        # Setup tkinter GUI
        self.gui = tk.Tk()
        self.gui.title("TOPICS MONITOR")
        self.gui.protocol("WM_DELETE_WINDOW", self.window_closing)

        # Setup frames
        self.sensors_frame = self.create_label_frame(self.gui, "HEALTH", row=0, column=0)
        self.gnss_status_frame = self.create_label_frame(self.gui, "GNSS STATUS", row=1, column=0)
        self.robot_status_frame = self.create_label_frame(self.gui, "STATUS", row=2, column=0)
        self.recording_frame = self.create_label_frame(self.gui, "RECORDING", row=3, column=0)
        self.exit_frame = self.create_label_frame(self.gui, "", row=4, column=0)

        self.subscribers = []
        self.setup_sensors()
        self.setup_gnss_status()
        self.setup_robot_status()
        self.setup_recording()
        self.setup_exit_button()

        # Timer to update GUI
        self.timer = self.create_timer(2, self.update_gui)

        # Initialize ROS 2 context
        self.is_initialized = True

        # Publisher for JSON topic status
        self.topic_status_publisher = self.create_publisher(
            String, '/monitor/status', qos_profile_sensor_data
        )

    def get_topic_by_sensor_name(self, name):
        for sensor in self.config['sensors']:
            if sensor['name'] == name:
                return sensor['topic']
        return None  # or raise an error if not found

    def create_label_frame(self, parent, text='', row=0, column=0, columnspan=1):
        frame = tk.LabelFrame(parent, text=text, font=("Arial Bold", self.font_size + 1))
        
        # Add to grid with expansion in both directions
        frame.grid(row=row, column=column, padx=10, pady=10, sticky="ew", columnspan=columnspan)
        
        # Configure columns in the frame to expand and center content
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_rowconfigure(0, weight=1)
        
        return frame

    def create_button(self, frame, text, row, column, command=None, background="red", activebackground="light gray", anchor="center"):
        button = tk.Button(frame, text=text, command=command, width=self.button_width, background=background,
                           activebackground=activebackground, font=("Arial Bold", self.font_size), anchor=anchor)
        button.grid(row=row, column=column, padx=5, pady=5)
        return button

    def setup_sensors(self):
        # Check if 'sensors' exists in the config
        if 'sensors' not in self.config:
            self.get_logger().warn("'sensors' key not found in config. No sensors will be set up.")

            return  # Exit the function early

        for index, sensor in enumerate(self.config['sensors']):
            button = self.create_button(self.sensors_frame, sensor['name'], row=index, column=0)
            setattr(self, f"{sensor['name'].lower().replace(' ', '_')}_button", button)

            self.topics.append(sensor['topic'])
        
        self.sensors_hz_monitor = MultiTopicHzMonitor(self.topics)


    def setup_gnss_status(self):
        # Check if 'gnss_status' exists in the config
        if 'gnss_status' not in self.config:
            self.get_logger().warn("'gnss_status' key not found in config. GNSS status will not be set up.")
            return  # Exit the function early

        gnss_status_config = self.config['gnss_status']

        # Check for required keys in gnss_status_config
        if 'message_type' not in gnss_status_config or 'topic' not in gnss_status_config:
            self.get_logger().warn("'message_type' or 'topic' key not found in 'gnss_status' config. GNSS status will not be set up.")
            return  # Exit if necessary keys are missing

        self.gnss_status_button = self.create_button(self.gnss_status_frame, text='NO FIX', row=0, column=1)
        self.gnss_service_button = self.create_button(self.gnss_status_frame, text='GPS', row=1, column=1)
        self.gnss_std_east = self.create_button(self.gnss_status_frame, text='STD_E', row=2, column=1)
        self.gnss_std_north = self.create_button(self.gnss_status_frame, text='STD_N', row=3, column=1)

        msg_type = self.import_message_type(gnss_status_config['message_type'])
        
        try:
            self.gnss_status_subscriber = self.create_subscription(
                msg_type, 
                gnss_status_config['topic'], 
                self.gnss_status_callback, 
                qos_profile_sensor_data
            )
        except Exception as e:
            self.get_logger().error(f"Error creating GNSS status subscription: {e}")

    def setup_robot_status(self):
        # Check if 'robot_status' exists in the config
        if 'robot_status' not in self.config:
            # Log a warning or error message
            self.get_logger().warn("'robot_status' key not found in config. Using default settings.")
            self.robot_status = {}  # or set it to a default value if necessary
            return  # Exit the function early

        # If it exists, proceed as normal
        for index, status in enumerate(self.config['robot_status']):
            self.robot_status[status['name']] = 0.0
            button = self.create_button(self.robot_status_frame, status['name'], row=index, column=0, anchor="w", background='grey')
            setattr(self, f"{status['name'].lower().replace(' ', '_')}_button", button)
            
            msg_type = self.import_message_type(status['message_type'])
            callback = self.create_status_callback(status['name'])
            
            try:
                subscriber = self.create_subscription(msg_type, status['topic'], callback, qos_profile_sensor_data)
                self.subscribers.append(subscriber)
            except Exception as e:
                self.get_logger().error(f"error creating subscription for topic '{status['topic']}': {e}")


    def setup_recording(self):
        # Check if 'recording' exists in the config
        if 'recording' not in self.config:
            self.get_logger().warn("Warning: 'recording' key not found in config. Recording will not be set up.")

            return  # Exit the function early

        recording_config = self.config['recording']

        # Check for required keys in recording_config
        if 'message_type' not in recording_config or 'topic' not in recording_config:
            self.get_logger().warn("Warning: 'message_type' or 'topic' key not found in 'recording' config. Recording will not be set up.")

            return  # Exit if necessary keys are missing

        self.recording_button = self.create_button(self.recording_frame, text="Recording\nin progress", row=0, column=2)

        msg_type = self.import_message_type(recording_config['message_type'])
        
        try:
            self.recording_subscriber = self.create_subscription(
                msg_type,
                recording_config['topic'],
                self.rosbag_recording_callback,
                qos_profile_sensor_data
            )
        except Exception as e:
            self.get_logger().error(f"Error creating recording subscription: {e}")

    def setup_exit_button(self):
        # Create the exit button with grey color
        self.exit_button = self.create_button(
            self.exit_frame, 
            text="Exit", 
            row=0, 
            column=0, 
            command=self.window_closing
        )
        self.exit_button.configure(
            background="grey",        # Sets the background color to grey
            activebackground="light gray"  # Sets the color when the button is pressed
        )
        # Make sure to set the columnspan in the grid configuration
        self.exit_button.grid(row=0, column=0, columnspan=3, padx=10, pady=10, sticky="ew")

        # Ensure the parent frame is correctly configured
        self.exit_frame.grid_columnconfigure(0, weight=1)
        self.exit_frame.grid_columnconfigure(1, weight=1)
        self.exit_frame.grid_columnconfigure(2, weight=1)

    def import_message_type(self, message_type_str):
        module_name, class_name = message_type_str.rsplit('.', 1)
        module = importlib.import_module(module_name)
        return getattr(module, class_name)

    def publish_json_status(self):
        status_data = {}

        for index, sensor in enumerate(self.config['sensors']):
            publish_rate = self.sensors_hz_monitor.get_rate(sensor['topic'])
            rate = publish_rate if publish_rate is not None else 0.0
            
            # Determine status
            status = "active" if rate != 0 else "inactive"

            status_data[sensor['name']] = {
                "status": status,
                "refresh_rate_hz": round(rate, 2)
            }

        # Add robot status
        status_data["robot_status"] = {
            key: round(value, 2) for key, value in self.robot_status.items()
        }

        # Convert to JSON
        json_message = json.dumps(status_data)

        # Publish the JSON string
        json_msg = String()
        json_msg.data = json_message
        self.topic_status_publisher.publish(json_msg)

    def create_status_callback(self, robot_status):
        def callback(msg):
            self.robot_status[robot_status] = msg.data
        return callback

    def window_closing(self):
        if self.is_initialized:
            try:
                self.gui.quit()  # Stop the Tkinter main loop
                self.is_initialized = False  # Mark as not initialized to avoid multiple shutdown attempts
                rclpy.shutdown()
            except Exception as e:
                self.get_logger().error(f"Error during shutdown: {e}")

    def dummy_callback(self):
        pass

    def gnss_status_callback(self, msg):
        self.gnss_status = msg.status.status
        self.gnss_service = msg.status.service
        # Extract diagonal terms (variance in m^2)
        cov_EE = msg.position_covariance[0]
        cov_NN = msg.position_covariance[4]
        cov_UU = msg.position_covariance[8]

        # Convert to standard deviation (σ) in meters
        self.std_E = math.sqrt(cov_EE)
        self.std_N = math.sqrt(cov_NN)
        self.std_U = math.sqrt(cov_UU)

        # self.get_logger().info(f'--- GPS Fix Received ---')
        # self.get_logger().info(f'Latitude: {msg.latitude:.8f}, Longitude: {msg.longitude:.8f}, Altitude: {msg.altitude:.2f} m')
        # self.get_logger().info(f'Standard deviation (XY): East = {self.std_E:.3f} m, North = {self.std_N:.3f} m')
        # self.get_logger().info(f'Standard deviation (Z): Up = {self.std_U:.3f} m')

        # Optionally, combine XY uncertainty as RMS
        self.xy_uncertainty = math.sqrt(self.std_E**2 + self.std_N**2)
        # self.get_logger().info(f'Combined XY plane pose uncertainty: {self.xy_uncertainty:.3f} m')

        # Extract diagonal terms (variance in m^2)
        cov_EE = msg.position_covariance[0]
        cov_NN = msg.position_covariance[4]
        cov_UU = msg.position_covariance[8]

        # Convert to standard deviation (σ) in meters
        self.std_E = math.sqrt(cov_EE)
        self.std_N = math.sqrt(cov_NN)
        self.std_U = math.sqrt(cov_UU)

    def update_gnss_status(self):
        if self.gnss_status == 2:
            text, color = 'GBAS FIX', "green"
        elif self.gnss_status == 1:
            text, color = 'SBAS FIX', "green"
        elif self.gnss_status == 0:
            text, color = 'FIX', "yellow"
        else:
            text, color = 'NO FIX', "red"

        if self.gnss_service == 8:
            service_text = 'GALILEO'
        elif self.gnss_service == 4:
            service_text = 'COMPASS'
        elif self.gnss_service == 2:
            service_text = 'GLONASS'
        else:
            service_text = 'GPS'
        
        self.gnss_status_button.configure(text=text, background=color, activebackground='light gray')
        self.gnss_service_button.configure(text='Service: '+service_text, background='lightblue', activebackground='light gray')

        std_e = f'STD_E: {self.std_E} m'
        self.gnss_std_east.configure(text=std_e, background='lightblue', activebackground='light gray')

        std_n = f'STD_N: {self.std_N} m'
        self.gnss_std_north.configure(text=std_n, background='lightblue', activebackground='light gray')
        
        std_e = f'STD_E: {self.std_E} m'
        self.gnss_std_east.configure(text=std_e, background='lightblue', activebackground='light gray')

        std_n = f'STD_N: {self.std_N} m'
        self.gnss_std_north.configure(text=std_n, background='lightblue', activebackground='light gray')

        self.gnss_status = -1

    def rosbag_recording_callback(self, msg):
        self.recording = True

    def update_sensors_frame_gui(self):
        
        for index, sensor in enumerate(self.config['sensors']):
            sensor_name = sensor['name']
            button = getattr(self, f"{sensor_name.lower().replace(' ', '_')}_button")

            # Calculate publish rate and update button text
            topic = self.get_topic_by_sensor_name(sensor_name)
            publish_rate = self.sensors_hz_monitor.get_rate(topic)
            rate = publish_rate if publish_rate is not None else 0.0
            button_text = f"{sensor_name} ({rate:.1f} Hz)"

            color = "green" if rate != 0 else "red"
            button.configure(text=button_text, background=color, activebackground='light gray')

    def update_status_frame_gui(self):
        for system_status, value in self.robot_status.items():
            button = getattr(self, f"{system_status.lower().replace(' ', '_')}_button")
            color = "coral" if value > 0 else "grey"

            # Update button text
            button_text = f"{system_status}: {value:.2f}"  # Update button text with publish rate

            button.configure(text=button_text, background=color, activebackground='light gray')        

    def update_recording_frame_gui(self):
        if 'recording' not in self.config:
            return
        color = "green" if self.recording else "red"
        self.recording_button.configure(background=color, activebackground='light gray')
        self.recording = False

    def update_gui(self):
        self.update_sensors_frame_gui()
        self.update_gnss_status()
        self.update_status_frame_gui()
        self.update_recording_frame_gui()
        
        try:
            self.gui.update()
        except tk.TclError:
            self.get_logger().warn("GUI closed unexpectedly.")
            rclpy.shutdown()
    
        self.publish_json_status()

def ros2_spin(node):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down ROS2 node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

def start_ros2_spin(node):
    threading.Thread(target=ros2_spin, args=(node,), daemon=True).start()

def main(args=None):
    rclpy.init(args=args)
    node = CheckTopicsGui()
    start_ros2_spin(node)  # spin in background
    node.gui.mainloop()

if __name__ == '__main__':
    main()