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
import yaml
import time
import rclpy
import importlib
import tkinter as tk
from rclpy.node import Node
from rclpy import Parameter
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
import threading


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
        self.sensors_status = {}
        self.robot_status = {}
        self.recording = False
        self.gnss_status = -1
        self.gnss_service = 1

        # Dictionary to store the message timestamps for each sensor
        self.sensors_timestamps = {}

        # Add dictionaries to hold the latest metrics
        self.battery_level = None
        self.traveled_distance = None
        self.robot_speed = None

        # Load configuration from YAML file
        with open(self.cfg_pth, 'r') as file:
            self.config = yaml.safe_load(file)

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
            print("Warning: 'sensors' key not found in config. No sensors will be set up.")
            self.sensors_status = {}  # Set a default value if necessary
            return  # Exit the function early

        for index, sensor in enumerate(self.config['sensors']):
            self.sensors_status[sensor['name']] = False
            button = self.create_button(self.sensors_frame, sensor['name'], row=index, column=0)
            setattr(self, f"{sensor['name'].lower().replace(' ', '_')}_button", button)

            # Use get method to avoid KeyError
            msg_type = self.import_message_type(sensor.get('message_type', 'default_message_type'))  # Default message type
            callback = self.create_callback(sensor['name'])

            try:
                subscriber = self.create_subscription(msg_type, sensor['topic'], callback, qos_profile=qos_profile_sensor_data)
                self.subscribers.append(subscriber)
            except Exception as e:
                print(f"Error creating subscription for sensor '{sensor['name']}': {e}")

    def setup_gnss_status(self):
        # Check if 'gnss_status' exists in the config
        if 'gnss_status' not in self.config:
            print("Warning: 'gnss_status' key not found in config. GNSS status will not be set up.")
            return  # Exit the function early

        gnss_status_config = self.config['gnss_status']

        # Check for required keys in gnss_status_config
        if 'message_type' not in gnss_status_config or 'topic' not in gnss_status_config:
            print("Warning: 'message_type' or 'topic' key not found in 'gnss_status' config. GNSS status will not be set up.")
            return  # Exit if necessary keys are missing

        self.gnss_status_button = self.create_button(self.gnss_status_frame, text='NO FIX', row=0, column=1)
        self.gnss_service_button = self.create_button(self.gnss_status_frame, text='GPS', row=1, column=1)

        msg_type = self.import_message_type(gnss_status_config['message_type'])
        
        try:
            self.gnss_status_subscriber = self.create_subscription(
                msg_type, 
                gnss_status_config['topic'], 
                self.gnss_status_callback, 
                qos_profile=qos_profile_sensor_data
            )
        except Exception as e:
            print(f"Error creating GNSS status subscription: {e}")

    def setup_robot_status(self):
        # Check if 'robot_status' exists in the config
        if 'robot_status' not in self.config:
            # Log a warning or error message
            print("Warning: 'robot_status' key not found in config. Using default settings.")
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
                subscriber = self.create_subscription(msg_type, status['topic'], callback, qos_profile=qos_profile_sensor_data)
                self.subscribers.append(subscriber)
            except Exception as e:
                print(f"Error creating subscription for topic '{status['topic']}': {e}")

    def setup_recording(self):
        # Check if 'recording' exists in the config
        if 'recording' not in self.config:
            print("Warning: 'recording' key not found in config. Recording will not be set up.")
            return  # Exit the function early

        recording_config = self.config['recording']

        # Check for required keys in recording_config
        if 'message_type' not in recording_config or 'topic' not in recording_config:
            print("Warning: 'message_type' or 'topic' key not found in 'recording' config. Recording will not be set up.")
            return  # Exit if necessary keys are missing

        self.recording_button = self.create_button(self.recording_frame, text="Recording\nin progress", row=0, column=2)

        msg_type = self.import_message_type(recording_config['message_type'])
        
        try:
            self.recording_subscriber = self.create_subscription(
                msg_type,
                recording_config['topic'],
                self.rosbag_recording_callback,
                qos_profile=qos_profile_sensor_data
            )
        except Exception as e:
            print(f"Error creating recording subscription: {e}")

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

    def create_callback(self, sensor_name):
        def callback(msg):
            self.sensors_status[sensor_name] = True

            # Track message arrival time for calculating publish rate
            if sensor_name not in self.sensors_timestamps:
                self.sensors_timestamps[sensor_name] = []
            
            current_time = time.time()
            self.sensors_timestamps[sensor_name].append(current_time)

            # Keep only the last few timestamps (for example, last 10 messages)
            if len(self.sensors_timestamps[sensor_name]) > 10:
                self.sensors_timestamps[sensor_name].pop(0)

        return callback

    def create_status_callback(self, robot_status):
        def callback(msg):
            self.robot_status[robot_status] = msg.data
        return callback

    def calculate_publish_rate(self, timestamps):
        """Calculates the publish rate (Hz) based on message timestamps."""
        if len(timestamps) < 2:
            return 0.0  # Not enough data to calculate rate

        # Calculate time differences between consecutive messages
        time_diffs = [timestamps[i] - timestamps[i - 1] for i in range(1, len(timestamps))]

        # Calculate the average time difference
        avg_time_diff = sum(time_diffs) / len(time_diffs) if time_diffs else 0

        # Calculate the rate in Hz
        publish_rate = 1.0 / avg_time_diff if avg_time_diff > 0 else 0
        return publish_rate

    def window_closing(self):
        if self.is_initialized:
            try:
                self.gui.quit()  # Stop the Tkinter main loop
                self.is_initialized = False  # Mark as not initialized to avoid multiple shutdown attempts
                rclpy.shutdown()
            except Exception as e:
                print(f"Error during shutdown: {e}")

    def dummy_callback(self):
        pass

    def gnss_status_callback(self, msg):
        self.gnss_status = msg.status.status
        self.gnss_service = msg.status.service

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
        
        self.gnss_status = -1

    def rosbag_recording_callback(self, msg):
        self.recording = True

    def update_sensors_frame_gui(self):
        for sensor_name, status in self.sensors_status.items():
            button = getattr(self, f"{sensor_name.lower().replace(' ', '_')}_button")
            color = "green" if status else "red"

            # Calculate publish rate and update button text
            timestamps = self.sensors_timestamps.get(sensor_name, [])
            publish_rate = self.calculate_publish_rate(timestamps)
            button_text = f"{sensor_name} ({publish_rate:.1f} Hz)"  # Update button text with publish rate

            button.configure(text=button_text, background=color, activebackground='light gray')

        # Reset status to be updated if we get new topics 
        self.sensors_status = {sensor_name: False for sensor_name in self.sensors_status}

    def update_status_frame_gui(self):
        for system_status, value in self.robot_status.items():
            button = getattr(self, f"{system_status.lower().replace(' ', '_')}_button")
            color = "coral" if value > 0 else "grey"

            # Update button text
            button_text = f"{system_status}: {value:.2f}"  # Update button text with publish rate

            button.configure(text=button_text, background=color, activebackground='light gray')        

    def update_recording_frame_gui(self):
        color = "green" if self.recording else "red"
        self.recording_button.configure(background=color, activebackground='light gray')
        self.recording = False

    def update_gui(self):
        self.update_sensors_frame_gui()
        self.update_gnss_status()
        self.update_status_frame_gui()
        self.update_recording_frame_gui()
        self.gui.update()


def ros2_spin(node):
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error during ROS 2 spin: {e}")


def main(args=None):
    rclpy.init(args=args)
    ctg = CheckTopicsGui()

    # Run ROS 2 spin in a separate thread
    ros2_thread = threading.Thread(target=ros2_spin, args=(ctg,))
    ros2_thread.start()

    try:
        ctg.gui.mainloop()  # Run Tkinter's main loop
    except KeyboardInterrupt:
        pass
    finally:
        ctg.window_closing()  # Call window_closing to ensure proper shutdown
        ros2_thread.join()  # Ensure the ROS 2 thread has finished

if __name__ == '__main__':
    main()
