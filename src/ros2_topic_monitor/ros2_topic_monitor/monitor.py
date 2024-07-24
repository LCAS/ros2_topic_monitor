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
import rclpy
import importlib
import tkinter as tk
from rclpy.node import Node
from rclpy import Parameter
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory


class CheckTopicsGui(Node):
    def __init__(self):
        super().__init__('topics_monitor_gui_node')

        # Params
        self.declare_parameter('cfg_pth', Parameter.Type.STRING)
        self.cfg_pth = self.get_parameter('cfg_pth').get_parameter_value().string_value

        package_share_directory = get_package_share_directory('ros2_topic_monitor')
        cfg_pth_default = os.path.join(package_share_directory, 'config', 'cfg_topics_monitor.yaml')
        self.cfg_pth = self.cfg_pth or cfg_pth_default

        self.button_width = 11
        self.font_size = 11

        # Setup flags
        self.sensors_status = {}
        self.recording = False
        self.gnss_status = -1
        self.gnss_service = 1

        # Load configuration from YAML file
        with open(self.cfg_pth, 'r') as file:
            self.config = yaml.safe_load(file)

        # Setup tkinter gui
        self.gui = tk.Tk()
        self.gui.title("TOPICS MONITOR")
        self.gui.protocol("WM_DELETE_WINDOW", self.window_closing)

        # Setup frames
        self.sensors_frame = self.create_label_frame(self.gui, "SENSORS", row=0, column=0)
        self.gnss_status_frame = self.create_label_frame(self.gui, "GNSS STATUS", row=0, column=1)
        self.recording_frame = self.create_label_frame(self.gui, "RECORDING", row=0, column=2)

        self.subscribers = []
        self.setup_sensors()
        self.setup_gnss_status()
        self.setup_recording()

        self.timer = self.create_timer(2, self.update_gui)

    def create_label_frame(self, parent, text='', row=0, column=0):
        frame = tk.LabelFrame(parent, text=text, font=("Arial Bold", self.font_size + 1))
        frame.grid(row=row, column=column, padx=10, pady=10, sticky="w")
        return frame

    def create_button(self, frame, text, row, column):
        button = tk.Button(frame, text=text, command=self.dummy_callback, width=self.button_width, background="red",
                           activebackground="red", font=("Arial Bold", self.font_size))
        button.grid(row=row, column=column, padx=5)
        return button


    '''
        The sensors button interface are created dynamically based on the topics that need
        to be monitored. 
    '''
    def setup_sensors(self):
        for index, sensor in enumerate(self.config['sensors']):
            self.sensors_status[sensor['name']] = False
            button = self.create_button(self.sensors_frame, sensor['name'], row=index, column=0)
            setattr(self, f"{sensor['name'].lower().replace(' ', '_')}_button", button)
            msg_type = self.import_message_type(sensor['message_type'])
            callback = self.create_callback(sensor['name'])
            subscriber = self.create_subscription(msg_type, sensor['topic'], callback, qos_profile=qos_profile_sensor_data)
            self.subscribers.append(subscriber)

    def setup_gnss_status(self):
        gnss_status_config = self.config['gnss_status']
        self.gnss_status_button = self.create_button(self.gnss_status_frame, text='NO FIX', row=0, column=1)
        self.gnss_service_button = self.create_button(self.gnss_status_frame, text='GPS'  , row=1, column=1)

        msg_type = self.import_message_type(gnss_status_config['message_type'])
        self.gnss_status_subscriber = self.create_subscription(msg_type, gnss_status_config['topic'], self.gnss_status_callback, qos_profile=qos_profile_sensor_data)

    def setup_recording(self):
        recording_config = self.config['recording']
        self.recording_button = self.create_button(self.recording_frame, text="Recording\nin progress", row=0, column=2)

        msg_type = self.import_message_type(recording_config['message_type'])
        self.recording_subscriber = self.create_subscription(msg_type, recording_config['topic'], self.rosbag_recording_callback, qos_profile=qos_profile_sensor_data)

    def import_message_type(self, message_type_str):
        module_name, class_name = message_type_str.rsplit('.', 1)
        module = importlib.import_module(module_name)
        return getattr(module, class_name)

    def create_callback(self, sensor_name):
        def callback(msg):
            self.sensors_status[sensor_name] = True
        return callback

    def window_closing(self):
        self.gui.destroy()
        rclpy.shutdown()

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
        
        self.gnss_status_button.configure(text=text, background=color, activebackground=color)
        self.gnss_service_button.configure(text='Service: '+service_text, background='blue', activebackground='blue')
        
        self.gnss_status = -1

    def rosbag_recording_callback(self, msg):
        self.recording = True

    def update_sensors_frame_gui(self):
        for sensor_name, status in self.sensors_status.items():
            button = getattr(self, f"{sensor_name.lower().replace(' ', '_')}_button")
            color = "green" if status else "red"
            button.configure(background=color, activebackground=color)
        # Reset status to be updated if we get new topics 
        self.sensors_status = {sensor_name: False for sensor_name in self.sensors_status}

    def update_recording_frame_gui(self):
        color = "green" if self.recording else "red"
        self.recording_button.configure(background=color, activebackground=color)
        self.recording = False

    def update_gui(self):
        self.update_sensors_frame_gui()
        self.update_gnss_status()
        self.update_recording_frame_gui()
        self.gui.update()


def main(args=None):
    rclpy.init(args=args)
    ctg = CheckTopicsGui()
    rclpy.spin(ctg)
    ctg.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
