#!/usr/bin/env python3

import os
import yaml
import rclpy
import importlib
import tkinter as tk

from rclpy.node import Node
from rclpy import Parameter 
from ament_index_python.packages import get_package_share_directory

class CheckTopicsGui(Node):
    def __init__(self):
        super().__init__('topics_monitor_gui_node')

        # Params
        self.declare_parameter( 'cfg_pth' , Parameter.Type.STRING )
        self.cfg_pth = self.get_parameter_or('cfg_pth', Parameter('str', Parameter.Type.STRING, '')).value

        package_share_directory = get_package_share_directory('ros2_topic_monitor')
        cfg_pth_default = os.path.join(package_share_directory,'config', 'cfg_topics_monitor.yaml')

        self.cfg_pth = self.cfg_pth or cfg_pth_default

        self.button_width = 11
        self.font_size = 11

        # setup flags
        self.sensors_status = {}
        self.recording = False

        # Load configuration from YAML file
        with open(self.cfg_pth, 'r') as file:
            self.config = yaml.safe_load(file)

        # setup tkinter gui
        self.gui = tk.Tk()
        self.gui.title("TOPICS MONITOR")
        self.gui.protocol("WM_DELETE_WINDOW", self.window_closing)

        self.spackensors_frame = tk.LabelFrame(self.gui, text="SENSORS", font=("Arial Bold", self.font_size + 1))
        self.sensors_frame.grid(row=0, column=0, padx=10, pady=10, sticky="w")

        self.recording_frame = tk.LabelFrame(self.gui, text="RECORDING", font=("Arial Bold", self.font_size + 1))
        self.recording_frame.grid(row=0, column=1, padx=10, pady=10, sticky="w")

        self.subscribers = []
        self.setup_sensors()
        self.setup_recording()

        self.timer = self.create_timer(0.5, self.update_gui)

    def setup_sensors(self):
        for index, sensor in enumerate(self.config['sensors']):
            self.sensors_status[sensor['name']] = False
            button = tk.Button(self.sensors_frame, text=sensor['name'], command=self.dummy_callback, width=self.button_width, background="red", activebackground="red", font=("Arial Bold", self.font_size))
            button.grid(row=index, column=0, padx=5)
            setattr(self, f"{sensor['name'].lower().replace(' ', '_')}_button", button)

            msg_type = self.import_message_type(sensor['message_type'])
            callback = self.create_callback(sensor['name'])
            subscriber = self.create_subscription(msg_type, sensor['topic'], callback, 10)
            self.subscribers.append(subscriber)

    def setup_recording(self):
        recording_config = self.config['recording']
        self.recording_button = tk.Button(self.recording_frame, text="Recording\nin progress", command=self.dummy_callback, width=self.button_width, background="red", activebackground="red", font=("Arial Bold", self.font_size))
        self.recording_button.grid(row=0, column=1, padx=5)

        msg_type = self.import_message_type(recording_config['message_type'])
        self.recording_subscriber = self.create_subscription(msg_type, recording_config['topic'], self.rosbag_recording_callback, 10)

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
        return

    def dummy_callback(self):
        return

    def rosbag_recording_callback(self, msg):
        self.recording = True

    def update_gui(self):
        for sensor_name, status in self.sensors_status.items():
            button = getattr(self, f"{sensor_name.lower().replace(' ', '_')}_button")
            if status:
                button.configure(background="green", activebackground="green")
            else:
                button.configure(background="red", activebackground="red")

        if self.recording:
            self.recording_button.configure(background="green", activebackground="green")
        else:
            self.recording_button.configure(background="red", activebackground="red")

        self.gui.update()

        for sensor_name in self.sensors_status.keys():
            self.sensors_status[sensor_name] = False
        self.recording = False

def main(args=None):
    rclpy.init(args=args)
    ctg = CheckTopicsGui()
    rclpy.spin(ctg)
    ctg.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
