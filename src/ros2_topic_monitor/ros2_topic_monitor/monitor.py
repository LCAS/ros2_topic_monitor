#!/usr/bin/env python3

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
        self.gnss_status = -1

        # Load configuration from YAML file
        with open(self.cfg_pth, 'r') as file:
            self.config = yaml.safe_load(file)

        # setup tkinter gui
        self.gui = tk.Tk()
        self.gui.title("TOPICS MONITOR")
        self.gui.protocol("WM_DELETE_WINDOW", self.window_closing)

        # Setup sensor frames
        self.sensors_frame      = self.create_label_frame(self.gui, "SENSORS"    , row=0, column=0)
        self.gnss_status_frame  = self.create_label_frame(self.gui, "GNSS STATUS", row=0, column=1)
        self.recording_frame    = self.create_label_frame(self.gui, "RECORDING"  , row=0, column=2)
        
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
        button = tk.Button(frame, text=text, command=self.dummy_callback, width=self.button_width, background="red", activebackground="red", font=("Arial Bold", self.font_size))
        button.grid(row=row, column=column, padx=5)
        return button

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
        return

    def dummy_callback(self):
        return

    def gnss_status_callback(self, msg):
        self.gnss_status = msg.status.status

    def update_gnss_status(self):
        if self.gnss_status == 2:
            self.gnss_status_button.configure(text='GBAS FIX', background="green", activebackground="green")
        elif self.gnss_status == 1:
            self.gnss_status_button.configure(text='SBAS FIX', background="green", activebackground="green")
        elif self.gnss_status == 0:
            self.gnss_status_button.configure(text='FIX', background="yellow", activebackground="yellow")
        else:
            self.gnss_status_button.configure(text='NO FIX', background="red", activebackground="red")
        
        self.gnss_status = -1

    def rosbag_recording_callback(self, msg):
        self.recording = True

    def update_sensosrs_frame_gui(self):
        for sensor_name, status in self.sensors_status.items():
            button = getattr(self, f"{sensor_name.lower().replace(' ', '_')}_button")
            if status:
                button.configure(background="green", activebackground="green")
            else:
                button.configure(background="red", activebackground="red")

        #reset status as to be updated if we get new topics 
        for sensor_name in self.sensors_status.keys():
            self.sensors_status[sensor_name] = False

    def update_recording_frame_gui(self):
        self.recording_button.configure(
            background="green" if self.recording else "red",
            activebackground="green" if self.recording else "red")
        self.recording = False

    def update_gui(self):
        self.update_sensosrs_frame_gui()
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
