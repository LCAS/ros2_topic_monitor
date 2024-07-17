import sys
import subprocess
import threading
import os
import yaml
import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import messagebox
from datetime import datetime
from rclpy import Parameter 
from std_msgs.msg import Empty
from ament_index_python.packages import get_package_share_directory

class BagRecorderNode(Node):
    def __init__(self):
        super().__init__('bag_recorder_node')

        # Params
        self.declare_parameter( 'cfg_pth' , Parameter.Type.STRING )
        self.cfg_pth = self.get_parameter_or('cfg_pth', Parameter('str', Parameter.Type.STRING, '')).value
        package_share_directory = get_package_share_directory('ros2_topic_monitor')
        cfg_pth_default = os.path.join(package_share_directory,'config', 'cfg_topics_record.yaml')
        self.cfg_pth = self.cfg_pth or cfg_pth_default

        with open(self.cfg_pth, 'r') as file:
            config = yaml.safe_load(file)
        
        self.topics = []
        for category in config['topics'].values():
            self.topics.extend(category) 

        self.bag_process = None

        self.publisher_ = self.create_publisher(Empty, 'rosbag_recording', 10)
        self.timer = self.create_timer(0.1, self.publish_message)

        self.recording_flag = False

    def publish_message(self):
        if self.recording_flag:
            msg = Empty()
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "rosbag_recording" with Empty message')

    def start_recording(self, output_path):
        self.recording_flag = True
        if self.bag_process is None:
            self.bag_process = subprocess.Popen([
                'ros2', 'bag', 'record', *self.topics,
                '--output', output_path
            ])
            self.get_logger().info(f'Started recording ROS2 bag to {output_path}.')

    def stop_recording(self):
        self.recording_flag = False
        if self.bag_process is not None:
            self.bag_process.terminate()
            self.bag_process.wait()
            self.bag_process = None
            self.get_logger().info('Stopped recording ROS2 bag.')

class BagRecorderGUI:
    def __init__(self, recorder_node):
        self.recorder_node = recorder_node
        self.root = tk.Tk()
        self.root.title('ROS2 Bag Recorder')

        self.dir = tk.Label(self.root, text="Enter Bag dir:")
        self.dir.pack(pady=5)

        self.bag_dir_entry = tk.Entry(self.root)
        self.bag_dir_entry.pack(pady=5)

        self.label = tk.Label(self.root, text="Enter Bag Name:")
        self.label.pack(pady=5)

        self.bag_name_entry = tk.Entry(self.root)
        self.bag_name_entry.pack(pady=5)

        self.start_button = tk.Button(self.root, text='Start Recording', command=self.start_recording)
        self.start_button.pack(pady=10)

        self.stop_button = tk.Button(self.root, text='Stop Recording', command=self.stop_recording)
        self.stop_button.pack(pady=10)

        self.end_session_button = tk.Button(self.root, text='End Session', command=self.end_session)
        self.end_session_button.pack(pady=10)

    def start_recording(self):
        bag_dir = self.bag_dir_entry.get()
        bag_name = self.bag_name_entry.get()
        if not bag_name:
            bag_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_path = os.path.join(bag_dir, bag_name)
        
        if os.path.exists(output_path):
            proceed = messagebox.askyesno("Warning", f"The bag name '{bag_name}' already exists. Do you want to overwrite it?")
            if not proceed:
                return

        self.recorder_node.start_recording(output_path)
        messagebox.showinfo("Info", f"Started recording ROS2 bag to {output_path}.")

    def stop_recording(self):
        self.recorder_node.stop_recording()
        messagebox.showinfo("Info", "Stopped recording ROS2 bag.")

    def end_session(self):
        self.stop_recording()
        self.root.quit()

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    recorder_node = BagRecorderNode()

    gui = BagRecorderGUI(recorder_node)

    def ros_spin():
        rclpy.spin(recorder_node)

    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.start()

    try:
        gui.run()
    finally:
        recorder_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()

if __name__ == '__main__':
    main()
