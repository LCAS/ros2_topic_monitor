import os
import tkinter as tk
from tkinter import messagebox
from datetime import datetime
import threading
import subprocess
import yaml
import rclpy
from rclpy.node import Node
from rclpy import Parameter
from std_msgs.msg import Empty
from ament_index_python.packages import get_package_share_directory

class BagRecorderNode(Node):
    def __init__(self):
        super().__init__('bag_recorder_node')

        # Params
        self.declare_parameter('cfg_pth', Parameter.Type.STRING)
        self.cfg_pth = self.get_parameter_or('cfg_pth', Parameter('str', Parameter.Type.STRING, '')).value
        package_share_directory = get_package_share_directory('ros2_topic_monitor')
        cfg_pth_default = os.path.join(package_share_directory, 'config', 'cfg_topics_record.yaml')
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
        self.recording_in_progress = False

        self.root = tk.Tk()
        self.root.title('ROS2 Bag Recorder')

        self.dir_label = tk.Label(self.root, text="Enter Bag Directory:")
        self.dir_label.pack(pady=5)

        self.bag_dir_entry = tk.Entry(self.root)
        self.bag_dir_entry.pack(pady=5)

        self.name_label = tk.Label(self.root, text="Enter Bag Name:")
        self.name_label.pack(pady=5)

        self.bag_name_entry = tk.Entry(self.root)
        self.bag_name_entry.pack(pady=5)

        self.toggle_button = tk.Button(self.root, text='Start Recording', command=self.toggle_recording)
        self.toggle_button.pack(pady=10)

        self.end_session_button = tk.Button(self.root, text='End Session', command=self.end_session)
        self.end_session_button.pack(pady=10)

        # Status frame to manage background color
        self.status_frame = tk.Frame(self.root, width=300, height=10)
        self.status_frame.pack(pady=10, fill=tk.X)
        self.update_status_color()

    def toggle_recording(self):
        if self.recording_in_progress:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        bag_dir = self.bag_dir_entry.get().strip()
        bag_name = self.bag_name_entry.get().strip()
        if not bag_name:
            bag_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        if not os.path.isdir(bag_dir):
            try:
                os.makedirs(bag_dir)
            except OSError as e:
                messagebox.showerror("Error", f"Failed to create directory '{bag_dir}': {e}")
                return

        output_path = os.path.join(bag_dir, bag_name)

        if os.path.exists(output_path):
            proceed = messagebox.askyesno("Warning", f"The bag name '{bag_name}' already exists. Do you want to overwrite it?")
            if not proceed:
                return

        try:
            self.recorder_node.start_recording(output_path)
            self.recording_in_progress = True
            self.update_status_color()
            self.toggle_button.config(text='Stop Recording')
            messagebox.showinfo("Info", f"Started recording ROS2 bag to {output_path}.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start recording: {e}")

    def stop_recording(self):
        try:
            self.recorder_node.stop_recording()
            self.recording_in_progress = False
            self.update_status_color()
            self.toggle_button.config(text='Start Recording')
            messagebox.showinfo("Info", "Stopped recording ROS2 bag.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to stop recording: {e}")

    def end_session(self):
        if self.recording_in_progress:
            self.stop_recording()
        self.root.quit()

    def update_status_color(self):
        if self.recording_in_progress:
            self.status_frame.config(bg='green')
        else:
            self.status_frame.config(bg='red')

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
