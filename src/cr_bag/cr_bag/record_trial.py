#!/usr/bin/env python3 
import ros2bag_convert.main
import rclpy
import signal
from rclpy.node import Node
from cr_bag.srv import StartBag, StopBag
import subprocess
import ros2bag_convert

"""
    bag files need:
        - motion type
        - slope angle
        - date and time
"""

class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')
        self.get_logger().info('Initialized trial recording services')
        self.stop_srv = self.create_service(StopBag, 'stop_recording', self.stop_recording_callback)
        self.start_srv = self.create_service(StartBag, 'start_recording', self.start_recording_callback)
        self.process = None
    
        self.topics_to_record = [
            "/vicon/cuberover/cuberover",
            "/imu",
            "/absolute_orientation",
            "/differential_drive_controller/cmd_vel",
            "/differential_drive_controller/odom"
            "/joint_states"
        ]

    def start_recording_callback(self, request, response):
        if self.process is not None:
            self.get_logger().warn('Recording already in progress')
            response.success = False
        else:
            bag_output = request.bag_output
            command = ['ros2', 'bag', 'record', '-o', bag_output, *self.topics_to_record]
            self.process = subprocess.Popen(command)
            self.get_logger().info('Started recording')
            self.get_logger().info(f'Outputting bag file to: {bag_output}')
            response.success = True
        return response

    def stop_recording_callback(self, request, response):
        if self.process:
            self.process.send_signal(signal.SIGINT)
            self.process.wait()
            self.process = None
            self.get_logger().info('Stopped recording')
            response.success = True
        else:
            self.get_logger().warn("Recording hasn't started")
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    bag_recorder = BagRecorder()

    rclpy.spin(bag_recorder)

    bag_recorder.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
