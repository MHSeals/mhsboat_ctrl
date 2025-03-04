#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message

from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from boat_interfaces.msg import BuoyMap

import rosbag2_py

import datetime
import os

parent_dir = "bag_files"  
os.makedirs(parent_dir, exist_ok=True)

bag_dir = os.path.join(parent_dir, f'bag_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}')

class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py.StorageOptions(
            uri=bag_dir, # Location of the bag directory
            storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py.TopicMetadata(
            name='/mhsboat_ctrl/map',
            type='boat_interfaces/msg/BuoyMap',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.map_subscription = self.create_subscription(
            BuoyMap,
            '/mhsboat_ctrl/map',
            self.map_callback,
            10)

    def map_callback(self, msg):
        self.writer.write(
            '/mhsboat_ctrl/map',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

def main(args=None):
    rclpy.init(args=args)
    br = BagRecorder()
    rclpy.spin(br)
    rclpy.shutdown()

if __name__ == '__main__':
    main()