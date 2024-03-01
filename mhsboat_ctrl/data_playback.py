#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSLivelinessPolicy,
    QoSHistoryPolicy,
)
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2, PointField, NavSatFix, Imu
from geometry_msgs.msg import Quaternion
import math
import numpy as np
from boat_interfaces.msg import AiOutput
from boat_interfaces.msg import BuoyCoordinates
from geographic_msgs.msg import GeoPoseStamped
import pickle
import threading
import time
import ast


class data_playback(Node):
    def __init__(self):
        super().__init__("logger")
        self.get_logger().info("logger: its working")
        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        # subscribes to AI Output
        self.AiOutput_publisher = self.create_publisher(AiOutput, "/AiOutput", 10)

        # subscribes to the center of cluster pointcloud
        self.lidarOutput_publisher = self.create_publisher(
            PointCloud2, "/center_of_clusters", 10
        )

        # subscribes to the GPS location
        self.gps_publisher = self.create_publisher(
            NavSatFix, "/mavros/global_position/global", qos_profile
        )

        # subscribes to the boat orientation
        self.imu_publisher = self.create_publisher(Imu, "/mavros/imu/data", qos_profile)

        self.coordinate_publisher = self.create_publisher(
            BuoyCoordinates, "/mhsboat_ctrl/buoy_coordinates", 10
        )

        self.average_buoy_coordinate_publisher = self.create_publisher(
            BuoyCoordinates, "/AverageBuoyCoordinates", 10
        )

        self.waypoint_publisher = self.create_publisher(
            GeoPoseStamped, "/mavros/setpoint_position/global", 10
        )

        self.base_path = "/root/roboboat_ws/src/mhsboat_ctrl/logs/text/"

    def ai(self):
        self.read_data(
            self.base_path + "AiOutput/AiOutput.txt", self.AiOutput_publisher
        )

    def lidar(self):
        self.read_data(
            self.base_path + "center_of_clusters/lidar.txt", self.lidarOutput_publisher
        )

    def gps(self):
        self.read_data(self.base_path + "gps/gps.txt", self.gps_publisher)

    def imu(self):
        self.read_data(self.base_path + "imu/imu.txt", self.imu_publisher)

    def buoy_coordinates(self):
        path = self.base_path + "buoy_coordinates/buoy_coordinates.txt"
        with open(path, "r") as f:
            for line in f.readlines():
                line_bytes = ast.literal_eval(line.strip())
                obj = pickle.loads(line_bytes)
                print(obj)

    def average_buoy_coordinates(self):
        self.read_data(
            self.base_path + "AverageBuoyCoordinates/AverageBuoyCoordinates.txt",
            self.average_buoy_coordinate_publisher,
        )

    def waypoint(self):
        path = self.base_path + "waypoint/waypoint.txt"

    def read_data(self, path, func):
        with open(path, "r") as f:
            for line in f.readlines():
                line_bytes = ast.literal_eval(line.strip())
                obj = pickle.loads(line_bytes)
                print(obj)
                # func.publish(obj)

    def parallel_functions(self):
        t1 = threading.Thread(target=self.ai)
        t2 = threading.Thread(target=self.lidar)
        t3 = threading.Thread(target=self.gps)
        t4 = threading.Thread(target=self.imu)
        t5 = threading.Thread(target=self.buoy_coordinates)
        t6 = threading.Thread(target=self.average_buoy_coordinates)
        t7 = threading.Thread(target=self.waypoint)

        t1.start()
        t2.start()
        t3.start()
        t4.start()
        t5.start()
        t6.start()
        t7.start()

        t1.join()
        t2.join()
        t3.join()
        t4.join()
        t5.join()
        t6.join()
        t7.join()

        time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = data_playback()
    # while True:
    #     node.parallel_functions()
    node.buoy_coordinates()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
