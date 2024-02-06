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
import time


class logger(Node):
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
        self.AiOutput_subscriber = self.create_subscription(
            AiOutput, "/AiOutput", self.ai_callback, 10
        )

        # subscribes to the center of cluster pointcloud
        self.lidarOutput_subscriber = self.create_subscription(
            PointCloud2, "/center_of_clusters", self.lidar_callback, 10
        )

        # subscribes to the GPS location
        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self.gps_callback, qos_profile
        )

        # subscribes to the boat orientation
        self.imu_subscriber = self.create_subscription(
            Imu, "/mavros/imu/data", self.imu_callback, qos_profile
        )

        self.coordinate_subscriber = self.create_subscription(
            BuoyCoordinates, "/mhsboat_ctrl/buoy_coordinates", self.buoy_coordinates_callback, 10
        )

        self.average_buoy_coordiante_subscriber = self.create_subscription(
            BuoyCoordinates, "/AverageBuoyCoordinates", self.average_buoy_coordinates_callback, 10
        )

        self.waypoint_subscriber = self.create_subscription(
            GeoPoseStamped, "/mavros/setpoint_position/global", self.waypoint_callback, 10
        )

        self.base_path = "/root/roboboat_ws/src/mhsboat_ctrl/logs/text/"
    def ai_callback(self, data: AiOutput):
        write_data(data,self.base_path+"AiOutput/")

    def lidar_callback(self, data: PointCloud2):
        write_data(data,self.base_path+"center_of_clusters/")

    def gps_callback(self, data: NavSatFix):
        write_data(data,self.base_path+"gps/")

    def imu_callback(self, data: Imu):
        write_data(data,self.base_path+"imu/")

    
    def buoy_coordinates_callback(self, data):
        write_data(data,self.base_path+"buoy_coordinates/")

    def average_buoy_coordinates_callback(self, data):
        write_data(data,self.base_path+"AverageBuoyCoordinates/")

    def waypoint_callback(self, data):
        write_data(data,self.base_path+"waypoint/")

def write_data(data,path):
        with open(path+str(time.time())+".txt","w") as f:
            f.write(str(data))
    

def main(args=None):
    rclpy.init(args=args)
    node = logger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()