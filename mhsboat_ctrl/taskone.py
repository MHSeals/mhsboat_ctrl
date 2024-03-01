#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from sensor_msgs.msg import PointCloud2, PointField, NavSatFix, Imu
from std_msgs.msg import Header
from mavros_msgs.msg import WaypointList, WaypointReached
from geometry_msgs.msg import Quaternion
import math
import numpy as np
from boat_interfaces.msg import AiOutput
from boat_interfaces.msg import BuoyCoordinates
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import TwistStamped, Vector3
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSLivelinessPolicy,
    QoSHistoryPolicy,
)
import time


class locate_buoys(Node):
    def __init__(self):
        super().__init__("TaskOne")
        self.get_logger().info("Task One: its working")
        self.latitudes = []
        self.longitudes = []
        self.types = []
        self.boat_latitude = 0
        self.boat_longitude = 0
        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )
        # subscribes to AI Output
        self.AiOutput_subscriber = self.create_subscription(
            BuoyCoordinates,
            "/AverageBuoyCoordinates",
            self.buoy_coordinates_callback,
            10,
        )

        # subscribes to the GPS location
        # self.gps_subscriber = self.create_subscription(
        #     NavSatFix, "/wamv/sensors/gps/gps/fix", self.gps_callback, 10
        # )

        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self.gps_callback, qos_profile
        )

        self.waypoint_reached_subscriber = self.create_subscription(
            WaypointReached, "/mavros/mission/reached", self.reached_callback, qos_profile
        )

        # self.waypoint_publisher = self.create_publisher(
        #     BuoyCoordinates, "/taskone/goal_waypoint", 10
        # )

        self.waypoint_publisher = self.create_publisher(
            GeoPoseStamped, "/mavros/setpoint_position/global", 10
        )

        self.buoy_publisher = self.create_publisher(
            WaypointList, "/mavros/geofence/fences", 10
        )

        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10
        )
        self.prev_waypoint_lat = 0
        self.prev_waypoint_long = 0
    def set_waypoint(self, lat: float, long: float):
        msg = GeoPoseStamped()
        
        msg.pose.position.latitude = lat
        msg.pose.position.longitude = long
        
        print("waypoint: "+str(lat)+" "+str(long))
        self.waypoint_publisher.publish(msg)

        #clear waypoint when reach waypoint
        if math.sqrt(abs(lat-self.boat_latitude)+abs(long-self.boat_longitude))<.000009:
            self.prev_waypoint_lat = 0
            self.prev_waypoint_long = 0

    def gps_callback(self, gps):
        self.boat_latitude = gps.latitude
        self.boat_longitude = gps.longitude
    def reached_callback(self, waypoint_list):
        twist = TwistStamped()
        twist.twist.linear.x = 0.5

        self.cmd_vel_publisher.publish(twist)
        print("GOING STRAIGHT")
        time.sleep(3)
        twist.twist.linear.x=0.0
        self.cmd_vel_publisher.publish(twist)


    def buoy_coordinates_callback(self, BuoyCoordinatesOutput):
        self.latitudes = BuoyCoordinatesOutput.latitudes
        self.longitudes = BuoyCoordinatesOutput.longitudes
        self.types = BuoyCoordinatesOutput.types
        self.ids = BuoyCoordinatesOutput.ids
        taskOne(self, self.latitudes, self.longitudes, self.types)


class Buoy:
    def __init__(self, t, long, lat):
        self.type = t
        self.longitude = long
        self.latitude = lat

    def __str__(self):
        return f"{self.type} {self.latitude} {self.longitude}"

    def __repr__(self):
        return f"{self.type} {self.latitude} {self.longitude}"

    def __eq__(self, other):
        return (
            self.type == other.type
            and self.latitude == other.latitude
            and self.longitude == other.longitude
        )


def taskOne(self, latitudes, longitudes, types):
    twist = TwistStamped()

    redPoleList = []
    greenPoleList = []
    print("types: " + str(types))
    for i, b in enumerate(types):
        if b == "red_pole_buoy":
            redPoleList.append(Buoy(b, longitudes[i], latitudes[i]))
        elif b == "green_pole_buoy":
            greenPoleList.append(Buoy(b, longitudes[i], latitudes[i]))

    print(redPoleList)
    print(greenPoleList)

    if len(redPoleList) == 0 or len(greenPoleList) == 0:
        #change later to get a good turn speed
        #also idk if this is good logic
        twist.twist.angular.z = .1
        self.cmd_vel_publisher.publish(twist)
 
        if self.prev_waypoint_lat!=0 and self.prev_waypoint_long!=0:
            self.set_waypoint(self.prev_waypoint_lat,self.prev_waypoint_long)
            # pass
        return
    else:
        twist.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
    
    redPoleDistances = [None]*len(redPoleList)
    greenPoleDistances = [None]*len(greenPoleList)
    
    for i, buoy in enumerate(redPoleList):
        redPoleDistances[i] = math.sqrt((self.boat_latitude-buoy.latitude)**2+(self.boat_longitude-buoy.longitude)**2)
    for i, buoy in enumerate(greenPoleList):
        greenPoleDistances[i] = math.sqrt((self.boat_latitude-buoy.latitude)**2+(self.boat_longitude-buoy.longitude)**2)
    
    for i in range(len(redPoleList)):
        for j in range(i + 1, len(redPoleList)):
            if redPoleDistances[i] > redPoleDistances[j]:
                tempDistance = redPoleDistances[i]
                tempPole = redPoleList[i]
                redPoleDistances[i] = redPoleDistances[j]
                redPoleList[i] = redPoleList[j]
                redPoleDistances[j] = tempDistance
                redPoleList[j] = tempPole

    for i in range(len(greenPoleList)):
        for j in range(i + 1, len(greenPoleList)):
            if greenPoleDistances[i] > greenPoleDistances[j]:
                tempDistance = greenPoleDistances[i]
                tempPole = greenPoleList[i]
                greenPoleDistances[i] = greenPoleDistances[j]
                greenPoleList[i] = greenPoleList[j]
                greenPoleDistances[j] = tempDistance
                greenPoleList[j] = tempPole

    closestRedPole = redPoleList[0]
    closestGreenPole = greenPoleList[0]

    midLat = (closestRedPole.latitude + closestGreenPole.latitude) / 2
    midLong = (closestRedPole.longitude + closestGreenPole.longitude) / 2

    # self.waypoint_publisher.publish(
    #     BuoyCoordinates(latitudes=[midLat], longitudes=[midLong], types=["waypoint"])
    # )

    self.set_waypoint(midLat, midLong)
    self.prev_waypoint_lat = midLat
    self.prev_waypoint_long = midLong

def main(args=None):
    rclpy.init(args=args)
    node = locate_buoys()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
