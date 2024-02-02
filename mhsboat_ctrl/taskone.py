#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2, PointField, NavSatFix, Imu
from geometry_msgs.msg import Quaternion
import math
import numpy as np
from boat_interfaces.msg import AiOutput
from boat_interfaces.msg import BuoyCoordinates
from geographic_msgs.msg import GeoPoseStamped


class locate_buoys(Node):
    def __init__(self):
        super().__init__("TaskOne")
        self.get_logger().info("Task One: its working")
        self.latitudes = []
        self.longitudes = []
        self.types = []
        self.boat_latitude = 0
        self.boat_longitude = 0
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
            NavSatFix, "/mavros/global_position/global", self.gps_callback, 10
        )

        # self.waypoint_publisher = self.create_publisher(
        #     BuoyCoordinates, "/taskone/goal_waypoint", 10
        # )

        self.waypoint_publisher = self.create_publisher(
            GeoPoseStamped, "/mavros/setpoint_position/global", 10
        )

    def set_waypoint(self, lat: float, long: float):
        msg = GeoPoseStamped()
        msg.pose.position.latitude = lat
        msg.pose.position.longitude = long
        self.waypoint_publisher.publish(msg)

    def gps_callback(self, gps):
        self.boat_latitude = gps.latitude
        self.boat_longitude = gps.longitude

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
        return
    redPoleDistances = []
    greenPoleDistances = []

    minRedDist = 1000000
    minRedIndex = 1000000
    for i, redPole in enumerate(redPoleList):
        dist = math.sqrt(
            (redPole.latitude - self.boat_latitude) ** 2
            + (redPole.longitude - self.boat_longitude) ** 2
        )
        redPoleDistances.append(dist)
        if dist < minRedDist:
            minRedDist = dist
            minRedIndex = i

    minGreenDist = 1000000
    minGreenIndex = 1000000
    for i, greenPole in enumerate(greenPoleList):
        dist = math.sqrt(
            (greenPole.latitude - self.boat_latitude) ** 2
            + (greenPole.longitude - self.boat_longitude) ** 2
        )
        greenPoleDistances.append(dist)
        if dist < minGreenDist:
            minGreenDist = dist
            minGreenIndex = i

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


def main(args=None):
    rclpy.init(args=args)
    node = locate_buoys()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
