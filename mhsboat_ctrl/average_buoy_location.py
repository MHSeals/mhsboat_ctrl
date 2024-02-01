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


class average_buoy_location(Node):
    def __init__(self):
        super().__init__("average_buoy_location")
        self.get_logger().info("average_buoy_location: its working")
        self.latitudes = []
        self.longitudes = []
        self.types = []
        self.tops = []
        self.lefts = []

        self.iterations = 1
        self.latTotals = []
        self.longTotals = []
        # record previous tops and lefts and total lat and long
        self.prevTops = []
        self.prevLefts = []
        self.prevTypes = []

        # subscribes to AI Output
        self.AiOutput_subscriber = self.create_subscription(
            BuoyCoordinates,
            "/mhsboat_ctrl/buoy_coordinates",
            self.buoy_coordinates_callback,
            10,
        )

        self.average_coordinate_publisher = self.create_publisher(
            BuoyCoordinates, "AverageBuoyCoordinates", 10
        )

    def buoy_coordinates_callback(self, BuoyCoordinatesOutput):
        self.latitudes = BuoyCoordinatesOutput.latitudes
        self.longitudes = BuoyCoordinatesOutput.longitudes
        self.types = BuoyCoordinatesOutput.types
        self.tops = BuoyCoordinatesOutput.tops
        self.lefts = BuoyCoordinatesOutput.lefts
        averageLocation(
            self, self.latitudes, self.longitudes, self.types, self.tops, self.lefts
        )

        averageLat = [x / self.iterations for x in self.latTotals]
        averageLong = [x / self.iterations for x in self.longTotals]
        print(averageLat)
        print(averageLong)
        print(self.iterations)

        self.average_coordinate_publisher.publish(
            BuoyCoordinates(
                latitudes=averageLat,
                longitudes=averageLong,
                types=self.types,
                ids=BuoyCoordinatesOutput.ids,
            )
        )


def averageLocation(self, latitudes, longitudes, types, tops, lefts):
    # check if type of buoys changed also

    fail = False
    if len(latitudes) == len(self.latTotals):
        # need to pair new lats and longs to old lats and longs make sure sorted in order
        # latitudes, longitudes, types, tops, lefts = pairBuoys(self, latitudes, longitudes, types, tops, lefts)

        if not None in latitudes:
            for i, x in enumerate(latitudes):
                self.latTotals[i] += x
            for i, x in enumerate(longitudes):
                self.longTotals[i] += x

            self.prevTops = tops
            self.prevLefts = lefts
            self.prevTypes = types
            self.types = types
            self.iterations += 1
        else:
            print("Something is wrong")
            fail = True
    elif (not len(latitudes) == len(self.latTotals)) or fail:
        print("reset")
        self.latTotals = latitudes
        self.longTotals = longitudes
        self.iterations = 1
        self.prevTops = tops
        self.prevLefts = lefts
        self.prevTypes = types


def pairBuoys(self, latitudes, longitudes, types, tops, lefts):
    newLatitudes = [None] * len(latitudes)
    newLongitudes = [None] * len(longitudes)
    newTypes = [None] * len(types)
    newTops = [None] * len(tops)
    newLefts = [None] * len(lefts)

    print("before" + str(types))
    for i, (top, left, t) in enumerate(zip(tops, lefts, types)):
        minDistDist = 1000000
        minDistDistIndex = -1
        for j, (prevTop, prevLeft, prevType) in enumerate(
            zip(self.prevTops, self.prevLefts, self.prevTypes)
        ):
            # prevDist = math.sqrt(prevTop**2+prevLeft**2)
            print(j, top, prevTop, left, prevLeft, t, prevType)
            dist = math.sqrt((top - prevTop) ** 2 + (left - prevLeft) ** 2)

            if dist < minDistDist and t == prevType:
                minDistDist = dist
                minDistDistIndex = j

        print("minDistDistIndex: " + str(minDistDistIndex))
        print("minDistDist: " + str(minDistDist))

        newLatitudes[minDistDistIndex] = latitudes[i]
        newLongitudes[minDistDistIndex] = longitudes[i]
        newTypes[minDistDistIndex] = types[i]
        newTops[minDistDistIndex] = tops[i]
        newLefts[minDistDistIndex] = lefts[i]
    print("before: " + str(types))
    print("after" + str(newTypes))
    return newLatitudes, newLongitudes, newTypes, newTops, newLefts


def main(args=None):
    rclpy.init(args=args)
    node = average_buoy_location()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
