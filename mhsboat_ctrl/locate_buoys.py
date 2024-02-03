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


class locate_buoys(Node):
    def __init__(self):
        super().__init__("locate_buoys")
        self.get_logger().info("its working")
        self.pointCloud = PointCloud2()
        self.current_lat = 0
        self.current_long = 0
        self.latitudes = []
        self.longitudes = []
        self.lefts = []
        self.tops = []
        self.ids = []
        self.buoy_types = []
        self.quaternion = Quaternion()

        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        # subscribes to AI Output
        self.AiOutput_subscriber = self.create_subscription(
            AiOutput, "/AiOutput", self.camera_callback, 10
        )

        # subscribes to the center of cluster pointcloud
        self.lidarOutput_subscriber = self.create_subscription(
            PointCloud2, "/center_of_clusters", self.lidar_callback, 10
        )

        # subscribes to the GPS location
        self.gps_subscriber = self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self.gps_callback, qos_profile
        )

        # self.gps_subscriber = self.create_subscription(
        # NavSatFix, "/wamv/sensors/gps/gps/fix", self.gps_callback,qos_profile)
        # subscribes to the boat orientation
        self.imu_subscriber = self.create_subscription(
            Imu, "/mavros/imu/data", self.imu_callback, qos_profile
        )
        # self.imu_subscriber = self.create_subscription(
        # Imu, "/wamv/sensors/imu/imu/data", self.imu_callback,qos_profile)

        # publishes buoy coordinates and types
        # float64[] latitudes
        # float64[] longitudes
        # string[] types
        self.coordinate_publisher = self.create_publisher(
            BuoyCoordinates, "/mhsboat_ctrl/buoy_coordinates", 10
        )

        # timer_period = .5
        # self.timer = self.create_timer(timer_period,self.publish_coordinates)

    def camera_callback(self, cameraAi_output: AiOutput):
        self.get_logger().info(str(cameraAi_output))
        self.ids = cameraAi_output.ids
        (
            self.latitudes,
            self.longitudes,
            self.buoy_types,
            self.lefts,
            self.tops,
            self.ids
        ) = analyze(
            cameraAi_output,
            self.pointCloud,
            self.current_lat,
            self.current_long,
            self.quaternion,
            self.ids
        )
        self.publish_coordinates()

    def lidar_callback(self, clusterCenters: PointCloud2):
        self.pointCloud = clusterCenters

    def gps_callback(self, data: NavSatFix):
        self.current_lat = data.latitude
        self.current_long = data.longitude

    def imu_callback(self, data: Imu):
        self.quaternion = data.orientation

    def publish_coordinates(self):
        self.coordinate_publisher.publish(
            BuoyCoordinates(
                latitudes=self.latitudes,
                longitudes=self.longitudes,
                types=self.buoy_types,
                lefts=self.lefts,
                tops=self.tops,
                ids=self.ids,
            )
        )


def analyze(cameraAi_output, pointCloud, current_lat, current_long, quaternion, pre_ids):
    latitudes = []
    longitudes = []
    buoy_types = []
    lefts = []
    tops = []
    ids = []

    if cameraAi_output.num!=len(cameraAi_output.lefts) or cameraAi_output.num!=len(cameraAi_output.tops) or cameraAi_output.num!=len(cameraAi_output.types) or cameraAi_output.num!=len(cameraAi_output.ids):
        return latitudes, longitudes, buoy_types, lefts, tops, ids

    for i in range(cameraAi_output.num):
        print("i: "+str(i))
        # calculate the 3D angle of each buoy location
        # returns a list of the left/right angle (theta) and the up/down angle (phi) relative to boat
        print("num: " + str(cameraAi_output.num))
        theta, phi = get_angle(cameraAi_output, i)

        # Use angle to get the XYZ coordinates of each buoy
        # returns X, Y, Z
        x, y, z = get_XYZ_coordinates(theta, phi, pointCloud, cameraAi_output.types[i])

        # skip point if no associated lidar points
        if x == 0 and y == 0 and z == 0:
            continue
        # convert the X,Y,Z coordinates into latitutde and longitude coordinates
        # returns latitude (lat) and longitude (long)
        lat, long = convert_to_lat_long(
            x, y, current_lat, current_long, quaternion, theta
        )

        latitudes.append(lat)
        longitudes.append(long)
        buoy_types.append(cameraAi_output.types[i])
        lefts.append(cameraAi_output.lefts[i])
        tops.append(cameraAi_output.tops[i])
        ids.append(pre_ids[i])

    return latitudes, longitudes, buoy_types, lefts, tops, ids


# calculate the 3D angle of each buoy location
# returns a list of the left/right angle (theta) and the up/down angle (phi)
def get_angle(cameraAi_output, index):
    # currently the FOV of the simulated camera
    # replace with FOV of real camera
    FOV_H = 80
    FOV_V = 56

    print("index: " + str(index))
    print("lefts: " + str(cameraAi_output.lefts))
    midX = cameraAi_output.lefts[index] + cameraAi_output.widths[index] / 2
    midY = cameraAi_output.tops[index] + cameraAi_output.heights[index] / 2
    width = cameraAi_output.img_width
    height = cameraAi_output.img_height

    degreesPerPixelH = FOV_H / width
    degreesPerPixelV = FOV_V / height

    print("midX: " + str(midX))
    print("degreesPerPixelH: " + str(degreesPerPixelH))
    print("FOV_H: " + str(FOV_H))
    theta = midX * degreesPerPixelH - FOV_H / 2
    phi = (midY * degreesPerPixelV - FOV_V / 2 )
    # phi = midY * degreesPerPixelV - FOV_V/2
    print("THETA: " + str(theta))
    print("PHI: "+ str(phi))
    # add 15.5 because camera is at slight angle down and the 15.5 corrects for it
    # shouldn't be a problem when the camera is mounted horizontally on the real boat

    return theta, phi


# Use angle to get the XYZ coordinates of each buoy
# returns X, Y, Z
def get_XYZ_coordinates(theta, phi, pointCloud, name):
    points = np.array(list(read_points(pointCloud)))
    mask = np.empty(points.shape[0], dtype=bool)
    for index, point in enumerate(points):
        x = point[0]
        y = point[1]
        z = point[2]
        r = math.sqrt(x**2 + y**2 + z**2)

        ### calculate angle from x axis, the camera always points towards the x axis so we only care about lidar points near the x axis
        # might have to change if camera doesn't point towards real lidar's x asix

        thetaPoint = (
            math.degrees(math.acos(x / math.sqrt(x**2 + y**2))) * y / abs(y) * -1
        )
        phiPoint = math.degrees(math.acos(x / math.sqrt(x**2 + z**2))) * z / abs(z)

        # if theta and phi are in list by some closeness
        # keep point, else delete point
        degrees = 5
        if (
            math.fabs(thetaPoint - theta) <= degrees
            or math.fabs(phiPoint - phi) <= degrees
        ):
            # print("theta: "+str(theta))
            # print("thetaPoint: "+str(thetaPoint))
            # print("theta-theta: "+str(math.fabs(thetaPoint-theta)))
            # print()
            # print("phi: "+str(phi))
            # print("phiPoint: "+str(phiPoint))
            # print("phi-phi: "+str(math.fabs(phiPoint-phi)))
            # print("\n")
            pass
        mask[index] = not (
            math.fabs(thetaPoint - theta) <= degrees
            and math.fabs(phiPoint - phi) <= degrees
        )

    points = np.delete(points, mask, axis=0)

    print(name)
    print("after points: " + str(points))
    if len(points) > 1:
        rp = np.mean(points, axis=0)
        return rp[0], rp[1], rp[2]
    if len(points) == 0:
        return (0, 0, 0)
    return (points[0][0], points[0][1], points[0][2])


# convert the X,Y,Z coordinates into latitutde and longitude coordinates
# returns latitude (lat) and longitude (long)
def convert_to_lat_long(x, y, current_lat, current_long, q, theta):
    # yaw of 0 is due east
    yaw = math.degrees(
        math.atan2(2.0 * (q.z * q.w + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    )
    yaw = 90-yaw
    if yaw<0:
        yaw+=360
    print("yaw: " + str(yaw) + " theta: " + str(theta))

    # current_lat = 30
    # current_long = 30

    distance = math.sqrt(x**2 + y**2)

    easting = distance * math.sin(math.radians(theta - yaw))
    northing = distance * math.cos(math.radians(theta - yaw)) * -1
    print("distance: " + str(distance))
    print("easting: " + str(easting))
    print("northing: " + str(northing))
    rad_earth = 6378.137
    pi = math.pi
    delta_lat_m = (northing / ((2 * pi / 360) * rad_earth)) / 1000
    delta_long_m = (easting / ((2 * pi / 360) * rad_earth)) / 1000

    new_lat = current_lat + delta_lat_m
    new_long = current_long + delta_long_m / math.cos(delta_lat_m * (pi / 180))
    return new_lat, new_long


def main(args=None):
    rclpy.init(args=args)
    node = locate_buoys()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


import sys
from collections import namedtuple
import ctypes
import math
import struct

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ("b", 1)
_DATATYPES[PointField.UINT8] = ("B", 1)
_DATATYPES[PointField.INT16] = ("h", 2)
_DATATYPES[PointField.UINT16] = ("H", 2)
_DATATYPES[PointField.INT32] = ("i", 4)
_DATATYPES[PointField.UINT32] = ("I", 4)
_DATATYPES[PointField.FLOAT32] = ("f", 4)
_DATATYPES[PointField.FLOAT64] = ("d", 8)


def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), "cloud is not a sensor_msgs.msg.PointCloud2"
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = (
        cloud.width,
        cloud.height,
        cloud.point_step,
        cloud.row_step,
        cloud.data,
        math.isnan,
    )
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = ">" if is_bigendian else "<"

    offset = 0
    for field in (
        f
        for f in sorted(fields, key=lambda f: f.offset)
        if field_names is None or f.name in field_names
    ):
        if offset < field.offset:
            fmt += "x" * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print(
                "Skipping unknown PointField datatype [%d]" % field.datatype,
                file=sys.stderr,
            )
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt
