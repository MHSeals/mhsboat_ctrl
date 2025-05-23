import rclpy
import rclpy.logging
from rclpy.node import Node
from boat_interfaces.msg import AiOutput, BuoyMap
import rclpy.utilities
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSLivelinessPolicy,
)
import os
import yaml
import math
import numpy as np
from typing import Dict, List, Tuple, Optional
import tf_transformations
from sensor_msgs.msg import PointField
from uuid import uuid1
import traceback

from mhsboat_ctrl.course_objects import CourseObject, Shape, Buoy, PoleBuoy, BallBuoy
from mhsboat_ctrl.enums import BuoyColors, Shapes
from mhsboat_ctrl.utils.lidar import read_points

buoy_color_mapping: Dict[str, BuoyColors] = {
    "black": BuoyColors.BLACK,
    "blue": BuoyColors.BLUE,
    "green": BuoyColors.GREEN,
    "red": BuoyColors.RED,
    "yellow": BuoyColors.YELLOW,
}

BUOY_DUPLICATE_THRESHOLD = 0.2
CLUSTER_DETECTION_THRESHOLD_ANGLE = 4

class Sensors(Node):
    def __init__(self):
        super().__init__("sensors")

        self.map: List[CourseObject] = []

        # cache for the sensors, limited to 3 items
        # this allows us to find the data that has the
        # smallest time difference, giving more accurate
        # results
        self._camera_cache: List[Tuple[AiOutput, float]] = []
        self._lidar_cache: List[Tuple[PointCloud2, float]] = []
        self._depth_cache: List[Tuple[PointCloud2, float]] = []
        self._depth_conf_cache: List[Tuple[PointCloud2, float]] = []
        self._odom_cache: List[Tuple[Odometry, float]] = []

        self._qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        self._ai_out_sub = self.create_subscription(
            AiOutput, "/AiOutput", self._camera_callback, self._qos_profile
        )

        self._lidar_out_sub = self.create_subscription(
            PointCloud2, "/center_of_clusters", self._lidar_callback, self._qos_profile
        )

        self._depth_out_sub = self.create_subscription(
            PointCloud2, "/zed/point_cloud/cloud_registered", self._depth_callback, self._qos_profile
        )

        self._depth_conf_out_sub = self.create_subscription(
            Image, "/zed/confidence/confidence_image", self._depth_conf_callback, self._qos_profile
        )

        self._odom_out_sub = self.create_subscription(
            Odometry(), "/odometry/filtered", self._odom_callback, self._qos_profile
        )

        self._process_timer = self.create_timer(0.1, self._safe_process_sensor_data)

        self.map_publisher = self.create_publisher(BuoyMap, "/mhsboat_ctrl/map", self._qos_profile)

        self.map: List[CourseObject] = []

        self.buoy_cluster_pub = self.create_publisher(PointCloud2, "/mhsboat_ctrl/buoy_clusters", self._qos_profile)

        self.previous_odom_data = None

        self.get_logger().info("Sensors Node Initialized")
        
        self._fps_counter = 0
        self._fps_start = self.get_clock().now().nanoseconds
        self.fps = 0

    def _camera_callback(self, msg: AiOutput):
        self.get_logger().info("Received camera data")
        self._camera_cache.append((msg, self.get_clock().now().nanoseconds))
        if len(self._camera_cache) > 3:
            self._camera_cache.pop(0)

    def _lidar_callback(self, msg: PointCloud2):
        self.get_logger().info("Received lidar data")
        self._lidar_cache.append((msg, self.get_clock().now().nanoseconds))
        if len(self._lidar_cache) > 3:
            self._lidar_cache.pop(0)

    def _depth_callback(self, msg: PointCloud2):
        self.get_logger().info("Received depth data")
        self._depth_cache.append((msg, self.get_clock().now().nanoseconds))
        if len(self._depth_cache) > 3:
            self._depth_cache.pop(0)

    def _depth_conf_callback(self, msg: PointCloud2):
        self.get_logger().info("Received depth confidence data")
        self._depth_conf_cache.append((msg, self.get_clock().now().nanoseconds))
        if len(self._depth_conf_cache) > 3:
            self._depth_conf_cache.pop(0)

    def _odom_callback(self, msg: Odometry):
        self.get_logger().info("Received odometry data")
        self._odom_cache.append((msg, self.get_clock().now().nanoseconds))
        if len(self._odom_cache) > 3:
            self._odom_cache.pop(0)

    @property
    def ai_output(self) -> Optional[AiOutput]:
        return self._camera_cache[-1][0] if len(self._camera_cache) > 0 else None

    @property
    def lidar_output(self) -> Optional[PointCloud2]:
        return self._lidar_cache[-1][0] if len(self._lidar_cache) > 0 else None

    @property
    def odom_output(self) -> Optional[Odometry]:
        return self._odom_cache[-1][0] if len(self._odom_cache) > 0 else None
    
    def _safe_process_sensor_data(self) -> None:
        try:
            self._process_sensor_data()
        except Exception as e:
            self.get_logger().error(f"Error processing sensor data: {e}")
            traceback.print_exc()
        # Increment FPS counter and log FPS every second
        self._fps_counter += 1
        now = self.get_clock().now().nanoseconds
        elapsed_time = (now - self._fps_start) / 1e9  # seconds
        if elapsed_time >= 1.0:
            self.fps = self._fps_counter / elapsed_time
            self._fps_counter = 0
            self._fps_start = now

        self.get_logger().info(f"FPS: {self.fps:.2f}")

    def _process_sensor_data(self) -> None:
        """
        Process the sensor data
        """
        if len(self._camera_cache) == 0:
            self.get_logger().warn("No camera data available")
            return

        if len(self._lidar_cache) == 0:
            self.get_logger().warn("No lidar data available")
            return

        if len(self._depth_cache) == 0:
            self.get_logger().warn("No depth data available")
            return

        if len(self._odom_cache) == 0:
            self.get_logger().warn("No odometry data available")
            return

        now = self.get_clock().now().nanoseconds

        ref_time = self._camera_cache[-1][1]

        # find the data that has the smallest time difference
        # and process it
        camera_data, camera_time = min(
            self._camera_cache,
            key=lambda x: abs(x[1] - ref_time),
        )
        lidar_data, lidar_time = min(
            self._lidar_cache,
            key=lambda x: abs(x[1] - ref_time),
        )
        depth_data, depth_time = min(
            self._depth_cache,
            key=lambda x: abs(x[1] - ref_time),
        )
        # TODO: Where can we use depth confidence data?
        depth_conf_data, depth_conf_time = min(
            self._depth_conf_cache,
            key=lambda x: abs(x[1] - ref_time),
        )
        odom_data, odom_time = min(
            self._odom_cache,
            key=lambda x: abs(x[1] - ref_time),
        )

        if self.previous_odom_data is None:
            self.previous_odom_data = odom_data
            return

        time_diff = max(camera_time, lidar_time, depth_time, depth_conf_time, odom_time) - min(
            camera_time, lidar_time, depth_time, depth_conf_time, odom_time
        )
        self.get_logger().info(f"Data time difference: {time_diff} ns")

        # Get which data is laggging behind
        if time_diff > 1e9:
            if camera_time < lidar_time and camera_time < depth_time and camera_time < odom_time and camera_time < depth_conf_time:
                self.get_logger().warn("Camera data is lagging behind")
            elif lidar_time < camera_time and lidar_time < depth_time and lidar_time < odom_time and lidar_time < depth_conf_time:
                self.get_logger().warn("Lidar data is lagging behind")
            elif depth_time < camera_time and depth_time < lidar_time and depth_time < odom_time and depth_time < depth_conf_time:
                self.get_logger().warn("Depth data is lagging behind")
            elif odom_time < camera_time and odom_time < lidar_time and odom_time < depth_time and odom_time < depth_conf_time:
                self.get_logger().warn("Odometry data is lagging behind")
            else:
                self.get_logger().warn("Unknown data is lagging behind")

        # TODO: find a good average value to base this off of
        # Currently set to 1 second
        if time_diff > 1e9:
            self.get_logger().warn(f"Data time difference is large: {time_diff} ns")

        if camera_data.num != len(camera_data.lefts):
            self.get_logger().warn(
                "Inconsistent camera data: number of bounding boxes does not match lefts"
            )
            self.get_logger().warn(
                f"Num: {camera_data.num}, Lefts: {len(camera_data.lefts)}"
            )

        if camera_data.num != len(camera_data.tops):
            self.get_logger().warn(
                "Inconsistent camera data: number of bounding boxes does not match tops"
            )
            self.get_logger().warn(
                f"Num: {camera_data.num}, Tops: {len(camera_data.tops)}"
            )

        if camera_data.num != len(camera_data.widths):
            self.get_logger().warn(
                "Inconsistent camera data: number of bounding boxes does not match widths"
            )
            self.get_logger().warn(
                f"Num: {camera_data.num}, Widths: {len(camera_data.widths)}"
            )

        if camera_data.num != len(camera_data.heights):
            self.get_logger().warn(
                "Inconsistent camera data: number of bounding boxes does not match heights"
            )
            self.get_logger().warn(
                f"Num: {camera_data.num}, Heights: {len(camera_data.heights)}"
            )

        if camera_data.num != len(camera_data.types):
            self.get_logger().warn(
                "Inconsistent camera data: number of bounding boxes does not match types"
            )
            self.get_logger().warn(
                f"Num: {camera_data.num}, Types: {len(camera_data.types)}"
            )

        if camera_data.num != len(camera_data.confidences):
            self.get_logger().warn(
                "Inconsistent camera data: number of bounding boxes does not match confidences"
            )
            self.get_logger().warn(
                f"Num: {camera_data.num}, Confidences: {len(camera_data.confidences)}"
            )

        local_detected_objects: List[CourseObject] = []

        for i in range(camera_data.num):
            # calculate the 3D angle of each buoy location
            # returns a list of the left/right angle (theta) and the up/down angle (phi) relative to boat
            theta, phi = self.get_angle(camera_data, i)
            self.get_logger().info(f"Buoy {i}: Theta: {theta}, Phi: {phi}")

            # Use angle to get the XYZ coordinates of each buoy
            # returns X, Y, Z
            result = self.get_XYZ_coordinates(
                theta, phi, lidar_data, camera_data.types[i]
            )

            if result is None:
                continue

            x, y, z, pts = result

            self._publish_cluster(pts, lidar_data.header)

            # skip point if no associated lidar points
            if x == 0 and y == 0 and z == 0:
                continue

            self.get_logger().info(f"Buoy {i}: X: {x}, Y: {y}, Z: {z}")

            # Publish cluster points for detected objects for debugging
            #self._publish_cluster(cluster_points, lidar_data.header)

            if camera_data.types[i].endswith("pole_buoy"):
                buoy_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                # type: ignore - this will always be either red or green
                local_detected_objects.append(PoleBuoy(x, y, z, buoy_color)) # type: ignore
            elif camera_data.types[i].endswith("buoy"):
                buoy_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                local_detected_objects.append(BallBuoy(x, y, z, buoy_color))
            elif camera_data.types[i].endswith("racquet_ball"):
                # TODO: can we effectively track racquet balls given that they are so small?
                # Do we need to track them?
                pass
            elif camera_data.types[i].endswith("circle"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                local_detected_objects.append(
                    Shape(x, y, z, Shapes.CIRCLE, shape_color)
                )
            elif camera_data.types[i].endswith("triangle"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                local_detected_objects.append(
                    Shape(x, y, z, Shapes.TRIANGLE, shape_color)
                )
            elif camera_data.types[i].endswith("cross"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                local_detected_objects.append(Shape(x, y, z, Shapes.CROSS, shape_color))
            elif camera_data.types[i].endswith("square"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                local_detected_objects.append(
                    Shape(x, y, z, Shapes.SQUARE, shape_color)
                )
            elif camera_data.types[i].endswith("duck_image"):
                # TODO: do we need to track duck images?
                pass

            local_detected_objects[-1].last_seen = now

        self.get_logger().info(
            f"Detected objects in this frame: {local_detected_objects}"
        )

        # Get change in position and orientation from odometry
        if odom_data is None:
            self.get_logger().warn("No odometry data available")
            return

        # Translate the map based on odometry
        trans = [
            odom_data.pose.pose.position.x
            - self.previous_odom_data.pose.pose.position.x,
            odom_data.pose.pose.position.y
            - self.previous_odom_data.pose.pose.position.y,
            odom_data.pose.pose.position.z
            - self.previous_odom_data.pose.pose.position.z,
        ]

        quat = [
            odom_data.pose.pose.orientation.x,
            odom_data.pose.pose.orientation.y,
            odom_data.pose.pose.orientation.z,
            odom_data.pose.pose.orientation.w,
        ]

        previous_quat = [
            self.previous_odom_data.pose.pose.orientation.x,
            self.previous_odom_data.pose.pose.orientation.y,
            self.previous_odom_data.pose.pose.orientation.z,
            self.previous_odom_data.pose.pose.orientation.w,
        ]

        delta_quat = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_inverse(previous_quat), quat
        )

        self.get_logger().info(f"Odometry translation: {trans}")
        self.get_logger().info(f"Odometry quaternion: {delta_quat}")

        transformation_matrix = tf_transformations.quaternion_matrix(delta_quat)
        transformation_matrix[0:3, 3] = trans

        self.get_logger().info(f"Transformation matrix: \n{transformation_matrix}")

        items_to_remove = []

        self.map += local_detected_objects

        # Check for objects that aren't seen by camera, but are seen by lidar
        for map_obj in self.map:
            if map_obj in items_to_remove:
                continue

            # Transform the map object based on the odometry
            point_hom = np.array([map_obj.x, map_obj.y, map_obj.z, 1])

            point_trans = np.dot(transformation_matrix, point_hom)

            map_obj.x = point_trans[0]
            map_obj.y = point_trans[1]
            map_obj.z = point_trans[2]

            theta = math.degrees(math.atan2(map_obj.y, map_obj.x))
            phi = math.degrees(math.atan2(map_obj.z, map_obj.x))

            coords = self.get_XYZ_coordinates(theta, phi, depth_data, "lidar")
            if coords is None:
                continue

            x, y, z, _ = coords

            if x == 0 and y == 0 and z == 0:
                continue

            map_obj.x = x
            map_obj.y = y
            map_obj.z = z
            map_obj.last_seen = now

            # check for objects that are really close to each other
            for obj2 in self.map:
                if obj2 in items_to_remove:
                    continue

                if map_obj == obj2:
                    continue

                same = True

                if isinstance(map_obj, Shape) and isinstance(obj2, Shape):
                    if map_obj.shape != obj2.shape or map_obj.color != obj2.color:
                        same = False
                elif isinstance(map_obj, Buoy) and isinstance(obj2, Buoy):
                    if map_obj.color != obj2.color:
                        same = False
                elif type(map_obj) != type(obj2):
                    same = False

                if same:
                    self.get_logger().info(
                        f"Checking for duplicates: {map_obj} (original) and {obj2} (this frame)"
                    )

                    distance = math.sqrt(
                        (map_obj.x - obj2.x) ** 2
                        + (map_obj.y - obj2.y) ** 2
                        + (map_obj.z - obj2.z) ** 2
                    )

                    self.get_logger().info(f"Distance: {distance}")

                    if distance < BUOY_DUPLICATE_THRESHOLD:
                        self.get_logger().info("Found duplicate; merging objects")
                        # make position the average of the two
                        map_obj.x = (map_obj.x + obj2.x) / 2
                        map_obj.y = (map_obj.y + obj2.y) / 2
                        map_obj.z = (map_obj.z + obj2.z) / 2
                        map_obj.uid = map_obj.uid or obj2.uid

                        # remove the other object
                        items_to_remove.append(obj2)


        # Remove objects that are really close to each other
        self.map = [obj for obj in self.map if obj not in items_to_remove]

        # Handle other objects that havent been seen in for 5 seconds
        self.map = [obj for obj in self.map if now - obj.last_seen < 5e9]

        # Add UID to objects that don't have one
        for obj in self.map:
            if obj.uid is None:
                obj.uid = uuid1()

        self.get_logger().info(f"Map: {self.map}")

        # Publish the map
        msg = BuoyMap()
        x, y, z, types, colors, uids = [], [], [], [], [], []
        for obj in self.map:
            x.append(obj.x)
            y.append(obj.y)
            z.append(obj.z)
            uids.append(str(obj.uid))

            if isinstance(obj, Shape):
                types.append(obj.shape.value)
                colors.append(obj.color.value)
            elif isinstance(obj, PoleBuoy):
                types.append("pole")
                colors.append(obj.color.value)
            elif isinstance(obj, BallBuoy):
                types.append("ball")
                colors.append(obj.color.value)
            else:
                types.append("course_object")
                colors.append("none")

        msg.x = x
        msg.y = y
        msg.z = z
        msg.types = types
        msg.colors = colors
        msg.uids = uids

        self.map_publisher.publish(msg)

        self.previous_odom_data = odom_data

    def _is_match(self, detected_obj: CourseObject, map_obj: CourseObject) -> bool:
        """
        Check if the detected object matches with the map object

        :param detected_obj: The detected object
        :type  detected_obj: class:`mhsboat_ctrl.course_objects.CourseObject`
        :param map_obj: The map object
        :type  map_obj: class:`mhsboat_ctrl.course_objects.CourseObject`
        :return: True if the objects match, False otherwise
        :rtype:  bool
        """
        if type(detected_obj) != type(map_obj):
            return False

        if isinstance(detected_obj, Buoy) and isinstance(map_obj, Buoy):
            if detected_obj.color != map_obj.color:
                return False

        if isinstance(detected_obj, Shape) and isinstance(map_obj, Shape):
            if (
                detected_obj.shape != map_obj.shape
                or detected_obj.color != map_obj.color
            ):
                return False

        distance_threshold = 0.6  # TODO: tune this value

        self.get_logger().info(f"Checking if {detected_obj} matches original {map_obj}")
        distance = math.sqrt(
            (detected_obj.x - map_obj.x) ** 2
            + (detected_obj.y - map_obj.y) ** 2
            + (detected_obj.z - map_obj.z) ** 2
        )
        self.get_logger().info(f"Distance: {distance}")
        self.get_logger().info(f"Match: {distance < distance_threshold}")
        return distance < distance_threshold

    # calculate the 3D angle of each buoy location
    # returns a list of the left/right angle (theta) and the up/down angle (phi)
    def get_angle(self, camera_data: AiOutput, index: int) -> Tuple[float, float]:
        """
        Calculate the 3D angle of a bounding box in the camera image relative to the camera

        :param camera_data: The camera data
        :type  camera_data: class:`boat_interfaces.msg.AiOutput`
        :param index: The index of the bounding box
        :type  index: int
        :return: The left/right angle (theta) and the up/down angle (phi)
        :rtype:  Tuple[float, float]
        """

        # TODO: Use /zed/left/camera_info to get the camera parameters

        # our camera is D435
        # https://www.intelrealsense.com/depth-camera-d435/
        FOV_H = 69
        FOV_V = 42

        centerX = camera_data.img_width / 2.0
        centerY = camera_data.img_height / 2.0

        pointX = camera_data.lefts[index] + camera_data.widths[index] / 2.0
        pointY = camera_data.tops[index] + camera_data.heights[index] / 2.0

        deltaX = pointX - centerX
        deltaY = pointY - centerY

        fX = camera_data.img_width / (2.0 * math.tan(math.radians(FOV_H / 2.0)))
        fY = camera_data.img_height / (2.0 * math.tan(math.radians(FOV_V / 2.0)))

        theta = math.degrees(math.atan2(deltaX, fX))
        phi = math.degrees(math.atan2(deltaY, fY))

        return theta, phi

    def get_XYZ_coordinates(
        self, theta: float, phi: float, pointCloud: PointCloud2, name: str
    ) -> Optional[Tuple[float, float, float, np.ndarray]]:
        """
        Get the XYZ coordinates of a buoy based on the angle from the camera

        :param theta: The angle from the x-axis
        :type  theta: float
        :param phi: The angle from the z-axis
        :type  phi: float
        :param pointCloud: The point cloud data
        :type  pointCloud: class:`sensor_msgs.msg.PointCloud2`
        :param name: The name of the buoy
        :type  name: str
        :return: The X, Y, Z coordinates of the buoy, or None if the buoy is not found
        :rtype:  Tuple[float, float, float] | None
        """
        points = np.array(list(read_points(pointCloud)))

        min_distance = float("inf")
        min_point = None

        for point in points:
            x = point[0]
            y = point[1]
            z = point[2]

            pointTheta = math.degrees(math.atan2(y, x))
            pointPhi = math.degrees(math.atan2(z, x))

            # is pointTheta and pointPhi within a certain threshold of theta and phi?
            if (
                abs(pointTheta - theta) > CLUSTER_DETECTION_THRESHOLD_ANGLE
                or abs(pointPhi - phi) > CLUSTER_DETECTION_THRESHOLD_ANGLE
            ):
                continue

            dist = math.sqrt((pointTheta - theta) ** 2 + (pointPhi - phi) ** 2)

            if dist < min_distance:
                min_distance = dist
                min_point = point

        if min_point is None:
            return None
        
        return min_point[0], min_point[1], min_point[2], points

    # New helper function to publish cluster points as PointCloud2
    def _publish_cluster(self, points: np.ndarray, header) -> None:
        if points.size == 0:
            return

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        points = points.reshape(-1, 3).astype(np.float32)

        cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,
            row_step=12 * len(points),
            data=points.tobytes(),
        )

        self.buoy_cluster_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)

    sensors = Sensors()

    try:
        rclpy.spin(sensors)
    except KeyboardInterrupt:
        sensors.get_logger().info("Shutting down")
    finally:
        sensors.destroy_node()
        rclpy.shutdown()