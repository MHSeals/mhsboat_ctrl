import rclpy
import rclpy.logging
from rclpy.node import Node
from boat_interfaces.msg import AiOutput, BuoyMap
import rclpy.utilities
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
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

from mhsboat_ctrl.course_objects import CourseObject, Shape, Buoy, PoleBuoy, BallBuoy
from mhsboat_ctrl.enums import BuoyColors, Shapes
from mhsboat_ctrl.utils.lidar import read_points
from mhsboat_ctrl.utils.thruster_controller import (
    ThrusterController,
    SimulatedController,
)

buoy_color_mapping: Dict[str, BuoyColors] = {
    "black": BuoyColors.BLACK,
    "blue": BuoyColors.BLUE,
    "green": BuoyColors.GREEN,
    "red": BuoyColors.RED,
    "yellow": BuoyColors.YELLOW,
}


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
        self._odom_cache: List[Tuple[Odometry, float]] = []

        self._ai_out_sub = self.create_subscription(
            AiOutput, "/AiOutput", self._camera_callback, 10
        )

        self._lidar_out_sub = self.create_subscription(
            PointCloud2, "/center_of_clusters", self._lidar_callback, 10
        )

        self._qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        self._odom_out_sub = self.create_subscription(
            Odometry(), "/odometry/filtered", self._odom_callback, self._qos_profile
        )

        self._process_timer = self.create_timer(0.1, self._process_sensor_data)

        self.map_publisher = self.create_publisher(BuoyMap, "/mhsboat_ctrl/map", 10)

        self.map: List[CourseObject] = []

        self.get_logger().info("Sensors Node Initialized")

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

    def _process_sensor_data(self) -> None:
        """
        Process the sensor data
        """
        if (
            len(self._camera_cache) == 0
            or len(self._lidar_cache) == 0
            or len(self._odom_cache) == 0
        ):
            self.get_logger().info("Not enough data to process; waiting for more data")
            return

        # find the data that has the smallest time difference
        # and process it
        camera_data, camera_time = min(
            self._camera_cache,
            key=lambda x: abs(x[1] - self.get_clock().now().nanoseconds),
        )
        lidar_data, lidar_time = min(
            self._lidar_cache,
            key=lambda x: abs(x[1] - self.get_clock().now().nanoseconds),
        )
        odom_data, odom_time = min(
            self._odom_cache,
            key=lambda x: abs(x[1] - self.get_clock().now().nanoseconds),
        )

        time_diff = max(camera_time, lidar_time, odom_time) - min(
            camera_time, lidar_time, odom_time
        )
        self.get_logger().info(f"Data time difference: {time_diff} ns")

        # Get which data is laggging behind
        if time_diff > 1e9:
            if camera_time < lidar_time and camera_time < odom_time:
                self.get_logger().warn("Camera data is lagging behind")
            elif lidar_time < camera_time and lidar_time < odom_time:
                self.get_logger().warn("Lidar data is lagging behind")
            elif odom_time < camera_time and odom_time < lidar_time:
                self.get_logger().warn("Odometry data is lagging behind")
            else:
                self.get_logger().warn("Unknown data is lagging behind")

        # TODO: find a good average value to base this off of
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
            coords = self.get_XYZ_coordinates(
                theta, phi, lidar_data, camera_data.types[i]
            )

            self.get_logger().info(f"Buoy {i}: Coords: {coords}")

            if coords is None:
                continue

            x, y, z = coords

            # skip point if no associated lidar points
            if x == 0 and y == 0 and z == 0:
                continue

            self.get_logger().info(f"Buoy {i}: X: {x}, Y: {y}, Z: {z}")

            # latitudes.append(x)
            # longitudes.append(y)
            # buoy_types.append(camera_data.types[i])
            # lefts.append(camera_data.lefts[i])
            # tops.append(camera_data.tops[i])

            # types: ['black_buoy', 'black_circle', 'black_cross', 'black_triangle', 'blue_buoy', 'blue_circle', 'blue_cross', 'blue_racquet_ball', 'blue_triangle', 'dock', 'duck_image', 'green_buoy', 'green_cross', 'green_pole_buoy', 'green_triangle', 'misc_buoy', 'red_buoy', 'red_circle', 'red_cross', 'red_pole_buoy', 'red_racquet_ball', 'red_square', 'rubber_duck', 'yellow_buoy', 'yellow_racquet_ball']

            if camera_data.types[i].endswith("pole_buoy"):
                buoy_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                # type: ignore - this will always be either red or green
                local_detected_objects.append(PoleBuoy(x, y, z, buoy_color))
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

            local_detected_objects[-1].last_seen = self.get_clock().now().nanoseconds

        self.get_logger().info(
            f"Detected objects in this frame: {local_detected_objects}"
        )

        # Get change in position and orientation from odometry
        if odom_data is None:
            self.get_logger().warn("No odometry data available")
            return

        # Translate the map based on odometry
        translation_matrix = self._get_homogenous_transform(odom_data)
        self._translate_map(translation_matrix)

        # Check if the detected objects match with the map objects
        # TODO: Is there a more efficient way to do this?
        for detected_obj in local_detected_objects:
            matched = False
            for map_obj in self.map:
                if self._is_match(detected_obj, map_obj):
                    map_obj.x = detected_obj.x
                    map_obj.y = detected_obj.y
                    map_obj.last_seen = detected_obj.last_seen
                    matched = True
                    break
            else:
                ...
            if not matched:
                self.map.append(detected_obj)

        # Check for objects that aren't seen by camera, but are seen by lidar
        for map_obj in self.map:
            theta = math.degrees(math.atan2(map_obj.y, map_obj.x))
            phi = math.degrees(math.atan2(map_obj.z, map_obj.x))

            coords = self.get_XYZ_coordinates(theta, phi, lidar_data, "lidar")
            if coords is None:
                continue

            x, y, z = coords

            if x == 0 and y == 0 and z == 0:
                continue

            map_obj.x = x
            map_obj.y = y
            map_obj.z = z
            map_obj.last_seen = self.get_clock().now().nanoseconds

        # Handle other objects that havent been seen in for 5 seconds
        self.map = [
            obj
            for obj in self.map
            if self.get_clock().now().nanoseconds - obj.last_seen < 5e9
        ]

        self.get_logger().info(f"Map: {self.map}")

        # Publish the map
        msg = BuoyMap()
        x, y, z, types, colors = [], [], [], [], []
        for obj in self.map:
            x.append(obj.x)
            y.append(obj.y)
            z.append(obj.z)

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

        self.map_publisher.publish(msg)

    def _get_homogenous_transform(self, odom: Odometry) -> np.ndarray:
        """
        Create a translation matrix from a PoseStamped message.
        """
        orientation = odom.pose.pose.orientation
        position = odom.pose.pose.position

        self.get_logger().info(f"Orientation: {orientation}")
        self.get_logger().info(f"Position: {position}")

        # Convert quaternion to rotation matrix
        rotation_matrix = self._quaternion_to_rotation_matrix(orientation)

        self.get_logger().info(f"Rotation matrix: {rotation_matrix}")

        # Build the translation matrix with position data
        translation_matrix = np.eye(4)
        translation_matrix[:3, :3] = rotation_matrix
        translation_matrix[:3, 3] = [position.x, position.y, position.z]

        return translation_matrix

    def _quaternion_to_rotation_matrix(self, q: Quaternion) -> np.ndarray:
        """
        Convert a quaternion to a rotation matrix

        :param q: The quaternion
        :type  q: class:`geometry_msgs.msg.Quaternion`
        :return: The rotation matrix
        :rtype:  numpy.ndarray
        """
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array(
            [
                [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
            ]
        )

    def _translate_map(self, translation_matrix: np.ndarray) -> None:
        """
        Translate the map based on the translation matrix

        :param translation_matrix: The translation matrix
        :type  translation_matrix: numpy.ndarray
        """
        for obj in self.map:
            self.get_logger().info(f"Translating {obj}")
            point = np.array([obj.x, obj.y, obj.z, 1])
            translated_point = translation_matrix @ point
            obj.x, obj.y, obj.z = translated_point[:3]
            self.get_logger().info(f"Translated point: {translated_point}")

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
        distance_threshold = 0.3  # TODO: tune this value
        self.get_logger().info(f"Checking if {detected_obj} matches original {map_obj}")
        distance = math.sqrt(
            (detected_obj.x - map_obj.x) ** 2 + (detected_obj.y - map_obj.y) ** 2
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
    ) -> Optional[Tuple[float, float, float]]:
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
        mask = np.empty(points.shape[0], dtype=bool)
        for index, point in enumerate(points):
            x = point[0]
            y = point[1]
            z = point[2]

            # calculate angle from x axis, the camera always points towards the x axis so we only care about lidar points near the x axis
            # might have to change if camera doesn't point towards real lidar's x asix

            thetaPoint = (
                math.degrees(math.acos(x / math.sqrt(x**2 + y**2))) * y / abs(y) * -1
            )
            phiPoint = math.degrees(math.acos(x / math.sqrt(x**2 + z**2))) * z / abs(z)

            # if this works im a genius
            # thetaPoint -= 180

            # normalize theta and phi to be between 0 and 360
            # thetaPoint = (thetaPoint + 360) % 360
            # phiPoint = (phiPoint + 360) % 360

            # max angle difference to consider a point a match
            degrees = 5
            mask[index] = not (
                math.fabs(thetaPoint - theta) <= degrees
                and math.fabs(phiPoint - phi) <= degrees
            )

        points = np.delete(points, mask, axis=0)

        if len(points) > 1:
            rp = np.mean(points, axis=0)
            return rp[0], rp[1], rp[2]
        if len(points) == 0:
            return None

        return (points[0][0], points[0][1], points[0][2])


# TODO: rewrite this class as its own node


class SensorsSimulated(Node):
    # TODO: fix this class
    def __init__(self, map_file: str):
        super().__init__("sensors")

        if not os.path.exists(map_file):
            raise FileNotFoundError(f"Map file {map_file} does not exist")

        self.map: List[CourseObject] = []

        with open(map_file, "r") as f:
            try:
                map_data = yaml.safe_load(f)

                for obj in map_data["objects"]:
                    if obj.get("pole_buoy") is not None:
                        if obj["pole_buoy"]["color"].upper() not in ["RED", "GREEN"]:
                            raise ValueError(
                                f"Invalid color for pole buoy: {obj['pole_buoy']['color']} at {obj['pole_buoy']['x']}, {obj['pole_buoy']['y']}"
                            )

                        self.map.append(
                            PoleBuoy(
                                obj["pole_buoy"]["x"],
                                obj["pole_buoy"]["y"],
                                # type: ignore
                                # type: ignore - this will always be either red or green
                                BuoyColors[obj["pole_buoy"]["color"].upper()],
                            )
                        )
                    elif obj.get("ball_buoy") is not None:
                        self.map.append(
                            BallBuoy(
                                obj["ball_buoy"]["x"],
                                obj["ball_buoy"]["y"],
                                obj["ball_buoy"]["z"],
                                BuoyColors[obj["ball_buoy"]["color"].upper()],
                            )
                        )
                    elif obj.get("buoy") is not None:
                        self.map.append(
                            Buoy(
                                obj["buoy"]["x"],
                                obj["buoy"]["y"],
                                obj["buoy"]["z"],
                                BuoyColors[obj["buoy"]["color"].upper()],
                            )
                        )
                    elif obj.get("shape") is not None:
                        self.map.append(
                            Shape(
                                obj["shape"]["x"],
                                obj["shape"]["y"],
                                obj["shape"]["z"],
                                Shapes[obj["shape"]["shape"].upper()],
                                BuoyColors[obj["shape"]["color"].upper()],
                            )
                        )
                    elif obj.get("course_object") is not None:
                        self.map.append(
                            CourseObject(
                                obj["course_object"]["x"],
                                obj["course_object"]["y"],
                                obj["course_object"]["z"],
                            )
                        )
                    else:
                        raise ValueError(f"Unknown buoy type in map file: {obj}")
            except yaml.YAMLError as e:
                raise ValueError(f"Error parsing map file: {e}")

        self.get_logger().info("Simualted Sensors Node Initialized")


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
