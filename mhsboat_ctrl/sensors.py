import rclpy
from rclpy.node import Node
from boat_interfaces.msg import AiOutput
from sensor_msgs.msg import PointCloud2, Imu, PointField
from geometry_msgs.msg import Quaternion
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
import os
import yaml
import math
import numpy as np

from mhsboat_ctrl.course_objects import CourseObject, Shape, Buoy, PoleBuoy, BallBuoy
from mhsboat_ctrl.enums import BuoyColors, Shapes
from mhsboat_ctrl.utils.lidar import read_points
from mhsboat_ctrl.utils.thruster_controller import ThrusterController, SimulatedController

buoy_color_mapping: dict[str, BuoyColors] = {
    "black": BuoyColors.BLACK,
    "blue": BuoyColors.BLUE,
    "green": BuoyColors.GREEN,
    "red": BuoyColors.RED,
    "yellow": BuoyColors.YELLOW,
}


class Sensors(Node):
    def __init__(self):
        super().__init__('sensors')

        self.map: list[CourseObject] = []

        self.controller = ThrusterController()

        # cache for the sensors, limited to 3 items
        # this allows us to find the data that has the
        # smallest time difference, giving more accurate
        # results
        self._camera_cache: list[tuple[AiOutput, float]] = []
        self._lidar_cache: list[tuple[PointCloud2, float]] = []
        self._imu_cache: list[tuple[Imu, float]] = []

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

        self._imu_out_sub = self.create_subscription(
            Imu, "/mavros/imu/data", self._imu_callback, self._qos_profile
        )

        self._process_timer = self.create_timer(0.1, self._process_sensor_data)
        super().__init__('sensors')

        self.map: list[CourseObject] = []

        self.get_logger().info("Sensors Node Initialized")

    def _camera_callback(self, msg: AiOutput):
        self._camera_cache.append((msg, self.get_clock().now().nanoseconds))
        if len(self._camera_cache) > 3:
            self._camera_cache.pop(0)

    def _lidar_callback(self, msg: PointCloud2):
        self._lidar_cache.append((msg, self.get_clock().now().nanoseconds))
        if len(self._lidar_cache) > 3:
            self._lidar_cache.pop(0)

    def _imu_callback(self, msg: Imu):
        self._imu_cache.append((msg, self.get_clock().now().nanoseconds))
        if len(self._imu_cache) > 3:
            self._imu_cache.pop(0)

    @property
    def ai_output(self) -> AiOutput | None:
        return self._camera_cache[-1][0] if len(self._camera_cache) > 0 else None

    @property
    def lidar_output(self) -> PointCloud2 | None:
        return self._lidar_cache[-1][0] if len(self._lidar_cache) > 0 else None

    @property
    def imu_output(self) -> Imu | None:
        return self._imu_cache[-1][0] if len(self._imu_cache) > 0 else None

    def _process_sensor_data(self) -> None:
        """
        Process the sensor data
        """
        if len(self._camera_cache) == 0 or len(self._lidar_cache) == 0 or len(self._imu_cache) == 0:
            self.get_logger().info("Not enough data to process; waiting for more data")
            return

        # find the data that has the smallest time difference
        # and process it
        camera_data, camera_time = min(self._camera_cache, key=lambda x: abs(
            x[1] - self.get_clock().now().nanoseconds))
        lidar_data, lidar_time = min(self._lidar_cache, key=lambda x: abs(
            x[1] - self.get_clock().now().nanoseconds))
        imu_data, imu_time = min(self._imu_cache, key=lambda x: abs(
            x[1] - self.get_clock().now().nanoseconds))
        
        # static type checking jargon
        assert type(camera_data.types) == list[str]
        
        time_diff = max(camera_time, lidar_time, imu_time) - min(camera_time, lidar_time, imu_time)

        # TODO: find a good average value to base this off of
        if time_diff > 1e9:
            self.get_logger().warn(f"Data time difference is large: {time_diff} ns")

        if camera_data.num != len(camera_data.lefts) or camera_data.num != len(camera_data.tops) or camera_data.num != len(camera_data.types):
            self.get_logger().warn("Camera data is not consistent")
            return

        local_detected_objects: list[CourseObject] = []

        for i in range(camera_data.num):
            # calculate the 3D angle of each buoy location
            # returns a list of the left/right angle (theta) and the up/down angle (phi) relative to boat
            theta, phi = self.get_angle(camera_data, i)

            # Use angle to get the XYZ coordinates of each buoy
            # returns X, Y, Z
            coords = self.get_XYZ_coordinates(
                theta, phi, lidar_data, camera_data.types[i])
            
            if coords is None:
                continue

            x, y, z = coords

            # skip point if no associated lidar points
            if x == 0 and y == 0 and z == 0:
                continue

            # latitudes.append(x)
            # longitudes.append(y)
            # buoy_types.append(camera_data.types[i])
            # lefts.append(camera_data.lefts[i])
            # tops.append(camera_data.tops[i])

            # types: ['black_buoy', 'black_circle', 'black_cross', 'black_triangle', 'blue_buoy', 'blue_circle', 'blue_cross', 'blue_racquet_ball', 'blue_triangle', 'dock', 'duck_image', 'green_buoy', 'green_cross', 'green_pole_buoy', 'green_triangle', 'misc_buoy', 'red_buoy', 'red_circle', 'red_cross', 'red_pole_buoy', 'red_racquet_ball', 'red_square', 'rubber_duck', 'yellow_buoy', 'yellow_racquet_ball']

            if camera_data.types[i].endswith("pole_buoy"):
                buoy_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                local_detected_objects.append(PoleBuoy(x, y, buoy_color)) # type: ignore - this will always be either red or green
            elif camera_data.types[i].endswith("buoy"):
                buoy_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                local_detected_objects.append(BallBuoy(x, y, buoy_color))
            elif camera_data.types[i].endswith("racquet_ball"):
                # TODO: can we effectively track racquet balls given that they are so small?
                # Do we need to track them?
                pass
            elif camera_data.types[i].endswith("circle"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                local_detected_objects.append(Shape(x, y, Shapes.CIRCLE, shape_color))
            elif camera_data.types[i].endswith("triangle"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                local_detected_objects.append(Shape(x, y, Shapes.TRIANGLE, shape_color))
            elif camera_data.types[i].endswith("cross"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                local_detected_objects.append(Shape(x, y, Shapes.CROSS, shape_color))
            elif camera_data.types[i].endswith("square"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                local_detected_objects.append(Shape(x, y, Shapes.SQUARE, shape_color))
            elif camera_data.types[i].endswith("duck_image"):
                # TODO: do we need to track duck images?
                pass

            local_detected_objects[-1].last_seen = self.get_clock().now().nanoseconds
        
        # Get change in position and orientation from odometry
        if imu_data is None:
            self.get_logger().warn("No odometry data available")
            return

        # Translate the map based on odometry
        translation_matrix = self._get_translation_matrix(imu_data)
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
    
    def _get_translation_matrix(self, odom_data: Imu) -> np.ndarray:
        """
        Get the translation matrix based on odometry data

        :param odom_data: The odometry data
        :type  odom_data: class:`sensor_msgs.msg.Imu`
        :return: The translation matrix
        :rtype:  numpy.ndarray
        """
        # Assuming odom_data contains orientation as quaternion and position as x, y, z
        orientation = odom_data.orientation
        position = odom_data.position

        # Convert quaternion to rotation matrix
        rotation_matrix = self._quaternion_to_rotation_matrix(orientation)

        # Create translation matrix
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
        return np.array([
            [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)]
        ])

    def _translate_map(self, translation_matrix: np.ndarray) -> None:
        """
        Translate the map based on the translation matrix

        :param translation_matrix: The translation matrix
        :type  translation_matrix: numpy.ndarray
        """
        for obj in self.map:
            point = np.array([obj.x, obj.y, 0, 1])
            translated_point = translation_matrix @ point
            obj.x, obj.y = translated_point[:2]

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
        distance_threshold = 1.0  # Define a threshold for matching objects
        distance = np.sqrt((detected_obj.x - map_obj.x) ** 2 + (detected_obj.y - map_obj.y) ** 2)
        return distance < distance_threshold

    # calculate the 3D angle of each buoy location
    # returns a list of the left/right angle (theta) and the up/down angle (phi)
    def get_angle(self, camera_data: AiOutput, index: int) -> tuple[float, float]:
        """
        Calculate the 3D angle of a bounding box in the camera image relative to the camera

        :param camera_data: The camera data
        :type  camera_data: class:`boat_interfaces.msg.AiOutput`
        :param index: The index of the bounding box
        :type  index: int
        :return: The left/right angle (theta) and the up/down angle (phi)
        :rtype:  tuple[float, float]
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

        fX = camera_data.img_width / \
            (2.0 * math.tan(math.radians(FOV_H / 2.0)))
        fY = camera_data.img_height / \
            (2.0 * math.tan(math.radians(FOV_V / 2.0)))

        theta = math.degrees(math.atan(deltaX / fX))
        phi = math.degrees(math.atan(deltaY / fY))

        return theta, phi

    # Use angle to get the XYZ coordinates of each buoy
    # returns X, Y, Z

    def get_XYZ_coordinates(self, theta: float, phi: float, pointCloud: PointCloud2, name: str):
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
        :rtype:  tuple[float, float, float] | None
        """
        points = np.array(list(read_points(pointCloud)))
        mask = np.empty(points.shape[0], dtype=bool)
        for index, point in enumerate(points):
            x = point[0]
            y = point[1]
            z = point[2]
            r = math.sqrt(x**2 + y**2 + z**2)

            # calculate angle from x axis, the camera always points towards the x axis so we only care about lidar points near the x axis
            # might have to change if camera doesn't point towards real lidar's x asix

            thetaPoint = (
                math.degrees(math.acos(x / math.sqrt(x**2 + y**2))
                             ) * y / abs(y) * -1
            )
            phiPoint = math.degrees(
                math.acos(x / math.sqrt(x**2 + z**2))) * z / abs(z)

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

        if len(points) > 1:
            rp = np.mean(points, axis=0)
            return rp[0], rp[1], rp[2]
        if len(points) == 0:
            return None
        return (points[0][0], points[0][1], points[0][2])

# TODO: Make SensorsSimulated relative to the boat
# TODO: Move buoy locations according to fake odometry
class SensorsSimulated(Node):
    def __init__(self, map_file: str):
        super().__init__('sensors')

        self.controller = SimulatedController()

        if not os.path.exists(map_file):
            raise FileNotFoundError(f"Map file {map_file} does not exist")

        self.map: list[CourseObject] = []

        with open(map_file, "r") as f:
            try:
                map_data = yaml.safe_load(f)

                for obj in map_data["objects"]:
                    if obj.get('pole_buoy') is not None:
                        if obj['pole_buoy']['color'].upper() not in ["RED", "GREEN"]:
                            raise ValueError(f"Invalid color for pole buoy: {obj['pole_buoy']['color']} at {obj['pole_buoy']['x']}, {obj['pole_buoy']['y']}")
                        
                        self.map.append(
                            PoleBuoy(
                                obj['pole_buoy']['x'],
                                obj['pole_buoy']['y'],
                                # type: ignore
                                BuoyColors[obj['pole_buoy']['color'].upper()] # type: ignore - this will always be either red or green
                            )
                        )
                    elif obj.get('ball_buoy') is not None:
                        self.map.append(
                            BallBuoy(
                                obj['ball_buoy']['x'],
                                obj['ball_buoy']['y'],
                                BuoyColors[obj['ball_buoy']['color'].upper()]
                            )
                        )
                    elif obj.get('buoy') is not None:
                        self.map.append(
                            Buoy(
                                obj['buoy']['x'],
                                obj['buoy']['y'],
                                BuoyColors[obj['buoy']['color'].upper()]
                            )
                        )
                    elif obj.get('shape') is not None:
                        self.map.append(
                            Shape(
                                obj['shape']['x'],
                                obj['shape']['y'],
                                Shapes[obj['shape']['shape'].upper()],
                                BuoyColors[obj['shape']['color'].upper()]
                            )
                        )
                    elif obj.get('course_object') is not None:
                        self.map.append(
                            CourseObject(
                                obj['course_object']['x'],
                                obj['course_object']['y']
                            )
                        )
                    else:
                        raise ValueError(f"Unknown buoy type in map file: {obj}")
            except yaml.YAMLError as e:
                raise ValueError(f"Error parsing map file: {e}")

        self.get_logger().info("Simualted Sensors Node Initialized")
