import rclpy
from rclpy.node import Node
from boat_interfaces.msg import AiOutput
from sensor_msgs.msg import PointCloud2, Imu, PointField
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
import os
import yaml
import math
import numpy as np
import struct
import sys

from mhsboat_ctrl.course_objects import CourseObject, Shape, Buoy, PoleBuoy, BallBuoy
from mhsboat_ctrl.enums import BuoyColors, Shapes

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

        # cache for the sensors, limited to 3 items
        # this allows us to find the data that has the
        # smallest time difference, giving more accurate
        # results
        self._camera_cache: list[tuple[AiOutput, float]] = []
        self._lidar_cache: list[tuple[PointCloud2, float]] = []
        self._imu_cache: list[tuple[Imu, float]] = []

        self._DATATYPES = {}
        self._DATATYPES[PointField.INT8] = ("b", 1)
        self._DATATYPES[PointField.UINT8] = ("B", 1)
        self._DATATYPES[PointField.INT16] = ("h", 2)
        self._DATATYPES[PointField.UINT16] = ("H", 2)
        self._DATATYPES[PointField.INT32] = ("i", 4)
        self._DATATYPES[PointField.UINT32] = ("I", 4)
        self._DATATYPES[PointField.FLOAT32] = ("f", 4)
        self._DATATYPES[PointField.FLOAT64] = ("d", 8)

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

    def _process_sensor_data(self):
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
            x, y, z = self.get_XYZ_coordinates(
                theta, phi, lidar_data, camera_data.types[i])

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

        # TODO: Match the detected objects from frame with the map

    # calculate the 3D angle of each buoy location
    # returns a list of the left/right angle (theta) and the up/down angle (phi)
    def get_angle(self, camera_data: AiOutput, index):
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

    def get_XYZ_coordinates(self, theta, phi, pointCloud, name):
        points = np.array(list(self.read_points(pointCloud)))
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

        print(name)
        print("after points: " + str(points))
        if len(points) > 1:
            rp = np.mean(points, axis=0)
            return rp[0], rp[1], rp[2]
        if len(points) == 0:
            return (0, 0, 0)
        return (points[0][0], points[0][1], points[0][2])
    
    def read_points(self, cloud, field_names=None, skip_nans=False, uvs=[]):
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
        fmt = self._get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
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


    def _get_struct_fmt(self, is_bigendian, fields, field_names=None):
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
            if field.datatype not in self._DATATYPES:
                print(
                    "Skipping unknown PointField datatype [%d]" % field.datatype,
                    file=sys.stderr,
                )
            else:
                datatype_fmt, datatype_length = self._DATATYPES[field.datatype]
                fmt += field.count * datatype_fmt
                offset += field.count * datatype_length

        return fmt

    """ 
    TODO:
        1. get slam - done
        2. wait for clustering - done
        3. wait for ai output - done
        4. match ai output with clustering - done
        5. mark detected buoys on the map
        6. profit
    """


class SensorsSimulated(Node):
    def __init__(self, map_file: str):
        super().__init__('sensors')

        if not os.path.exists(map_file):
            raise FileNotFoundError(f"Map file {map_file} does not exist")

        self.detected_objects: list[CourseObject] = []

        with open(map_file, "r") as f:
            try:
                map_data = yaml.safe_load(f)

                for obj in map_data["objects"]:
                    if obj.get('pole_buoy') is not None:
                        if obj['pole_buoy']['color'].upper() not in ["RED", "GREEN"]:
                            raise ValueError(f"Invalid color for pole buoy: {obj['pole_buoy']['color']} at {obj['pole_buoy']['x']}, {obj['pole_buoy']['y']}")
                        
                        self.detected_objects.append(
                            PoleBuoy(
                                obj['pole_buoy']['x'],
                                obj['pole_buoy']['y'],
                                # type: ignore
                                BuoyColors[obj['pole_buoy']['color'].upper()] # type: ignore - this will always be either red or green
                            )
                        )
            except yaml.YAMLError as e:
                raise ValueError(f"Error parsing map file: {e}")

        self.get_logger().info("Simualted Sensors Node Initialized")
