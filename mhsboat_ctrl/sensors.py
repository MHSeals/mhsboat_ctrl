import rclpy
import rclpy.logging
from rclpy.node import Node
from boat_interfaces.msg import AiOutput, BuoyMap
import rclpy.utilities
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from zed_msgs.msg import ObjectsStamped
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
from typing import Dict, List, Tuple, Optional, cast, Literal
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

# ZED object detection label mapping to our course objects
zed_label_mapping = {
    "black_buoy": ("buoy", BuoyColors.BLACK),
    "blue_buoy": ("buoy", BuoyColors.BLUE), 
    "green_buoy": ("buoy", BuoyColors.GREEN),
    "red_buoy": ("buoy", BuoyColors.RED),
    "yellow_buoy": ("buoy", BuoyColors.YELLOW),
    "black_pole_buoy": ("pole_buoy", BuoyColors.BLACK),
    "blue_pole_buoy": ("pole_buoy", BuoyColors.BLUE),
    "green_pole_buoy": ("pole_buoy", BuoyColors.GREEN),
    "red_pole_buoy": ("pole_buoy", BuoyColors.RED),
    "yellow_pole_buoy": ("pole_buoy", BuoyColors.YELLOW),
    "black_circle": ("circle", BuoyColors.BLACK),
    "blue_circle": ("circle", BuoyColors.BLUE),
    "green_circle": ("circle", BuoyColors.GREEN),
    "red_circle": ("circle", BuoyColors.RED),
    "yellow_circle": ("circle", BuoyColors.YELLOW),
    "black_triangle": ("triangle", BuoyColors.BLACK),
    "blue_triangle": ("triangle", BuoyColors.BLUE),
    "green_triangle": ("triangle", BuoyColors.GREEN),
    "red_triangle": ("triangle", BuoyColors.RED),
    "yellow_triangle": ("triangle", BuoyColors.YELLOW),
    "black_cross": ("cross", BuoyColors.BLACK),
    "blue_cross": ("cross", BuoyColors.BLUE),
    "green_cross": ("cross", BuoyColors.GREEN),
    "red_cross": ("cross", BuoyColors.RED),
    "yellow_cross": ("cross", BuoyColors.YELLOW),
    "black_square": ("square", BuoyColors.BLACK),
    "blue_square": ("square", BuoyColors.BLUE),
    "green_square": ("square", BuoyColors.GREEN),
    "red_square": ("square", BuoyColors.RED),
    "yellow_square": ("square", BuoyColors.YELLOW),
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
        self._zed_objects_cache: List[Tuple[ObjectsStamped, float]] = []
        self._lidar_cache: List[Tuple[PointCloud2, float]] = []
        self._odom_cache: List[Tuple[Odometry, float]] = []

        self._qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        self._zed_objects_sub = self.create_subscription(
            ObjectsStamped, "/zed/zed_node/obj_det/objects", self._zed_objects_callback, self._qos_profile
        )

        self._lidar_out_sub = self.create_subscription(
            PointCloud2, "/center_of_clusters", self._lidar_callback, self._qos_profile
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

    def _zed_objects_callback(self, msg: ObjectsStamped):
        self.get_logger().info("Received ZED object detection data")
        self._zed_objects_cache.append((msg, self.get_clock().now().nanoseconds))
        if len(self._zed_objects_cache) > 3:
            self._zed_objects_cache.pop(0)

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
    def zed_objects_output(self) -> Optional[ObjectsStamped]:
        return self._zed_objects_cache[-1][0] if len(self._zed_objects_cache) > 0 else None

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
        Process the sensor data using ZED object detection
        """
        if len(self._zed_objects_cache) == 0:
            self.get_logger().warn("No ZED object detection data available")
            return

        if len(self._lidar_cache) == 0:
            self.get_logger().warn("No lidar data available; however, it is not necessary")
            # Set lidar data to None since it's optional
            lidar_data, lidar_time = None, 0
        else:
            lidar_data, lidar_time = min(
                self._lidar_cache,
                key=lambda x: abs(x[1] - ref_time),
            )

        if len(self._odom_cache) == 0:
            self.get_logger().warn("No odometry data available")
            return

        now = self.get_clock().now().nanoseconds

        ref_time = self._zed_objects_cache[-1][1]

        # find the data that has the smallest time difference
        # and process it
        zed_objects_data, zed_objects_time = min(
            self._zed_objects_cache,
            key=lambda x: abs(x[1] - ref_time),
        )
        odom_data, odom_time = min(
            self._odom_cache,
            key=lambda x: abs(x[1] - ref_time),
        )

        if self.previous_odom_data is None:
            self.previous_odom_data = odom_data
            return

        # Calculate time difference, excluding lidar if not available
        if lidar_data is not None:
            time_diff = max(zed_objects_time, lidar_time, odom_time) - min(
                zed_objects_time, lidar_time, odom_time
            )
        else:
            time_diff = max(zed_objects_time, odom_time) - min(
                zed_objects_time, odom_time
            )
        self.get_logger().info(f"Data time difference: {time_diff} ns")

        # Get which data is lagging behind
        if time_diff > 1e9:
            if lidar_data is not None:
                if zed_objects_time < lidar_time and zed_objects_time < odom_time:
                    self.get_logger().warn("ZED object detection data is lagging behind")
                elif lidar_time < zed_objects_time and lidar_time < odom_time:
                    self.get_logger().warn("Lidar data is lagging behind")
                elif odom_time < zed_objects_time and odom_time < lidar_time:
                    self.get_logger().warn("Odometry data is lagging behind")
                else:
                    self.get_logger().warn("Unknown data is lagging behind")
            else:
                if zed_objects_time < odom_time:
                    self.get_logger().warn("ZED object detection data is lagging behind")
                elif odom_time < zed_objects_time:
                    self.get_logger().warn("Odometry data is lagging behind")
                else:
                    self.get_logger().warn("Unknown data is lagging behind")

        # TODO: find a good average value to base this off of
        # Currently set to 1 second
        if time_diff > 1e9:
            self.get_logger().warn(f"Data time difference is large: {time_diff} ns")

        local_detected_objects: List[CourseObject] = []

        # Process ZED object detection results
        for obj in zed_objects_data.objects:
            # Skip objects with low confidence
            if obj.confidence < 70.0:  # Adjustable confidence threshold
                continue
                
            # Skip objects that are not being tracked properly
            if obj.tracking_state == 0:  # OFF - object no longer valid
                continue
                
            # Get label and map to our course object types
            label = obj.label.lower()
            if label not in zed_label_mapping:
                self.get_logger().warn(f"Unknown object label: {label}")
                continue
                
            obj_type, color = zed_label_mapping[label]
            
            # Use the 3D position directly from ZED (already in camera frame)
            x, y, z = obj.position[0], obj.position[1], obj.position[2]
            
            self.get_logger().info(f"Detected {label}: X: {x}, Y: {y}, Z: {z}, Confidence: {obj.confidence}")

            # Create the appropriate course object
            course_obj = None
            if obj_type == "pole_buoy":
                # PoleBuoy only accepts RED or GREEN colors
                if color in [BuoyColors.RED, BuoyColors.GREEN]:
                    # Use cast to satisfy the literal type constraint
                    course_obj = PoleBuoy(x, y, z, cast("Literal[BuoyColors.RED, BuoyColors.GREEN]", color))
                else:
                    self.get_logger().warn(f"Invalid color {color} for pole buoy, skipping")
                    continue
            elif obj_type == "buoy":
                course_obj = BallBuoy(x, y, z, color)
            elif obj_type == "circle":
                course_obj = Shape(x, y, z, Shapes.CIRCLE, color)
            elif obj_type == "triangle":
                course_obj = Shape(x, y, z, Shapes.TRIANGLE, color)
            elif obj_type == "cross":
                course_obj = Shape(x, y, z, Shapes.CROSS, color)
            elif obj_type == "square":
                course_obj = Shape(x, y, z, Shapes.SQUARE, color)
                
            if course_obj:
                course_obj.last_seen = now
                # Store ZED tracking ID for future reference
                course_obj.zed_tracking_id = obj.sublabel  # Use sublabel as tracking ID
                local_detected_objects.append(course_obj)

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

        # Validate transformation values to prevent NaN
        if np.any(np.isnan(trans)) or np.any(np.isnan(delta_quat)):
            self.get_logger().warn("NaN detected in transformation, skipping frame")
            return

        transformation_matrix = tf_transformations.quaternion_matrix(delta_quat)
        transformation_matrix[0:3, 3] = trans

        # Validate transformation matrix
        if np.any(np.isnan(transformation_matrix)):
            self.get_logger().warn("NaN detected in transformation matrix, skipping frame")
            return

        self.get_logger().info(f"Transformation matrix: \n{transformation_matrix}")

        items_to_remove = []

        # Store existing map objects before processing new detections
        existing_map_objects = self.map.copy()

        # Check for objects that aren't seen by camera, but are seen by lidar

        # TODO: Use ZED tracking ID to match objects more accurately
        # Only process existing map objects, not newly detected ones
        for map_obj in existing_map_objects:
            if map_obj in items_to_remove:
                continue

            # Transform the map object based on the odometry
            point_hom = np.array([map_obj.x, map_obj.y, map_obj.z, 1])

            point_trans = np.dot(transformation_matrix, point_hom)

            # Validate transformed coordinates
            if np.any(np.isnan(point_trans)):
                self.get_logger().warn(f"NaN detected in transformed coordinates for object {map_obj}, marking for removal")
                items_to_remove.append(map_obj)
                continue

            map_obj.x = point_trans[0]
            map_obj.y = point_trans[1]
            map_obj.z = point_trans[2]

            # Try to find a matching object in the current detections using ZED tracking ID
            matched = False
            for detected_obj in local_detected_objects:
                if (hasattr(map_obj, 'zed_tracking_id') and 
                    hasattr(detected_obj, 'zed_tracking_id') and
                    map_obj.zed_tracking_id is not None and
                    detected_obj.zed_tracking_id is not None and
                    map_obj.zed_tracking_id == detected_obj.zed_tracking_id):
                    # Update position with the new detection
                    map_obj.x = detected_obj.x
                    map_obj.y = detected_obj.y
                    map_obj.z = detected_obj.z
                    map_obj.last_seen = now
                    matched = True
                    # Remove from local detections to avoid duplicates
                    items_to_remove.append(detected_obj)
                    break

            # If no tracking ID match, check for spatial proximity (fallback)
            if not matched:
                for detected_obj in local_detected_objects:
                    if detected_obj in items_to_remove:
                        continue
                        
                    # Check if this is the same type of object
                    if not self._is_match(detected_obj, map_obj):
                        continue
                        
                    distance = math.sqrt(
                        (map_obj.x - detected_obj.x) ** 2
                        + (map_obj.y - detected_obj.y) ** 2
                        + (map_obj.z - detected_obj.z) ** 2
                    )
                    
                    if distance < BUOY_DUPLICATE_THRESHOLD:
                        # Update the map object with new detection
                        map_obj.x = detected_obj.x
                        map_obj.y = detected_obj.y  
                        map_obj.z = detected_obj.z
                        map_obj.last_seen = now
                        # Update tracking ID if it was missing
                        if hasattr(detected_obj, 'zed_tracking_id'):
                            map_obj.zed_tracking_id = detected_obj.zed_tracking_id
                        items_to_remove.append(detected_obj)
                        matched = True
                        break

        # Add any new detections that weren't matched to existing objects
        for obj in local_detected_objects:
            if obj not in items_to_remove:
                self.map.append(obj)

        # Remove objects that are really close to each other (duplicates)
        additional_removals = []
        for i, obj1 in enumerate(self.map):
            if obj1 in items_to_remove or obj1 in additional_removals:
                continue
                
            for j, obj2 in enumerate(self.map[i+1:], i+1):
                if obj2 in items_to_remove or obj2 in additional_removals:
                    continue

                # Skip if they have the same ZED tracking ID (they're the same object)
                if (hasattr(obj1, 'zed_tracking_id') and hasattr(obj2, 'zed_tracking_id') and
                    obj1.zed_tracking_id is not None and obj2.zed_tracking_id is not None and
                    obj1.zed_tracking_id == obj2.zed_tracking_id):
                    continue

                same = self._is_match(obj1, obj2)

                if same:
                    self.get_logger().info(
                        f"Checking for duplicates: {obj1} and {obj2}"
                    )

                    distance = math.sqrt(
                        (obj1.x - obj2.x) ** 2
                        + (obj1.y - obj2.y) ** 2
                        + (obj1.z - obj2.z) ** 2
                    )

                    self.get_logger().info(f"Distance: {distance}")

                    if distance < BUOY_DUPLICATE_THRESHOLD:
                        self.get_logger().info("Found duplicate; merging objects")
                        # Keep the one with more recent detection or ZED tracking ID
                        if (hasattr(obj1, 'zed_tracking_id') and obj1.zed_tracking_id is not None and
                            (not hasattr(obj2, 'zed_tracking_id') or obj2.zed_tracking_id is None)):
                            # Keep obj1, remove obj2
                            additional_removals.append(obj2)
                        elif (hasattr(obj2, 'zed_tracking_id') and obj2.zed_tracking_id is not None and
                              (not hasattr(obj1, 'zed_tracking_id') or obj1.zed_tracking_id is None)):
                            # Keep obj2, remove obj1
                            additional_removals.append(obj1)
                        else:
                            # Make position the average of the two and keep the more recent one
                            if obj1.last_seen >= obj2.last_seen:
                                obj1.x = (obj1.x + obj2.x) / 2
                                obj1.y = (obj1.y + obj2.y) / 2
                                obj1.z = (obj1.z + obj2.z) / 2
                                additional_removals.append(obj2)
                            else:
                                obj2.x = (obj1.x + obj2.x) / 2
                                obj2.y = (obj1.y + obj2.y) / 2
                                obj2.z = (obj1.z + obj2.z) / 2
                                additional_removals.append(obj1)

        # Remove all marked objects
        items_to_remove.extend(additional_removals)
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
            # Skip objects with NaN coordinates
            if np.isnan(obj.x) or np.isnan(obj.y) or np.isnan(obj.z):
                self.get_logger().warn(f"Skipping object with NaN coordinates: {obj}")
                continue
                
            x.append(float(obj.x))
            y.append(float(obj.y))
            z.append(float(obj.z))
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

    # New helper function to publish cluster points as PointCloud2 (for debugging)
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