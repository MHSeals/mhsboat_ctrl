import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSLivelinessPolicy,
)
from cv_bridge import CvBridge
from boat_interfaces.msg import AiOutput, BuoyMap, BoatMovement
from sensor_msgs.msg import Odometry, CameraInfo, Image 
import numpy as np
import tf_transformations
from typing import Dict

from mhsboat_ctrl.course_objects import CourseObject, Shape, Buoy, PoleBuoy, BallBuoy
from mhsboat_ctrl.enums import BuoyColors, Shapes

buoy_color_mapping: Dict[str, BuoyColors] = {
    "black": BuoyColors.BLACK,
    "blue": BuoyColors.BLUE,
    "green": BuoyColors.GREEN,
    "red": BuoyColors.RED,
    "yellow": BuoyColors.YELLOW,
}

BUOY_DUPLICATE_THRESHOLD = 0.2

class Vision(Node):
    
    """
    This stripped down navigation logic relies on good odometry input data
    and properly tuned PID parameters.

    First, we detect all buoys and create an approximate 2d position
    by mapping the center of the bounding box to a position in 3d space
    with the camera's intrinsic parameters.

    Using the new 3d positions, we can take the midpoint between the
    buoys and calculate the slope of the line we want to travel along
    relative to the robot's heading, we can save the current odometry
    and use it as our fixed frame, then we can travel as straight as
    possible until we update the goal position to maintain reliable
    closed-loop control.

    INPUTS:
    - CV BBox Centers
    - Camera intrinsic parameters
    - Odometry (visual + IMU fused)
    
    PROCESSING:
    - Filter the outlier depths in the bounding box by using their z-scores
    - Take the mean of the points then map the the point using the depth
      and position with the intrinsic parameters.
    - Using the positions of all the buoys, determine the task and decide
      which ones are relevant
    - Calculate the slope of the path relative to the robot.
    """

    def __init__(self):
        super().__init__("vision")
        
        self._ai_out_sub = self.create_subscription(
            AiOutput, "/AiOutput", self._camera_callback, 10
        )
        
        self._depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self._depth_callback, 10
        )
        
        self._info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/depth/camera_info', self._camera_info_callback, 10
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
        
        self.map_publisher = self.create_publisher(BuoyMap, "/mhsboat_ctrl/map", 10)
        self.movement_publisher = self.create_publisher(BoatMovement, "/mhsboat_ctrl/movement", 10)
        
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None  # Camera intrinsics

        self.get_logger().info("Vision Node Initialized")
    
    def _camera_info_callback(self, msg) -> None:
        """ Get camera intrinsics from CameraInfo topic. """
        self.fx = msg.k[0]  # Focal length in x
        self.fy = msg.k[4]  # Focal length in y
        self.cx = msg.k[2]  # Optical center x
        self.cy = msg.k[5]  # Optical center y
        
    # TODO: Add better error handling and logging
    def _camera_callback(self, camera_data: AiOutput) -> None:
        self.get_logger().info("Received buoy detections")
        
        lefts = np.array([camera_data.lefts])
        tops = np.array([camera_data.tops])
        widths = np.array([camera_data.widths])
        heights = np.array([camera_data.heights])
        detected_objects = []
        
        # Calculate the centers of each buoy detection to transform point later
        centers = np.array([lefts + widths / 2, tops + heights / 2]).T
        items_to_remove = []
        

        for i in range(camera_data.num):
            x, y, z = centers[i]
            if camera_data.types[i].endswith("pole_buoy"):
                buoy_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                # type: ignore - this will always be either red or green
                detected_objects.append(PoleBuoy(x, y, z, buoy_color)) # type: ignore
            elif camera_data.types[i].endswith("buoy"):
                buoy_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                detected_objects.append(BallBuoy(x, y, z, buoy_color))
            elif camera_data.types[i].endswith("circle"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                detected_objects.append(
                    Shape(x, y, z, Shapes.CIRCLE, shape_color)
                )
            elif camera_data.types[i].endswith("triangle"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                detected_objects.append(
                    Shape(x, y, z, Shapes.TRIANGLE, shape_color)
                )
            elif camera_data.types[i].endswith("cross"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                detected_objects.append(Shape(x, y, z, Shapes.CROSS, shape_color))
            elif camera_data.types[i].endswith("square"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                detected_objects.append(
                    Shape(x, y, z, Shapes.SQUARE, shape_color)
                )
            elif camera_data.types[i].endswith("duck_image"):
                pass
            elif camera_data.types[i].endswith("racquet_ball"):
                pass
            
        for i in range(camera_data.num):
            # Search for duplicate buoys
            for j in range(camera_data.num)[::-1]:
                if detected_objects[j] in items_to_remove:
                    continue
                
                if i == j:
                    continue
                
                same = True

                # Check for different features
                if isinstance(detected_objects[i], Shape) and isinstance(detected_objects[j], Shape):
                    if detected_objects[i].shape != detected_objects[j].shape or detected_objects[i].color != detected_objects[j].color:
                        same = False
                elif isinstance(detected_objects[i], Buoy) and isinstance(detected_objects[j], Buoy):
                    if detected_objects[i].color != detected_objects[j].color:
                        same = False
                elif type(detected_objects[i]) != type(detected_objects[j]):
                    same = False

                if same:
                    self.get_logger().info(
                        f"Checking for duplicates: {detected_objects[i]} (original) and {detected_objects[j]} (this frame)"
                    )

                    distance = np.linalg.norm(detected_objects[i], detected_objects[j])

                    self.get_logger().info(f"Distance: {distance}")

                    if distance < BUOY_DUPLICATE_THRESHOLD:
                        self.get_logger().info("Found duplicate; merging objects")
                        # make position the average of the two
                        detected_objects[i] = (detected_objects[i] + detected_objects[j]) / 2

                        # remove the other object
                        items_to_remove.append(detected_objects[j])
            
        # Remove objects that are really close to each other
        detected_objects = [obj for obj in detected_objects if obj not in items_to_remove]

        msg = BuoyMap()
        x, y, z, types, colors = [], [], [], [], []
        for obj in detected_objects:
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
        
    def _depth_callback(self, msg: Image) -> None:
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def _odom_callback(self, odom_data: Odometry) -> None:
        self.get_logger().info("Received odom data")
        
        delta_trans = [
            odom_data.pose.pose.position.x - self.prev_x, 
            odom_data.pose.pose.position.y - self.prev_y,
            odom_data.pose.pose.position.z - self.prev_z
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

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        delta_euler = tf_transformations.euler_from_quaternion(delta_quat)
    
        self.get_logger().info(f"Translation: {delta_trans}")
        self.get_logger().info(f"Change in rotation: {delta_euler}")
        
        msg = BoatMovement()
        msg.dx = delta_trans[0]
        msg.dy = delta_trans[1]
        msg.dzr = delta_euler[2]
        
        self.movement_publisher.publish(msg)
        
    def pixel_to_3d(self, point: np.ndarray, msg: Image) -> np.ndarray:
        if self.fx is None:
            self.get_logger().warn("Camera info not received yet")
            return

        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        
        # Define the pixel coordinates (x, y)
        u, v = point[0], point[1]
    
        
        z = depth_image[v, u] / 1000.0  # Convert mm to meters
        
        # Compute 3D coordinates
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        
        return np.array([x, y, z])
        
def main(args=None):
    rclpy.init(args=args)
    vision = Vision()
    
    try:
        rclpy.spin(vision)
    except KeyboardInterrupt:
        vision.get_logger().info("Shutting down vision node")
    finally:
        vision.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()