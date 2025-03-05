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

from boat_interfaces.msg import AiOutput, BuoyMap
from sensor_msgs.msg import Odometry, CameraInfo, Image 
import numpy as np
from typing import List, Tuple, Dict

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
    This stripped down navigation relies on good odometry input data
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
        super().__init__("pure_vision")
        
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
        
        self._process_timer = self.create_timer(0.1, self._safe_process_sensor_data)
        
        self.map_publisher = self.create_publisher(BuoyMap, "/mhsboat_ctrl/map", 10)
        self.map: List[CourseObject] = []
        
        self.previous_odom_data = None
        
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None  # Camera intrinsics

        self.get_logger().info("Sensors Node Initialized")
    
    def _camera_info_callback(self, msg) -> None:
        """ Get camera intrinsics from CameraInfo topic. """
        self.fx = msg.k[0]  # Focal length in x
        self.fy = msg.k[4]  # Focal length in y
        self.cx = msg.k[2]  # Optical center x
        self.cy = msg.k[5]  # Optical center y
        
    # TODO: Add better error handling and logging
    def _camera_callback(self, cam_data: AiOutput) -> None:
        self.get_logger().info("Received buoy detections")
        
        lefts = np.array([cam_data.lefts])
        tops = np.array([cam_data.tops])
        widths = np.array([cam_data.widths])
        heights = np.array([cam_data.heights])
        
        # Calculate the centers of each buoy detection to transform point later
        centers = np.array([lefts + widths / 2, tops + heights / 2]).T
        
        for i in range(cam_data.num):
            center1 == centers[i]
            self.pixel_to_3d(center1)

            # Search for duplicate buoys
            for j in range(cam_data.num)[::-1]:
                center2 == centers[j]

                if center2 in items_to_remove:
                    continue
                
                if center1 == center2:
                    continue
                
                same = True

                # Check for different features
                if isinstance(center1, Shape) and isinstance(center2, Shape):
                    if center1.shape != center2.shape or center1.color != center2.color:
                        same = False
                elif isinstance(center1, Buoy) and isinstance(center2, Buoy):
                    if center1.color != center2.color:
                        same = False
                elif type(center1) != type(center2):
                    same = False

                if same:
                    self.get_logger().info(
                        f"Checking for duplicates: {center1} (original) and {center2} (this frame)"
                    )

                    distance = np.linalg.norm(center1, center2)

                    self.get_logger().info(f"Distance: {distance}")

                    if distance < BUOY_DUPLICATE_THRESHOLD:
                        self.get_logger().info("Found duplicate; merging objects")
                        # make position the average of the two
                        center1 = (center1 + center2) / 2

                        # remove the other object
                        items_to_remove.append(center2)
            
        # Remove objects that are really close to each other
        self.map = [obj for obj in self.map if obj not in items_to_remove]

        cam_data = BuoyMap()
        x, y, z, types, colors = [], [], [], [], []
        
    def _depth_callback(self, msg: Image) -> None:
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def _odom_callback(self, msg: Odometry) -> None:
        self.get_logger().info("Received odom data")
        
    def pixel_to_3d(self, point: np.ndarray, width: int, height: int) -> np.ndarray:
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
        sensors.get_logger().info("Shutting down vision node")
    finally:
        sensors.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()