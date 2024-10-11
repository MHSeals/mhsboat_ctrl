import rclpy
from rclpy.node import Node
from boat_interfaces.msg import AiOutput
from sensor_msgs.msg import PointCloud2, Imu
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy

from mhsboat_ctrl.course_objects import CourseObject, Shape, Buoy, PoleBuoy, BallBuoy
from mhsboat_ctrl.enums import BuoyColors, Shapes


class Sensors(Node):
    def __init__(self):
        super().__init__('sensors')

        self.detected_objects: list[CourseObject] = []

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
    
    def _process_sensor_data(self):
        if len(self._camera_cache) == 0 or len(self._lidar_cache) == 0 or len(self._imu_cache) == 0:
            self.get_logger().info("Not enough data to process; waiting for more data")
            return
        
        # find the data that has the smallest time difference
        # and process it
        camera_data = min(self._camera_cache, key=lambda x: abs(x[1] - self.get_clock().now().nanoseconds))
        lidar_data = min(self._lidar_cache, key=lambda x: abs(x[1] - self.get_clock().now().nanoseconds))
        imu_data = min(self._imu_cache, key=lambda x: abs(x[1] - self.get_clock().now().nanoseconds))

        # TODO: locate buoys

    """ 
    TODO:
        1. run slam
        2. run clustering
        3. wait for ai output
        4. match ai output with clustering
        5. mark detected buoys on the map
        6. expose the map of the detected buoys to the task code
        7. profit

    eventually:
        - simulated map to test the task code
    """