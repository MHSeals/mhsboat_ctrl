import rclpy
from rclpy.node import Node
from boat_interfaces.msg import AiOutput
from sensor_msgs.msg import PointCloud2, Imu
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy

class Sensors(Node):
    def __init__(self):
        super().__init__('sensors')

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

        self.get_logger().info("Sensors Node Initialized")

    def _camera_callback(self, msg):
        pass

    def _lidar_callback(self, msg):
        pass

    def _imu_callback(self, msg):
        pass

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