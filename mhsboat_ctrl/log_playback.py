import pickle
import rclpy
from rclpy.node import Node
# Import the message type for each log file
from sensor_msgs.msg import PointCloud2

class log_playback(Node):
    def __init__(self):
        super().__init__("log_playback")
        self._logger = self.get_logger()
        self._logger.info("log playback node started")
        
        # Change the message type and topic as needed for each type of log file
        self.publisher = self.create_publisher(PointCloud2, '/velodyne/points', 10)

        # Log file path
        self.file_path = "/root/roboboat_ws/src/mhs_roboboat/text/velodyne/velodyne.txt"

        with open(self.file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        # Deserialize the object
                        obj = pickle.loads(eval(line))
                        # Publish the object
                        self.publisher.publish(obj)
                        
                    except Exception as e:
                        print(f"Error parsing line: {line}")
                        print(f"Error message: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    publisher = log_playback()
    rclpy.spin(publisher)


if __name__ == "__main__":
    main()
