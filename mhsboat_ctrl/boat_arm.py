import sys
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool

class ArmingService(Node):
    def __init__(self):
        super().__init__("arming_service")
        self.arm_service = self.create_client(CommandBool, "/mavros/cmd/arming")
        
    def arm_boat(self, mode: bool):
        request = CommandBool.Request()
        request.value = mode
        self.future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    service_caller = ArmingService()
    service_caller.arm_boat(mode=bool(eval(sys.argv[1])))
    service_caller.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
