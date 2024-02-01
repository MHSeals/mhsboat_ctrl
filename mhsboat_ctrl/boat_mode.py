import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode


class ModeService(Node):
    def __init__(self):
        super().__init__("mode_service")
        self.mode_service = self.create_client(SetMode, "/mavros/set_mode")

    def set_mode(self, mode):
        """Mode options: MANUAL GUIDED HOLD AUTO"""
        request = SetMode.Request()
        request.custom_mode = mode
        self.future = self.mode_service.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    service_caller = ModeService()
    mode = input("Available modes: GUIDED HOLD MANUAL\nEnter one: ")
    service_caller.set_mode(mode=mode)
    service_caller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
