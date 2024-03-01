import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn

UINT16_MAX = 65535


class Waterpump(Node):
    def __init__(self):
        super().__init__("waterpump")
        self.rc_publisher = self.create_publisher(
            OverrideRCIn, "/mavros/rc/override", 10
        )

    def waterpump(self):
        self.rcin = OverrideRCIn()
        self.rcin.channels = [
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            2006,
            UINT16_MAX - 1,
            UINT16_MAX - 1,
            UINT16_MAX - 1,
            UINT16_MAX - 1,
            UINT16_MAX - 1,
            UINT16_MAX - 1,
            UINT16_MAX - 1,
        ]
        self.rc_publisher.publish(self.rcin)


def main(args=None):
    rclpy.init(args=args)
    waterpump = Waterpump()
    while True:
        waterpump.waterpump()

    waterpump.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
