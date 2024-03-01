import rclpy
from mavros_msgs.msg import RCIn


class RCChannelPublisher:
    def __init__(self, node_name, topic_name, channel_index, signal_value, publish_rate):
        self.node = rclpy.create_node(node_name)
        self.publisher = self.node.create_publisher(RCIn, topic_name, 10)
        self.channel_index = channel_index
        self.signal_value = signal_value
        self.rate = self.node.create_rate(publish_rate)

    def send_signal(self):
        message = RCIn()
        message.channels[self.channel_index] = self.signal_value
        self.publisher.publish(message)
        self.rate.sleep()

    def run(self):
        while rclpy.ok():
            self.send_signal()

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdow()


def main():
    rclpy.init()
    publisher = RCChannelPublisher("rc_signal_sender", "mavros/rc/in", 9, 2006, 10)
    publisher.run()
    publisher.shutdown()


if __name__ == "__main__":
    main()
