import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Vector3
import curses
from curses import wrapper
import time


class MavRosController(Node):
    def __init__(self):
        super().__init__("mavros_controller")
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10
        )

    def controller(self, stdscr):
        twist = TwistStamped()

        # +x: forward
        # -x: backward
        # +z: left
        # -z: right

        # Create static info menu
        stdscr.clear()
        stdscr.addstr(0, 0, "-----------------------------")
        stdscr.addstr(1, 0, "Moving around:")
        stdscr.addstr(2, 0, "Control boat")
        stdscr.addstr(3, 0, "       w      ")
        stdscr.addstr(4, 0, "   a   s   d  ")
        stdscr.addstr(5, 0, "       x      ")
        stdscr.addstr(6, 0, " ")
        stdscr.addstr(7, 0, "w/x: increase linear velocity")
        stdscr.addstr(8, 0, "a/d: increase angular velocity")
        stdscr.addstr(9, 0, "s: stop")
        stdscr.addstr(10, 0, " ")
        stdscr.addstr(11, 0, "Press Ctrl C to exit")
        stdscr.move(12, 0)
        stdscr.refresh()

        # Create window with updating movement info
        output_win = curses.newwin(0, 0, 12, 0)

        while True:
            try:
                input = stdscr.getkey()

                if input == "w":
                    twist.twist.linear.x += 0.01
                elif input == "a":
                    twist.twist.angular.z += 0.01
                elif input == "s":
                    twist.twist.linear.x = 0.0
                    twist.twist.angular.z = 0.0
                    twist.twist.linear.y = 0.0
                elif input == "d":
                    twist.twist.angular.z -= 0.01
                elif input == "x":
                    # twist.twist.linear.x -= 1
                    twist.twist.linear.y -= 0.01

                output_win.clear()
                output_win.addstr(
                    f"currently:   linear x: {twist.twist.linear.x}m/s  angular z: {twist.twist.angular.z}m/s linear y: {twist.twist.linear.y}m/s"
                )
                output_win.refresh()

            except KeyboardInterrupt:
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = 0.0
                break

            self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    mavros_controller = MavRosController()
    wrapper(mavros_controller.controller)
    mavros_controller.destroy_node()


if __name__ == "__main__":
    main()
