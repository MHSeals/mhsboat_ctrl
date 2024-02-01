import time
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSLivelinessPolicy,
)

from mavros_msgs.srv import CommandHome
from mavros_msgs.msg import WaypointReached
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from geographic_msgs.msg import GeoPoseStamped
from boat_interfaces.msg import BuoyCoordinates

from boat_ctrl.boat_arm import ArmingService
from boat_ctrl.boat_mode import ModeService
from boat_ctrl.boat_controller import BoatController


class TaskoneWaypoint(Node):
    def __init__(self):
        super().__init__("taskone_waypoint_node")

        self.boat_controller = BoatController()
        # self.arm_boat = ArmingService()
        # self.arm_boat.arm_boat(True)

        # self.boat_mode = ModeService()
        # self.boat_mode.set_mode("GUIDED")

        self.qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        self.current_lat = 0
        self.current_long = 0

        self.current_waypoint_goal = []
        self.gates = []
        self.gate_found = False

        self.set_home_service = self.create_client(CommandHome, "/mavros/cmd/set_home")
        self.waypoint_publisher = self.create_publisher(
            GeoPoseStamped, "/mavros/setpoint_position/global", 10
        )
        self.create_subscription(
            BuoyCoordinates,
            "/taskone/goal_waypoint",
            self.taskone_waypoints_callback,
            10,
        )
        self.pole_subscriber = self.create_subscription(
            String, "/taskone/poles", self.pole_callback, 10
        )
        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.gps_callback,
            qos_profile=self.qos_profile,
        )
        self.create_subscription(
            WaypointReached,
            "/mavros/mission/reached",
            self.waypoint_reached_callback,
            10,
        )

    def taskone_waypoints_callback(self, data: BuoyCoordinates):
        lat = data.latitudes[0]
        long = data.longitudes[0]
        buoy_type = data.types[0]

        if buoy_type == "waypoint":
            self.current_waypoint_goal = [lat, long]

    def gps_callback(self, data: NavSatFix):
        self.current_lat = data.latitude
        self.current_long = data.longitude

    def waypoint_reached_callback(self, data: WaypointReached):
        # if self.gate_found:
        #     self.gates.append(1)
        #     self.gate_found = False
        #     print("gate reached")
        #     self.boat_controller.set_forward_velocity(0.0)
        #     self.boat_controller.set_angular_velocity(0.0)
        #     self.boat_controller.cmd_vel_publisher.publish(self.boat_controller.cmd_vel)
        #     time.sleep(5)
        #     self.set_waypoint(
        #         self.current_waypoint_goal[0], self.current_waypoint_goal[1]
        #     )
        # else:
        #     self.boat_controller.set_forward_velocity(0.0)
        #     self.boat_controller.set_angular_velocity(0.0)
        #     self.boat_controller.cmd_vel_publisher.publish(self.boat_controller.cmd_vel)
        #     time.sleep(5)
        #     self.set_waypoint(
        #         self.current_waypoint_goal[0], self.current_waypoint_goal[1]
        #     )
        reached = data.wp_seq

        # TODO
        """
        Topics to investigate:
        /mavros/mission/reached
        /mavros/mission/waypoints
        /mavros/global_position/set_gp_origin
        /mavros/geofence/fences
        """

    def set_home(self):
        request = CommandHome.Request()
        request.latitude = float(self.current_lat)
        request.longitude = float(self.current_long)
        self.future = self.set_home_service.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def set_waypoint(self, lat: float, long: float):
        msg = GeoPoseStamped()
        msg.pose.position.latitude = lat
        msg.pose.position.longitude = long
        self.waypoint_publisher.publish(msg)

    def pole_callback(self, data: String):
        if not self.start:
            input("Start? Press any key to start!")
            self.start = True

        poles = json.loads(data.data)

        # if self.gates >= 2:
        #     exit(0)

        # Screen middle is the x size of the camera output divided by 2
        SCREEN_MIDDLE = 640 / 2

        # If it doesn't detect more than one pole, rotate in place to try to find buoys
        if len(poles) < 2:
            print("Only one or no pole found, rotating to try to locate buoys...")
            self.boat_controller.set_forward_velocity(0.0)
            self.boat_controller.set_angular_velocity(0.1)
            self.boat_controller.cmd_vel_publisher.publish(self.boat_controller.cmd_vel)
            return

        # Sort the list of poles by area in descending order
        sort_by_area = sorted(poles, key=lambda x: x["area"], reverse=True)

        # Get the two largest areas (closest poles to boat)
        poles = [pole for pole in sort_by_area[:2]]
        print("sorted poles")
        print(poles)

        if poles[0]["name"] == poles[1]["name"]:
            print("Two poles of same color found, keep going forward")
            return

        # Set left pole and right pole based on x values
        if poles[0]["x_left"] > poles[1]["x_left"]:
            left_buoy = poles[1]
            right_buoy = poles[0]

            if (
                left_buoy["name"] == "red_buoy"
                and right_buoy["name"] == "green_buoy"
                and len(self.gates) < 2
            ):
                print("gate found")
                self.gate_found = True
                # self.set_waypoint(
                #     self.current_waypoint_goal[0], self.current_waypoint_goal[1]
                # )
        elif poles[0]["x_left"] < poles[1]["x_left"]:
            left_buoy = poles[0]
            right_buoy = poles[1]

            if (
                left_buoy["name"] == "red_buoy"
                and right_buoy["name"] == "green_buoy"
                and len(self.gates) < 2
            ):
                print("gate found")
                self.gate_found = True
                # self.set_waypoint(
                #     self.current_waypoint_goal[0], self.current_waypoint_goal[1]
                # )

        # Calculate middle of two poles
        buoy_middle = left_buoy["x_right"] + (
            (right_buoy["x_left"] - left_buoy["x_right"]) / 2
        )

        # If both poles are on left side of boat, angle towards left
        # Vice versa for poles on right side of screen

        # print(buoy_middle)

        if (
            left_buoy["x_right"] < SCREEN_MIDDLE
            and right_buoy["x_left"] < SCREEN_MIDDLE
        ):
            # Turn left to orientate boat between two buoys
            print("TWO BUOYS ON LEFT SIDE OF SCREEN")
            self.boat_controller.set_angular_velocity(0.01)
        elif (
            left_buoy["x_right"] > SCREEN_MIDDLE
            and right_buoy["x_left"] > SCREEN_MIDDLE
        ):
            # Turn right to orientate boat between two buoys
            print("TWO BUOYS ON RIGHT SIDE OF SCREEN")
            self.boat_controller.set_angular_velocity(-0.01)
        elif buoy_middle - SCREEN_MIDDLE > 5:
            print("TOO LEFT")
            self.boat_controller.set_angular_velocity(-0.01)
        elif SCREEN_MIDDLE - buoy_middle > 5:
            print("TOO RIGHT")
            self.boat_controller.set_angular_velocity(0.1)

        elif SCREEN_MIDDLE - buoy_middle < 5 and buoy_middle - SCREEN_MIDDLE < 5:
            print("BOAT IS CENTERED")
            # print("MOVING FORWARD")
            # self.boat_controller.set_linear_velocity(10.0)
            if self.gate_found:
                self.set_waypoint(
                    self.current_waypoint_goal[0], self.current_waypoint_goal[1]
                )

        # Publish velocity
        self.boat_controller.cmd_vel_publisher.publish(self.boat_controller.cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    taskone = TaskoneWaypoint()

    taskone.set_home()
    # time.sleep(5)
    # taskone.set_waypoint(-35.3632556, 149.1652358)
    # taskone.set_waypoint(-35.3632694, 149.1652390)
    # taskone.set_waypoint(-35.3632680, 149.1652475)
    rclpy.spin(taskone)

    taskone.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
