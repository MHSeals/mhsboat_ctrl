import numpy as np
from typing import Tuple, Optional

from mhsboat_ctrl.vision_mhsboat_ctrl import VisionBoatController
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus, BuoyColors
from mhsboat_ctrl.course_objects import PoleBuoy
from mhsboat_ctrl.utils.math_util import distance, midpoint, calculate_buoy_angle

# TODO: What is a good value for these?
ANGLE_ALLOWED_DEVIATION = 30  # degrees
DIST_ALLOWED_DEVIATION = 2  # meters
ALLOWED_TURN_DEVIATION = 10  # degrees
FORWARD_VELOCITY = 1  # m/s
ANGULAR_VELOCITY = 0.5  # rad/s
BOAT_X = 0 # meters
BOAT_Y = 0 # meters
MIN_PAIR_DIST = 1.8288 # meters
MAX_PAIR_DIST = 3.048 # meters
MIN_GATE_DIST = 7.62 # meters
MAX_GATE_DIST = 30.48 # meters
LAST_SEEN_BACKTRACK_DEVIATION = 1 # seconds
LAST_SEEN_STOP_DEVIATION = 3 # seconds
LAST_SEEN_FAILURE_DEVIATION = 8 # seconds

class TaskOne(Task):
    status = TaskStatus.NOT_STARTED

    def __init__(self, boat_controller: VisionBoatController):
        self.boat_controller = boat_controller
        self.pid = self.boat_controller.pid
        self.buoy_map = self.boat_controller.buoy_map
        self._buoys = []
        self.red_pole_buoys = []
        self.green_pole_buoys = []

        self.x = 0.0
        self.y = 0.0
        self.zr = 0.0

    def search(self) -> Optional[Tuple[float, float]]:
        # self.boat_controller.get_logger().info("Searching for TaskOne")
        """
        Search for the following pattern where R is a red pole buoy, and G is a green pole buoy
        There could be any number of other objects between the buoys, and the rectangle could be rotated in any direction
        R   G
        
        
        R   G
        """
        

        self.red_pole_buoys = [
            buoy
            for buoy in self.buoy_map
            if isinstance(buoy, PoleBuoy) and buoy.color == BuoyColors.RED
        ]
        self.green_pole_buoys = [
            buoy
            for buoy in self.buoy_map
            if isinstance(buoy, PoleBuoy) and buoy.color == BuoyColors.GREEN
        ]
        
        # self.boat_controller.get_logger().info(f"RPB: {self.red_pole_buoys}, GPB: {self.green_pole_buoys}")

        if len(self.red_pole_buoys) < 2 or len(self.green_pole_buoys) < 2:
            return None

        for red1 in self.red_pole_buoys:
            for red2 in self.red_pole_buoys:
                if red1 == red2:
                    continue

                for green1 in self.green_pole_buoys:
                    for green2 in self.green_pole_buoys:
                        if green1 == green2:
                            continue

                        """
                        
                        Desired buoy pattern

                        R1   G1


                        R2   G2
                        
                        Pairs
                            - R2, R1, G1
                            - R1, G1, G2
                            - G1, G2, R2
                            - G2, R2, R1

                        """

                        # Check angles on the 4 corners
                        angle_R1 = calculate_buoy_angle(green1, red1, red2)
                        angle_G1 = calculate_buoy_angle(red1, green1, green2)
                        angle_R2 = calculate_buoy_angle(green2, red2, red1)
                        angle_G2 = calculate_buoy_angle(red2, green2, green1)

                        self.boat_controller.get_logger().info(
                            f"Angles: {angle_R1}, {angle_G1}, {angle_R2}, {angle_G2}"
                        )

                        if abs(angle_R1 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue
                        if abs(angle_G1 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue
                        if abs(angle_R2 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue
                        if abs(angle_G2 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue

                        # Get distance between same colored buoys (calculated in meters)
                        dist_g = distance(green1.x, green1.y, green2.x, green2.y)
                        if (
                            dist_g > MAX_GATE_DIST + DIST_ALLOWED_DEVIATION
                            and dist_g < MIN_GATE_DIST - DIST_ALLOWED_DEVIATION
                        ):
                            continue
                        dist_r = distance(red1.x, red1.y, red2.x, red2.y)
                        if (
                            dist_r > MAX_GATE_DIST + DIST_ALLOWED_DEVIATION
                            and dist_r < MIN_GATE_DIST - DIST_ALLOWED_DEVIATION
                        ):
                            continue

                        # Get distance between buoy pairs (calculated in meters)
                        dist_pair = distance(green1.x, green1.y, red1.x, red1.y)
                        if (
                            dist_pair > MAX_PAIR_DIST + DIST_ALLOWED_DEVIATION
                            and dist_pair < MIN_PAIR_DIST - DIST_ALLOWED_DEVIATION
                        ):
                            continue
                        dist_pair2 = distance(green2.x, green2.y, red2.x, red2.y)
                        if (
                            dist_pair2 > MAX_PAIR_DIST + DIST_ALLOWED_DEVIATION
                            and dist_pair2 < MIN_PAIR_DIST - DIST_ALLOWED_DEVIATION
                        ):
                            continue

                        self.boat_controller.get_logger().info(
                            f"Distances: {dist_g}, {dist_r}, {dist_pair}, {dist_pair2}"
                        )

                        # if everything is cool and right, we will continue with the run function

                        # TODO: check that no other buoys are inside the rectangle

                        self._buoys = [red1, green1, red2, green2]

                        return (
                            int((red1.x + green1.x + red2.x + green2.x) / 4),
                            int((red1.y + green1.y + red2.y + green2.y) / 4),
                        )

    def run(self) -> TaskCompletionStatus:
        """
        Executes Task One by navigating the boat between the closest red and green buoys.
        The method performs the following steps:
        1. Logs the start of Task One.
        2. Sets the boat's forward velocity.
        3. Determines the closest red buoy from the two available red buoys.
        4. Determines the closest green buoy from the two available green buoys.
        5. Calculates the midpoint between the closest red and green buoys.
        6. Adjusts the boat's angular velocity based on the deviation of the midpoint from the allowed turn deviation.
        7. Returns the task completion status as SUCCESS.
        Returns:
            TaskCompletionStatus: The status of the task completion, always SUCCESS.
        """
        self.boat_controller.get_logger().info("Running Task One")

        completion_status = TaskCompletionStatus.NOT_STARTED

        self.last_seen = -1 # Only set to a positive value when we actually don't see buoys

        while (
            not completion_status == TaskCompletionStatus.SUCCESS
            and not completion_status == TaskCompletionStatus.FAILURE
        ):
            # TODO: Finish this part of the code
            # self.x += self.boat_controller.dx
            # self.y += self.boat_controller.dy
            # self.zr += self.boat_controller.dzr

            # angular_velocity = self.boat_controller.pid.pure_pursuit(np.pi / 3, (self.x, self.y), self.zr)
            # angular_velocity *= 5 * np.pi / 180
            # 
            # self.boat_controller.set_angular_velocity(angular_velocity)
            # self.boat_controller.set_forward_velocity(FORWARD_VELOCITY)

            last_seen_deviation = self.boat_controller.get_clock().now().nanoseconds / 1e9 - self.last_seen

            # Handle case where all buoys aren't detected
            if(len(self.red_pole_buoys) < 1 or len(self.green_pole_buoys) < 1):
                if(self.last_seen == -1):
                    self.last_seen = self.boat_controller.get_clock().now().nanoseconds / 1e9
                    continue
                elif(last_seen_deviation > LAST_SEEN_BACKTRACK_DEVIATION):
                    self.boat_controller.set_forward_velocity(-FORWARD_VELOCITY / 2) # backtrack slowly until buoys are seen
                    continue
                elif(last_seen_deviation > LAST_SEEN_STOP_DEVIATION):
                    self.boat_controller.set_forward_velocity(0) # stop the boat
                    continue
                elif(last_seen_deviation > LAST_SEEN_FAILURE_DEVIATION):
                    completion_status = TaskCompletionStatus.FAILURE
                    continue
            
            if(len(self.red_pole_buoys) > 2 or len(self.green_pole_buoys) > 2):
                ... # TODO: Handle when more of 2 of each buoy are detected

            # if first red buoy is closer, set that to closest_red_buoy. if second buoy is closer, set that to close_red_buoy instead.
            if distance(
                self.red_pole_buoys[0].x, self.red_pole_buoys[0].y, BOAT_X, BOAT_Y
            ) < distance(
                self.red_pole_buoys[1].x, self.red_pole_buoys[1].y, BOAT_X, BOAT_Y
            ):
                closest_red_buoy = self.red_pole_buoys[0]
            else:
                closest_red_buoy = self.red_pole_buoys[1]

            #
            if distance(
                self.green_pole_buoys[0].x, self.green_pole_buoys[0].y, BOAT_X, BOAT_Y
            ) < distance(
                self.green_pole_buoys[1].x, self.green_pole_buoys[1].y, BOAT_X, BOAT_Y
            ):
                closest_green_buoy = self.green_pole_buoys[0]
            else:
                closest_green_buoy = self.green_pole_buoys[1]

            # Get the midpoint
            mid = midpoint(
                closest_red_buoy.x,
                closest_red_buoy.y,
                closest_green_buoy.x,
                closest_green_buoy.y,
            )
            #
            if abs(mid[0]) > ALLOWED_TURN_DEVIATION:
                self.boat_controller.set_angular_velocity(
                    -ANGULAR_VELOCITY if mid[0] > 0 else ANGULAR_VELOCITY
                )
            else:
                self.boat_controller.set_angular_velocity(0)

            if (
                self.green_pole_buoys[0].y < 0
                and self.green_pole_buoys[1].y < 0
                and self.red_pole_buoys[0].y < 0
                and self.red_pole_buoys[1].y < 0
            ):
                completion_status = TaskCompletionStatus.SUCCESS

        return completion_status


def main(controller: VisionBoatController):
    controller.add_task(TaskOne(controller))