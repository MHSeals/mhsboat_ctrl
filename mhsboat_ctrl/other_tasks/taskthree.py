import numpy as np
import math
from typing import Tuple, Union, Optional

from mhsboat_ctrl.mhsboat_ctrl import BoatController
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus, BuoyColors
from mhsboat_ctrl.course_objects import BallBuoy
from mhsboat_ctrl.utils.math_util import distance, midpoint, calculate_buoy_angle

MIN_GREEN_BUOYS = 2
MIN_RED_BUOYS = 2
MIN_YELLOW_BUOYS = 1

# TODO: What is a good value for these?
ANGLE_ALLOWED_DEVIATION = 15  # degrees
DIST_ALLOWED_DEVIATION = 1  # meters
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

class TaskThree(Task):
    status = TaskStatus.NOT_STARTED

    def __init__(self, boat_controller: BoatController):
        self.boat_controller = boat_controller
        self._buoy_map = boat_controller.buoy_map
        self.red_ball_buoys = []
        self.green_ball_buoys = []
        self.yellow_ball_buoys = []

    def search(self) -> Optional[Tuple[float, float]]:
        """
        TODO: 
        """
        images = [
            buoy
            for buoy in self.boat_controller.buoy_map
            if isinstance(buoy, BallBuoy) and buoy.color == BuoyColors.RED
            ]

    def run(self) -> TaskCompletionStatus:
        self.boat_controller.get_logger().info("Running Task Two")

        completion_status = TaskCompletionStatus.NOT_STARTED

        while (
            not completion_status == TaskCompletionStatus.SUCCESS
            or not completion_status == TaskCompletionStatus.FAILURE
        ):
            red_array = [
            buoy
            for buoy in self.boat_controller.buoy_map
            if isinstance(buoy, BallBuoy) and buoy.color == BuoyColors.RED
            ]
            green_array = [
                buoy
                for buoy in self.boat_controller.buoy_map
                if isinstance(buoy, BallBuoy) and buoy.color == BuoyColors.GREEN
            ]
            yellow_array = [
                buoy
                for buoy in self.boat_controller.buoy_map
                if isinstance(buoy, BallBuoy) and buoy.color == BuoyColors.YELLOW
            ]
            self.red_ball_buoys: list[BallBuoy] = []
            self.green_ball_buoys: list[BallBuoy] = []
            self.yellow_ball_buoys: list[BallBuoy] = []
            self.midpoints = []

            # sorts all ball buoys by distance, ball buoys separated by colors can be seen as pairs by their index
            # closest buoys are infered to be index 0
            minimum_red = 1e99
            for i in red_array:
                _distance = math.sqrt(i.x ** 2 + i.y ** 2 + i.z ** 2)
                if _distance < minimum_red:
                    minimum_red = _distance
                    self.red_ball_buoys.append(i)
            
            minimum_green = 1e99
            for i in green_array:
                _distance = math.sqrt(i.x ** 2 + i.y ** 2 + i.z ** 2)
                if _distance < minimum_green:
                    minimum_green = _distance
                    self.green_ball_buoys.append(i)
            
            minimum_yellow = 1e99
            for i in yellow_array:
                _distance = math.sqrt(i.x ** 2 + i.y ** 2 + i.z ** 2)
                if _distance < minimum_yellow:
                    minimum_yellow = _distance
                    self.yellow_ball_buoys.append(i)
            
            last_seen_deviation = self.boat_controller.get_clock().now().nanoseconds / 1e9 - self.last_seen

            # Handle case where all buoys aren't detected
            if(len(self.red_ball_buoys) < 1 or len(self.green_ball_buoys) < 1):
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
            
            # when yellow ball buoys are detected
            # NOTE: even though the first yellow ball buoy detected may not be matched with necessarily the first pair,
            #       we are only using it to compare the distances between red and green anyway, and therefore it doesn't matter.
            if len(self.yellow_ball_buoys) >= 0:
                for i in range(0, len(self.yellow_ball_buoys)):
                    if distance(self.red_ball_buoys[i].x, self.red_ball_buoys[i].y, self.yellow_ball_buoys[i].x, self.yellow_ball_buoys[i].y) < distance(self.green_ball_buoys[i].x, self.green_ball_buoys[1].y, self.yellow_ball_buoys[i].x, self.yellow_ball_buoys[i].y):
                        self.midpoints.append(midpoint(self.red_ball_buoys[i].x, self.red_ball_buoys[i].y, self.yellow_ball_buoys[i].x, self.yellow_ball_buoys[i].y))
                    elif distance(self.red_ball_buoys[i].x, self.red_ball_buoys[i].y, self.yellow_ball_buoys[i].x, self.yellow_ball_buoys[i].y) > distance(self.green_ball_buoys[i].x, self.green_ball_buoys[1].y, self.yellow_ball_buoys[i].x, self.yellow_ball_buoys[i].y):
                        self.midpoints.append(midpoint(self.green_ball_buoys[i].x, self.green_ball_buoys[1].y, self.yellow_ball_buoys[i].x, self.yellow_ball_buoys[i].y))
            
            # alec's boat control.
            for _midpoint in self.midpoints:
                if abs(_midpoint[0]) > ALLOWED_TURN_DEVIATION:
                    self.boat_controller.set_angular_velocity(
                        -ANGULAR_VELOCITY if _midpoint[0] > 0 else ANGULAR_VELOCITY
                    )
                else:
                    self.boat_controller.set_angular_velocity(0)
            """
            TODO:
                - Implement the buoy pairing algorithm
                - Test the algorithm to see if the boat ever gets stuck (when it can no longer see buoys)
                - Substitute the yellow buoys for the nearest buoy and ensure it creates a new pair
                - Use the closest buoy pair's midpoint as the next goal

            NOTE: It probably makes sense to organize things like boat velocity
                  into a parameter file, so we don't have to change things individually in
                  code, and there is an easier way to see our constants.
            """
            if (
                self.green_ball_buoys[0].y < 0
                and self.green_ball_buoys[1].y < 0
                and self.red_ball_buoys[0].y < 0
                and self.red_ball_buoys[1].y < 0
            ):
                completion_status = TaskCompletionStatus.SUCCESS

        return completion_status


def main(controller: BoatController):
    controller.add_task(TaskTwo(controller))