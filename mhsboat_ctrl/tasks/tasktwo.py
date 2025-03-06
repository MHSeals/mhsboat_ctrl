import numpy as np

from typing import Tuple, Union, Optional

from mhsboat_ctrl.mhsboat_ctrl import BoatController
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus, BuoyColors
from mhsboat_ctrl.course_objects import BallBuoy
from mhsboat_ctrl.utils.math_util import distance, midpoint, calculate_buoy_angle

MIN_GREEN_BUOYS = 2
MIN_RED_BUOYS = 2
MIN_YELLOW_BUOYS = 1
class TaskTwo(Task):
    status = TaskStatus.NOT_STARTED

    def __init__(self, boat_controller: BoatController):
        self.boat_controller = boat_controller
        self._buoy_map = boat_controller.buoy_map
        self.red_ball_buoys = []
        self.green_ball_buoys = []
        self.yellow_ball_buoys = []

    def search(self) -> Optional[Tuple[float, float]]:
        """
        There are two primary ways of ensuring that
        task two is properly detected and completed
        at the optimal time (after task one)

        1. Detect nearby clusters of yellow buoys (no
           other tasks have yellow buoys)
        2. Using any yellow buoys detected immediately
           after completing task one and using the closest
           pair as the starting point (maybe require the
           detection of another pair following it)

        Ultimately, this function should return the midpoint
        between the first pair of buoys to give the boat a
        good starting point
        """
        self.yellow_ball_buoys = [
            buoy
            for buoy in self.buoy_map
            if isinstance(buoy, BallBuoy) and buoy.color == BuoyColors.YELLOW
        ]
        
        if len(yellow_ball_buoys) >= 

    def run(self) -> TaskCompletionStatus:
        self.boat_controller.get_logger().info("Running Task Two")

        completion_status = TaskCompletionStatus.NOT_STARTED

        self.red_ball_buoys = [
            buoy
            for buoy in self.boat_controller.buoy_map
            if isinstance(buoy, BallBuoy) and buoy.color == BuoyColors.RED
        ]
        self.green_ball_buoys = [
            buoy
            for buoy in self.boat_controller.buoy_map
            if isinstance(buoy, BallBuoy) and buoy.color == BuoyColors.GREEN
        ]
        self.yellow_ball_buoys = [
            buoy
            for buoy in self.boat_controller.buoy_map
            if isinstance(buoy, BallBuoy) and buoy.color == BuoyColors.YELLOW
        ]

        while (
            not completion_status == TaskCompletionStatus.SUCCESS
            or not completion_status == TaskCompletionStatus.FAILURE
        ):
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

        return completion_status


def main(controller: BoatController):
    controller.add_task(TaskTwo(controller))


"""
Steps to solve:

1-Checking and starting:
    search for yellow bouys
    once found run logic
    
2-Running logic
    find all buoy pairs (red and green)
    using the same logic as task one
    (finding the two closest red and green buoys)
    
    after finding all the pairs of buoys,
    pair each yellow buoy to the closest red
    or green buoy
    
    
3- Check for completion
    check if past the closest bouy
    if not within certain range, repeat logic
"""
