import numpy as np
import math
from typing import Tuple, Union, Optional

from mhsboat_ctrl.vision_mhsboat_ctrl import VisionBoatController
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus, BuoyColors
from mhsboat_ctrl.course_objects import BallBuoy
from mhsboat_ctrl.utils.math_util import distance, midpoint, calculate_buoy_angle

FORWARD_VELOCITY = 1  # m/s
ANGULAR_VELOCITY = 0.5  # rad/s
DRIVE_DEVIATION = 3
END_DEVIATION = 6
ANGLE_DEV = 15
YELLOW_DIST_DEVIATION = 2

class TaskTwo(Task):
    status = TaskStatus.NOT_STARTED

    def __init__(self, boat_controller: VisionBoatController):
        self.boat_controller = boat_controller
        self.pid = self.boat_controller.pid
        self.buoy_map = self.boat_controller.buoy_map
        self._buoys = []
        self.red_ball_buoys = []
        self.green_ball_buoys = [] 

        self.x = 0.0
        self.y = 0.0
        self.zr = 0.0

        self.last_seen = -1

        self.prev_angle = 1e99

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
        if (self.boat_controller.through_gates) return (0.0, 0.0)

    def run(self) -> TaskCompletionStatus:
        self.boat_controller.get_logger().info("Running Task Two")

        completion_status = TaskCompletionStatus.NOT_STARTED

        while (
            not completion_status == TaskCompletionStatus.SUCCESS
            and not completion_status == TaskCompletionStatus.FAILURE
        ):
            self.green_ball_buoys = [buoy for buoy in self.buoy_map if(
                isinstance(buoy, BallBuoy) and buoy.color == BuoyColors.GREEN)]

            self.red_ball_buoys = [buoy for buoy in self.buoy_map if(
                isinstance(buoy, BallBuoy) and buoy.color == BuoyColors.RED)]

            self.yellow_ball_buoys = [buoy for buoy in self.buoy_map if(
                isinstance(buoy, BallBuoy) and buoy.color == BuoyColors.YELLOW)]

            closest_green_ball_buoy = min(self.green_ball_buoys, key=lambda x: distance(0, 0, x.x, x.y))
            closest_red_ball_buoy = min(self.red_ball_buoys, key=lambda x: distance(0, 0, x.x, x.y))
            closest_yellow_ball_buoy = min(self.yellow_ball_buoys, key=lambda x: distance(0, 0, x.x, x.y))
            
            dist_green = distance(closest_green_ball_buoy.x, closest_green_ball_buoy.y, closest_yellow_ball_buoy.x, closest_yellow_ball_buoy.y)
            dist_red = distance(closest_red_ball_buoy.x, closest_red_ball_buoy.y, closest_yellow_ball_buoy.x, closest_yellow_ball_buoy.y)
            
            if dist_green > dist_red and dist_red < YELLOW_DIST_DEVIATION:
                closest_green_ball_buoy = closest_yellow_ball_buoy
            elif dist_red > dist_green and dist_green < YELLOW_DIST_DEVIATION:
                closest_red_ball_buoy = closest_yellow_ball_buoy 
            
            mdpt = midpoint(closest_green_ball_buoy.x, closest_green_ball_buoy.y, closest_red_ball_buoy.x, closest_red_ball_buoy.y)
            
            self.angle = np.arctan2(mdpt[1], mdpt[0])
            
            if abs(self.angle - self.prev_angle) > ANGLE_DEV:
                self.prev_angle = self.angle

            self.x += self.boat_controller.dx
            self.y += self.boat_controller.dy
            self.zr += self.boat_controller.dzr

            angular_velocity = self.boat_controller.pid.pure_pursuit(self.prev_angle, (self.x, self.y), self.zr)
            angular_velocity *= ANGULAR_VELOCITY * np.pi / 180
            
            self.boat_controller.set_angular_velocity(angular_velocity)
            self.boat_controller.set_forward_velocity(FORWARD_VELOCITY)

            # Handle case where all buoys aren't detected
            if(len(self.red_ball_buoys) < 1 or len(self.green_ball_buoys) < 1):
                if(self.last_seen == -1):
                    self.last_seen = self.boat_controller.get_clock().now().nanoseconds / 1e9
                    continue
                elif(last_seen_deviation > DRIVE_DEVIATION):
                    self.set_forward_velocity(FORWARD_VELOCITY)
                    self.set_angular_velocity(0)
                elif(last_seen_deviation > END_DEVIATION):
                    self.boat_controller.through_gates = True
                    completion_status = TaskCompletionStatus.SUCCESS

        return completion_status

def main(controller: BoatController):
    controller.add_task(TaskTwo(controller))