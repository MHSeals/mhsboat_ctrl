import numpy as np
from typing import Tuple, Union, Optional

from mhsboat_ctrl.mhsboat_ctrl import BoatController
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus, BuoyColors
from mhsboat_ctrl.course_objects import PoleBuoy
from mhsboat_ctrl.sensors import Sensors, SensorsSimulated
from mhsboat_ctrl.utils.math_util import distance, midpoint, calculate_buoy_angle

# TODO: What is a good value for these?
ANGLE_ALLOWED_DEVIATION = 15 # degrees
DIST_ALLOWED_DEVIATION = 1 # meters
ALLOWED_TURN_DEVIATION = 10 # degrees
FORWARD_VELOCITY = 1 # m/s
ANGULAR_VELOCITY = 0.5 # rad/s
BOAT_X = 0
BOAT_Y = 0


class TaskOne(Task):
    status = TaskStatus.NOT_STARTED

    def __init__(self, boat_controller: BoatController, sensors: Union[Sensors, SensorsSimulated]):
        self.boat_controller = boat_controller
        self.sensors = sensors
        self._buoys = []
        self.red_pole_buoys = []
        self.green_pole_buoys = []

    def search(self) -> Optional[Tuple[float, float]]:
        self.boat_controller.get_logger().info("Searching for TaskOne")
        """
        Search for the following pattern where R is a red pole buoy, and G is a green pole buoy
        There could be any number of other objects between the buoys, and the rectangle could be rotated in any direction
        R   G
        
        
        R   G
        """

        self.red_pole_buoys = [buoy for buoy in self.sensors.map if isinstance(buoy, PoleBuoy) and buoy.color == BuoyColors.RED]
        self.green_pole_buoys = [buoy for buoy in self.sensors.map if isinstance(buoy, PoleBuoy) and buoy.color == BuoyColors.GREEN]

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

                        # Check angles on the 4 corners
                        angle_R1 = calculate_buoy_angle(green1, red1, red2)
                        if abs(angle_R1 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue
                        angle_G1 = calculate_buoy_angle(red1, green1, green2)
                        if abs(angle_G1 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue
                        angle_R2 = calculate_buoy_angle(green2, red2, red1)
                        if abs(angle_R2 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue
                        angle_G2 = calculate_buoy_angle(red2, green2, green1)
                        if abs(angle_G2 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue

                        self.boat_controller.get_logger().info(f"Angles: {angle_R1}, {angle_G1}, {angle_R2}, {angle_G2}")

                        # Get distance between same colored buoys (calculated in meters)
                        dist_g = distance(green1.x, green1.y, green2.x, green2.y)
                        if (dist_g < 30.48 + DIST_ALLOWED_DEVIATION and dist_g > 6.096 - DIST_ALLOWED_DEVIATION): 
                            continue
                        dist_r = np.sqrt((red2.x - red1.x) ** 2 + (red2.y - red1.y) ** 2)
                        if (dist_r < 30.48 + DIST_ALLOWED_DEVIATION and dist_r > 6.096 - DIST_ALLOWED_DEVIATION): 
                            continue

                        # Get distance between buoy pairs (calculated in meters)
                        dist_pair = distance(green1.x, green1.y, red2.x, red2.y)
                        if (dist_pair < 3.048 + DIST_ALLOWED_DEVIATION and dist_pair > 1.8288 - DIST_ALLOWED_DEVIATION): 
                            continue
                        dist_pair2 = np.sqrt((red2.x - green2.x) ** 2 + (red2.y - green2.y) ** 2)
                        if (dist_pair2 < 3.048 + DIST_ALLOWED_DEVIATION and dist_pair2 > 1.8288 - DIST_ALLOWED_DEVIATION): 
                            continue
                        
                        self.boat_controller.get_logger().info(f"Distances: {dist_g}, {dist_r}, {dist_pair}, {dist_pair2}")

                        #if everything is cool and right, we will continue with the run function
                            
                        # TODO: check that no other buoys are inside the rectangle

                        self._buoys = [red1, green1, red2, green2]

                        return (int((red1.x + green1.x + red2.x + green2.x) / 4), int((red1.y + green1.y + red2.y + green2.y) / 4))

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
        
        while not completion_status == TaskCompletionStatus.SUCCESS or not completion_status == TaskCompletionStatus.FAILURE:
            self.sensors.controller.set_forward_velocity(FORWARD_VELOCITY)
            
            # if first red buoy is closer, set that to closest_red_buoy. if second buoy is closer, set that to close_red_buoy instead.
            if(distance(self.red_pole_buoys[0].x, self.red_pole_buoys[0].y, BOAT_X, BOAT_Y) < distance(self.red_pole_buoys[1].x, self.red_pole_buoys[1].y, BOAT_X, BOAT_Y)):
                closest_red_buoy = (self.red_pole_buoys[0].x, self.red_pole_buoys[0].y)
            else:
                closest_red_buoy = (self.red_pole_buoys[1].x, self.red_pole_buoys[1].y)
                
            # 
            if(distance(self.green_pole_buoys[0].x, self.green_pole_buoys[0].y, BOAT_X, BOAT_Y) < distance(self.green_pole_buoys[1].x, self.green_pole_buoys[1].y, BOAT_X, BOAT_Y)):
                closest_green_buoy = (self.green_pole_buoys[0].x, self.green_pole_buoys[0].y)
            else:
                closest_green_buoy = (self.green_pole_buoys[1].x, self.green_pole_buoys[1].y)
            
            # Get the midpoint
            mid = midpoint(closest_red_buoy[0], closest_red_buoy[1], closest_green_buoy[0], closest_green_buoy[1])
            # 
            if abs(mid[0]) > ALLOWED_TURN_DEVIATION:
                self.sensors.controller.set_angular_velocity(-ANGULAR_VELOCITY if mid[0] > 0 else ANGULAR_VELOCITY)
            else:
                self.sensors.controller.set_angular_velocity(0)

            if self.green_pole_buoys[0].y < 0 and self.green_pole_buoys[1].y < 0  and self.red_pole_buoys[0].y < 0  and self.red_pole_buoys[1].y < 0:
                completion_status = TaskCompletionStatus.SUCCESS

            
        return completion_status

def main(controller: BoatController):
    controller.add_task(TaskOne(controller, controller.sensors))
