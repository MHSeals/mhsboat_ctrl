import matplotlib.pyplot as plt
import numpy as np

from mhsboat_ctrl.mhsboat_ctrl import BoatController
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus, BuoyColors
from mhsboat_ctrl.course_objects import PoleBuoy
from mhsboat_ctrl.sensors import Sensors, SensorsSimulated
from mhsboat_ctrl.mhsboat_ctrl.utils.math_util import distance, angle, calculate_polebouy_angle

# TODO: What is a good value for these?
ANGLE_ALLOWED_DEVIATION = 15 # degrees
DIST_ALLOWED_DEVIATION = 1 # meters


class TaskOne(Task):
    status = TaskStatus.NOT_STARTED

    def __init__(self, controller: BoatController, sensors: Sensors | SensorsSimulated):
        self.controller = controller
        self.sensors = sensors
        self._buoys = []
        self.red_pole_buoys = []
        self.green_pole_buoys = []

    def search(self) -> None | tuple[float, float]:
        self.controller.get_logger().info("Searching for TaskOne")
        """
        Search for the following pattern where R is a red pole buoy, and G is a green pole buoy
        There could be any number of other objects between the buoys, and the rectangle could be rotated in any direction
        R   G
        
        
        R   G
        """

        # self.plot_all_buoys_and_angles()

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
                        angle_R1 = self.calculate_polebouy_angle(green1, red1, red2)
                        if abs(angle_R1 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue
                        angle_G1 = self.calculate_polebouy_angle(red1, green1, green2)
                        if abs(angle_G1 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue
                        angle_R2 = self.calculate_polebouy_angle(green2, red2, red1)
                        if abs(angle_R2 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue
                        angle_G2 = self.calculate_polebouy_angle(red2, green2, green1)
                        if abs(angle_G2 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue

                        self.controller.get_logger().info(f"Angles: {angle_R1}, {angle_G1}, {angle_R2}, {angle_G2}")

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
                        
                        self.controller.get_logger().info(f"Distances: {dist_g}, {dist_r}, {dist_pair}, {dist_pair2}")

                        #if everything is cool and right, we will continue with the run function
                            
                        # TODO: check that no other buoys are inside the rectangle

                        self._buoys = [red1, green1, red2, green2]

                        return (int((red1.x + green1.x + red2.x + green2.x) / 4), int((red1.y + green1.y + red2.y + green2.y) / 4))

    def run(self) -> TaskCompletionStatus:
        self.controller.get_logger().info("Running TaskOne")
        
        self.sensors.controller.set_forward_velocity(self, FORWARD_VELOCITY)
        
        if(distance(self.red_pole_buoys[0].x, self.red_pole_buoys[0].y, 0, 0) < distance(self.red_pole_buoys[1].x, self.red_pole_buoys[1].y, 0, 0)):
            closest_red_bouy = distance(self.red_pole_buoys[0].x, self.red_pole_buoys[0].y, 0, 0)
        else:
            closest_red_bouy = (self.red_pole_buoys[0].x, self.red_pole_buoys[0].y, 0, 0)
        if(distance(self.green_pole_buoys[0].x, self.green_pole_buoys[0].y, 0, 0) < distance(self.green_pole_buoys[1].x, self.green_pole_buoys[1].y, 0, 0)):
            closest_green_bouy = distance(self.green_pole_buoys[0].x, self.green_pole_buoys[0].y, 0, 0)
        else:
            closest_green_bouy = (self.green_pole_buoys[0].x, self.green_pole_buoys[0].y, 0, 0)
        
        mid = midpoint(closest_red_bouy.x, closest_red_bouy.y, closest_green_bouy.y, closest_green_bouy.y)
        if(mid[2]>ALLOWED_TURN_DEVIATION):
            self.sensors.controller.set_angular_velocity(self, -ANGULAR_VELOCITY)
        elif(mid[2]<-ALLOWED_TURN_DEVIATION):
            self.sensors.controller.set_angular_velocity(self, ANGULAR_VELOCITY)
        else:
            self.sensors.controller.set_angular_velocity(self, 0)
            
        return TaskCompletionStatus.SUCCESS

def main(controller: BoatController):
    controller.add_task(TaskOne(controller, controller.sensors))
