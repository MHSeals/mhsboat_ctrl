import matplotlib.pyplot as plt
import numpy as np

from mhsboat_ctrl.mhsboat_ctrl import BoatController
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus, BuoyColors
from mhsboat_ctrl.course_objects import PoleBuoy
from mhsboat_ctrl.sensors import Sensors, SensorsSimulated

ANGLE_ALLOWED_DEVIATION = 5 # degrees


class TaskOne(Task):
    status = TaskStatus.NOT_STARTED

    def __init__(self, controller: BoatController, sensors: Sensors | SensorsSimulated):
        self.controller = controller
        self.sensors = sensors
        self._buoys = []

    def search(self) -> None | tuple[float, float]:
        self.controller.get_logger().info("Searching for TaskOne")
        # Search for the following pattern where R is a red pole buoy, and G is a green pole buoy
        # There could be any number of other objects between the buoys, and the rectangle could be rotated in any direction
        # R   G
        #
        #
        # R   G

        # self.plot_all_buoys_and_angles()

        red_pole_buoys = [buoy for buoy in self.sensors.map if isinstance(buoy, PoleBuoy) and buoy.color == BuoyColors.RED]
        green_pole_buoys = [buoy for buoy in self.sensors.map if isinstance(buoy, PoleBuoy) and buoy.color == BuoyColors.GREEN]

        if len(red_pole_buoys) < 2 or len(green_pole_buoys) < 2:
            return None

        for red1 in red_pole_buoys:
            for red2 in red_pole_buoys:
                if red1 == red2:
                    continue

                for green1 in green_pole_buoys:
                    for green2 in green_pole_buoys:
                        if green1 == green2:
                            continue

                        # Check angles on the 4 corners
                        angle_R1 = self.calculate_angle(green1, red1, red2)
                        if abs(angle_R1 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue
                        angle_G1 = self.calculate_angle(red1, green1, green2)
                        if abs(angle_G1 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue
                        angle_R2 = self.calculate_angle(green2, red2, red1)
                        if abs(angle_R2 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue
                        angle_G2 = self.calculate_angle(red2, green2, green1)
                        if abs(angle_G2 - 90) > ANGLE_ALLOWED_DEVIATION:
                            continue

                        self.controller.get_logger().info(f"Angles: {angle_R1}, {angle_G1}, {angle_R2}, {angle_G2}")

                        # TODO: check that no other buoys are inside the rectangle, and that the rectangle is not too big or too small

                        self._buoys = [red1, green1, red2, green2]

                        return (int((red1.x + green1.x + red2.x + green2.x) / 4), int((red1.y + green1.y + red2.y + green2.y) / 4))

    def calculate_angle(self, b1: PoleBuoy, b2: PoleBuoy, b3: PoleBuoy) -> float:
        """
        Calculate the angle between the lines formed by the 3 points

        :param b1: The first buoy
        :type b1: class:`mhsboat_ctrl.course_objects.PoleBuoy`
        :param b2: The second buoy
        :type b2: class:`mhsboat_ctrl.course_objects.PoleBuoy`
        :param b3: The third buoy
        :type b3: class:`mhsboat_ctrl.course_objects.PoleBuoy`
        :return: The angle between the lines
        :rtype: float
        """
        v1 = np.array([b1.x - b2.x, b1.y - b2.y])
        v2 = np.array([b3.x - b2.x, b3.y - b2.y])
        angle = np.arctan2(np.linalg.det([v1, v2]), np.dot(v1, v2))
        return abs(np.degrees(angle))

    def run(self) -> TaskCompletionStatus:
        self.controller.get_logger().info("Running TaskOne")

        # get pose

        # build waypoints to pass through the navigation gate while avoiding obstacles

        # follow waypoints

        return TaskCompletionStatus.SUCCESS


def main(controller: BoatController):
    controller.add_task(TaskOne(controller, controller.sensors))
