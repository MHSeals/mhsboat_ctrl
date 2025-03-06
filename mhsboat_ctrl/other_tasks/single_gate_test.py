from typing import Tuple, Optional

from mhsboat_ctrl.mhsboat_ctrl import BoatController
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus, BuoyColors
from mhsboat_ctrl.course_objects import PoleBuoy
from mhsboat_ctrl.utils.math_util import midpoint


class SingleGateTest(Task):
    status = TaskStatus.NOT_STARTED

    def __init__(self, boat_controller: BoatController):
        self.boat_controller = boat_controller
        self.buoy_map = self.boat_controller.buoy_map
        self._buoys = []
        self.red_pole = None
        self.green_pole = None

    def search(self) -> Optional[Tuple[float, float]]:
        self.boat_controller.get_logger().info("Searching for SingleGateTest")
        # Find red buoy to the left and green buoy to the right of the boat
        for b1 in self.buoy_map:
            for b2 in self.buoy_map:
                if b1 == b2:
                    continue
                if (
                    isinstance(b1, PoleBuoy)
                    and isinstance(b2, PoleBuoy)
                    and b1.color == BuoyColors.RED
                    and b2.color == BuoyColors.GREEN
                    and b1.x < b2.x
                ):
                    self.red_pole = b1
                    self.green_pole = b2

    def run(self) -> TaskCompletionStatus:
        if self.red_pole is None or self.green_pole is None:
            return TaskCompletionStatus.NOT_STARTED
        
        self.boat_controller.get_logger().info("Running SingleGateTest")
        
        # Compute gate midpoint using the red and green poles.
        mid = midpoint(
            self.red_pole.x, self.red_pole.y, self.green_pole.x, self.green_pole.y
        )
        self.boat_controller.get_logger().info(f"Computed gate midpoint: {mid}")

        # Set forward velocity (example value).
        self.boat_controller.set_forward_velocity(0.2)

        # Define an acceptable lateral deviation (in pixels/meters, adjust as needed).
        ALLOWED_LATERAL_DEVIATION = 10  # adjust threshold

        # Adjust angular velocity according to the lateral deviation of the computed midpoint.
        if abs(mid[0]) > ALLOWED_LATERAL_DEVIATION:
            angular = -0.5 if mid[0] > 0 else 0.5
            self.boat_controller.set_angular_velocity(angular)
            self.boat_controller.get_logger().info(
                f"Adjusting angular velocity to {angular}"
            )
        else:
            self.boat_controller.set_angular_velocity(0)
            self.boat_controller.get_logger().info(
                "Lateral deviation within threshold; heading aligned."
            )
            # Once aligned, we assume the boat can pass through the gate.
            return TaskCompletionStatus.SUCCESS

        return TaskCompletionStatus.NOT_STARTED


def main(controller: BoatController):
    controller.add_task(SingleGateTest(controller))
