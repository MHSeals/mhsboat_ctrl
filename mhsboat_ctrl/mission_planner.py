import rclpy
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush, WaypointClear, WaypointPull
from mavros_msgs.msg import Waypoint, GlobalPositionTarget
from std_srvs.srv import Trigger

class mission_planner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        self.wp_push = self.create_client(WaypointPush, '/boat/mission/push')
        self.wp_clear = self.create_client(WaypointClear, '/boat/mission/clear')
        self.wp_pull = self.create_client(WaypointPull, '/boat/mission/pull')
        self.arming_client = self.create_client(Trigger, '/boat/cmd/arming')
        self.takeoff_client = self.create_client(GlobalPositionTarget, '/boat/cmd/takeoff')

    async def clear_mission(self):
        while not self.wp_clear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Clear mission service not available, waiting...')

        req = WaypointClear.Request()
        resp = await self.wp_clear(req)
        if resp.success:
            self.get_logger().info('Mission cleared successfully!')
        else:
            self.get_logger().error('Failed to clear mission: %s' % resp.reason)

    async def push_waypoints(self, waypoints):
        while not self.wp_push.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Push waypoint service not available, waiting...')

        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = waypoints
        resp = await self.wp_push(req)
        if resp.success:
            self.get_logger().info('Waypoints pushed successfully!')
        else:
            self.get_logger().error('Failed to push waypoints: %s' % resp.reason)

    async def arm(self):
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting...')

        req = Trigger.Request()
        resp = await self.arming_client(req)
        if resp.success:
            self.get_logger().info('Vehicle armed successfully!')
        else:
            self.get_logger().error('Failed to arm vehicle: %s' % resp.message)

    async def takeoff(self, latitude, longitude, altitude):
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Takeoff service not available, waiting...')

        req = GlobalPositionTarget()
        req.latitude = latitude
        req.longitude = longitude
        req.altitude = altitude
        resp = await self.takeoff_client(req)
        if resp:
            self.get_logger().info('Vehicle took off successfully!')
        else:
            self.get_logger().error('Failed to take off: %s' % resp.message)

def main(args=None):
    # Example waypoints
    waypoints = [
        Waypoint(frame=3, command=22, is_current=True, autocontinue=True,
                 param1=0.0, param2=0.0, param3=0.0, param4=0.0, x_lat=47.397742, y_long=8.545593, z_alt=10.0),
                # Add more waypoints as needed
    ]

    mission_planner.clear_mission()
    mission_planner.push_waypoints(waypoints)
    mission_planner.arm()
    mission_planner.takeoff(47.397742, 8.545593, 10)

    rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    mission_planner = mission_planner()

    rclpy.spin_until_future_complete(main(mission_planner))
    rclpy.shutdown()
