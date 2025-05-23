import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSLivelinessPolicy,
)

from mhsboat_ctrl.course_objects import Shape, Buoy, PoleBuoy, BallBuoy
from mhsboat_ctrl.enums import BuoyColors

from boat_interfaces.msg import BuoyMap, BoatMovement
from geometry_msgs.msg import TwistStamped

import numpy as np
import os
import copy
from uuid import uuid1
from typing import Tuple, List 

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"  # Hide annoying prompt
import pygame

# Constants
FONT_SIZE = 40
FONT = "Segoe UI Medium"
FONT_COLOR = "black"
BUOY_RADIUS = 0.4
BOAT_WIDTH = 1
BOAT_HEIGHT = 2
BOAT_COLOR = "gray12"
BACKGROUND_COLOR = "lightblue"
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

WORLD_TO_PIXEL = 30

ANGLE = np.pi / 4
LOOKAHEAD = 3 # meters
KP = 3
KI = 0.06
KD = 0.1
INTEGRAL_BOUND = 1

BUOY_RADIUS *= WORLD_TO_PIXEL

class GUI(Node):
    def __init__(self):
        super().__init__("boat_gui")

        self.map_publisher = self.create_publisher(BuoyMap, "/mhsboat_ctrl/map", 10)

        self.movement_publisher = self.create_publisher(BoatMovement, "/mhsboat_ctrl/movement", 10)

        # Start Pygame window
        pygame.init()

        # Dynamic GUI variables
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.text = pygame.font.SysFont(FONT, FONT_SIZE)
        self.boat = Boat(
            convert_color(BOAT_COLOR),
            BOAT_WIDTH,
            BOAT_HEIGHT,
            0,
            0
        )
        self.prev_x = self.boat.x
        self.prev_y = self.boat.y
        self.prev_zr = self.boat.orientation
        self.buoys = []
        self.original_buoys = []
        self.buoy_color = BuoyColors.GREEN
        self.buoy_type = PoleBuoy
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.words = ""
        self.run = True
        self.load = False
        self.save = False
        
        self._qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", self.cmd_vel_callback, self._qos_profile
        )

        self.display_timer = self.create_timer(0, self.display_loop)
        
    def cmd_vel_callback(self, msg: TwistStamped):
        # self.get_logger().info("Received movement commands")
        self.boat.linear_vel = msg.twist.linear.x
        self.boat.angular_vel = msg.twist.angular.z

    def display_loop(self) -> None:
        # Event handler
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_l:
                    self.load = True
                    self.words = "LOAD MAP FILE?"
                elif event.key == pygame.K_s:
                    self.save = True
                    self.words = "SAVE MAP FILE?"
                elif event.key == pygame.K_g:
                    self.buoy_color = BuoyColors.GREEN
                elif event.key == pygame.K_r:
                    self.buoy_color = BuoyColors.RED
                elif event.key == pygame.K_y:
                    self.buoy_color = BuoyColors.YELLOW
                elif event.key == pygame.K_b:
                    self.buoy_color = BuoyColors.BLACK
                elif event.key == pygame.K_u:
                    self.buoy_color = BuoyColors.BLUE
                elif event.key == pygame.K_1:
                    self.buoy_type = BallBuoy
                elif event.key == pygame.K_2:
                    self.buoy_type = PoleBuoy
            elif event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                if (
                    self.buoy_type == PoleBuoy
                    and self.buoy_color != BuoyColors.RED
                    and self.buoy_color != BuoyColors.GREEN
                ):
                    self.get_logger().info("Buoy color not available for type")
                else:
                    self.original_buoys.append(self.buoy_type(*self.shift_back(pixel_to_world(translate_click_point(pos))), 0, self.buoy_color))  # type: ignore
            elif event.type == pygame.QUIT or not self.run:
                self.quit()
                return

        # Calculate delta time to compensate for flucuating framerate
        self.time = self.get_clock().now().nanoseconds / 1e9
        self.dt = self.time - self.prev_time

        # self.get_logger().info(f"dt: {self.dt}")

        # Wipes the screen by drawing the background
        self.screen.fill(convert_color(BACKGROUND_COLOR))

        # Update and draw the boat
        self.boat.turn(self.dt)
        self.boat.move(self.dt)

        msg = BoatMovement()
        msg.dx = float(self.boat.x - self.prev_x)
        msg.dy = float(self.boat.y - self.prev_y)
        msg.dzr = float(self.boat.orientation - self.prev_zr) 
        
        self.movement_publisher.publish(msg)

        self.prev_x = self.boat.x
        self.prev_y = self.boat.y
        self.prev_zr = self.boat.orientation

        self.draw_boat(self.boat.get_draw_points())

        # Draw the buoys
        self.draw_buoys()
        
        draw_path_points = self.draw_path(ANGLE)

        for point in draw_path_points:
            pygame.draw.circle(self.screen, (0, 0, 0), point, 2)

        # Update and draw the text
        if not (self.load or self.save):
            x, y = pygame.mouse.get_pos()
            self.words = f"{self.buoy_type.__name__}_{self.buoy_color.value} ({round((x / WORLD_TO_PIXEL), 2)}, {round(y / WORLD_TO_PIXEL, 2)})".upper()
        self.draw_text(self.words)

        # Update the display and save current time for delta time
        self.prev_time = self.time
        pygame.display.update()

        msg = BuoyMap()
        x, y, z, types, colors, uids = [], [], [], [], [], []

        for obj in self.buoys:
            x.append(float(obj.x))
            y.append(float(obj.y))
            z.append(0.0)

            uids.append(str(uuid1()))
            
            if isinstance(obj, Shape):
                types.append(obj.shape.value)
                colors.append(obj.color.value)
            elif isinstance(obj, PoleBuoy):
                types.append("pole")
                colors.append(obj.color.value)
            elif isinstance(obj, BallBuoy):
                types.append("ball")
                colors.append(obj.color.value)
            else:
                types.append("course_object")
                colors.append("none")

            # self.get_logger().info(f"Buoy: x={obj.x}, y={obj.y}, color={obj.color.value}")

        # self.get_logger().info(f"x:{self.boat.x}, y:{self.boat.y}, o:{self.boat.orientation}")

        msg.x = x
        msg.y = y
        msg.z = z
        msg.types = types
        msg.colors = colors
        msg.uids = uids

        self.map_publisher.publish(msg)

    def draw_boat(self, points):
        for i in range(len(points)):
            points[i] = translate_draw_point(self.shift_point(points[i]))
            
        pygame.draw.polygon(self.screen, self.boat.color, points)

    def draw_text(self, words):
        font_surface = self.text.render(words, True, FONT_COLOR)
        self.screen.blit(font_surface, (10, 10))
        
    def draw_path(self, angle: float) -> List[Tuple[int, int]]:
        points = []
        for x in range(int(-SCREEN_WIDTH / 2), int(SCREEN_WIDTH / 2)):
            shift_x, _ = self.shift_back((x, 0))
            y = np.tan(angle) * shift_x 
            points.append(translate_draw_point(self.shift_point((shift_x, y))))
        return points

    def draw_buoys(self):
        self.buoys = copy.deepcopy(self.original_buoys)

        for buoy in self.buoys:
            if isinstance(buoy, Buoy):
                color = convert_color(buoy.color.value)

                buoy.x, buoy.y = self.shift_point((buoy.x, buoy.y))

                x = buoy.x * WORLD_TO_PIXEL
                y = buoy.y * WORLD_TO_PIXEL
                
                center = translate_draw_point((x, y))

                if isinstance(buoy, PoleBuoy):
                    pygame.draw.circle(self.screen, color, center, BUOY_RADIUS)
                    pygame.draw.circle(
                        self.screen,
                        darken_color(color, 80),
                        center,
                        BUOY_RADIUS * 5 / 6,
                    )
                    pygame.draw.circle(self.screen, color, center, BUOY_RADIUS * 2 / 5)
                elif isinstance(buoy, BallBuoy):
                    pygame.draw.circle(self.screen, color, center, BUOY_RADIUS)
                    pygame.draw.circle(
                        self.screen,
                        darken_color(convert_color("blue"), 120),
                        center,
                        BUOY_RADIUS / 2,
                    )
                else:
                    self.get_logger().info("Buoy variation unspecified")
            else:
                self.get_logger().info("Buoy type error")

    def shift_back(self, point: Tuple[float, float]) -> Tuple[float, float]:
        cos_angle = np.cos(self.boat.orientation)
        sin_angle = np.sin(self.boat.orientation)
        
        x_rotated = point[0] * cos_angle - point[1] * sin_angle
        y_rotated = point[0] * sin_angle + point[1] * cos_angle
        
        x_world = x_rotated + self.boat.x
        y_world = y_rotated + self.boat.y

        return (x_world, y_world)

    def shift_back(self, point: Tuple[float, float]) -> Tuple[float, float]:
        x, y = point[0] + self.boat.x, point[1] + self.boat.y
        r, theta = np.hypot(x, y), np.arctan2(y, x)
        theta -= self.boat.orientation
        return (r * np.cos(theta), r * np.sin(theta))

    def write_map(self): ...

    def read_map(self): ...

    def quit(self):
        self.run = False
        pygame.quit()
        self.get_logger().info("GUI Quit")

class Boat:
    # Initialize starting values
    def __init__(
        self,
        color: pygame.Color,
        width: int,
        height: int,
        x=0.0,
        y=0.0,
        angular_vel=0.0,
        linear_vel=0.0,
        orientation=0.0,
    ):
        # Properties
        self.color = color
        self.width = width
        self.height = height
        self.angular_vel = angular_vel
        self.linear_vel = linear_vel
        self.orientation = orientation
        self.x = x
        self.y = y

        self.a1 = np.arctan(self.width / self.height) * 2
        self.a2 = np.pi - self.a1

        # Helper values for drawing the boat
        self.diagonal = ((width * WORLD_TO_PIXEL) ** 2 + (height * WORLD_TO_PIXEL) ** 2) ** (1 / 2)

    # Print string
    def __str__(self):
        return f"Boat(x:{self.x}, y:{self.y}, o:{self.orientation}, ω:{self.angular_vel}, v:{self.linear_vel})"

    # Move the boat based on linear velocity and orientation
    def move(self, dt: float):
        self.x += self.linear_vel * np.cos(self.orientation) * dt
        self.y += self.linear_vel * np.sin(self.orientation) * dt

    # Turn the boat based on angular velocity in radians per second
    def turn(self, dt: float):
        self.orientation += self.angular_vel * dt
        self.orientation %= np.pi * 2

    # Redraw the boat each frame by using the angles formed by diagonals
    def get_draw_points(self) -> list[tuple[int, int]]:
        angles = [self.orientation + self.a1 / 2]
        angles.append(angles[0] + self.a2)
        angles.append(angles[1] + self.a1)
        angles.append(angles[2] + self.a2)
    
        points = []
        for i in range(4):
            points.append(
                (
                    self.x + np.cos(angles[i]) * self.diagonal / 2,
                    self.y - np.sin(angles[i]) * self.diagonal / 2,
                )
            )
    
        tip_length = self.height * 0.3
        tip_x = self.x + np.cos(self.orientation) * (self.diagonal / 2 + tip_length)
        tip_y = self.y - np.sin(self.orientation) * (self.diagonal / 2 + tip_length)
    
        points.append((tip_x, tip_y))
    
        return points

# Helper functions
def convert_color(color: str) -> pygame.Color:
    try:
        return pygame.Color(color)
    except ValueError as e:
        print(f"Color not defined in colordict: {e}")
        return pygame.Color("pink")  # Default color to indicate error

# Function that darkens any given color by subtracting a certain amount from the color value
def darken_color(color: pygame.Color, amount: int) -> pygame.Color:
    new_color = []
    for i in range(3):
        new_color.append(max(color[i] - amount, 0))
    return pygame.Color(tuple(new_color))

def translate_draw_point(point: Tuple[int, int]) -> Tuple[int, int]:
    return (point[0] + SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 - point[1])

def translate_click_point(point: Tuple[int, int]) -> Tuple[int, int]:
    return (point[0] - SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 - point[1])

def world_to_pixel(point: Tuple[float, float]) -> Tuple[int, int]:
    return (int(point[0] * WORLD_TO_PIXEL), int(point[1] * WORLD_TO_PIXEL))

def pixel_to_world(point: Tuple[int, int]) -> Tuple[float, float]:
    return (point[0] / WORLD_TO_PIXEL, point[1] / WORLD_TO_PIXEL)

def main(args=None):
    rclpy.init(args=args)

    gui = GUI()

    try:
        rclpy.spin(gui)
    except KeyboardInterrupt:
        gui.get_logger().info("Shutting down")
    finally:
        gui.destroy_node()
        rclpy.shutdown()