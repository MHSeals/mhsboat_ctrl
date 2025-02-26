from boat_interfaces.msg import BuoyMap
from mhsboat_ctrl.enums import BuoyColors
from mhsboat_ctrl.course_objects import Buoy, PoleBuoy, BallBuoy, CourseObject
import pygame
import rclpy
from rclpy.node import Node
import os
from typing import Tuple
from mhsboat_ctrl.utils.custom_types import numeric

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"  # Hide annoying prompt

# BuoyMap.x float64[]
# BuoyMap.y float64[]
# BuoyMap.types String[]
# BuoyMap.colors String[]

# Constants
FONT_SIZE = 40
FONT = "Segoe UI Medium"
FONT_COLOR = "black"
BUOY_RADIUS = 10
BOAT_WIDTH = 40
BOAT_HEIGHT = 70
BOAT_COLOR = "gray12"
BACKGROUND_COLOR = "white"
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
SCALE = 50


def world_to_screen(x: float, y: float, scale: numeric = SCALE) -> Tuple[int, int]:
    screen_x = SCREEN_WIDTH // 2 + int(round(y * scale))
    screen_y = SCREEN_HEIGHT // 2 - int(round(x * scale))
    return (screen_x, screen_y)


class TESTGUI(Node):
    def __init__(self):
        super().__init__("boat_test_gui")

        # Start Pygame window
        pygame.init()

        # Dynamic GUI variables
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.text = pygame.font.SysFont(FONT, FONT_SIZE)
        self.buoys: list[CourseObject] = []
        self.buoy_color = BuoyColors.GREEN
        self.buoy_type = PoleBuoy
        self.prev_time = pygame.time.get_ticks() / 1000
        self.words = ""
        self.run = True

        self.map_sub = self.create_subscription(
            BuoyMap, "/mhsboat_ctrl/map", self.map_callback, 10
        )

        self.display_timer = self.create_timer(0, self.display_loop)

    def map_callback(self, msg: BuoyMap):
        self.buoys = []
        for i in range(len(msg.x)):
            color = BuoyColors(msg.colors[i].lower())
            if msg.types[i] == "pole":
                self.buoys.append(PoleBuoy(msg.x[i], msg.y[i], msg.z[i], color))  # type: ignore color will always only be red or green
            elif msg.types[i] == "ball":
                self.buoys.append(BallBuoy(msg.x[i], msg.y[i], msg.z[i], color))

    def display_loop(self):
        # Event handler
        for event in pygame.event.get():
            if event.type == pygame.QUIT or not self.run:
                self.run = False
                pygame.quit()
                self.get_logger().info("Quit GUI")
                return

        self.screen.fill(BACKGROUND_COLOR)
        self.draw_buoys()
        self.draw_boat((SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2))
        pygame.display.update()
    
    # python
    def draw_boat(self, center: Tuple[int, int]):
        x, y = center
        # Define dimensions for the boat shape
        width = 20      # width of the square (base)
        roof_height = 10  # height of the triangular roof
        half_width = width // 2
        
        # The square bottom is drawn with the given center as its center.
        # Compute square vertices (clockwise)
        top_left = (x - half_width, y - half_width)
        top_right = (x + half_width, y - half_width)
        bottom_right = (x + half_width, y + half_width)
        bottom_left = (x - half_width, y + half_width)
        
        # Compute the roof apex (centered above the square)
        roof_apex = (x, top_left[1] - roof_height)
        
        # Order vertices to form a house shape:
        # bottom_left, bottom_right, top_right, roof_apex, top_left
        points = [bottom_left, bottom_right, top_right, roof_apex, top_left]
        pygame.draw.polygon(self.screen, BOAT_COLOR, points)

    def draw_buoys(self):
        for buoy in self.buoys:
            self.get_logger().info(f"Drawing buoy {buoy} at {buoy.x}, {buoy.y}")
            if isinstance(buoy, Buoy):
                color = self.convert_color(buoy.color.value)
                # Transform world coordinates to screen coordinates
                x, y = world_to_screen(buoy.x, buoy.y)

                if isinstance(buoy, PoleBuoy):
                    pygame.draw.circle(self.screen, color, (x, y), BUOY_RADIUS)
                    pygame.draw.circle(
                        self.screen,
                        self.darken_color(color, 80),
                        (x, y),
                        int(BUOY_RADIUS * 5 / 6),
                    )
                    pygame.draw.circle(
                        self.screen, color, (x, y), int(BUOY_RADIUS * 2 / 5)
                    )
                elif isinstance(buoy, BallBuoy):
                    pygame.draw.circle(self.screen, color, (x, y), BUOY_RADIUS)
                    pygame.draw.circle(
                        self.screen,
                        self.darken_color(self.convert_color("blue"), 120),
                        (x, y),
                        int(BUOY_RADIUS / 2),
                    )
                else:
                    self.get_logger().info("Buoy variation unspecified")
            else:
                self.get_logger().info("Buoy type error")

    def convert_color(self, color: str) -> pygame.Color:
        try:
            return pygame.Color(color)
        except ValueError as e:
            self.get_logger().info(f"Color not defined in colordict: {e}")
            return pygame.Color("pink")  # Default color to indicate error

    # Function that darkens any given color by subtracting a certain amount from the color value
    def darken_color(self, color: pygame.Color, amount: int) -> pygame.Color:
        new_color = []
        for i in range(3):
            new_color.append(max(color[i] - amount, 0))
        return pygame.Color(tuple(new_color))


def main(args=None):
    rclpy.init(args=args)

    testgui = TESTGUI()

    try:
        rclpy.spin(testgui)
    except KeyboardInterrupt:
        testgui.get_logger().info("Shutting down")
    finally:
        testgui.destroy_node()
        rclpy.shutdown()
