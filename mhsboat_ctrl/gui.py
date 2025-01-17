import numpy as np
import yaml
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide" # Hide annoying prompt
import pygame
from mhsboat_ctrl.course_objects import Buoy, PoleBuoy, BallBuoy
from mhsboat_ctrl.enums import BuoyColors

# Constants
FONT_SIZE = 40
FONT = "Segoe UI Medium"
FONT_COLOR = "black"
BUOY_RADIUS = 10
BOAT_WIDTH = 40
BOAT_HEIGHT = 70
BOAT_COLOR = "gray12"
BACKGROUND_COLOR = "lightblue"
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

class GUI():
    def __init__(self):
        # Start Pygame window
        pygame.init()

        # Dynamic GUI variables
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.text = pygame.font.SysFont(FONT, FONT_SIZE)
        self.boat = Boat(convert_color(BOAT_COLOR), BOAT_WIDTH, BOAT_HEIGHT, SCREEN_WIDTH/2, SCREEN_HEIGHT/2)
        self.buoys = []
        self.buoy_color = BuoyColors.GREEN
        self.buoy_type = PoleBuoy
        self.prev_time = pygame.time.get_ticks() / 1000
        self.words = ""
        self.run = True
        self.load = False 
        self.save = False
    
    def display_loop(self):
        # Event handler
        for event in pygame.event.get():
            if(event.type == pygame.KEYDOWN):
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
                if self.buoy_type == PoleBuoy and self.buoy_color != BuoyColors.RED and self.buoy_color != BuoyColors.GREEN:
                    print("Buoy color not available for type")
                else:
                    self.buoys.append(self.buoy_type(*pos, self.buoy_color)) # type: ignore
            elif event.type == pygame.QUIT or not self.run:
                self.quit()
                return

        # Calculate delta time to compensate for flucuating framerate
        self.time = pygame.time.get_ticks() / 1000
        self.dt = self.time - self.prev_time

        # Wipes the screen by drawing the background
        self.screen.fill(convert_color(BACKGROUND_COLOR))

        # Update and draw the boat
        self.boat.turn(self.dt)
        self.boat.move(self.dt)
        self.draw_boat(self.boat.get_draw_points())
        
        # Draw the buoys
        self.draw_buoys()
        
        # Update and draw the text
        if not (self.load or self.save):
            self.words = f"{self.buoy_type.__name__}_{self.buoy_color.value}".upper()
        self.draw_text(self.words)

        # Update the display and save current time for delta time
        self.prev_time = self.time
        pygame.display.update()

    def draw_boat(self, points):
        pygame.draw.polygon(self.screen, self.boat.color, points)

    def draw_text(self, words):
        font_surface = self.text.render(words, True, FONT_COLOR)    
        self.screen.blit(font_surface, (10, 10))
    
    def draw_buoys(self):
        for buoy in self.buoys:
            if isinstance(buoy, Buoy):
                color = convert_color(buoy.color.value)
                x = buoy.x
                y = buoy.y
        
                if isinstance(buoy, PoleBuoy):
                    pygame.draw.circle(self.screen, color, (x, y), BUOY_RADIUS)
                    pygame.draw.circle(self.screen, darken_color(color, 80), (x, y), BUOY_RADIUS * 5 / 6)
                    pygame.draw.circle(self.screen, color, (x, y), BUOY_RADIUS * 2 / 5)
                elif isinstance(buoy, BallBuoy):
                    pygame.draw.circle(self.screen, color, (x, y), BUOY_RADIUS)
                    pygame.draw.circle(self.screen, darken_color(convert_color("blue"), 120), (x, y), BUOY_RADIUS / 2)
                else:
                    print("Buoy variation unspecified")
            else:
                print("Buoy type error")

    def write_map(self):
        ...

    def read_map(self):
        ...
        
    def quit(self):
        self.run = False
        pygame.quit()
        print("GUI Quit")
        

class Boat():
    # Initialize starting values
    def __init__(self, color: pygame.Color, width: int, height: int, x=0.0, y=0.0, angular_vel=0.0, linear_vel=0.0, orientation=0.0):
        # Properties
        self.color = color
        self.width = width 
        self.height = height 
        self.angular_vel = angular_vel 
        self.linear_vel = linear_vel
        self.orientation = orientation
        self.x = x
        self.y = y
        
        # Helper values for drawing the boat
        self.diagonal = (width ** 2 + height ** 2) ** (1/2)

    # Print string
    def __str__(self):
        return f"Boat(x:{self.x}, y:{self.y}, o:{self.orientation}, ω:{self.angular_vel}, v:{self.linear_vel})"

    # Move the boat based on linear velocity and orientation
    def move(self, dt: float):
       self.x += np.cos(self.orientation) * self.linear_vel * dt
       self.y -= np.sin(self.orientation) * self.linear_vel * dt
   
    # Turn the boat based on angular velocity in radians per second
    def turn(self, dt: float):
       self.orientation += self.angular_vel * dt
       self.orientation %= np.pi * 2
       
    # Redraw the boat each frame by using the angles formed by diagonals
    def get_draw_points(self) -> list[tuple[int, int]]:
        a1 = np.arctan(self.width / self.height) * 2
        a2 = np.pi - a1
        angles = [self.orientation + a1 / 2]
        angles.append(angles[0] + a2)
        angles.append(angles[1] + a1)
        angles.append(angles[2] + a2)
        
        points = []
        for i in range(4):
            points.append((self.x + np.cos(angles[i]) * self.diagonal / 2, self.y - np.sin(angles[i]) * self.diagonal / 2))
        
        return points

# Helper functions
def convert_color(color: str) -> pygame.Color:
        try:
            return pygame.Color(color)
        except ValueError as e:
            print(f"Color not defined in colordict: {e}")
            return pygame.Color("pink") # Default color to indicate error

# Function that darkens any given color by subtracting a certain amount from the color value
def darken_color(color: pygame.Color, amount: int) -> pygame.Color:
    new_color = []
    for i in range(3):
        new_color.append(max(color[i] - amount, 0))
    return pygame.Color(tuple(new_color))
