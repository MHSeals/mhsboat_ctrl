import numpy as np
from mhsboat_ctrl.course_objects import Buoy, PoleBuoy, BallBuoy
from mhsboat_ctrl.enums import BuoyColors
import yaml
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide" # Hide annoying prompt
import pygame

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
    def draw(self):
        a1 = np.atan(self.width / self.height) * 2
        a2 = np.pi - a1
        angles = [self.orientation + a1 / 2]
        angles.append(angles[0] + a2)
        angles.append(angles[1] + a1)
        angles.append(angles[2] + a2)
        
        points = []
        for i in range(4):
            points.append((self.x + np.cos(angles[i]) * self.diagonal / 2, self.y - np.sin(angles[i]) * self.diagonal / 2))

        pygame.draw.polygon(screen, self.color, points)
    
    def update(self, linear_vel, angular_vel):
        self.linear_vel = linear_vel
        self.angular_vel = angular_vel

def draw_text(words):
    font_surface = text.render(words, True, FONT_COLOR)    
    screen.blit(font_surface, (10, 10))

def draw_buoys():
    for buoy in buoys:
        if isinstance(buoy, Buoy):
            color = convert_color(buoy.color.value)
            x = buoy.x
            y = buoy.y
    
            if isinstance(buoy, PoleBuoy):
                pygame.draw.circle(screen, color, (x, y), BUOY_RADIUS)
                pygame.draw.circle(screen, darken_color(color, 80), (x, y), BUOY_RADIUS * 5 / 6)
                pygame.draw.circle(screen, color, (x, y), BUOY_RADIUS * 2 / 5)
            elif isinstance(buoy, BallBuoy):
                pygame.draw.circle(screen, color, (x, y), BUOY_RADIUS)
                pygame.draw.circle(screen, darken_color(convert_color("blue"), 120), (x, y), BUOY_RADIUS / 2)
            else:
                print("Buoy variation unspecified")
        else:
            print("Buoy type error")

def convert_color(color: str) -> pygame.Color:
        try:
            return pygame.Color(color)
        except ValueError as e:
            print(f"Color not defined in colordict: {e}")
            return pygame.Color("pink") # Default color to indicate error

# Function that darkens any given color by subtractinga certain amount from the color value
def darken_color(color: pygame.Color, amount: int) -> pygame.Color:
    new_color = []
    for i in range(3):
        new_color.append(max(color[i] - amount, 0))
    return pygame.Color(tuple(new_color))

def write_map():
    ...

def read_map():
    ...

# Start Pygame window
pygame.init()

# Screen config
BACKGROUND_COLOR = "lightblue"
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

# Initial variables and constants
FONT_SIZE = 40
FONT = "Segoe UI Medium"
FONT_COLOR = "black"
BUOY_RADIUS = 10
BOAT_WIDTH = 40
BOAT_HEIGHT = 70
BOAT_COLOR = "gray12"
text = pygame.font.SysFont(FONT, FONT_SIZE)
boat = Boat(convert_color(BOAT_COLOR), BOAT_WIDTH, BOAT_HEIGHT, linear_vel=60, angular_vel=np.pi/22, orientation=-np.pi/4) # Arbitrary testing values
buoys = []
buoy_color = BuoyColors.GREEN
buoy_type = PoleBuoy
prev_time = pygame.time.get_ticks() / 1000
load = save = False
words = ""
run = True

while run:
    # Event handler
    for event in pygame.event.get():
        if(event.type == pygame.KEYDOWN):
            if event.key == pygame.K_l:
                load = True
                words = "LOAD MAP FILE?"
            elif event.key == pygame.K_s:
                save = True
                words = "SAVE MAP FILE?"
            elif event.key == pygame.K_g:
                buoy_color = BuoyColors.GREEN
            elif event.key == pygame.K_r:
                buoy_color = BuoyColors.RED
            elif event.key == pygame.K_y:
                buoy_color = BuoyColors.YELLOW
            elif event.key == pygame.K_b:
                buoy_color = BuoyColors.BLACK
            elif event.key == pygame.K_u:
                buoy_color = BuoyColors.BLUE
            elif event.key == pygame.K_1:
                buoy_type = BallBuoy
            elif event.key == pygame.K_2:
                buoy_type = PoleBuoy
        elif event.type == pygame.MOUSEBUTTONUP:
            pos = pygame.mouse.get_pos()
            if buoy_type == PoleBuoy and buoy_color != BuoyColors.RED and buoy_color != BuoyColors.GREEN:
                print("Buoy color not available for type")
            else:
                buoys.append(buoy_type(*pos, buoy_color)) # type: ignore
        elif event.type == pygame.QUIT:
            run = False

    # Calculate delta time to compensate for flucuating framerate
    time = pygame.time.get_ticks() / 1000
    dt = time - prev_time

    # Wipes the screen by drawing the background
    screen.fill(convert_color(BACKGROUND_COLOR))

    # Update the state of the screen
    boat.turn(dt)
    boat.move(dt)
    boat.draw()
    draw_buoys()
    if not (load or save):
        words = f"{buoy_type.__name__}_{buoy_color.value}".upper()
    draw_text(words)

    pygame.display.update()
    prev_time = time

# Properly shutdown Pygame after quit event
pygame.quit()