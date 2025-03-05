from sim_pid import PIDController
import numpy as np
import pygame
import noise
from time import time

NOISE_SCALE = 1.1
WAVE_STRENGTH = 10.0
WIDTH = 800
HEIGHT = 600
SLOPE = 5.0 / 3.0
BOAT_WIDTH = 20
BOAT_HEIGHT = 50
WAVE_SIM_SPEED = 0.2
LINEAR_VELOCITY = 60
TURN_SPEED = 1
SEED = 32

diagonal = (BOAT_WIDTH**2 + BOAT_HEIGHT**2) ** (1 / 2)

boat_pos = np.array([40.0, 20.0])
boat_orientation = np.pi / 2
boat_vel = np.array([0.0, 0.0])

start_time = time()
prev_t = 0
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))

positions = []

controller = PIDController(look_ahead=50, Kp=10, Ki=2, Kd=2, integral_bound=10)
angular_velocity = 0

def get_wave(t):
    t *= WAVE_SIM_SPEED
    u = noise.pnoise1(t * NOISE_SCALE, base=SEED) * WAVE_STRENGTH
    v = noise.pnoise1((t + 1000) * NOISE_SCALE, base=SEED) * WAVE_STRENGTH
    return np.array([u, v])

def draw_wave_field():
    step = 50
    for x in range(0, WIDTH, step):
        for y in range(0, HEIGHT, step):
            t = time() - start_time
            wave = get_wave(t + (x + y))
            wave_end = np.array([x, y]) + wave / WAVE_STRENGTH * 30

            color = scalar_to_color(np.linalg.norm(wave), 0, WAVE_STRENGTH)
            pygame.draw.line(screen, color, translate_draw_point((x, y)), translate_draw_point(wave_end), 2)
            pygame.draw.circle(screen, color, (x, y), 3)

def scalar_to_color(value, min_val, max_val):
    ratio = (value - min_val) / (max_val - min_val)
    ratio = max(0, min(1, ratio))

    r = int(255 * ratio)
    g = 0
    b = int(255 * (1 - ratio))

    return (r, g, b)

def draw_boat():
    a1 = np.arctan2(BOAT_WIDTH, BOAT_HEIGHT) * 2
    a2 = np.pi - a1
    angles = [boat_orientation + a1 / 2]
    angles.append(angles[0] + a2)
    angles.append(angles[1] + a1)
    angles.append(angles[2] + a2)

    points = []
    for i in range(4):
        points.append(translate_draw_point((
                        boat_pos[0] + np.cos(angles[i]) * diagonal / 2,
                        boat_pos[1] + np.sin(angles[i]) * diagonal / 2
                    )))

    pygame.draw.polygon(screen, "black", points)

def move(dt):
    boat_pos[0] += np.cos(boat_orientation) * LINEAR_VELOCITY * dt
    boat_pos[1] += np.sin(boat_orientation) * LINEAR_VELOCITY * dt
    
def turn(angular_velocity, dt):
    global boat_orientation
    boat_orientation += angular_velocity * dt

def translate_draw_point(point):
    return (point[0], HEIGHT - point[1])

while True:
    t = time() - start_time
    wave = get_wave(t)
    dt = t - prev_t
    boat_vel += wave * dt 
    boat_pos += boat_vel * dt 
    prev_t = t

    screen.fill("white")

    draw_boat()
    draw_wave_field() 
    move(dt)
    turn(angular_velocity, dt)
    
    positions.append(boat_pos.copy())
    
    # print(f"vel: {angular_velocity / np.pi * 180}, orientation: {boat_orientation / np.pi * 180}")
    
    goal, angular_velocity = controller.pure_pursuit(SLOPE, boat_pos.tolist(), boat_orientation) 
    angular_velocity *= TURN_SPEED * np.pi / 180
    goal = translate_draw_point(goal)

    pygame.draw.line(screen, "black", translate_draw_point((0, 0)), translate_draw_point((WIDTH, WIDTH * SLOPE)), 3)
    pygame.draw.circle(screen, "black", goal, 8)
    
    for pos in positions:
        pygame.draw.circle(screen, "black", translate_draw_point(pos.tolist()), 2)
    
    pygame.display.update()