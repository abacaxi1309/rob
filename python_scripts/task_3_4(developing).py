import pygame
import math
import matplotlib.pyplot as plt
from gamepad import gamepad_control
from robotCar import kinematic_model_update

# Constants for the simulation
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
LANE_WIDTH = 200
CAR_LENGTH = 70
CAR_WIDTH = 40
LANE_COLOR = (0, 0, 0)  # Black lane color
BACKGROUND_COLOR = (180, 165, 120)  # Sand color background
FPS = 60
LANE_LINE_WIDTH = 5  # Width of lane lines
CAR_SPRITE_PATH = "tesla-birdview.png"  # Car sprite image path

# Initialize car position (start in the middle of the lane)
initial_state = [SCREEN_WIDTH / 2, SCREEN_HEIGHT - CAR_LENGTH / 2 - 50, -math.pi / 2, 0]  # [x, y, theta, phi]
car_speed = 10  # Forward velocity in pixels per second
wheelbase = 50  # Distance between front and rear axles in pixels

# Lane boundaries
left_lane_x = (SCREEN_WIDTH - LANE_WIDTH) // 2
right_lane_x = (SCREEN_WIDTH + LANE_WIDTH) // 2

# Inside boundaries (considering line width)
inside_left_boundary = left_lane_x + LANE_LINE_WIDTH
inside_right_boundary = right_lane_x - LANE_LINE_WIDTH

# Data for plotting
positions_center = []  # Center of the car
times = []  # List to store time
collisions = []  # To track collisions with lane boundaries

# Time tracking
time_elapsed = 0

# Load car sprite
def load_car_sprite():
    """Load and scale the car sprite, rotate for proper orientation."""
    car_image = pygame.image.load(CAR_SPRITE_PATH).convert_alpha()
    car_image = pygame.transform.rotate(car_image, -90)  # Rotate sprite -90 degrees to fix orientation
    return pygame.transform.scale(car_image, (CAR_LENGTH, CAR_WIDTH))

# Pygame initialization
def init_pygame():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Lane Simulation with Car Sprite")
    return screen, pygame.time.Clock()

# Update car position using kinematics
def update_car_state(input_value, state, dt):
    """Update car state based on joystick input and kinematic model."""
    steering_rate = input_value * 0.5  # Scale joystick input to steering rate
    inputs = [car_speed, steering_rate]
    return kinematic_model_update(state, inputs, wheelbase, dt)

# Draw environment
def draw_environment(screen):
    """Draw the road and lane."""
    # Background
    screen.fill(BACKGROUND_COLOR)

    # Lane area (black color)
    pygame.draw.rect(screen, LANE_COLOR, (left_lane_x, 0, LANE_WIDTH, SCREEN_HEIGHT))

    # Lane lines
    pygame.draw.rect(screen, (255, 255, 255), (left_lane_x, 0, LANE_LINE_WIDTH, SCREEN_HEIGHT))
    pygame.draw.rect(screen, (255, 255, 255), (right_lane_x - LANE_LINE_WIDTH, 0, LANE_LINE_WIDTH, SCREEN_HEIGHT))

# Draw car sprite
def draw_car_sprite(screen, car_image, state):
    """Draw the car sprite at its current position and orientation."""
    x, y, theta, _ = state
    rotated_image = pygame.transform.rotate(car_image, -math.degrees(theta))
    new_rect = rotated_image.get_rect(center=(x, y))
    screen.blit(rotated_image, new_rect.topleft)

# Main simulation
def main():
    global time_elapsed

    screen, clock = init_pygame()
    car_image = load_car_sprite()
    state = initial_state.copy()

    try:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Get joystick input
            joystick_value = gamepad_control()

            # Update car state using kinematics
            state = update_car_state(joystick_value, state, dt=1 / FPS)

            # Record positions for plotting
            x, y, _, _ = state
            positions_center.append(x)
            times.append(time_elapsed)

            # Check for collisions
            if x - CAR_WIDTH / 2 < inside_left_boundary or x + CAR_WIDTH / 2 > inside_right_boundary:
                collisions.append(time_elapsed)

            time_elapsed += 1 / FPS

            # Draw environment and car sprite
            draw_environment(screen)
            draw_car_sprite(screen, car_image, state)

            pygame.display.flip()
            clock.tick(FPS)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        pygame.quit()
        plot_results()

# Plot results
def plot_results():
    """Plot the car's position over time after the simulation ends."""
    plt.figure()
    plt.plot(times, positions_center, label="Car Center Position", color="blue")
    plt.axhline(y=inside_left_boundary, color='r', linestyle='--', label="Inside Left Boundary")
    plt.axhline(y=inside_right_boundary, color='g', linestyle='--', label="Inside Right Boundary")

    # Mark collisions
    for t in collisions:
        plt.axvline(x=t, color='orange', linestyle=':', label="Collision" if t == collisions[0] else "")

    plt.title("Car Position and Collisions Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (x-coordinate)")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    main()