import pygame
import sys
import matplotlib.pyplot as plt
from gamepad import gamepad_control  # Import the gamepad control function

# Constants for the simulation
SCREEN_WIDTH = 800 
SCREEN_HEIGHT = 600
LANE_WIDTH = 200
CAR_WIDTH = 40
CAR_HEIGHT = 70
CAR_COLOR = (255, 0, 0)
LANE_COLOR = (0, 0, 0)  # Black lane color
BACKGROUND_COLOR = (180, 165, 120)  # Sand color background
FPS = 60
LANE_LINE_WIDTH = 5  # Width of lane lines
TOLERANCE = 2  # Small tolerance for detection (Prevents false collision)

# Task 3: Simulation of car movement within lane boundaries
# Initialize pygame
def init_pygame():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Lane Simulation")
    return screen, pygame.time.Clock()

# Initialize car position (start in the middle of the lane)
car_x = (SCREEN_WIDTH - CAR_WIDTH) // 2
car_y = SCREEN_HEIGHT - CAR_HEIGHT - 50
car_speed = 5  # Speed of lateral movement

# Lane boundaries
left_lane_x = (SCREEN_WIDTH - LANE_WIDTH) // 2
right_lane_x = (SCREEN_WIDTH + LANE_WIDTH) // 2

# Inside boundaries (considering line width)
inside_left_boundary = left_lane_x + LANE_LINE_WIDTH
inside_right_boundary = right_lane_x - LANE_LINE_WIDTH

# Data for plotting (Task 4)
positions_center = []  # Center of the car
positions_left = []  # Left edge of the car
positions_right = []  # Right edge of the car
times = []  # List to store time
collisions = []  # To track times when the car crosses the lane boundaries

# Time tracking
time_elapsed = 0
car_fully_inside = True  # Tracks if car is fully inside the lane

def update_car_position(input_value, current_x):
    """ Update car position using the input value and enforce screen boundaries. """
    new_x = current_x + input_value * car_speed
    # Ensure the car stays within screen boundaries
    new_x = max(0, min(new_x, SCREEN_WIDTH - CAR_WIDTH))
    return new_x

def draw_environment(screen):
    """Draw the road, lanes, and car."""
    # Background
    screen.fill(BACKGROUND_COLOR)

    # Lane area (black color)
    pygame.draw.rect(screen, LANE_COLOR, (left_lane_x, 0, LANE_WIDTH, SCREEN_HEIGHT))

    # Lane lines
    pygame.draw.rect(screen, (255, 255, 255), (left_lane_x, 0, LANE_LINE_WIDTH, SCREEN_HEIGHT))
    pygame.draw.rect(screen, (255, 255, 255), (right_lane_x - LANE_LINE_WIDTH, 0, LANE_LINE_WIDTH, SCREEN_HEIGHT))

    # Car
    pygame.draw.rect(screen, CAR_COLOR, (car_x, car_y, CAR_WIDTH, CAR_HEIGHT))

def main():
    global car_x, time_elapsed, car_fully_inside

    screen, clock = init_pygame()

    try:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Get joystick input
            joystick_value = gamepad_control()
            if joystick_value is not None:
                car_x = update_car_position(joystick_value, car_x)

            # Calculate car edges
            car_left_edge = car_x
            car_right_edge = car_x + CAR_WIDTH
            fully_inside_now = (car_left_edge > inside_left_boundary and car_right_edge < inside_right_boundary)

            # Create car and line rectangles
            car_rect = pygame.Rect(car_x, car_y, CAR_WIDTH, CAR_HEIGHT)
            left_line_rect = pygame.Rect(left_lane_x, 0, LANE_LINE_WIDTH, SCREEN_HEIGHT)
            right_line_rect = pygame.Rect(right_lane_x - LANE_LINE_WIDTH, 0, LANE_LINE_WIDTH, SCREEN_HEIGHT)

            currently_colliding = car_rect.colliderect(left_line_rect) or car_rect.colliderect(right_line_rect)

            # Refined collision logic with full inside check
            if fully_inside_now and not car_fully_inside:
                # The car has fully returned to the lane
                car_fully_inside = True
            elif not fully_inside_now and car_fully_inside:
                # Car is now partially outside and colliding
                if currently_colliding:
                    collisions.append(time_elapsed)
                    car_fully_inside = False

            # Record data for plotting (Task 4)
            positions_center.append(car_x + CAR_WIDTH / 2)
            positions_left.append(car_x)
            positions_right.append(car_x + CAR_WIDTH)
            times.append(time_elapsed)
            time_elapsed += 1 / FPS

            # Draw everything (Task 3)
            draw_environment(screen)

            # Update display even without joystick input
            pygame.display.flip()

            # Cap the frame rate
            clock.tick(FPS)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        pygame.quit()
        plot_results()  # Task 4

# Task 4: Plot car position and collisions over time
def plot_results():
    """Plot the car's position over time after the simulation ends."""
    plt.figure()
    plt.plot(times, positions_center, label="Car Center Position", color="blue")
    plt.plot(times, positions_left, label="Car Left Edge", color="purple", linestyle="--")
    plt.plot(times, positions_right, label="Car Right Edge", color="brown", linestyle="--")
    plt.axhline(y=inside_left_boundary, color='r', linestyle='--', label="Inside Left Boundary")
    plt.axhline(y=inside_right_boundary, color='g', linestyle='--', label="Inside Right Boundary")

    # Mark collisions
    for t in collisions:
        plt.axvline(x=t, color='orange', linestyle=':', label="Crossed Line" if t == collisions[0] else "")

    plt.title("Car Position Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (x-coordinate)")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    main()
