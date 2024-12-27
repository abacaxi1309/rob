import pygame
import math
import matplotlib.pyplot as plt
from task1_gamepad import gamepad_control
from task2_car_model import kinematic_model_update, get_car_corners

class LaneSimulation:

    def __init__(self):
        self.SCREEN_WIDTH = 700
        self.SCREEN_HEIGHT = 700
        self.LANE_WIDTH = 200
        self.CAR_LENGTH = 70
        self.CAR_WIDTH = 40
        self.LANE_COLOR = (0, 0, 0)  # Black lane color
        self.BACKGROUND_COLOR = (180, 165, 120)  # Sand color background
        self.FPS = 60
        self.LANE_LINE_WIDTH = 5 # Width of lane lines
        self.CAR_SPRITE_PATH = "tesla-birdview.png" # Car sprite image path

        # Initialize car position (start in the middle of the lane)
        self.initial_state = [
            self.SCREEN_WIDTH / 2,
            self.SCREEN_HEIGHT - self.CAR_LENGTH / 2 - 50,
            -math.pi / 2,
            0,
        ]

        self.car_speed = 50 # Forward velocity in pixels per second
        self.wheelbase = 50 # Distance between front and rear axles in pixels

        # Lane boundaries
        self.left_lane_line_x = (self.SCREEN_WIDTH - self.LANE_WIDTH) // 2
        self.right_lane_line_x = (self.SCREEN_WIDTH + self.LANE_WIDTH) // 2

        # Inside boundaries (considering line width)
        self.inside_left_boundary = self.left_lane_line_x + self.LANE_LINE_WIDTH
        self.inside_right_boundary = self.right_lane_line_x - self.LANE_LINE_WIDTH

        # Data for plotting
        self.positions_center = []
        self.corners = []
        self.times = []

        # Time tracking
        self.time_elapsed = 0

        self.screen = None
        self.clock = None

    # Load car sprite
    def load_car_sprite(self):
        """Load and scale the car sprite, rotate for proper orientation."""
        car_image = pygame.image.load(self.CAR_SPRITE_PATH).convert_alpha()
        car_image = pygame.transform.rotate(car_image, -90) # Rotate sprite -90 degrees to fix orientation
        return pygame.transform.scale(car_image, (self.CAR_LENGTH, self.CAR_WIDTH))

    # Pygame initialization
    def init_pygame(self):
        pygame.init()
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
        pygame.display.set_caption("Lane Simulation with Car Sprite")
        self.clock = pygame.time.Clock()

    # Update car position using kinematics
    def update_car_state(self, input_value, state, dt):
        """Update car state based on joystick input and kinematic model."""
        steering_rate = input_value * 1.5 # Scale joystick input to steering rate
        inputs = [self.car_speed, steering_rate]
        return kinematic_model_update(state, inputs, self.wheelbase, dt)

    # Draw environment with repeated pattern for infinite effect
    def draw_environment(self, vertical_offset):
        """Draw the road and lane with vertical offset for infinite scrolling."""
        # Fill background
        self.screen.fill(self.BACKGROUND_COLOR)

        # Calculate where to start drawing the road
        start_y = -vertical_offset % self.SCREEN_HEIGHT # Modular arithmetic for seamless repetition
        for i in range(3): # Draw three segments to ensure full coverage
            y_pos = start_y + i * self.SCREEN_HEIGHT
            pygame.draw.rect(
                self.screen,
                self.LANE_COLOR,
                (self.left_lane_line_x, y_pos, self.LANE_WIDTH, self.SCREEN_HEIGHT),
            )
            # Draw lane lines
            pygame.draw.rect(
                self.screen,
                (255, 255, 255),
                (self.left_lane_line_x, y_pos, self.LANE_LINE_WIDTH, self.SCREEN_HEIGHT),
            )
            pygame.draw.rect(
                self.screen,
                (255, 255, 255),
                (
                    self.right_lane_line_x - self.LANE_LINE_WIDTH,
                    y_pos,
                    self.LANE_LINE_WIDTH,
                    self.SCREEN_HEIGHT,
                ),
            )

    # Draw car sprite
    def draw_car_sprite(self, car_image, state):
        """Draw the car sprite at its current position and orientation."""
        x, y, theta, _ = state
        rotated_image = pygame.transform.rotate(car_image, -math.degrees(theta))
        new_rect = rotated_image.get_rect(center=(x, y))
        self.screen.blit(rotated_image, new_rect.topleft)

    # Plot results
    def plot_results(self):
        """Plot the car's position over time after the simulation ends."""

        plt.figure()
        # Draw lane boundaries
        plt.axhline(y=self.inside_left_boundary, color='r', linestyle='--', label="Left Lane")
        plt.axhline(y=self.inside_right_boundary, color='g', linestyle='--', label="Right Lane")

        # Draw car center of mass position
        plt.plot(self.times, self.positions_center, label="Car Center Position", color='blue')

        # Fill lane area for better visualization
        plt.fill_between(
            self.times, self.inside_right_boundary, self.inside_left_boundary, color="gray", alpha=0.2, label="Lane Area"
        )

        collision_times = []  # Track times of collisions for drawing
        collision_positions = []  # Track positions of collisions for markers

        # Initialize separate lists for the positions
        posLeft_list = []
        posRight_list = []

        for i, (_, _, posLeft_tuple, posRight_tuple) in enumerate(self.corners):
            posLeft = posLeft_tuple[0] # First element of third position
            posRight = posRight_tuple[0] # First element of fourth position

            if posLeft < self.inside_left_boundary:
                # If crossing below left boundary
                if len(collision_times) == 0 or self.times[i] != collision_times[-1]:
                    collision_times.append(self.times[i])
                    collision_positions.append(posLeft)
            elif posRight > self.inside_right_boundary:
                # If crossing above right boundary
                if len(collision_times) == 0 or self.times[i] != collision_times[-1]:
                    collision_times.append(self.times[i])
                    collision_positions.append(posRight)

            posLeft_list.append(posLeft)
            posRight_list.append(posRight)

        # Plot separate positions
        plt.plot(self.times, posLeft_list, label="Left Wheels", color="yellow")
        plt.plot(self.times, posRight_list, label="Right Wheels", color="black")

        # Mark collisions whith overlapping markers
        for t, c_pos in zip(collision_times, collision_positions):
            plt.scatter(t, c_pos, color="orange", zorder=5, label="Edge line detected" if t == collision_times[0] else "")

        plt.title("Car Position and Collisions Over Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (x-coordinate)")
        plt.legend()
        plt.grid()
        # Invert y-axis
        plt.gca().invert_yaxis()
        plt.show()

    # Simulation
    def run_simulation(self):
        self.init_pygame()
        car_image = self.load_car_sprite()
        state = self.initial_state.copy()
        # Initialize joystick
        pygame.joystick.init()
        joystick = None
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"Joystick connected: {joystick.get_name()}")
        else:
            print("No joystick found! Using keyboard controls instead.")

        vertical_offset = 0

        try:
            running = True
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False

                # Get joystick input
                joystick_value = gamepad_control(joystick)
                
                # Update car state using kinematics
                state = self.update_car_state(joystick_value, state, dt=1 / self.FPS)

                # Record positions for plotting
                x, _, _, _ = state
                self.positions_center.append(x)
                self.corners.append(get_car_corners(state, self.CAR_LENGTH, self.CAR_WIDTH))
                self.times.append(self.time_elapsed)

                # Check if the car crosses the top or bottom of the screen
                if state[1] < 0: # Crossed the top
                    vertical_offset += self.SCREEN_HEIGHT # Scroll the road down
                    state[1] += self.SCREEN_HEIGHT # Keep the car visible

                if state[1] > self.SCREEN_HEIGHT: # Crossed the bottom
                    vertical_offset -= self.SCREEN_HEIGHT # Scroll the road up
                    state[1] -= self.SCREEN_HEIGHT # Keep the car visible

                self.time_elapsed += 1 / self.FPS

                # Draw environment and car sprite with vertical offset
                self.draw_environment(vertical_offset)
                self.draw_car_sprite(car_image, state)

                pygame.display.flip()
                self.clock.tick(self.FPS)
        except Exception as e:
            print(f"Error: {e}")
        finally:
            pygame.quit()
            self.plot_results()

if __name__ == "__main__":
    simulation = LaneSimulation()
    simulation.run_simulation()
