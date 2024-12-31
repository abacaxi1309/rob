import numpy as np
from scipy.optimize import minimize
import pygame
import math
import matplotlib.pyplot as plt
import csv
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
        self.lta = []

        # Time tracking
        self.time_elapsed = 0

        self.screen = None
        self.clock = None

        # PID state variables
        self.pid_integral = 0
        self.pid_prev_error = 0

        self.best_pid_params = None

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


    def calculate_lateral_error(self, state):
        """Calculate lateral error based on front wheel positions."""

        corners = get_car_corners(state, self.CAR_LENGTH, self.CAR_WIDTH)
        front_left = corners[2][0]  # x-coordinate of front left wheel
        front_right = corners[3][0]  # x-coordinate of front right wheel

        if front_left < self.inside_left_boundary:
            return self.inside_left_boundary - front_left, 'left'
        elif front_right > self.inside_right_boundary:
            return front_right - self.inside_right_boundary, 'right'
        return 0, None

    def compute_pid_correction(self, Kp, Ki, Kd, error, dt):
        """Compute PID correction based on error."""
        self.pid_integral += error * dt
        derivative = (error - self.pid_prev_error) / dt
        self.pid_prev_error = error

        return Kp * error + Ki * self.pid_integral + Kd * derivative

    def pid_cost_function(self, params):
        """Cost function to evaluate PID performance."""

        Kp, Ki, Kd = params
        self.reset_simulation()

        total_error = 0
        state = self.initial_state.copy()
        for _ in range(int(10 * self.FPS)):  # Simulate for 10 seconds
            lateral_error, _ = self.calculate_lateral_error(state)

            dt = 1 / self.FPS
            correction = self.compute_pid_correction(Kp, Ki, Kd, lateral_error, dt)

            state = self.update_car_state(correction, state, dt)
            total_error += abs(lateral_error)  # Sum of absolute errors

            if total_error > 1e6:  # Avoid long simulations if error is too high
                return total_error

        return total_error

    def optimize_pid(self):

        """Optimize PID parameters."""
        #initial_guess = [3.4, 1.67, 0.033] 
        initial_guess = [1, 0.06, 0.16]
        bounds = [(0, 1), (0, 1), (0, 1)]  # Reasonable bounds for Kp, Ki, Kd

        result = minimize(self.pid_cost_function, initial_guess, bounds=bounds)
        self.best_pid_params = result.x

    def reset_simulation(self):
        """Reset simulation state for PID optimization."""
        self.positions_center = []
        self.corners = []
        self.times = []
        self.time_elapsed = 0
        self.pid_integral = 0
        self.pid_prev_error = 0

    def run_simulation(self):
        """Run the simulation using optimized PID parameters."""
        self.optimize_pid()
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
        line_detected = False  # Track if the line is detected
        # Font for alert message
        font = pygame.font.Font(None, 36)  # Default font, size 36
        alert_message = font.render("ALERT: EDGE LINE DETECTED", True, (255, 0, 0))  # Red text

        # TASK 7
        # Lists for data logging
        log_time = []
        log_joystick_value = []
        log_lateral_error = []
        log_pid_correction = []
        log_x_position = []
        log_y_position = []
        log_theta = []


        try:
            running = True
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False

                # Get joystick input
                joystick_value = gamepad_control(joystick)

                # Check for boundary detection
                lateral_error, side = self.calculate_lateral_error(state)
                pid_correction = 0
                if lateral_error != 0:  # Activate PID only when wheels detect the line
                    line_detected = True  # Line detected
                    Kp, Ki, Kd = self.best_pid_params
                    pid_correction = self.compute_pid_correction(Kp, Ki, Kd, lateral_error, 1 / self.FPS)
                else:
                    line_detected = False  # No line detected

                if side == 'left':
                    state = self.update_car_state(joystick_value + pid_correction, state, dt=1 / self.FPS)
                elif side == 'right':
                    state = self.update_car_state(joystick_value - pid_correction, state, dt=1 / self.FPS)
                else:
                    state = self.update_car_state(joystick_value, state, dt=1 / self.FPS)

                x, _, _, _ = state

                # TASK 7
                # Saving the values to the lists
                log_time.append(self.time_elapsed)
                log_joystick_value.append(joystick_value)
                log_lateral_error.append(lateral_error)
                log_pid_correction.append(pid_correction)
                log_x_position.append(x)
                log_y_position.append(state[1])
                log_theta.append(state[2])

                # Append the values to the lists
                self.positions_center.append(x)
                self.times.append(self.time_elapsed)  # Add the current time to times
                self.corners.append(get_car_corners(state, self.CAR_LENGTH, self.CAR_WIDTH))

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

                # Display alert message if line is detected
                if line_detected:
                    self.screen.blit(alert_message, (10, 10))  # Display message at (50, 50)

                pygame.display.flip()
                self.clock.tick(self.FPS)
        except Exception as e:
            print(f"Error: {e}")
        finally:
            pygame.quit()
            self.plot_results()

            ##### TASK 7

            # Plot of the lateral error and control inputs over
            plt.figure(figsize=(16, 12))

            # Subplot 1: Lateral error vs. time
            plt.subplot(3, 1, 1)
            plt.plot(log_time, log_lateral_error, label='Lateral Error', color='blue')
            plt.axhline(0, color='red', linestyle='--', label='Center Line')
            plt.xlabel('Time (s)')
            plt.ylabel('Lateral Error')
            plt.title('Lateral Error Over Time')
            plt.legend()

            # Subplot 2: Joystick input and PID correction vs. time
            plt.subplot(3, 1, 2)
            plt.plot(log_time, log_joystick_value, label='Joystick Input', color='green')
            plt.plot(log_time, log_pid_correction, label='PID Correction', color='orange')
            plt.xlabel('Time (s)')
            plt.ylabel('Control Input')
            plt.title('Joystick and PID Correction Over Time')
            plt.legend()

            # Subplot 3: Trajectory of the car with lanes
            plt.subplot(3, 1, 3)

            # Ajusting the y_position to be inverted
            y_position_inverted = max(log_y_position) - log_y_position + min(log_y_position)

            # Scatter plot with colormap
            scatter = plt.scatter(log_x_position, y_position_inverted,  
                                c=log_time,  
                                cmap='viridis', 
                                marker='o',
                                s=30,
                                label='Car Trajectory')

            # Colorbar
            plt.colorbar(scatter, label='Time (s)')

            # Draw lane boundaries
            road_center = self.SCREEN_WIDTH / 2
            plt.axvline(x=self.inside_left_boundary, color='r', linestyle='--', label="Left Lane")
            plt.axvline(x=self.inside_right_boundary, color='g', linestyle='--', label="Right Lane")
            plt.axvline(x=road_center, color='red', linestyle='-', label='Center Line')

            # Labels
            plt.xlabel('Lateral Position')
            plt.ylabel('Forward Position')
            plt.title('Car Trajectory with Lane Boundaries')
            plt.legend()
            plt.tight_layout()
            plt.show()

            # Save the logged data to a file

            with open('task7_log.csv', mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Time', 'Joystick Value', 'Lateral Error', 'PID Correction', 'X Position'])
                for t, js, le, pc, x_pos in zip(log_time, log_joystick_value, log_lateral_error, log_pid_correction, log_x_position):
                    writer.writerow([t, js, le, pc, x_pos])
            
            print("Data saved to task7_log.csv")

if __name__ == "__main__":
    simulation = LaneSimulation()
    simulation.run_simulation()




