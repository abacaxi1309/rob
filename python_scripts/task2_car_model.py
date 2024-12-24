import numpy as np

def kinematic_model_update(state, inputs, wheelbase, dt):
    """
    Update the state vector using the kinematic model and matrix representation as shown in the provided image.
    :param state: Current state vector [x, y, theta, phi].
    :param inputs: Control inputs [velocity (V), steering rate (ws)].
    :param wheelbase: Distance between front and rear axles in meters.
    :param dt: Time step for simulation in seconds.
    :return: Updated state vector [x, y, theta, phi].
    """
    x, y, theta, phi = state
    V, ws = inputs

    # Limit the steering angle phi
    max_steering_angle = np.pi / 4  # Limite de 45 graus
    phi = np.clip(phi, -max_steering_angle, max_steering_angle)

    # State-space model based on the provided matrix representation
    dx = V * np.cos(theta) * np.cos(phi)
    dy = V * np.sin(theta) * np.cos(phi)
    dtheta = (V / wheelbase) * np.sin(phi)
    dphi = ws

    # Discretize using Euler method
    new_state = np.array([
        x + dx * dt,
        y + dy * dt,
        theta + dtheta * dt,
        phi + dphi * dt
    ])

    return new_state

def get_car_corners(state, car_length, car_width):
    """
    Calculate the corners of the car based on its state.
    :param state: Current state vector [x, y, theta, phi].
    :param car_length: Length of the car.
    :param car_width: Width of the car.
    :return: Coordinates of the four corners of the car.
    """
    x, y, theta, phi = state

    # Rear axle center
    rear_x = x - (car_length / 2) * np.cos(theta)
    rear_y = y - (car_length / 2) * np.sin(theta)

    # Front axle center
    front_x = x + (car_length / 2) * np.cos(theta + phi)
    front_y = y + (car_length / 2) * np.sin(theta + phi)

    # Corners of the car
    corners = {
        "rear_left": (rear_x - (car_width / 2) * np.sin(theta), rear_y + (car_width / 2) * np.cos(theta)),
        "rear_right": (rear_x + (car_width / 2) * np.sin(theta), rear_y - (car_width / 2) * np.cos(theta)),
        "front_left": (front_x - (car_width / 2) * np.sin(theta + phi), front_y + (car_width / 2) * np.cos(theta + phi)),
        "front_right": (front_x + (car_width / 2) * np.sin(theta + phi), front_y - (car_width / 2) * np.cos(theta + phi))
    }

    return corners

def simulate(initial_state, inputs_fn, wheelbase, dt):
    """
    Simulate the car's motion indefinitely until interrupted.
    :param initial_state: Initial state vector [x, y, theta, phi].
    :param inputs_fn: Function that provides control inputs [V, ws] at a given time.
    :param wheelbase: Distance between front and rear axles in meters.
    :param dt: Time step for simulation in seconds.
    :return: Final state vector [x, y, theta, phi].
    """
    state = initial_state

    t = 0  # Time starts at zero
    while True:  # Infinite loop until manually stopped
        inputs = inputs_fn(t)
        state = kinematic_model_update(state, inputs, wheelbase, dt)
        t += dt  # Increment time

        yield state  # Yield the current state for real-time use

