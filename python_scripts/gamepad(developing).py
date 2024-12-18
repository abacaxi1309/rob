import pygame

def gamepad_control():
    """
    Captures analog stick or keyboard input for continuous steering.
    Returns a value indicating left (-1), right (1), or neutral (0).
    """
    pygame.init()
    pygame.joystick.init()

    # Check if a joystick is connected
    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Joystick connected: {joystick.get_name()}")
    else:
        print("No joystick found! Using keyboard controls instead.")

    steering_value = 0  # Default steering value

    # Check continuous input state
    keys = pygame.key.get_pressed()  # Check current state of all keys
    if keys[pygame.K_LEFT]:
        steering_value = -1  # Continuous left turn
    elif keys[pygame.K_RIGHT]:
        steering_value = 1  # Continuous right turn

    # Handle joystick input with dead zone
    if joystick:
        horizontal_value = joystick.get_axis(0)  # Axis 0: left/right
        if abs(horizontal_value) > 0.1:  # Dead zone filter
            steering_value = horizontal_value

    return steering_value