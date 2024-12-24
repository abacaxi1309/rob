import pygame

def gamepad_control(joystick):
    """
    Captures analog stick or keyboard input for continuous steering.
    Returns a value indicating left (-1), right (1), or neutral (0).
    """
    steering_value = 0  # Default steering value

    # Continuous keyboard input
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        steering_value = -1
    elif keys[pygame.K_RIGHT]:
        steering_value = 1

    # Continuous joystick input
    if joystick:
        horizontal_value = joystick.get_axis(0)  # Axis 0: left/right
        if abs(horizontal_value) > 0.1:  # Dead zone filter
            steering_value = horizontal_value

    return steering_value
