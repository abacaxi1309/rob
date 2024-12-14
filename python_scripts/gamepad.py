import pygame

def gamepad_control():
    """
    Initializes the gamepad and captures analog stick horizontal movements (left/right).
    Returns the axis value when moved left or right.
    """
    # Initialize pygame and joystick support
    pygame.init()
    pygame.joystick.init()

    # Check if a joystick is connected
    if pygame.joystick.get_count() == 0:
        print("No joystick found!")
        return None

    joystick = pygame.joystick.Joystick(0)  # Connect to the first joystick
    joystick.init()
    print(f"Joystick connected: {joystick.get_name()}")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break

            # Detect horizontal axis movements (Axis 0 for most controllers)
            if event.type == pygame.JOYAXISMOTION:
                if event.axis == 0:  # Typically, axis 0 corresponds to left/right movement
                    horizontal_value = event.value

                    # Only return values for significant movements (to filter noise)
                    if abs(horizontal_value) > 0.1:  # Ignore small movements
                        return horizontal_value

    pygame.quit()

if __name__ == "__main__":
    print("Starting gamepad control...")

    while True:
        axis_value = gamepad_control()
        if axis_value is not None:
            if axis_value < 0:
                print("Turning left")
            elif axis_value > 0:
                print("Turning right")
