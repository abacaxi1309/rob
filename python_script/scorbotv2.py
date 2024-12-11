import serial
from scipy.io import loadmat
import numpy as np
import time

# Load the .mat file
data = loadmat('trajectory_data.mat')

# Extract positions and orientations
positions = data['positions']         # Position matrix (X, Y, Z)
orientations = data['orientations']   # Orientation matrix (Theta_X, Theta_Y, Theta_Z)

# Separate the columns for individual analysis
position_x, position_y, position_z = positions[:, 0], positions[:, 1], positions[:, 2]
theta_x, theta_y, theta_z = orientations[:, 0], orientations[:, 1], orientations[:, 2]

# Extract theta positions and convert to degrees
theta_x = np.degrees(theta_x.flatten())
theta_y = np.degrees(theta_y.flatten())
theta_z = np.degrees(theta_z.flatten())

# Process theta_x and theta_y (round to one decimal, multiply by 10, convert to int)
theta_x_processed = [int(round(val, 1) * 10) for val in theta_x]
theta_y_processed = [int(round(val, 1) * 10) for val in theta_y]

# Extract and convert positions
x_positions = position_x.flatten() * 1000  # Convert to mm
y_positions = position_y.flatten() * 1000  # Convert to mm
z_positions = position_z.flatten() * 1000  # Convert to mm

x_positions_processed = [int(round(val, 1) * 10) for val in x_positions]
y_positions_processed = [int(round(val, 1) * 10) for val in y_positions]
z_positions_processed = [int(round(val, 1) * 10) for val in z_positions]

# Ensure connection to the serial port
try:
    ser = serial.Serial(
        port='COM4',           
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )
    print("Connected to SCORBOT-ER VII.")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

# Function to send commands to SCORBOT
def send_command(command):
    final_command = command + '\r'
    ser.write(final_command.encode('utf-8'))
    print("Command sent:", repr(final_command.encode('utf-8')))
    ser.flush()
    # Wait briefly before reading
    import time
    time.sleep(0.5)  # Adjust as needed
    response = ser.read_all().decode('utf-8', errors='ignore').strip()
    return response

# Initialize the robot
print("Initializing SCORBOT...")
response = send_command(f"DELP ZZ")
print(f"Response: {repr(response)}")
response = send_command(f"YES")
print(f"Response: {repr(response)}")
response = send_command(f"DEFP ZZ")
print(f"Response: {repr(response)}")
response = send_command(f"HERE ZZ")
print(f"Response: {repr(response)}")

i = 0
for i in range(1):
    x = x_positions_processed[i]
    y = y_positions_processed[i]
    z = z_positions_processed[i]
    p = theta_y_processed[i]
    r = theta_x_processed[i]
    
    print(x, y, z)
    response = send_command(f"SETPVC ZZ X {int(x)}")
    print(f"Response: {repr(response)}")
    response = send_command(f"SETPVC ZZ Y {int(y)}")
    print(f"Response: {repr(response)}")
    response = send_command(f"SETPVC ZZ Z {int(z)}")
    print(f"Response: {repr(response)}")
    #response = send_command(f"SETPVC ZZ P {int(p)}")
    #print(f"Response: {repr(response)}")
    #response = send_command(f"SETPVC ZZ R {int(r)}")
    #print(f"Response: {repr(response)}")
    response = send_command(f"LISTPV ZZ")
    print(f"Response: {repr(response)}")
    #response = send_command(f"MOVE P")
    #print(f"Response: {repr(response)}")
