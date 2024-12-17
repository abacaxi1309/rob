import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

#constant velocity 10 m/s
v = 10
L = 2
dt =0.1
ws = 0

x, y, theta,phi = 0,0,0,0
x_traj, y_traj = [],[] 

# Simulation loop
for t in np.arange(0, 10, dt):  # Simulate for 10 seconds
    x = (v * np.cos(theta)) * dt + x
    y = (v * np.sin(theta)) * dt + y
    theta = ((v / L) * np.tan(phi) * dt) + theta
    phi = ws*dt + phi
    # Save trajectory
    x_traj.append(x)
    y_traj.append(y)
    print(theta)   
    
    if t>5:
        phi =np.deg2rad(10)
        
#print(theta,x_traj, y_traj)
# Plot the trajectory
plt.plot(x_traj, y_traj, label="Car trajectory")
plt.axhline(y=-1, color='yellow', label="Left Bounding Lane")
plt.axhline(y=0, color='k', linestyle='--', label="Lane center")
plt.axhline(y=1, color='yellow', label="Right Bounding Lane")
plt.ylim(-5, 5)  # Limites para visualizar melhor as faixas
plt.xlim(0, max(x_traj))  # Limitar o eixo X ao máximo da trajetória
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.title("Kinematic Model of Car")
plt.legend()
plt.grid()
plt.show()

