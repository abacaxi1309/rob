import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
import pandas as pd

# Define symbolic variables for Lyapunov analysis
def define_lyapunov_analysis():
    e = sp.Symbol('e', real=True)  # Lateral error
    de = sp.Symbol('de', real=True)  # Derivative of lateral error

    # Lyapunov candidate function
    V = 0.5 * e**2  # Quadratic form
    dV = sp.diff(V, e) * (-1.0 * e - 0.06 * de)  # Using error dynamics

    return V, dV

# Simulate the dynamics for lateral error convergence
def simulate_lateral_error(Kp=1.0, Kd=0.06, Ki=0.16, t_end=40, dt=0.01):
    """Simulate the lateral error dynamics with given PID parameters."""
    time = np.arange(0, t_end, dt)
    error = np.zeros_like(time)

    # Initial conditions
    error[0] = 50  # Initial lateral deviation in pixels
    integral_error = 0
    derivative_error = 0

    for i in range(1, len(time)):
        # PID computation
        derivative_error = (error[i-1] - error[i-2]) / dt if i > 1 else 0
        integral_error += error[i-1] * dt

        correction = -Kp * error[i-1] - Kd * derivative_error - Ki * integral_error

        # Update error based on correction (simplified dynamics)
        error[i] = error[i-1] + correction * dt

    return time, error, derivative_error

# Analyze data from simulation logs
def analyze_simulation_logs(csv_file):
    """Analyze logged simulation data for validation."""
    df = pd.read_csv(csv_file)

    # Extract relevant columns
    time = df['Time']
    lateral_error = df['Lateral Error']
    pid_correction = df['PID Correction']

    # Plot lateral error over time
    plt.figure(figsize=(10, 6))
    plt.plot(time, lateral_error, label='Lateral Error', color='blue')
    plt.axhline(0, color='black', linestyle='--', label='Lane Center')
    plt.xlabel('Time (s)')
    plt.ylabel('Lateral Error (pixels)')
    plt.title('Lateral Error Over Time')
    plt.legend()
    plt.grid(True)
    plt.show()

    # Summary statistics
    max_error = lateral_error.abs().max()
    mean_error = lateral_error.abs().mean()
    response_times = np.diff(np.where(pid_correction != 0)[0]) * (time[1] - time[0])
    avg_response_time = np.mean(response_times) if len(response_times) > 0 else None

    print(f"Max Error: {max_error:.2f} pixels")
    print(f"Mean Error: {mean_error:.2f} pixels")
    print(f"Average Response Time: {avg_response_time:.4f} seconds")

# New feature: Compute Lyapunov function and its derivative over logged data
def compute_lyapunov_from_logs(time, error):
    """Compute V(e) and dV(e) over time using logged data."""
    derivative_error = np.gradient(error, time)  # Compute de/dt
    V = 0.5 * error**2
    dV = error * (-0.06 * derivative_error - 1.0 * error)

    # Plot results
    plt.figure(figsize=(10, 6))

    # Plot V(e)
    plt.subplot(2, 1, 1)
    plt.plot(time, V, label='Lyapunov Function (V)', color='green')
    plt.xlabel('Time (s)')
    plt.ylabel('V(e)')
    plt.title('Lyapunov Function Over Logged Data')
    plt.legend()
    plt.grid(True)

    # Plot dV(e)
    plt.subplot(2, 1, 2)
    plt.plot(time, dV, label='Derivative of Lyapunov Function (dV)', color='purple')
    plt.axhline(0, color='black', linestyle='--', label='Stability Threshold (dV=0)')
    plt.xlabel('Time (s)')
    plt.ylabel('dV(e)')
    plt.title('Derivative of Lyapunov Function Over Logged Data')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

def main():
    # Lyapunov analysis
    V, dV = define_lyapunov_analysis()
    print("Lyapunov Function:", V)
    print("Derivative of Lyapunov Function:", dV)
    
    # Stability validation
    e, de = sp.symbols('e de', real=True)
    condition = sp.simplify(dV.subs({'e': e, 'de': de}))
    print("Simplified Derivative of Lyapunov Function:", condition)

    # Check sign of dV symbolically
    if sp.simplify(condition).has(e):  # Checks if condition depends on 'e'
        print("The derivative of the Lyapunov function depends on error (e).")
        print("Inspect specific ranges of e and de to confirm stability.")
    else:
        print("The system stability can be concluded directly.")

    # Simulate error dynamics
    time, error, derivative_error = simulate_lateral_error()

    # Compute Lyapunov function and its derivative over logged data
    csv_file = "task7_log.csv"  # Adjust this to your file path
    df = pd.read_csv(csv_file)
    time_logged = df['Time'].to_numpy()
    lateral_error_logged = df['Lateral Error'].to_numpy()
    compute_lyapunov_from_logs(time_logged, lateral_error_logged)

    # Plot simulation results
    plt.figure(figsize=(10, 6))
    plt.plot(time, error, label='Lateral Error', color='red')
    plt.axhline(0, color='black', linestyle='--', label='Lane Center')
    plt.xlabel('Time (s)')
    plt.ylabel('Lateral Error (pixels)')
    plt.title('Lateral Error Convergence')
    plt.legend()
    plt.grid(True)
    plt.show()

    # Analyze simulation logs
    analyze_simulation_logs(csv_file)

if __name__ == "__main__":
    main()
