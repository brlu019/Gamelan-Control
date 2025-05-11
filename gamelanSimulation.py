import numpy as np
import matplotlib.pyplot as plt

# Mechanical parameters (adjust these to match your physical system)
J = 0.01         # Moment of inertia (kg*m^2)
B = 0.02         # Damping coefficient (N*m*s/rad) - reduced to allow bounce
Kt = 0.05        # Motor torque constant (N*m/A)
restitution = 0.5  # Coefficient of restitution for impact (0 = no bounce, 1 = perfect bounce)

# Simulation parameters
dt = 0.001        # Time step (seconds)
total_time = 2.0  # Total simulation duration (seconds)
time = np.arange(0, total_time, dt)

# Define input torque as a function of time (stronger step torque)
def input_torque(t):
    if t < 0.5:
        return -0.2  # Increased torque to ensure striker reaches drum
    else:
        return 0.0   # Torque off

# Initialize arrays for angular position and angular velocity
theta = np.zeros(len(time))
omega = np.zeros(len(time))

# Set initial position at 90 degrees (pi/2 radians)
theta[0] = np.pi / 2

# Euler integration loop for mechanical dynamics
for i in range(1, len(time)):
    T = input_torque(time[i-1])

    # Compute angular acceleration
    omega_dot = (T - B * omega[i-1]) / J

    # Update angular velocity and position
    omega[i] = omega[i-1] + omega_dot * dt
    theta[i] = theta[i-1] + omega[i] * dt

    # Collision with drum at theta = 0 (impact model)
    if theta[i] < 0:
        theta[i] = 0
        omega[i] = -restitution * omega[i]  # reverse and damp velocity

# Plotting the results
plt.figure(figsize=(12, 6))

# Angular position plot
plt.subplot(2, 1, 1)
plt.plot(time, theta, label='Angular Position (rad)')
plt.title('Gamelan Striker Dynamics with Impact')
plt.xlabel('Time (s)')
plt.ylabel('Angular Position (rad)')
plt.grid(True)
plt.legend()

# Angular velocity plot
plt.subplot(2, 1, 2)
plt.plot(time, omega, label='Angular Velocity (rad/s)', color='orange')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
