import numpy as np
import matplotlib.pyplot as plt

# Mechanical parameters (adjust these to match your physical system)
J = 0.01         # Moment of inertia (kg*m^2)
B = 0.01         # Damping coefficient (N*m*s/rad) - reduced to allow bounce
Kt = 0.05        # Motor torque constant (N*m/A)
restitution = 0.4  # Coefficient of restitution for impact (0 = no bounce, 1 = perfect bounce)

# Active-damping control gains
Kp_damping = 0.03
Kd_damping = 0.04
Kp_reset = 0.01
Ki_reset = 0.00 # Zero for now, can tune
Kd_reset = 0.01

# Simulation parameters
dt = 0.001        # Time step (seconds)
total_time = 10.0  # Total simulation duration (seconds)
time = np.arange(0, total_time, dt)

# Initialize arrays for angular position and angular velocity
theta = np.zeros(len(time))
omega = np.zeros(len(time))

# Set initial position at 90 degrees (pi/2 radians)
theta[0] = np.pi / 2

# Initialize phase and threshold
phase = 1
angular_position_threshold = 0.1

# Damping phase parameters
target_theta_damping = np.pi / 4 # Target end position after impact
damping_start_time = None
damping_settle_time = 3 # Time to wait before reset

# Reset phase parameters
reset_theta = np.pi / 2 # Reset position after impact
integral_error = 0 # Initalize integral error for reset phase

# Input torque initially applied
T_applied = -0.1

# Euler integration loop for mechanical dynamics
for i in range(1, len(time)):

    if phase == 1:
        # Apply constant torque until impact is detected (phase 1)
        T = T_applied
    elif phase == 2:
        if damping_start_time is None:
            damping_start_time = time[i]

        position_error = target_theta_damping - theta[i-1] # Comes from encoder data
        velocity_error = -omega[i-1] # Comes from IMU data
        T = Kp_damping * position_error + Kd_damping * velocity_error

        if (abs(target_theta_damping - theta[i-1]) <= angular_position_threshold and 
            time[i] - damping_start_time > damping_settle_time): # After impact, apply position-based damping (phase 2)
            phase = 3
    elif phase == 3: # After impact, apply reset torque (phase 3)
        position_error_reset = reset_theta - theta[i-1]
        velocity_error_reset = -omega[i-1]
        integral_error += position_error_reset * dt
        T = Kp_reset * position_error_reset + Ki_reset * integral_error + Kd_reset * velocity_error_reset

    omega_dot = (T - B * omega[i-1]) / J

    # Update angular velocity and position
    omega[i] = omega[i-1] + omega_dot * dt
    theta[i] = theta[i-1] + omega[i] * dt

    # Collision with drum at theta = 0 (impact model)
    if theta[i] < 0:
        theta[i] = 0
        omega[i] = -restitution * omega[i]  # reverse and damp velocity
        phase = 2 # Transition to damping phase

# Plotting the results
plt.figure(figsize=(12, 6))

# Angular position plot
plt.subplot(2, 1, 1)
plt.plot(time, theta, label='Angular Position (rad)')
plt.title('Gamelan Striker Dynamics with Impact Triggered Damping')
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
