import math
import numpy as np
import matplotlib.pyplot as plt

# --- Initial State ---
x = float(input("Enter x starting pos of the robot: "))
y = float(input("Enter y starting pos of the robot: "))
theta = float(input("Enter theta (in degrees): "))

theta = math.radians(theta)  # convert to radians
theta_start = theta  # store initial orientation for plotting

print("Start:", x, y, theta)

# --- Target ---
x_tar = float(input("Enter x target: "))
y_tar = float(input("Enter y target: "))

print("Target:", x_tar, y_tar)

# --- Parameters ---
dt = 0.1
k_v = 0.8        # linear gain
k_theta = 2.0    # angular gain
tolerance = 0.1

path_x = []
path_y = []
path_theta = []

# --- Control Loop ---
while True:
    # Distance error
    dx = x_tar - x
    dy = y_tar - y
    distance = math.sqrt(dx**2 + dy**2)

    if distance < tolerance:
        break

    # Desired angle
    alpha = math.atan2(dy, dx)

    # Angle error (normalized)
    theta_error = alpha - theta
    theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))

    # Control laws
    v = k_v * distance
    omega = k_theta * theta_error

    # Update state
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta += omega * dt

    # Store path
    path_x.append(x)
    path_y.append(y)
    path_theta.append(theta)

# --- Plot ---
plt.figure()

# Path
plt.plot(path_x, path_y, label="Robot Path")

# Target
plt.plot(x_tar, y_tar, 'ro', label="Target")

# --- START POSITION + ORIENTATION ---
start_x = path_x[0] if path_x else x
start_y = path_y[0] if path_y else y

# Mark start point
plt.plot(start_x, start_y, 'go', label="Start")

# Draw orientation arrow at start
arrow_length = 0.4
plt.arrow(start_x, start_y,
          arrow_length * math.cos(theta_start), 
          arrow_length * math.sin(theta_start),
          head_width=0.2, color='g')

# Plot arrows along the path
arrow_length = 0.2

for i in range(0, len(path_x), 1): 
    plt.arrow(path_x[i], path_y[i],
              arrow_length * math.cos(path_theta[i]),
              arrow_length * math.sin(path_theta[i]),
              head_width=0.15, head_length=0.2, fc='blue', ec='blue')

# --- END POSITION (optional) ---
plt.plot(x, y, 'kx', label="Final Position")

# Tolerance circle
ang = np.linspace(0, 2*np.pi, 100)
plt.plot(x_tar + 0.1*np.cos(ang),
         y_tar + 0.1*np.sin(ang), 'r--')

plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("Waypoint Navigation")
plt.axis("equal")
plt.grid()

plt.show()