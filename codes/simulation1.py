import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Before we will do linear transformation to set cordinate space
# from the one of the optitrack to the one of the robot


# Parameters
r = 5  # Radius of the working sphere
target = np.array([5, 4, 2])  # Target coordinates (x, y, z)

# Calculate intersection point with the sphere
distance = np.linalg.norm(target)  # Calculate the norm (magnitude) of the target vector
scaling_factor = r / distance  # Calculate the scaling factor to bring the point onto the sphere
intersection = scaling_factor * target  # Scale the target vector to get the intersection coordinates on the sphere

# Calculate orientation (yaw, pitch, roll) to point the end effector towards the target
yaw = np.arctan2(target[1], target[0])  # Yaw: rotation around the Z-axis (from X to Y)
pitch = np.arctan2(target[2], np.sqrt(target[0]**2 + target[1]**2))  # Pitch: rotation around the Y-axis (from Z to X-Y plane)
roll = 0  # Roll: rotation around the X-axis, assuming no roll is needed

# Print results to the console
print(f"Intersection Point: {intersection}")
print(f"Yaw: {np.degrees(yaw)}°")
print(f"Pitch: {np.degrees(pitch)}°")
print(f"Roll: {roll}°")

# Plotting the results
fig = plt.figure()  # Create a new figure
ax = fig.add_subplot(111, projection='3d')  # Add a 3D subplot

# Plot the sphere representing the working space of the robot
u = np.linspace(0, 2 * np.pi, 100)  # Parametric variable for the sphere's longitude
v = np.linspace(0, np.pi, 100)  # Parametric variable for the sphere's latitude
x = r * np.outer(np.cos(u), np.sin(v))  # X coordinates of the sphere
y = r * np.outer(np.sin(u), np.sin(v))  # Y coordinates of the sphere
z = r * np.outer(np.ones(np.size(u)), np.cos(v))  # Z coordinates of the sphere
ax.plot_surface(x, y, z, color='b', alpha=0.1)  # Plot the surface of the sphere with light blue color and transparency

# Plot the target point in red
ax.scatter(target[0], target[1], target[2], color='r', label='Target')

# Plot the intersection point (end effector position) in green
ax.scatter(intersection[0], intersection[1], intersection[2], color='g', label='Intersection')

# Plot the line from the robot base (origin) to the target point
ax.plot([0, target[0]], [0, target[1]], [0, target[2]], color='k', linestyle='--')  # Dashed line in black

# Plot the line representing the robot arm from the base to the intersection point
ax.plot([0, intersection[0]], [0, intersection[1]], [0, intersection[2]], color='g')  # Solid line in green

# Add labels to the axes
ax.set_xlabel('X')  # Label for the X-axis
ax.set_ylabel('Y')  # Label for the Y-axis
ax.set_zlabel('Z')  # Label for the Z-axis

# Add a legend to the plot
ax.legend()

# Show the plot
plt.show()
