import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Ellipsoid parameters
a, b, c = 3, 2, 1  # semi-axes of the ellipsoid

# Point in 3D space
x0, y0, z0 = 5, 4, 2

# System of equations (Lagrange multiplier system)
def equations(vars):
    x, y, z, lambd = vars
    eq1 = 2 * (x - x0) + 2 * lambd * (x / a**2)
    eq2 = 2 * (y - y0) + 2 * lambd * (y / b**2)
    eq3 = 2 * (z - z0) + 2 * lambd * (z / c**2)
    eq4 = (x**2 / a**2) + (y**2 / b**2) + (z**2 / c**2) - 1  # ellipsoid equation
    return [eq1, eq2, eq3, eq4]

# Initial guess for the solver
initial_guess = [1, 1, 1, 1]

# Solve the system of equations
solution = fsolve(equations, initial_guess)
x_min, y_min, z_min = solution[0], solution[1], solution[2]

# Minimum distance
min_distance = np.sqrt((x_min - x0)**2 + (y_min - y0)**2 + (z_min - z0)**2)

print(f"The closest point on the ellipsoid is ({x_min:.2f}, {y_min:.2f}, {z_min:.2f})")
print(f"The minimum distance from the point to the ellipsoid is {min_distance:.2f}")

# Create a meshgrid for the ellipsoid surface
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
x_ellipsoid = a * np.outer(np.cos(u), np.sin(v))
y_ellipsoid = b * np.outer(np.sin(u), np.sin(v))
z_ellipsoid = c * np.outer(np.ones_like(u), np.cos(v))

# Plotting the ellipsoid, the point, and the closest point
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot the ellipsoid
ax.plot_surface(x_ellipsoid, y_ellipsoid, z_ellipsoid, color='c', alpha=0.6, rstride=4, cstride=4, edgecolor='none')

# Plot the original point
ax.scatter(x0, y0, z0, color='r', s=100, label='Original Point (x0, y0, z0)')

# Plot the closest point on the ellipsoid
ax.scatter(x_min, y_min, z_min, color='b', s=100, label='Closest Point on Ellipsoid')

# Plot a line connecting the point to the ellipsoid
ax.plot([x0, x_min], [y0, y_min], [z0, z_min], color='g', linestyle='--', label='Minimum Distance')

# Labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Minimum Distance from Point to Ellipsoid')

# Set the aspect ratio for better visualization
ax.set_box_aspect([a, b, c])

# Add a legend
ax.legend()

plt.show()
