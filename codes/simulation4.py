import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def closest_point_on_ellipsoid(center, semi_axes, point, plot=False):
    """
    Calculates the closest point on the ellipsoid, minimum distance, and pitch/roll/yaw.

    Parameters:
    center (tuple): Coordinates of the center of the ellipsoid (x_c, y_c, z_c).
    semi_axes (tuple): Semi-axes lengths of the ellipsoid (a, b, c).
    point (tuple): Coordinates of the target point (x0, y0, z0).
    plot (bool): Whether to plot the ellipsoid and points in 3D.

    Returns:
    dict: Contains closest point on the ellipsoid, minimum distance, and pitch/roll/yaw.
    """

    # Ellipsoid parameters (semi-axes lengths and center coordinates)
    a, b, c = semi_axes
    x_c, y_c, z_c = center

    # Point in 3D space (target point)
    x0, y0, z0 = point

    # Shift target point by the center of the ellipsoid
    x0_shifted, y0_shifted, z0_shifted = x0 - x_c, y0 - y_c, z0 - z_c

    # Calculate lambda (scaling factor)
    lambda_factor = np.sqrt((x0_shifted**2 / a**2) + (y0_shifted**2 / b**2) + (z0_shifted**2 / c**2))

    # Closest point on the ellipsoid
    x_min = x0_shifted / lambda_factor + x_c
    y_min = y0_shifted / lambda_factor + y_c
    z_min = z0_shifted / lambda_factor + z_c

    # Minimum distance
    min_distance = np.sqrt((x_min - x0)**2 + (y_min - y0)**2 + (z_min - z0)**2)

    # Calculate pitch, roll, yaw
    # Pitch: Rotation about the y-axis
    pitch = np.arctan2(np.sqrt((x0_shifted - x_min)**2 + (z0_shifted - z_min)**2), y0_shifted - y_min)

    # Roll: Rotation about the x-axis
    roll = np.arctan2(z_min - z_c, y_min - y_c)

    # Yaw: Rotation about the z-axis
    yaw = np.arctan2(y_min - y_c, x_min - x_c)

    # Results dictionary
    results = {
        "closest_point": (x_min, y_min, z_min),
        "min_distance": min_distance,
        "pitch": np.degrees(pitch),
        "roll": np.degrees(roll),
        "yaw": np.degrees(yaw)
    }

    if plot:
        # Create a meshgrid for the ellipsoid surface
        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x_ellipsoid = x_c + a * np.outer(np.cos(u), np.sin(v))
        y_ellipsoid = y_c + b * np.outer(np.sin(u), np.sin(v))
        z_ellipsoid = z_c + c * np.outer(np.ones_like(u), np.cos(v))

        # Plotting the ellipsoid, the point, and the closest point
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Plot the ellipsoid
        ax.plot_surface(x_ellipsoid, y_ellipsoid, z_ellipsoid, color='c', alpha=0.6, rstride=4, cstride=4, edgecolor='none')

        # Plot the original point
        ax.scatter(x0, y0, z0, color='r', s=100, label='Original Point')

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

    return results

# Example Usage:
center = (0, 0, 10)  # Center of the ellipsoid
semi_axes = (4, 4, 3)  # Semi-axes lengths (a, b, c)
target_point = (0, 0, 2)  # Target point in space
plot_3d = True  # Set to True to visualize the result

results = closest_point_on_ellipsoid(center, semi_axes, target_point, plot_3d)

# Output the results
print(f"The closest point on the ellipsoid is {results['closest_point']}")
print(f"The minimum distance from the point to the ellipsoid is {results['min_distance']:.2f}")
print(f"Pitch: {results['pitch']:.2f}°, Roll: {results['roll']:.2f}°, Yaw: {results['yaw']:.2f}°")