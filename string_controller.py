import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

# Constants
R = 4.5  # Radius of the sphere
D = 0.25  # Plane cuts the sphere at z = D

# Compute the center of the segment
segment_center = np.array([0, 0, D])

# Compute the three fixed points P1, P2, P3 (120° apart)
theta1 = np.radians(0)
theta2 = np.radians(120)
theta3 = np.radians(240)

segment_radius = np.sqrt(R**2 - D**2)  # Radius of the spherical segment

p1 = np.array([segment_radius * np.cos(theta1), segment_radius * np.sin(theta1), D])
p2 = np.array([segment_radius * np.cos(theta2), segment_radius * np.sin(theta2), D])
p3 = np.array([segment_radius * np.cos(theta3), segment_radius * np.sin(theta3), D])

# Function to compute point X on the curved surface of the segment
def compute_x(alpha, beta):
    alpha_rad = np.radians(alpha)
    beta_rad = np.radians(beta)
    
    x_x = R * np.sin(alpha_rad) * np.cos(beta_rad)
    x_y = R * np.sin(alpha_rad) * np.sin(beta_rad)
    x_z = R * np.cos(alpha_rad)
    
    # Ensure X is within the spherical segment
    if x_z < D:
        return None  # Invalid position

    return np.array([x_x, x_y, x_z])

# Function to compute geodesic distance along the sphere
def spherical_distance(p, x):
    dot_product = np.dot(p, x) / (np.linalg.norm(p) * np.linalg.norm(x))
    angle = np.arccos(np.clip(dot_product, -1.0, 1.0))  # Avoid numerical issues
    return R * angle  # Arc length (geodesic distance)

# Initial values for alpha and beta
alpha_init = 70
beta_init = 180

def map_range(x):
    return round(1.5 - 3 * ((x - 1.32) / 11),2)

# Create figure
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# Function to update plot dynamically
def update(val):
    ax.clear()
    
    alpha = slider_alpha.val
    beta = slider_beta.val
    
    x = compute_x(alpha, beta)
    if x is None:
        ax.set_title("Point X is outside the segment!", color='red')
        return
    
    # Compute distances
    d1 = spherical_distance(p1, x)
    d2 = spherical_distance(p2, x)
    d3 = spherical_distance(p3, x)
    
    # Print the coordinates of all points in the terminal
    print(f"Coordinates of Points:")
    print(f"P1: {p1}")
    print(f"P2: {p2}")
    print(f"P3: {p3}")
    print(f"Servos: {map_range(d1)}, {map_range(d2)}, {map_range(d3)}")
    print()

    # Plot sphere
    u, v = np.mgrid[0:2*np.pi:50j, 0:np.pi:25j]
    sphere_x = R * np.cos(u) * np.sin(v)
    sphere_y = R * np.sin(u) * np.sin(v)
    sphere_z = R * np.cos(v)
    ax.plot_surface(sphere_x, sphere_y, sphere_z, color='c', alpha=0.3)

    # Plot segment plane
    theta = np.linspace(0, 2*np.pi, 100)
    seg_x = segment_radius * np.cos(theta)
    seg_y = segment_radius * np.sin(theta)
    seg_z = np.full_like(seg_x, D)
    ax.plot(seg_x, seg_y, seg_z, 'g', linewidth=2)

    # Plot points
    ax.scatter(*p1, color='r', s=100, label=f"P1 (d={d1:.2f})")
    ax.scatter(*p2, color='b', s=100, label=f"P2 (d={d2:.2f})")
    ax.scatter(*p3, color='y', s=100, label=f"P3 (d={d3:.2f})")
    ax.scatter(*x, color='m', s=100, label="X (Moving)")

    # Plot geodesic lines (curved distance between points)
    ax.plot([p1[0], x[0]], [p1[1], x[1]], [p1[2], x[2]], 'r--')
    ax.plot([p2[0], x[0]], [p2[1], x[1]], [p2[2], x[2]], 'b--')
    ax.plot([p3[0], x[0]], [p3[1], x[1]], [p3[2], x[2]], 'y--')

    # Labels and limits
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    ax.set_xlim([-R, R])
    ax.set_ylim([-R, R])
    ax.set_zlim([-R, R])
    ax.set_title(f"Point X at (α={alpha:.1f}°, β={beta:.1f}°)", fontsize=12)
    ax.legend()

    plt.draw()

# Create sliders for alpha and beta
ax_alpha = plt.axes([0.2, 0.02, 0.65, 0.03])
ax_beta = plt.axes([0.2, 0.06, 0.65, 0.03])

slider_alpha = Slider(ax_alpha, "Alpha (°)", 0, 90, valinit=alpha_init)
slider_beta = Slider(ax_beta, "Beta (°)", 0, 360, valinit=beta_init)

# Update function when sliders change
slider_alpha.on_changed(update)
slider_beta.on_changed(update)

# Initial plot
update(None)
plt.show()
