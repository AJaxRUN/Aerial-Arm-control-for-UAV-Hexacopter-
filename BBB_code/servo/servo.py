import numpy as np
import subprocess

# Constants
R = 4.5  # Radius of the sphere
D = 0.25  # Plane cuts the sphere at z = D

# Compute the center of the segment
segment_center = np.array([0, 0, D])

# Compute the three fixed points P1, P2, P3 (120 apart)
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
alpha = 70
beta = 180

def map_range(x):
    return round(1.5 - 3 * ((x - 1.32) / 11),2)


def set_servo_positions(val1, val2, val3):
    try:
        result = subprocess.run(
            ['sudo','./servo', str(val1), str(val2), str(val3)],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        print("Servo program output:")
        print(result.stdout)
    except subprocess.CalledProcessError as e:
        print("Error calling servo_control:")
        print(e.stderr)


while True:   
    x = compute_x(alpha, beta)
    
    # Compute distances
    d1 = spherical_distance(p1, x)
    d2 = spherical_distance(p2, x)
    d3 = spherical_distance(p3, x)

    s1 = map_range(d1)
    s2 = map_range(d2)
    s3 = map_range(d3)
    
    print("Servos: {s1}, {s2}, {s3}")
    
    set_servo_positions(s1, s2, s3)
    print()
    
    alpha = int(input())
    beta = int(input())

