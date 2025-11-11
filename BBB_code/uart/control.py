#!/usr/bin/env python3
import serial
import struct
import time
import math
import sys
import numpy as np
import signal

PORT = "/dev/ttyS5"
BAUD = 115200
START1 = 0xAA
START2 = 0x55
PACKET_SIZE = 33
PACKET_FMT = "<fffffffBH"

PI = math.pi
R = 4.5
D = 2.5

running = True
def sigint_handler(sig, frame):
    global running
    running = False
signal.signal(signal.SIGINT, sigint_handler)

# servo send placeholder
def send_servo(channel, value):
    print(f"[servo] ch{channel} -> {value:.2f}")

def deg2rad(d): return d * PI / 180.0
def rad2deg(r): return r * 180.0 / PI

def compute_x(alpha_deg, beta_deg):
    alpha = deg2rad(alpha_deg)
    beta  = deg2rad(beta_deg)
    return np.array([
        R * math.sin(alpha) * math.cos(beta),
        R * math.sin(alpha) * math.sin(beta),
        R * math.cos(alpha)
    ])

def spherical_distance(p, x):
    dot = np.dot(p, x) / (np.linalg.norm(p) * np.linalg.norm(x))
    dot = float(np.clip(dot, -1.0, 1.0))
    angle = math.acos(dot)
    return R * angle

def map_to_servo(distance):
    servo_min, servo_max = -1.5, 0.3
    d_min, d_max = 0.0, 2.0 * R * math.acos(D / R)
    scaled = (distance - d_min) / (d_max - d_min)
    scaled = min(max(scaled, 0.0), 1.0)
    val = servo_min + (servo_max - servo_min) * scaled
    val = min(max(val, -1.5), 1.5)
    return round(val, 2)

def quat_to_euler(qw, qx, qy, qz):
    sinr = 2 * (qw * qx + qy * qz)
    cosr = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)

    sinp = 2 * (qw * qy - qz * qx)
    pitch = math.copysign(PI / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny = 2 * (qw * qz + qx * qy)
    cosy = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)

    return roll, pitch, yaw

def read_packet(ser):
    while True:
        b = ser.read(1)
        if not b: return None
        if b[0] == START1:
            b2 = ser.read(1)
            if b2 and b2[0] == START2:
                data = ser.read(PACKET_SIZE - 2)
                if len(data) == PACKET_SIZE - 2:
                    return data
    return None

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
    except Exception as e:
        print(f"❌ Failed to open {PORT}: {e}")
        sys.exit(1)

    seg_r = math.sqrt(R*R - D*D)
    p1 = np.array([seg_r*math.cos(deg2rad(0)),   seg_r*math.sin(deg2rad(0)),   D])
    p2 = np.array([seg_r*math.cos(deg2rad(120)), seg_r*math.sin(deg2rad(120)), D])
    p3 = np.array([seg_r*math.cos(deg2rad(240)), seg_r*math.sin(deg2rad(240)), D])

    print(f"✅ Listening on {PORT} ({BAUD} baud)...\n")

    while running:
        packet_data = read_packet(ser)
        if not packet_data:
            continue

        try:
            x, y, z, qw, qx, qy, qz, waypoint_state, checksum = struct.unpack(PACKET_FMT, packet_data)

            # quaternion -> Euler
            roll, pitch, yaw = quat_to_euler(qw, qx, qy, qz)
            roll = rad2deg(roll)
            pitch = rad2deg(pitch)
            yaw = rad2deg(yaw)

            # compensation
            k_pitch, k_roll, k_yaw = 0.8, 0.8, 0.3
            alpha = -pitch * k_pitch
            beta  = roll * k_roll

            yaw_rad = deg2rad(yaw)
            alpha += k_yaw * math.sin(yaw_rad)
            beta  += k_yaw * math.cos(yaw_rad)

            cosY, sinY = math.cos(yaw_rad), math.sin(yaw_rad)
            alpha_rot = alpha * cosY - beta * sinY
            beta_rot  = alpha * sinY + beta * cosY

            x_vec = compute_x(alpha_rot, beta_rot)
            print("calculated x: ",x_vec[2])
            print("x > D: ",x_vec[2] >= D)

            if x_vec[2] >= D:
                print("In here")
                d1 = spherical_distance(p1, x_vec)
                d2 = spherical_distance(p2, x_vec)
                d3 = spherical_distance(p3, x_vec)

                s1 = map_to_servo(d1)
                s2 = map_to_servo(d2)
                s3 = map_to_servo(d3)
                print("Ss", s1, s2, s3)

                send_servo(6, s1)
                send_servo(7, s2)
                send_servo(8, s3)
            else:
                print("[INFO] x_vec.z < D, skipping actuation")

            # debug print
            print(f"Waypoint: {waypoint_state}, Pos: ({x:.3f},{y:.3f},{z:.3f}), Euler: ({roll:.2f},{pitch:.2f},{yaw:.2f})")

            time.sleep(0.02)

        except struct.error:
            continue

    ser.close()
    print("\n✅ Serial connection closed.")

if __name__ == "__main__":
    main()

