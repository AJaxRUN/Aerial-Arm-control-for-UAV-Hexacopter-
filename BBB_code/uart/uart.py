import serial
import struct
import time
import os
import sys

# UART5 port on BeagleBone
PORT = "/dev/ttyS5"
BAUD = 115200

# Expected start bytes
START1 = 0xAA
START2 = 0x55

# Packet format: 2B header + 7 floats + 1 uint8 + 2B checksum = 2 + 28 + 1 + 2 = 33 bytes
PACKET_SIZE = 33

# struct format: < = little-endian, f = float, B = uint8, H = uint16
# 2 start bytes handled manually
PACKET_FMT = "<fffffffBH"

def clear_line():
    sys.stdout.write("\033[2J\033[H")  # clear screen and move cursor to top-left
    sys.stdout.flush()

def read_packet(ser):
    # Look for start bytes
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == START1:
            b2 = ser.read(1)
            if b2 and b2[0] == START2:
                # Read rest of packet
                data = ser.read(PACKET_SIZE - 2)
                if len(data) == PACKET_SIZE - 2:
                    return data
    return None

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"❌ Failed to open {PORT}: {e}")
        sys.exit(1)

    print(f"✅ Listening on {PORT} ({BAUD} baud)...\n")
    time.sleep(1)

    while True:
        packet_data = read_packet(ser)
        if not packet_data:
            continue

        try:
            fields = struct.unpack(PACKET_FMT, packet_data)
            x, y, z, qw, qx, qy, qz, waypoint_state, checksum = fields

            clear_line()
            print("=== 🦾 ARM CONTROL PACKET ===\n")
            print(f"X: {x:8.3f}   Y: {y:8.3f}   Z: {z:8.3f}")
            print(f"QW: {qw:8.3f}  QX: {qx:8.3f}  QY: {qy:8.3f}  QZ: {qz:8.3f}")
            print(f"Waypoint State: {waypoint_state}")
            print(f"Checksum: {checksum:04X}")
            print("\n(Press Ctrl+C to exit)")

        except struct.error:
            continue
        except KeyboardInterrupt:
            break

    ser.close()
    print("\n✅ Serial connection closed.")

if __name__ == "__main__":
    main()

