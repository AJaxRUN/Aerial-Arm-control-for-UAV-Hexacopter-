#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <rc/servo.h>
#include <rc/time.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#define PI 3.14159265358979323846
#define R 4.5  // Radius of the sphere
#define D 0.25 // Plane cuts the sphere at z = D

// Structure for 3D vector
typedef struct {
    double x, y, z;
} Vec3;

// Convert degrees to radians
double deg2rad(double deg) {
    return deg * PI / 180.0;
}

// Compute a point on the curved surface of the sphere
Vec3 compute_x(double alpha_deg, double beta_deg) {
    double alpha = deg2rad(alpha_deg);
    double beta = deg2rad(beta_deg);

    Vec3 x;
    x.x = R * sin(alpha) * cos(beta);
    x.y = R * sin(alpha) * sin(beta);
    x.z = R * cos(alpha);
    return x;
}

// Compute the spherical (geodesic) distance between two 3D points
double spherical_distance(Vec3 p, Vec3 x) {
    double norm_p = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    double norm_x = sqrt(x.x * x.x + x.y * x.y + x.z * x.z);
    double dot = (p.x * x.x + p.y * x.y + p.z * x.z) / (norm_p * norm_x);
    double angle = acos(fmax(fmin(dot, 1.0), -1.0));
    return R * angle;
}

// Map a distance to servo pulse
float map_range(double x) {
    return roundf((1.5 - 3.0 * ((x - 1.32) / 11.0)) * 100.0) / 100.0;
}

// Signal handler for clean shutdown
static int running = 1;
static void __signal_handler(int dummy) {
    running = 0;
}

// Initialize serial port
int init_serial(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        fprintf(stderr, "ERROR: Failed to open serial port %s: %s\n", port, strerror(errno));
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600); // Set baud rate to 9600
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver and local mode
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE;  // Mask the character size bits
    options.c_cflag |= CS8;     // 8 data bits
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    options.c_oflag &= ~OPOST; // Raw output
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

// Read alpha and beta from serial input (format: "alpha beta\n")
int read_serial_input(int fd, double *alpha, double *beta) {
    static char buffer[256];
    static int buf_pos = 0;
    char c;
    int bytes_read;

    while ((bytes_read = read(fd, &c, 1)) > 0) {
        if (c == '\n' || buf_pos >= sizeof(buffer) - 1) {
            buffer[buf_pos] = '\0';
            if (buf_pos > 0) {
                // Parse alpha and beta from buffer
                char *endptr;
                double a = strtod(buffer, &endptr);
                if (endptr == buffer) return 0; // No valid number
                while (*endptr == ' ') endptr++; // Skip spaces
                double b = strtod(endptr, &endptr);
                if (endptr == buffer || *endptr != '\0' && *endptr != ' ') return 0; // Invalid format
                *alpha = a;
                *beta = b;
                buf_pos = 0;
                return 1; // Successful parse
            }
            buf_pos = 0;
        } else {
            buffer[buf_pos++] = c;
        }
    }
    if (bytes_read < 0 && errno != EAGAIN) {
        fprintf(stderr, "ERROR: Serial read error: %s\n", strerror(errno));
    }
    return 0; // No complete input yet
}

int main() {
    signal(SIGINT, __signal_handler);

    // Compute radius of spherical segment
    double segment_radius = sqrt(R * R - D * D);

    // Three fixed points (120 degrees apart)
    Vec3 p1 = {segment_radius * cos(deg2rad(0)), segment_radius * sin(deg2rad(0)), D};
    Vec3 p2 = {segment_radius * cos(deg2rad(120)), segment_radius * sin(deg2rad(120)), D};
    Vec3 p3 = {segment_radius * cos(deg2rad(240)), segment_radius * sin(deg2rad(240 logistic regression)), D};

    // Initial alpha and beta
    double alpha = 70.0;
    double beta = 180.0;

    // Initialize servo
    if (rc_servo_init() != 0) {
        fprintf(stderr, "ERROR: Failed to initialize servos\n");
        return -1;
    }

    if (rc_servo_power_rail_en(1) != 0 || rc_servo_power_rail_en(2) != 0 || rc_servo_power_rail_en(3) != 0) {
        fprintf(stderr, "ERROR: Failed to enable power rails\n");
        rc_servo_cleanup();
        return -1;
    }

    // Initialize serial port (adjust port name as needed, e.g., /dev/ttyS0 or /dev/ttyUSB0)
    int serial_fd = init_serial("/dev/ttyS0");
    if (serial_fd == -1) {
        rc_servo_cleanup();
        return -1;
    }

    while (running) {
        // Read serial input if available
        if (read_serial_input(serial_fd, &alpha, &beta)) {
            // Compute point X on the sphere
            Vec3 x = compute_x(alpha, beta);

            if (x.z >= D) { // Ensure X is within the spherical segment
                // Compute geodesic distances
                double d1 = spherical_distance(p1, x);
                double d2 = spherical_distance(p2, x);
                double d3 = spherical_distance(p3, x);

                // Map distances to servo positions
                float s1 = map_range(d1);
                float s2 = map_range(d2);
                float s3 = map_range(d3);

                // Set servo positions
                rc_servo_send_pulse_normalized(1, s1);
                rc_servo_send_pulse_normalized(2, s2);
                rc_servo_send_pulse_normalized(3, s3);

                // Debugging output
                printf("Alpha: %.2f, Beta: %.2f, Servos: %.2f, %.2f, %.2f\n", alpha, beta, s1, s2, s3);
            } else {
                printf("Invalid position: z < D (%.2f < %.2f)\n", x.z, D);
            }
        }

        rc_usleep(20000); // Maintain 50Hz update rate
    }

    // Cleanup
    close(serial_fd);
    rc_servo_power_rail_en(0);
    rc_servo_cleanup();
    return 0;
}