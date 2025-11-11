#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <rc/servo.h>
#include <rc/time.h>
#include <sys/select.h>
#include <unistd.h>
#include <termios.h>
#include <float.h>

#define PI 3.14159265358979323846
#define R 4.5
#define D 2.5

// Helper to check if input is ready without blocking
int input_available() {
    struct timeval tv = {0, 0};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
}

// Disable canonical mode so input isn't line-buffered
void set_terminal_mode(int enable) {
    static struct termios oldt, newt;

    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON); // Disable buffering
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restore old settings
    }
}

static int running = 1;

static void __signal_handler(int dummy) {
    running = 0;
}

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
    double dot = (p.x * x.x + p.y * x.y + p.z * x.z) /
                 (sqrt(p.x * p.x + p.y * p.y + p.z * p.z) * sqrt(x.x * x.x + x.y * x.y + x.z * x.z));
    double angle = acos(fmax(fmin(dot, 1.0), -1.0));
    return R * angle;
}

static float map_to_servo(int servo_id, double distance) {
    // Servo-specific calibrated physical range
    float servo_min, servo_max;
    double d_min = 0.0, d_max = 2.0 * R * acos(D / R);

    switch (servo_id) {
        case 1:
            servo_min = -1.5f; servo_max = 0.3f;
            break;
        case 2:
            servo_min = -1.5f; servo_max = 0.3f;
            break;
        case 3:
            servo_min = -1.5f; servo_max = 0.3f;
            break;
        default:
            servo_min = -1.5f; servo_max = 1.5f;
    }

    // Shift distance relative to center
    double scaled = (distance - d_min) / (d_max - d_min);
    if (scaled < 0.0) scaled = 0.0;
    if (scaled > 1.0) scaled = 1.0;

    float servo_val = servo_max - (servo_max - servo_min) * (float)scaled;

    // Clamp and round
    if (servo_val > 1.5f) servo_val = 1.5f;
    if (servo_val < -1.5f) servo_val = -1.5f;
    servo_val = roundf(servo_val * 100.0f) / 100.0f;

    return servo_val;
}

int main() {
    signal(SIGINT, __signal_handler);

    // Compute radius of spherical segment
    double segment_radius = sqrt(R * R - D * D);

    // 120 degree separated points
    Vec3 p1 = {segment_radius * cos(deg2rad(0)), segment_radius * sin(deg2rad(0)), D};
    Vec3 p2 = {segment_radius * cos(deg2rad(120)), segment_radius * sin(deg2rad(120)), D};
    Vec3 p3 = {segment_radius * cos(deg2rad(240)), segment_radius * sin(deg2rad(240)), D};

    double alpha = 0;
    double beta = 0;

    if (rc_servo_init() != 0) {
        fprintf(stderr, "ERROR: Failed to initialize servos\n");
        return -1;
    }

    if (rc_servo_power_rail_en(1) != 0) {
        fprintf(stderr, "ERROR: Failed to enable power rail\n");
        rc_servo_cleanup();
        return -1;
    }

    char buffer[256] = {0};
    int buf_idx = 0;

    set_terminal_mode(1);
    while (running) {
        Vec3 x = compute_x(alpha, beta);
    
        if (x.z >= D) {
            double d1 = spherical_distance(p1, x);
            double d2 = spherical_distance(p2, x);
            double d3 = spherical_distance(p3, x);
    
            float s1 = map_to_servo(1, d1);
            float s2 = map_to_servo(2, d2);
            float s3 = map_to_servo(3, d3);
    
            rc_servo_send_pulse_normalized(6, s1);
            rc_servo_send_pulse_normalized(7, s2);
            rc_servo_send_pulse_normalized(8, s3);
        }
    
        // Check for user input
        while (input_available()) {
            int c = getchar();
            if (c == EOF) continue;
            buffer[buf_idx++] = (char)c;
            if (buf_idx >= 255) {
                buf_idx = 0;
            }
            if (c == '\n') {
                buffer[buf_idx] = '\0';
                sscanf(buffer, "%lf %lf", &alpha, &beta);
                buf_idx = 0;
            }
        }
    
        rc_usleep(20000); // Maintain 50Hz
    }
    
    set_terminal_mode(0);  
    rc_servo_power_rail_en(0);
    rc_servo_cleanup();
    return 0;
}
