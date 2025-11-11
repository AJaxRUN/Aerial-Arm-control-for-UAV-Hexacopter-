// full_actuation.c
// Port of full_actuation.py to C, with servo actuation using rc/servo.h

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <rc/servo.h>
#include <rc/time.h>

#define PORT_PATH "/dev/ttyS5"
#define BAUD_RATE B115200
#define START1 0xAA
#define START2 0x55
#define PACKET_SIZE 33  // includes 2 start bytes

static volatile int running = 1;
static const double PI = 3.14159265358979323846;
static const double R = 4.5;
static const double D = 2.5;

static void __signal_handler(int dummy){
    (void)dummy;
    running = 0;
}

static int open_serial(const char *path){
    int fd = open(path, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if(fd < 0) return -1;

    struct termios tty;
    if(tcgetattr(fd, &tty) != 0){
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);
    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);

    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5; // 0.5s

    if(tcsetattr(fd, TCSANOW, &tty) != 0){
        close(fd);
        return -1;
    }

    return fd;
}

// Read exactly len bytes into buf, return 0 on success, -1 on EOF/error
static int read_full(int fd, uint8_t *buf, size_t len){
    size_t off = 0;
    while(off < len && running){
        ssize_t r = read(fd, buf + off, len - off);
        if(r > 0) off += r;
        else if(r == 0) return -1;
        else {
            if(errno == EAGAIN || errno == EWOULDBLOCK){
                rc_usleep(10000); // 10ms
                continue;
            }
            return -1;
        }
    }
    return (off == len) ? 0 : -1;
}

static float le_bytes_to_float(const uint8_t *b){
    uint32_t u = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

static double deg2rad(double d){ return d * PI / 180.0; }
static double rad2deg(double r){ return r * 180.0 / PI; }

static void compute_x(double alpha_deg, double beta_deg, double out[3]){
    double alpha = deg2rad(alpha_deg);
    double beta  = deg2rad(beta_deg);
    out[0] = R * sin(alpha) * cos(beta);
    out[1] = R * sin(alpha) * sin(beta);
    out[2] = R * cos(alpha);
}

static double vec_norm(const double v[3]){
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static double dot3(const double a[3], const double b[3]){
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static double spherical_distance(const double p[3], const double x[3]){
    double dot = dot3(p,x) / (vec_norm(p) * vec_norm(x));
    if(dot > 1.0) dot = 1.0;
    if(dot < -1.0) dot = -1.0;
    double angle = acos(dot);
    return R * angle;
}

static float map_to_servo(double distance){
    const float servo_min = -1.5f;
    const float servo_max = 0.3f;
    double d_min = 0.0;
    double d_max = 2.0 * R * acos(D / R);
    double scaled = (distance - d_min) / (d_max - d_min);
    if(scaled < 0.0) scaled = 0.0;
    if(scaled > 1.0) scaled = 1.0;
    double val = servo_min + (servo_max - servo_min) * scaled;
    if(val < -1.5) val = -1.5;
    if(val > 1.5) val = 1.5;
    // round to 2 decimals
    double rv = round(val * 100.0) / 100.0;
    return (float)rv;
}

// quaternion (qw,qx,qy,qz) -> roll,pitch,yaw in degrees
static void quat_to_euler(double qw,double qx,double qy,double qz, double *roll, double *pitch, double *yaw){
    double sinr = 2.0 * (qw * qx + qy * qz);
    double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
    double r = atan2(sinr, cosr);

    double sinp = 2.0 * (qw * qy - qz * qx);
    double p;
    if(fabs(sinp) >= 1.0) p = copysign(PI / 2.0, sinp);
    else p = asin(sinp);

    double siny = 2.0 * (qw * qz + qx * qy);
    double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
    double y = atan2(siny, cosy);

    *roll = r; *pitch = p; *yaw = y;
}

int main(int argc, char *argv[]){
    (void)argc; (void)argv;

    signal(SIGINT, __signal_handler);

    int fd = open_serial(PORT_PATH);
    if(fd < 0){
        fprintf(stderr, "Failed to open %s: %s\n", PORT_PATH, strerror(errno));
        return 1;
    }

    // prepare points p1,p2,p3 (same as python)
    double seg_r = sqrt(R*R - D*D);
    double p1[3] = { seg_r * cos(deg2rad(0)),   seg_r * sin(deg2rad(0)),   D };
    double p2[3] = { seg_r * cos(deg2rad(120)), seg_r * sin(deg2rad(120)), D };
    double p3[3] = { seg_r * cos(deg2rad(240)), seg_r * sin(deg2rad(240)), D };

    printf("Listening on %s (115200 baud)...\n", PORT_PATH);

    // initialize servos
    printf("Initializing servo system...\n");
    if(rc_servo_init() != 0){
        fprintf(stderr, "ERROR: Failed to initialize servos\n");
        // continue but warn
    }
    // Enable power rails that may be used by channels. Some boards expose multiple rails.
    if(rc_servo_power_rail_en(1) != 0 || rc_servo_power_rail_en(2) != 0 || rc_servo_power_rail_en(3) != 0){
        fprintf(stderr, "WARNING: Failed to enable one or more servo power rails (1..3)\n");
    }

    // buffer-based, low-latency read: read available bytes, process packets immediately
    uint8_t buf[512];
    size_t buf_len = 0;
    uint8_t payload[PACKET_SIZE - 2]; // 31 bytes
    float last_s1 = 0.0f, last_s2 = 0.0f, last_s3 = 0.0f;

    struct timespec last_send_time;
    clock_gettime(CLOCK_MONOTONIC, &last_send_time);

    while(running){
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv;
        // tiny timeout for low latency (5 ms)
        tv.tv_sec = 0; tv.tv_usec = 5000;
        int sel = select(fd + 1, &rfds, NULL, NULL, &tv);
        if(sel > 0 && FD_ISSET(fd, &rfds)){
            ssize_t n = read(fd, buf + buf_len, sizeof(buf) - buf_len);
            if(n > 0) buf_len += (size_t)n;
            else if(n < 0){
                if(errno != EAGAIN && errno != EWOULDBLOCK){
                    // non-recoverable read error
                    break;
                }
            }
        }

        // process complete packets immediately
        size_t i = 0;
        while(buf_len >= PACKET_SIZE){
            // find start sequence in buffer
            if(buf[i] != START1 || buf[i+1] != START2){
                // drop first byte and continue searching
                memmove(buf, buf+1, buf_len-1);
                buf_len -= 1;
                continue;
            }

            // have full packet at buf[0..PACKET_SIZE-1]
            memcpy(payload, buf + 2, PACKET_SIZE - 2);

            // parse 7 floats + uint8 + uint16 (little-endian)
            float x = le_bytes_to_float(payload + 0);
            float y = le_bytes_to_float(payload + 4);
            float z = le_bytes_to_float(payload + 8);
            float qw = le_bytes_to_float(payload + 12);
            float qx = le_bytes_to_float(payload + 16);
            float qy = le_bytes_to_float(payload + 20);
            float qz = le_bytes_to_float(payload + 24);
            uint8_t waypoint_state = payload[28];
            uint16_t checksum = (uint16_t)payload[29] | ((uint16_t)payload[30] << 8);

            // quaternion -> Euler
            double roll_rad, pitch_rad, yaw_rad;
            quat_to_euler(qw, qx, qy, qz, &roll_rad, &pitch_rad, &yaw_rad);
            double roll = rad2deg(roll_rad);
            double pitch = rad2deg(pitch_rad);
            double yaw = rad2deg(yaw_rad);

            // compensation
            double k_pitch = 0.8, k_roll = 0.8, k_yaw = 0.3;
            double alpha = -pitch * k_pitch;
            double beta  = roll * k_roll;

            double yawr = deg2rad(yaw);
            alpha += k_yaw * sin(yawr);
            beta  += k_yaw * cos(yawr);

            double cosY = cos(yawr), sinY = sin(yawr);
            double alpha_rot = alpha * cosY - beta * sinY;
            double beta_rot  = alpha * sinY + beta * cosY;

            double x_vec[3];
            compute_x(alpha_rot, beta_rot, x_vec);

            if(x_vec[2] >= D){
                double d1 = spherical_distance(p1, x_vec);
                double d2 = spherical_distance(p2, x_vec);
                double d3 = spherical_distance(p3, x_vec);

                last_s1 = map_to_servo(d1);
                last_s2 = map_to_servo(d2);
                last_s3 = map_to_servo(d3);
            }

            // send immediately using most recent computed values
            if(rc_servo_send_pulse_normalized(6, last_s1) != 0 ||
               rc_servo_send_pulse_normalized(7, last_s2) != 0 ||
               rc_servo_send_pulse_normalized(8, last_s3) != 0){
                // non-fatal: continue
            }

            // remove this packet from buffer
            memmove(buf, buf + PACKET_SIZE, buf_len - PACKET_SIZE);
            buf_len -= PACKET_SIZE;
        }

        // if no incoming packet for >20ms, resend last values to maintain servo hold
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - last_send_time.tv_sec) * 1000 + (now.tv_nsec - last_send_time.tv_nsec) / 1000000;
        if(elapsed_ms > 20){
            rc_servo_send_pulse_normalized(6, last_s1);
            rc_servo_send_pulse_normalized(7, last_s2);
            rc_servo_send_pulse_normalized(8, last_s3);
            last_send_time = now;
        }
    }

    close(fd);
    printf("\nSerial connection closed.\n");

    printf("Disabling servo power rail...\n");
    rc_servo_power_rail_en(0);
    printf("Cleaning up servo system...\n");
    rc_servo_cleanup();

    printf("Program completed.\n");
    return 0;
}

