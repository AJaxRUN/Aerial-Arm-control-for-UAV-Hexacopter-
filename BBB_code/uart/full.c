#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <rc/servo.h>
#include <rc/time.h>

// ===============================
// === CONSTANTS ===
// ===============================
#define START_BYTE1 0xAA
#define START_BYTE2 0x55
#define PI 3.14159265358979323846
#define R 4.5
#define D 2.5

// ===============================
// === PACKET STRUCT (MATCHES PYTHON) ===
// ===============================
#pragma pack(push, 1)
typedef struct {
    uint8_t start_byte1;     // 0xAA
    uint8_t start_byte2;     // 0x55
    float position[3];       // x, y, z
    float yaw;               // yaw
    uint8_t waypoint_state;  // state
    uint16_t checksum;       // Fletcher-16
} arm_control_packet_t;
#pragma pack(pop)

// ===============================
// === CHECKSUM ===
// ===============================
uint16_t fletcher16(uint8_t *data, size_t len) {
    uint16_t sum1 = 0, sum2 = 0;
    for (size_t i = 0; i < len; i++) {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}

// ===============================
// === UTILS ===
// ===============================
typedef struct { double x, y, z; } Vec3;

static double deg2rad(double d){ return d * PI / 180.0; }
static double rad2deg(double r){ return r * 180.0 / PI; }

Vec3 compute_x(double alpha_deg, double beta_deg){
    double alpha = deg2rad(alpha_deg);
    double beta  = deg2rad(beta_deg);
    Vec3 x;
    x.x = R * sin(alpha) * cos(beta);
    x.y = R * sin(alpha) * sin(beta);
    x.z = R * cos(alpha);
    return x;
}

double spherical_distance(Vec3 p, Vec3 x){
    double dot = (p.x*x.x + p.y*x.y + p.z*x.z) /
                 (sqrt(p.x*p.x + p.y*p.y + p.z*p.z) *
                  sqrt(x.x*x.x + x.y*x.y + x.z*x.z));
    double angle = acos(fmax(fmin(dot,1.0),-1.0));
    return R * angle;
}

float map_to_servo(int id, double distance){
    float servo_min=-1.5f, servo_max=0.3f;
    double d_min=0.0, d_max=2.0*R*acos(D/R);
    double scaled=(distance-d_min)/(d_max-d_min);
    if(scaled<0.0)scaled=0.0;
    if(scaled>1.0)scaled=1.0;
    float val=servo_min+(servo_max-servo_min)*scaled;
    if(val>1.5f)val=1.5f;
    if(val<-1.5f)val=-1.5f;
    return roundf(val*100.0f)/100.0f;
}

// ===============================
// === UART ===
// ===============================
int uart_open(const char *device){
    int fd=open(device,O_RDWR|O_NOCTTY|O_NDELAY);
    if(fd==-1){ perror("UART open failed"); return -1; }
    struct termios opt;
    tcgetattr(fd,&opt);
    opt.c_cflag=B115200|CS8|CLOCAL|CREAD;
    opt.c_iflag=IGNPAR;
    opt.c_oflag=0;
    opt.c_lflag=0;
    tcflush(fd,TCIFLUSH);
    tcsetattr(fd,TCSANOW,&opt);
    return fd;
}

// ===============================
// === MAIN ===
// ===============================
static int running=1;
void __signal_handler(int _){ running=0; }

int main(){
    signal(SIGINT,__signal_handler);
    int uart_fd=uart_open("/dev/ttyS5");
    if(uart_fd<0) return -1;

    if(rc_servo_init()!=0){ fprintf(stderr,"Servo init fail\n"); return -1; }
    rc_servo_power_rail_en(1);

    double seg_r=sqrt(R*R-D*D);
    Vec3 p1={seg_r*cos(deg2rad(0)),   seg_r*sin(deg2rad(0)),   D};
    Vec3 p2={seg_r*cos(deg2rad(120)), seg_r*sin(deg2rad(120)), D};
    Vec3 p3={seg_r*cos(deg2rad(240)), seg_r*sin(deg2rad(240)), D};

    arm_control_packet_t pkt;
    size_t psize=sizeof(pkt);
    printf("Listening for orientation packets...\n");

    while(running){
        uint8_t b1, b2;
        if(read(uart_fd,&b1,1)<=0) continue;
        if(b1!=START_BYTE1) continue;

        if(read(uart_fd,&b2,1)<=0) continue;
        if(b2!=START_BYTE2) continue;

        uint8_t buf[sizeof(pkt)];
        buf[0]=b1; buf[1]=b2;
        ssize_t r=read(uart_fd,buf+2,psize-2);
        if(r!=psize-2) continue;

        uint16_t recv_cs=*(uint16_t*)&buf[psize-2];
        uint16_t calc_cs=fletcher16(buf,psize-2);
        if(recv_cs!=calc_cs) continue;

        memcpy(&pkt,buf,psize);

        // ============================
        // === DATA PROCESSING ===
        // ============================
        double yaw_deg = rad2deg(pkt.yaw);
        printf("\n[Packet] X=%.3f  Y=%.3f  Z=%.3f  Yaw=%.2f deg  State=%d\n",
                pkt.position[0], pkt.position[1], pkt.position[2],
                yaw_deg, pkt.waypoint_state);

        // Example computation using yaw
        double alpha=-pkt.position[0]*0.8;
        double beta = pkt.position[1]*0.8;
        double yaw_rad=deg2rad(yaw_deg);
        alpha += 0.3*sin(yaw_rad);
        beta  += 0.3*cos(yaw_rad);

        Vec3 x=compute_x(alpha,beta);
        printf("Computed vector x=(%.2f, %.2f, %.2f)\n", x.x, x.y, x.z);

        if(x.z>=D){
            double d1=spherical_distance(p1,x);
            double d2=spherical_distance(p2,x);
            double d3=spherical_distance(p3,x);

            float s1=map_to_servo(1,d1);
            float s2=map_to_servo(2,d2);
            float s3=map_to_servo(3,d3);

            rc_servo_send_pulse_normalized(6,s1);
            rc_servo_send_pulse_normalized(7,s2);
            rc_servo_send_pulse_normalized(8,s3);
        }

        rc_usleep(20000); // 50 Hz loop
    }

    rc_servo_power_rail_en(0);
    rc_servo_cleanup();
    close(uart_fd);
    return 0;
}

