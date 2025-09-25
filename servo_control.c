// https://old.beagleboard.org/static/librobotcontrol/group___servo.html#gaa3227ad3e226810d64a7266f27f39c3b


/*
+ve is clockwise(release) and -ve is counter clockwise(tight)

servo :
e: -1.5
b: 1.5

origin: 0 0 0  ---- d = 6.82


limits
180 - 12.32 -  1.5
70  - 1.32  - -1.5

alpha_max = 70
alpha_min = 0
*/
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <rc/servo.h>
#include <rc/time.h>

static int running = 1;

static void __signal_handler(int dummy) {
    running = 0;
}

int main(int argc, char *argv[]) {
    if (argc != 4) {
        printf("Usage: %s <servo1_val> <servo2_val> <servo3_val>\n", argv[0]);
        return -1;
    }

    float val1 = atof(argv[1]);
    float val2 = atof(argv[2]);
    float val3 = atof(argv[3]);

    printf("Initializing servo system...\n");
    if (rc_servo_init() != 0) {
        printf("ERROR: Failed to initialize servos\n");
        return -1;
    }

    printf("Enabling servo power rails...\n");
    if (rc_servo_power_rail_en(1) != 0 || rc_servo_power_rail_en(2) != 0 || rc_servo_power_rail_en(3) != 0) {
        printf("ERROR: Failed to enable power rail\n");
        rc_servo_cleanup();
        return -1;
    }

    signal(SIGINT, __signal_handler);

    printf("Setting servo positions to: %.2f, %.2f, %.2f\n", val1, val2, val3);

    for (int i = 0; i < 100 && running; i++) {
        if (rc_servo_send_pulse_normalized(1, val1) != 0 ||
            rc_servo_send_pulse_normalized(2, val2) != 0 ||
            rc_servo_send_pulse_normalized(3, val3) != 0) {
            printf("ERROR: Failed to send pulse\n");
        }
        rc_usleep(20000);  // 20ms = 50Hz
    }

    printf("Disabling servo power rail...\n");
    rc_servo_power_rail_en(0);
    printf("Cleaning up servo system...\n");
    rc_servo_cleanup();

    printf("Program completed.\n");
    return 0;
}
