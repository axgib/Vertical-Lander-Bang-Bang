#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strtof()
#include <stdint.h>
#include <math.h> // for M_PI
// #include <iostream>
#include <rc/servo.h>
#include <robotcontrol.h>
#include "hop_defs.h"

float t, t_start;
float pause = 3e9;
float prop_test_duration = 6e9;
int i = 0;
float loop_hz = 5500;

int main() {

    if(rc_kill_existing_process(2.0)<-2) return -1;

    // start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
            fprintf(stderr,"ERROR: failed to start signal handler\n");
            return -1;
    }

    // initialize servos and propeller
    if(rc_servo_init()) {
            fprintf(stderr,"ERROR: failed to initialize servos\n");
            return -1;
    }
    rc_servo_power_rail_en(1); //turn on servo power rail

    rc_set_state(RUNNING);
    int prop_or_servo = 0; // 0 for prop, 1 for servo

    t_start = rc_nanos_since_boot();

    while(rc_get_state()!=EXITING) {
        t = rc_nanos_since_boot();
        if(t-t_start < pause) {                                             // Set all actuators to zero
            if(i==0){
                printf("Powering on 1st propeller...");
            }
            i = 1;
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < pause + prop_test_duration) {                    // Power on 1st prop
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
        }
        elseif(t-t_start < 2*pause + prop_test_duration) {                  // Power off 1st prop
            if(i==1){
                printf("Powering on 2st propeller...");
            }
            i = 2;
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
        }
        elseif(t-t_start < 2*pause + 2*prop_test_duration) {                // Power on 2nd prop
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,0.1);
        }
        elseif(t-t_start < 3*pause + 2*prop_test_duration) {                // Power off 2nd prop
            if(i==2){
                printf("Powering on both propellers to 10%...");
            }
            i = 3;
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
        }


        elseif(t-t_start < 3*pause + 3*prop_test_duration) {                // Power on both propellers to 10%
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,0.1);
        }
        elseif(t-t_start < 4*pause + 3*prop_test_duration) {                // Power off both propellers
            if(i==3){
                printf("Powering on both propellers to 50%...");
            }
            i = 4;
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
        }
        elseif(t-t_start < 4*pause + 4*prop_test_duration) {                // Power on both propellers to 50%
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,0.5);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,0.5);
        }
        elseif(t-t_start < 5*pause + 4*prop_test_duration) {                // Power off both propellers
            if(i==4){
                printf("Powering on both propellers to 100%...");
            }
            i = 5;
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
        }
        elseif(t-t_start < 5*pause + 5*prop_test_duration) {                // Power on both propellers to 100%
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,1);
        }
        elseif(t-t_start < 6*pause + 5*prop_test_duration) {                // Power off both propellers
            if(i==5){
                printf("Propeller test is complete!");
            }
            i = 6;
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
        }
        else {
            rc_set_state(EXITING);
            rc_servo_cleanup();
            return;
        }
        rc_nanosleep(1e9/loop_hz);
    }

}