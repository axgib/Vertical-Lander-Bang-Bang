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
float prop_pause = 3e9;
float prop_test_duration = 6e9;
float servo_test_duration = 5e8;
float servo_test_duration_fast = 1e8;
float servo_max_angle = 0.2;//servo at 60deg at 1, want max ~12 ---> 12/60 = 0.2
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
                printf("Ensure vehicle is properly secured! Exit now if not\nPowering on 1st propeller...");
            }
            i = 1;
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }

        // Test each propeller individually %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        elseif(t-t_start < pause + prop_test_duration) {                    // Power on 1st prop
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 2*pause + prop_test_duration) {                  // Power off 1st prop
            if(i==1){
                printf("Powering on 2st propeller...");
            }
            i = 2;
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 2*pause + 2*prop_test_duration) {                // Power on 2nd prop
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 3*pause + 2*prop_test_duration) {                // Power off 2nd prop
            if(i==2){
                printf("Powering on both propellers to 10%...");
            }
            i = 3;
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }

        // Test propellers together %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        elseif(t-t_start < 3*pause + 3*prop_test_duration) {                // Power on both propellers to 10%
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 4*pause + 3*prop_test_duration) {                // Power off both propellers
            if(i==3){
                printf("Powering on both propellers to 50%...");
            }
            i = 4;
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 4*pause + 4*prop_test_duration) {                // Power on both propellers to 50%
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,0.5);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,0.5);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 5*pause + 4*prop_test_duration) {                // Power off both propellers
            if(i==4){
                printf("Powering on both propellers to 100%...");
            }
            i = 5;
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 5*pause + 5*prop_test_duration) {                // Power on both propellers to 100%
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 6*pause + 5*prop_test_duration) {                // Power off both propellers
            if(i==5){
                printf("Propeller test is complete!\nNow testing servos, beginning with X-servo...");
            }
            i = 6;
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }

        // Test each servo individually %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        elseif(t-t_start < 6*pause + 5*prop_test_duration + 1*servo_test_duration) {                // X Servo
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 6*pause + 5*prop_test_duration + 2*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,-servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 6*pause + 5*prop_test_duration + 3*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,-0.8*servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 6*pause + 5*prop_test_duration + 4*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,-0.6*servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 6*pause + 5*prop_test_duration + 5*servo_test_duration) {             
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,-0.4*servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 6*pause + 5*prop_test_duration + 6*servo_test_duration) {
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,-0.2*servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 6*pause + 5*prop_test_duration + 7*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 6*pause + 5*prop_test_duration + 8*servo_test_duration) {              
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0.2*servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 6*pause + 5*prop_test_duration + 9*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0.4*servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 6*pause + 5*prop_test_duration + 10*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0.6*servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 6*pause + 5*prop_test_duration + 11*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0.8*servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 6*pause + 5*prop_test_duration + 12*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 7*pause + 5*prop_test_duration + 12*servo_test_duration) {
            if(i==6){
                printf("Testing Y-servo...");
            }
            i = 7;      
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }

        elseif(t-t_start < 7*pause + 5*prop_test_duration + 13*servo_test_duration) {                // Y Servo
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,servo_max_angle);
        }
        elseif(t-t_start < 7*pause + 5*prop_test_duration + 14*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,-servo_max_angle);
        }
        elseif(t-t_start < 7*pause + 5*prop_test_duration + 15*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,-0.8*servo_max_angle);
        }
        elseif(t-t_start < 7*pause + 5*prop_test_duration + 16*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,-0.6*servo_max_angle);
        }
        elseif(t-t_start < 7*pause + 5*prop_test_duration + 17*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,-0.4*servo_max_angle);
        }
        elseif(t-t_start < 7*pause + 5*prop_test_duration + 18*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,-0.2*servo_max_angle);
        }
        elseif(t-t_start < 7*pause + 5*prop_test_duration + 19*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 7*pause + 5*prop_test_duration + 20*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0.2*servo_max_angle);
        }
        elseif(t-t_start < 7*pause + 5*prop_test_duration + 21*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0.4*servo_max_angle);
        }
        elseif(t-t_start < 7*pause + 5*prop_test_duration + 22*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0.6*servo_max_angle);
        }
        elseif(t-t_start < 7*pause + 5*prop_test_duration + 23*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0.8*servo_max_angle);
        }
        elseif(t-t_start < 7*pause + 5*prop_test_duration + 24*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,servo_max_angle);
        }
        elseif(t-t_start < 8*pause + 5*prop_test_duration + 24*servo_test_duration) { 
            if(i==7){
                printf("Individual servo tests complete! Testing servos together...");
            }
            i = 8;               
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }

        // Test both servos together %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        elseif(t-t_start < 8*pause + 5*prop_test_duration + 25*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,servo_max_angle);
        }
        elseif(t-t_start < 8*pause + 5*prop_test_duration + 26*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,-servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,servo_max_angle);
        }
        elseif(t-t_start < 8*pause + 5*prop_test_duration + 27*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,-servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,-servo_max_angle);
        }
        elseif(t-t_start < 8*pause + 5*prop_test_duration + 28*servo_test_duration) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,-servo_max_angle);
        
        // Circle                                                                                      //Circle
        elseif(t-t_start < 8*pause + 5*prop_test_duration + 28*servo_test_duration + 1*servo_test_duration_fast) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,-servo_max_angle);
        }
        elseif(t-t_start < 8*pause + 5*prop_test_duration + 28*servo_test_duration + 2*servo_test_duration_fast) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,-0.707*servo_max_angle);//sqrt(2)/2
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,-0.707*servo_max_angle);
        }
        elseif(t-t_start < 8*pause + 5*prop_test_duration + 27*servo_test_duration + 3*servo_test_duration_fast) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,-1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 8*pause + 5*prop_test_duration + 28*servo_test_duration + 4*servo_test_duration_fast) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,-0.707*servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y, 0.707*servo_max_angle);
        }
        elseif(t-t_start < 8*pause + 5*prop_test_duration + 25*servo_test_duration + 5*servo_test_duration_fast) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,1);
        }
        elseif(t-t_start < 8*pause + 5*prop_test_duration + 26*servo_test_duration + 6*servo_test_duration_fast) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0.707);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0.707);
        }
        elseif(t-t_start < 8*pause + 5*prop_test_duration + 27*servo_test_duration + 7*servo_test_duration_fast) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }
        elseif(t-t_start < 8*pause + 5*prop_test_duration + 28*servo_test_duration + 8*servo_test_duration_fast) {                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X, 0.707*servo_max_angle);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,-0.707*servo_max_angle);
        }
        elseif(t-t_start < 9*pause + 5*prop_test_duration + 25*servo_test_duration + 8*servo_test_duration_fast) {
            if(i==8){
                printf("Servo tests complete! Testing all actuators together...");
            }
            i = 9;                
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,-0.1);
            rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,-0.1);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
            rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
        }

        // Test all actuators together %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
        else {
            rc_set_state(EXITING);
            rc_servo_cleanup();
            return;
        }
        rc_nanosleep(1e9/loop_hz);
    }

}