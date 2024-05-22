// Propeller system id

/**
 * @file rc_test_esc.c
 * @example    rc_test_esc
 *
 *
 * @author     James Strawson
 * @date       3/20/2018
 */
// #include <stdio.h>
// #include <getopt.h>
// #include <stdlib.h> // for atoi
#include <signal.h>
// #include <rc/time.h>
// #include <rc/dsm.h>
// #include <rc/servo.h>
#include <stdio.h>
#include <robotcontrol.h>


static int running = 0;

// typedef enum test_mode_t{
//         DISABLED,
//         NORM,
//         SWEEP,
// }test_mode_t;

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        running=0;
        return;
}

int main(int argc, char *argv[])
{
        int c,i,ret;            // misc variables
        double sweep_limit = 0; // max throttle allowed when sweeping
        double thr = 0;         // normalized throttle
        int ch = 0;             // channel to test, 0 means all channels
        double dir = 1;         // switches between 1 & -1 in sweep mode
        // test_mode_t mode;       // current operating mode
        uint64_t dsm_nanos;     // nanoseconds since last dsm packet
        int frequency_hz = 50;  // default 50hz frequency to send pulses
        int wakeup_en = 1;      // wakeup period enabled by default
        double wakeup_s = 3.0;  // wakeup period in seconds
        double wakeup_val = -0.1;// wakeup value
        int min_us = RC_ESC_DEFAULT_MIN_US;
        int max_us = RC_ESC_DEFAULT_MAX_US;
        // start with mode as disabled

        thr = 0.1;
        // mode = NORM;

        // set signal handler so the loop can exit cleanly
        signal(SIGINT, __signal_handler);
        running=1;

        // initialize PRU and make sure power rail is OFF
        if(rc_servo_init()) return -1;
        if(rc_servo_set_esc_range(min_us,max_us)) return -1;
        rc_servo_power_rail_en(0);
        
        // send throttle of 0 first
        // otherwise it will go into calibration mode
        if(wakeup_en){
                printf("waking ESC up from idle for 3 seconds\n");
                for(i=0;i<=frequency_hz*wakeup_s;i++){
                        if(running==0) return 0;
                        if(rc_servo_send_esc_pulse_normalized(ch,wakeup_val)==-1) return -1;
                        rc_usleep(1000000/frequency_hz);
                }
                printf("done with wakeup period\n");
        }

        // Main loop runs at frequency_hz
        while(running){
            rc_servo_send_esc_pulse_normalized(ch,thr);
            // break;
                // case SWEEP:
                //         // increase or decrease position each loop
                //         // scale with frequency
                //         thr += dir * sweep_limit / frequency_hz;
                //         // reset pulse width at end of sweep
                //         if(thr > sweep_limit){
                //                 thr = sweep_limit;
                //                 dir = -1;
                //         }
                //         else if(thr < 0){
                //                 thr = 0;
                //                 dir = 1;
                //         }
                //         // send result
                //         if(oneshot_en) rc_servo_send_oneshot_pulse_normalized(ch,thr);
                //         else rc_servo_send_esc_pulse_normalized(ch,thr);
                //         break;
                
                // sleep roughly enough to maintain frequency_hz
            rc_usleep(1000000/frequency_hz);
        }
        // cleanup
        rc_servo_send_esc_pulse_normalized(ch,-0.1);
        rc_usleep(50000);
        rc_servo_cleanup();
        rc_dsm_cleanup();
        printf("\n");
        return 0;
}