/**
* @example rc_balance
*
* Reference solution for balancing EduMiP
**/
#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strtof()
// #include <stdint.h> // for 64int_t
#include <math.h> // for M_PI
#include <chrono>
// #include <iostream>
#include <rc/servo.h>
#include <robotcontrol.h>
#include "hop_defs.h"

// /**
//  * NOVICE: Drive rate and turn rate are limited to make driving easier.
//  * ADVANCED: Faster drive and turn rate for more fun.
//  */

// typedef enum drive_mode_t{
//         NOVICE,
//         ADVANCED
// }drive_mode_t;
/**
 * ARMED or DISARMED to indicate if the controller is running
 */
typedef enum arm_state_t{
        ARMED,
        DISARMED
}arm_state_t;
/**
 * Feedback controller setpoint written to by setpoint_manager and read by the
 * controller.
 */
typedef struct setpoint_t{
        arm_state_t arm_state;  ///< see arm_state_t declaration
        // drive_mode_t drive_mode;///< NOVICE or ADVANCED
        double phix;            // gimbal position x (rad)
        double phiy;            // gimbal position y (rad)
        double phix_dot;         ///< rate at which phi reference updates (rad/s)
        double phiy_dot;        // TODO: Need one for each phi?
        double height;
        double h_dot;
        double thetax;           ///< body lean angle (rad)
        double thetay;
        double thetax_dot;         ///< rate at which phi reference updates (rad/s)
        double thetay_dot;
        // double gamma;           ///< body turn angle (rad)
        // double gamma_dot;       ///< rate at which gamma setpoint updates (rad/s)
}setpoint_t;
/**
 * This is the system state written to by the balance controller.
 */
typedef struct core_state_t{
        double phix;     // Gimbal angle relative to body
        double phiy;     // Gimbal angle relative to body
        double thetax;           ///< body angle radians
        double thetay;
        double xPos;            // x position
        double yPos;
        double height;
        double vBattBoard;           ///< board battery voltage
        double vBattProp;            ///< propeller battery voltage
        // 5 controllers needed: height, thetax, thetay, phix, phiy. 1 denotes inner loop, 2 denotes outer loop. Roll controller is 6th
        // Assume roll = 0, body and global coordinates aligned
        double d1x_u;            ///< output of attitude controller D1x to servo
        double d1y_u;            ///< output of attitude controller D1y to servo
        double d2h_u;            ///< output of height controller D1h to propeller
        double d2x_u;            ///< output of position controller D1x (theta_ref)
        double d2y_u;            ///< output of position controller D1y (theta_ref)
        double mot_drive;       ///< u compensated for battery voltage
} core_state_t;
// possible modes, user selected with command line arguments (ONLY HOP IS IMPLEMENTED)
typedef enum m_input_mode_t{
        NONE,
        DSM,
        STDIN,
        HOP
} m_input_mode_t;

static void __print_usage(void);
static void __balance_controller(void);         ///< mpu interrupt routine
static void* __setpoint_manager(void* ptr);     ///< background thread
static void* __battery_checker(void* ptr);      ///< background thread
static void* __printf_loop(void* ptr);          ///< background thread
static int __zero_out_controller(void);
static int __disarm_controller(void);
static int __arm_controller(void);
static int __wait_for_starting_condition(void);
static void __on_pause_press(void);
// static void __on_mode_release(void);

// global variables
core_state_t cstate;
setpoint_t setpoint;
rc_filter_t D1x = RC_FILTER_INITIALIZER;
rc_filter_t D1y = RC_FILTER_INITIALIZER;
rc_filter_t D2h = RC_FILTER_INITIALIZER;
rc_filter_t D2x = RC_FILTER_INITIALIZER;
rc_filter_t D2y = RC_FILTER_INITIALIZER;

rc_mpu_data_t mpu_data;
m_input_mode_t m_input_mode = HOP;
uint64_t tstart, ti, t, t_takeoff,t_decend,t_land;
int board_thread_id = 1;
int prop_thread_id = 2;
int rf_bus;
uint8_t rf_addr, rf_height;

/*
 * printed if some invalid argument was given
 */
static void __print_usage(void)
{
        printf("\n");
        printf("-i {dsm|stdin|none}     specify input\n");
        printf("-h                      print this help message\n");
        printf("\n");
}
/**
 * Initialize the filters, mpu, threads, & wait until shut down
 *
 * @return     0 on success, -1 on failure
 */
int main(int argc, char *argv[])
{
        // int c;
        pthread_t setpoint_thread = 0;
        pthread_t board_battery_thread = 0;
        pthread_t prop_battery_thread = 0;
        pthread_t printf_thread = 0;
        
        // make sure another instance isn't running
        // if return value is -3 then a background process is running with
        // higher privaledges and we couldn't kill it, in which case we should
        // not continue or there may be hardware conflicts. If it returned -4
        // then there was an invalid argument that needs to be fixed.
        if(rc_kill_existing_process(2.0)<-2) return -1;

        // start signal handler so we can exit cleanly
        if(rc_enable_signal_handler()==-1){
                fprintf(stderr,"ERROR: failed to start signal handler\n");
                return -1;
        } 
        
        // // initialize enocders %% WE DON'T HAVE ENCODERS
        // if(rc_encoder_eqep_init()==-1){
        //         fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        //         return -1;
        // }
        // // initialize motors
        // if(rc_motor_init()==-1){
        //         fprintf(stderr,"ERROR: failed to initialize motors\n");
        //         return -1;
        // }
        // rc_motor_standby(1); // start with motors in standby
        
        printf("Initialization begun...\n")

        //Turn off green LED and turn on red LED while initializing
        if(rc_led_set(RC_LED_GREEN, 0)==-1){
                fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_GREEN\n");
                return -1;
        }
        if(rc_led_set(RC_LED_RED, 1)==-1){
                fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_RED\n");
                return -1;
        }

        // initialize servos and propeller
        if(rc_servo_init()) {
                fprintf(stderr,"ERROR: failed to initialize servos\n");
                return -1;
        }

        // turn on power rail
        printf("Turning On 6V Servo Power Rail\n");
        rc_servo_power_rail_en(1);

        // initialize adc
        if(rc_adc_init()==-1){
                fprintf(stderr, "failed to initialize adc\n");
        }

        // make PID file to indicate your project is running
        // due to the check made on the call to rc_kill_existing_process() above
        // we can be fairly confident there is no PID file already and we can
        // make our own safely.
        rc_make_pid_file(); // (/run/shm/robotcontrol.pid)

        // set up mpu configuration
        rc_mpu_config_t mpu_config = rc_mpu_default_config();
        mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
        mpu_config.orient = ORIENTATION_Z_DOWN;

        // if gyro isn't calibrated, run the calibration routine
        // run rc_calibrate_gyro and rc_calibrate_accel before running this
        // if(!rc_mpu_is_gyro_calibrated()){
        //         printf("Gyro not calibrated, automatically starting calibration routine\n");
        //         printf("Let your MiP sit still on a firm surface\n");
        //         rc_mpu_calibrate_gyro_routine(mpu_config);
        // }

        // make sure setpoint starts at normal values
        setpoint.arm_state = DISARMED;
        // setpoint.drive_mode = NOVICE;

        // set up D1x and D1y Phi controllers
        //TODO: Adjust controllers in header file
        double D1x_num[] = D1X_NUM; 
        double D1x_den[] = D1X_DEN;
        if(rc_filter_alloc_from_arrays(&D1x, DT, D1x_num, D1X_NUM_LEN, D1x_den, D1X_DEN_LEN)){
                fprintf(stderr,"ERROR in rc_balance, failed to make filter D1x\n");
                return -1;
        }
        D1x.gain = D1X_GAIN;

        double D1y_num[] = D1Y_NUM; 
        double D1y_den[] = D1Y_DEN;
        if(rc_filter_alloc_from_arrays(&D1y, DT, D1y_num, D1Y_NUM_LEN, D1y_den, D1Y_DEN_LEN)){
                fprintf(stderr,"ERROR in rc_balance, failed to make filter D1y\n");
                return -1;
        }
        D1y.gain = D1Y_GAIN;

        rc_filter_enable_saturation(&D1x, -1.0, 1.0);
        rc_filter_enable_soft_start(&D1x, SOFT_START_SEC);// TODO: what is soft start
        rc_filter_enable_saturation(&D1y, -1.0, 1.0);
        rc_filter_enable_soft_start(&D1y, SOFT_START_SEC);
        
        // set up D2h height controller
        double D2h_num[] = D2H_NUM;
        double D2h_den[] = D2H_DEN;
        if(rc_filter_alloc_from_arrays(&D2h, DT, D2h_num, D2H_NUM_LEN, D2h_den, D2H_DEN_LEN)){
                fprintf(stderr,"ERROR in rc_balance, failed to make filter D2h\n");
                return -1;
        }
        D2h.gain = D2H_GAIN;

        rc_filter_enable_saturation(&D2h, -H_REF_MAX, H_REF_MAX);// TODO: define height limits (normalized?)
        rc_filter_enable_soft_start(&D2h, SOFT_START_SEC);

        // set up D2x and D2y theta controllers
        double D2x_num[] = D2X_NUM;
        double D2x_den[] = D2X_DEN;
        if(rc_filter_alloc_from_arrays(&D2x, DT, D2x_num, D2X_NUM_LEN, D2x_den, D2X_DEN_LEN)){
                fprintf(stderr,"ERROR in rc_balance, failed to make filter D2x\n");
                return -1;
        }
        D2x.gain = D2X_GAIN;

        double D2y_num[] = D2Y_NUM;
        double D2y_den[] = D2Y_DEN;
        if(rc_filter_alloc_from_arrays(&D2y, DT, D2y_num, D2Y_NUM_LEN, D2y_den, D2Y_DEN_LEN)){
                fprintf(stderr,"ERROR in rc_balance, failed to make filter D2y\n");
                return -1;
        }
        D2y.gain = D2Y_GAIN;

        rc_filter_enable_saturation(&D1x, -PHI_REF_MAX, PHI_REF_MAX);
        // rc_filter_enable_soft_start(&D1x, SOFT_START_SEC);
        rc_filter_enable_saturation(&D1y, -PHI_REF_MAX, PHI_REF_MAX);
        // rc_filter_enable_soft_start(&D1y, SOFT_START_SEC);
        rc_filter_enable_saturation(&D2x, -THETA_REF_MAX, THETA_REF_MAX);
        // rc_filter_enable_soft_start(&D2x, SOFT_START_SEC);
        rc_filter_enable_saturation(&D2y, -THETA_REF_MAX, THETA_REF_MAX);
        // rc_filter_enable_soft_start(&D2y, SOFT_START_SEC);

        printf("Inner Loop controller D1x:\n");
        rc_filter_print(D1x);
        printf("Inner Loop controller D1y:\n");
        rc_filter_print(D1y);
        printf("\nOuter Loop controller D2h:\n");
        rc_filter_print(D2h);
        printf("\nOuter Loop controller D2x:\n");
        rc_filter_print(D2x);
        printf("\nOuter Loop controller D2y:\n");
        rc_filter_print(D2y);

        // // start threads to slowly sample battery voltage
        // if(rc_pthread_create(&board_battery_thread, __battery_checker, (void*) NULL, SCHED_OTHER, 0)){
        //         fprintf(stderr, "failed to start board battery thread\n");
        //         return -1;
        // }

        // if(rc_pthread_create(&prop_battery_thread, __battery_checker, (void*) NULL, SCHED_OTHER, 0)){
        //         fprintf(stderr, "failed to start prop battery thread\n");
        //         return -1;
        // } 

        if(rc_pthread_create(&board_battery_thread, __battery_checker, (void*) &board_thread_id, SCHED_OTHER, 0)){
                fprintf(stderr, "failed to start board battery thread\n");
                return -1;
        }

        if(rc_pthread_create(&prop_battery_thread, __battery_checker, (void*) &prop_thread_id, SCHED_OTHER, 0)){
                fprintf(stderr, "failed to start prop battery thread\n");
                return -1;
        } //TODO: check

        // wait for the battery threads to make the first read
        while(cstate.vBattBoard<1.0 && cstate.vBattProp<1.0 && rc_get_state()!=EXITING) rc_usleep(10000);
        
        // start printf_thread if running from a terminal
        // if it was started as a background process then don't bother
        if(isatty(fileno(stdout))){
                if(rc_pthread_create(&printf_thread, __printf_loop, (void*) NULL, SCHED_OTHER, 0)){
                        fprintf(stderr, "failed to start battery thread\n");
                        return -1;
                }
        }

        // start mpu
        if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
                fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost!\n");
                rc_led_blink(RC_LED_RED, 5, 5);
                return -1;
        }

        // start balance stack to control setpoints
        if(rc_pthread_create(&setpoint_thread, __setpoint_manager, (void*) NULL, SCHED_OTHER, 0)){
                fprintf(stderr, "failed to start setpoint thread\n");
                return -1;
        }

        // this should be the last step in initialization
        // to make sure other setup functions don't interfere
        rc_mpu_set_dmp_callback(&__balance_controller);
        
        printf("\n Initialization complete! All threads running...\n")

        // start in the RUNNING state
        rc_set_state(RUNNING);

        // chill until something exits the program
        while(rc_get_state()!=EXITING){// TODO: How does it get out of this loop?
                rc_usleep(200000);
        }
        
        // join threads
        rc_pthread_timed_join(setpoint_thread, NULL, 1.5);
        rc_pthread_timed_join(board_battery_thread, NULL, 1.5);
        rc_pthread_timed_join(prop_battery_thread, NULL, 1.5);
        rc_pthread_timed_join(printf_thread, NULL, 1.5);
        
        // cleanup
        rc_filter_free(&D1x);
        rc_filter_free(&D1y);
        rc_filter_free(&D2h);
        rc_filter_free(&D2x);
        rc_filter_free(&D2y);

        rc_mpu_power_off();
        rc_led_set(RC_LED_GREEN, 0);
        rc_led_set(RC_LED_RED, 0);
        rc_led_cleanup();
        rc_servo_cleanup();
        // rc_encoder_eqep_cleanup();
        // rc_button_cleanup();    // stop button handlers
        rc_remove_pid_file();   // remove pid file LAST
        printf("Shutdown complete\n")
        return 0;
}


/**
 * This thread is in charge of adjusting the controller setpoint based on user
 * inputs from dsm radio control. Also detects pickup to control arming the
 * controller.
 *
 * @param      ptr   The pointer
 *
 * @return     { description_of_the_return_value }
 */
void* __setpoint_manager(__attribute__ ((unused)) void* ptr)
{
        // double drive_stick, turn_stick; // input sticks
        int i, ch, chan, stdin_timeout = 0; // for stdin input
        char in_str[11];

        ti = rc_nanos_since_boot();
        t = ti-tstart;// time since tstart was defined

        // wait for mpu to settle
        __disarm_controller();
        rc_usleep(2500000);
        rc_set_state(RUNNING);
        rc_led_set(RC_LED_RED,0);
        rc_led_set(RC_LED_GREEN,1);

        while(rc_get_state()!=EXITING){


                // sleep at beginning of loop so we can use the 'continue' statement
                rc_usleep(1000000/SETPOINT_MANAGER_HZ);

                // if we got here the state is RUNNING, but controller is not
                // necessarily armed. If DISARMED, wait for the user to pick MIP up
                // which will we detected by wait_for_starting_condition()
                if(setpoint.arm_state == DISARMED){
                        prinf("\nArming controller...\n")
                        if(__wait_for_starting_condition()==0){
                                __zero_out_controller();
                                __arm_controller();
                        }
                        else continue;
                }
                printf("\nController armed\n")
                
                // if dsm is active, update the setpoint rates

                        if(t < T_TAKEOFF){ // Wait on ground
                                setpoint.phix = 0;
                                setpoint.phiy = 0;
                                setpoint.phix_dot = 0;
                                setpoint.phiy_dot = 0;
                                setpoint.height = 0;
                                setpoint.h_dot = 0;
                                setpoint.thetax = 0;
                                setpoint.thetay = 0;
                                setpoint.thetax_dot = 0;
                                setpoint.thetay_dot = 0;
                        }
                        else if(t < T_DECEND){// at takeoff, set setpoint.height to hover_height
                                setpoint.height = HOVER_HEIGHT;
                        }
                        else if(t < T_LAND){// decending, slowly lower setpoint.height to 0
                                setpoint.height = 0;
                        }
                        else{// shutdown
                                rc_set_state(EXITING);
                        }

                        // if it has been more than 1 second since getting data
                        if(stdin_timeout >= SETPOINT_MANAGER_HZ){
                                setpoint.theta = 0;
                                setpoint.phix = 0;
                                setpoint.phiy = 0;
                                setpoint.phi_dot = 0;
                                setpoint.gamma_dot = 0;
                        }
                        else{
                                stdin_timeout++;
                        }

                        continue;

                }
        }
        // if state becomes EXITING the above loop exists and we disarm here
        __disarm_controller();
        return NULL;
}
/**
 * discrete-time balance controller operated off mpu interrupt Called at
 * SAMPLE_RATE_HZ
 */
static void __balance_controller(void)
{
        static int xinner_saturation_counter = 0;
        static int yinner_saturation_counter = 0;
        double posx, posy, dutyh; // posx and posy to servos, dutyh to prop
        /******************************************************************
        * STATE_ESTIMATION
        * read sensors and compute the state when either ARMED or DISARMED
        ******************************************************************/
        // angle theta is positive in the direction of forward tip around X axis
        cstate.thetax = mpu_data.dmp_TaitBryan[TB_PITCH_X] + BOARD_MOUNT_ANGLE_X;
        cstate.thetay = mpu_data.dmp_TaitBryan[TB_PITCH_Y] + BOARD_MOUNT_ANGLE_Y;// TODO

        // TODO: read from laser range finder
        rc_i2c_read_byte(rf_bus,rf_addr,rf_height);
        cstate.height = rf_height - RF_OFFSET;
        
        // collect encoder positions, right wheel is reversed
        // cstate.wheelAngleR = (rc_encoder_eqep_read(ENCODER_CHANNEL_R) * 2.0 * M_PI) \
        //                         /(ENCODER_POLARITY_R * GEARBOX * ENCODER_RES);
        // cstate.wheelAngleL = (rc_encoder_eqep_read(ENCODER_CHANNEL_L) * 2.0 * M_PI) \
        //                         /(ENCODER_POLARITY_L * GEARBOX * ENCODER_RES);
        
        // Phi is average wheel rotation also add theta body angle to get absolute
        // wheel position in global frame since encoders are attached to the body
        // cstate.phi = ((cstate.wheelAngleL+cstate.wheelAngleR)/2) + cstate.theta;
        // // steering angle gamma estimate
        // cstate.gamma = (cstate.wheelAngleR-cstate.wheelAngleL) \
        //                                 * (WHEEL_RADIUS_M/TRACK_WIDTH_M);
        /*************************************************************
        * check for various exit conditions AFTER state estimate
        ***************************************************************/
        if(rc_get_state()==EXITING){
                rc_motor_set(0,0.0);// set all motors to zero
                return;
        }

        // if controller is still ARMED while state is PAUSED, disarm it
        if(rc_get_state()!=RUNNING && setpoint.arm_state==ARMED){
                __disarm_controller();
                return;
        }

        // exit if the controller is disarmed
        if(setpoint.arm_state==DISARMED){
                return;
        }

        // check for a tipover
        if(fabs(cstate.theta) > TIP_ANGLE){
                __disarm_controller();
                printf("tip detected \n");
                return;
        }

        /************************************************************
        * OUTER LOOP Theta controllers D2x and D2y
        * Move the position setpoint based on theta_dot.
        * Input to the controller is theta error (setpoint-state).
        *************************************************************/
        if(ENABLE_POSITION_HOLD){
                if(fabs(setpoint.thetax_dot) > 0.001){
                    setpoint.thetax += setpoint.thetax_dot*DT;
                }
                cstate.d2x_u = rc_filter_march(&D2x,setpoint.thetax-cstate.thetax); //TODO: Don't understand this
                setpoint.theta = cstate.d2x_u;
        }
        else setpoint.thetax = 0.0;

        if(ENABLE_POSITION_HOLD){
                if(fabs(setpoint.theta_doty) > 0.001){
                    setpoint.thetay += setpoint.thetay_dot*DT;
                }
                cstate.d2y_u = rc_filter_march(&D2y,setpoint.thetay-cstate.thetay); //TODO: Don't understand this
                setpoint.theta = cstate.d2y_u;
        }
        else setpoint.thetay = 0.0;
        /************************************************************
        * INNER LOOP ANGLE Phi controllers D1x and D1y
        * Input to D1x and D1y is phi error (setpoint-state). Then scale the
        * output u to compensate for changing battery voltage.
        *************************************************************/
        D1x.gain = D1X_GAIN * V_NOMINAL/cstate.vBattBoard;
        cstate.d1x_u = rc_filter_march(&D1x,(setpoint.phix-cstate.phix));
        cstate.d1x_u = rc_filter_march(&D1x,(setpoint.phix-cstate.phix));

        D1y.gain = D1Y_GAIN * V_NOMINAL/cstate.vBattBoard;
        cstate.d1y_u = rc_filter_march(&D1y,(setpoint.phiy-cstate.phiy));
        cstate.d1y_u = rc_filter_march(&D1y,(setpoint.phiy-cstate.phiy));
        /*************************************************************
        * Check if the inner loop saturated. If it saturates for over
        * a second disarm the controller to prevent stalling motors.
        *************************************************************/
        if(fabs(cstate.d1x_u)>0.95) xinner_saturation_counter++;
        else xinner_saturation_counter = 0;
        if(fabs(cstate.d1y_u)>0.95) yinner_saturation_counter++;
        else yinner_saturation_counter = 0;

        // if saturate for a second, disarm for safety
        // TODO: Make sense to disarm controller for flying object?
        if(xinner_saturation_counter > (SAMPLE_RATE_HZ*D1_SATURATION_TIMEOUT) || yinner_saturation_counter > (SAMPLE_RATE_HZ*D1_SATURATION_TIMEOUT)){
                printf("inner loop controller saturated\n");
                __disarm_controller();
                inner_saturation_counter = 0;
                return;
        }
        // /**********************************************************
        // * gama (steering) controller D3
        // * move the setpoint gamma based on user input like phi
        // ***********************************************************/
        // if(fabs(setpoint.gamma_dot)>0.0001) setpoint.gamma += setpoint.gamma_dot * DT;
        // cstate.d3_u = rc_filter_march(&D3,setpoint.gamma - cstate.gamma);
        // /**********************************************************
        // * Send signal to motors
        // * add D1 balance control u and D3 steering control also
        // * multiply by polarity to make sure direction is correct.
        // ***********************************************************/
        // duty = cstate.d1_u - cstate.d3_u;
        // TODO: Check these motor commands
        rc_servo_send_pulse_normalized(SERVO_CHANNEL_X, SERVO_POLARITY_X * posx);
        rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y, SERVO_POLARITY_Y * posy);
        rc_motor_set(PROP_CHANNEL,dutyh)
        return;
}
/**
 * Clear the controller's memory and zero out setpoints.
 *
 * @return     { description_of_the_return_value }
 */
static int __zero_out_controller(void)
{
        rc_filter_reset(&D1x);
        rc_filter_reset(&D1y);
        rc_filter_reset(&D2h);
        rc_filter_reset(&D2x);
        rc_filter_reset(&D2y);
        setpoint.thetax = 0.0;
        setpoint.thetay = 0.0;
        setpoint.phix   = 0.0;
        setpoint.phiy   = 0.0;
        setpoint.height = 0.0;
        // setpoint.gamma = 0.0;
        // rc_motor_set(0,0.0); //TODO: turn off each motor every loop? We don't want to do this with propeller
        return 0;
}
/**
 * disable motors & set the setpoint.core_mode to DISARMED
 *
 * @return     { description_of_the_return_value }
 */
static int __disarm_controller(void)
{
        rc_motor_standby(1);
        rc_motor_free_spin(0);
        setpoint.arm_state = DISARMED;
        return 0;
}
/**
 * zero out the controller & encoders. Enable motors & arm the controller.
 *
 * @return     0 on success, -1 on failure
 */
static int __arm_controller(void)
{
        __zero_out_controller();
        rc_encoder_eqep_write(ENCODER_CHANNEL_X,0); // TODO: change to top and bottom for propeller
        rc_encoder_eqep_write(ENCODER_CHANNEL_Y,0);
        // prefill_filter_inputs(&D1,cstate.theta);
        rc_motor_standby(0);
        setpoint.arm_state = ARMED;
        return 0;
}
/**
 * Wait for MiP to be held upright long enough to begin. Returns
 *
 * @return     0 if successful, -1 if the wait process was interrupted by pause
 *             button or shutdown signal.
 */
static int __wait_for_starting_condition(void)
{
        int checks = 0;
        const int check_hz = 20;        // check 20 times per second
        int checks_needed = round(START_DELAY*check_hz);
        int wait_us = 1000000/check_hz;
        // wait for MiP to be tipped back or forward first
        // exit if state becomes paused or exiting
        while(rc_get_state()==RUNNING){
                // if within range, start counting
                if(fabs(cstate.thetax) > START_ANGLE || fabs(cstate.thetay) > START_ANGLE) checks++;
                // fell out of range, restart counter
                else checks = 0;
                // waited long enough, return
                if(checks >= checks_needed) break;
                rc_usleep(wait_us);
        }
        // now wait for MiP to be upright
        checks = 0;
        // exit if state becomes paused or exiting
        while(rc_get_state()==RUNNING){
                // if within range, start counting
                if(fabs(cstate.thetax) < START_ANGLE || fabs(cstate.thetay) < START_ANGLE) checks++;
                // fell out of range, restart counter
                else checks = 0;
                // waited long enough, return
                if(checks >= checks_needed) return 0;
                rc_usleep(wait_us);
        }
        return -1;
}
/**
 * Slow loop checking battery voltage. Also changes the D1 saturation limit
 * since that is dependent on the battery voltage.
 *
 * @return     nothing, NULL poitner
 */
// static void* __battery_checker(__attribute__ ((unused)) void* ptr)
// {
//         double new_v;
//         while(rc_get_state()!=EXITING){

//                 new_v = rc_adc_batt();
//                 // if the value doesn't make sense, use nominal voltage
//                 if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL;
//                 cstate.vBattBoard = new_v;
//                 rc_usleep(1000000 / BATTERY_CHECK_HZ);
//         }
//         return NULL;
// }
static void* __battery_checker(void* ptr)
{
        int thread_id = *(int*)ptr;// TODO: Check!
        double new_v;

        while(rc_get_state()!=EXITING){
                if(thread_id == 1){
                        new_v = rc_adc_batt();
                        // if the value doesn't make sense, use nominal voltage
                        if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL_BOARD;
                        cstate.vBattBoard = new_v;
                        rc_usleep(1000000 / BATTERY_CHECK_HZ);
                }
                else if(thread_id == 2){
                        new_v = V_NOMINAL_PROP;// TODO: Not right: should be reading from an input pin
                        // new_v = rc_adc_batt();
                        // // if the value doesn't make sense, use nominal voltage
                        // if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL_PROP;
                        cstate.vBattProp = new_v;
                        rc_usleep(1000000 / BATTERY_CHECK_HZ);
                }
                else {
                        prinf("\nthread_id not found\n")
                }
        }
        return NULL;
}

/**
 * prints diagnostics to console this only gets started if executing from
 * terminal
 *
 * @return     nothing, NULL pointer
 */

static void* __printf_loop(__attribute__ ((unused)) void* ptr)
{
        rc_state_t last_rc_state, new_rc_state; // keep track of last state
        last_rc_state = rc_get_state();
        while(rc_get_state()!=EXITING){
                new_rc_state = rc_get_state();
                // check if this is the first time since being paused
                if(new_rc_state==RUNNING && last_rc_state!=RUNNING){
                        printf("\nRUNNING: Hold upright to balance.\n");
                        printf("    θx    |");
                        printf("  θx_ref  |");
                        printf("    θy    |");
                        printf("  θy_ref  |");
                        printf("    φx    |");
                        printf("  φx_ref  |");
                        printf("    φy    |");
                        printf("  φy_ref  |");
                        printf("    h     |");
                        printf("  h_ref   |");
                        printf("  D1x_u   |");
                        printf("  D1y_u   |");
                        printf("  D2h_u   |");
                        printf("  D2x_u   |");
                        printf("  D2y_u   |");
                        printf("vBattBoard|");
                        printf(" vBattProp|");
                        printf("arm_state|");
                        printf("\n");
                }
                else if(new_rc_state==PAUSED && last_rc_state!=PAUSED){
                        printf("\nPAUSED: press pause again to start.\n");
                }
                last_rc_state = new_rc_state;
                // decide what to print or exit
                if(new_rc_state == RUNNING){
                        printf("\r");
                        printf("%7.3f  |", cstate.thetax);
                        printf("%7.3f  |", setpoint.thetax);
                        printf("%7.3f  |", cstate.thetay);
                        printf("%7.3f  |", setpoint.thetay);
                        printf("%7.3f  |", cstate.phix);
                        printf("%7.3f  |", setpoint.phix);
                        printf("%7.3f  |", cstate.phiy);
                        printf("%7.3f  |", setpoint.phiy);
                        printf("%7.3f  |", cstate.height);
                        printf("%7.3f  |", setpoint.height);
                        printf("%7.3f  |", cstate.d1x_u);
                        printf("%7.3f  |", cstate.d1y_u);
                        printf("%7.3f  |", cstate.d2h_u);
                        printf("%7.3f  |", cstate.d2x_u);
                        printf("%7.3f  |", cstate.d2y_u);
                        printf("%7.3f  |", cstate.vBattBoard);
                        printf("%7.3f  |", cstate.vBattProp);
                        if(setpoint.arm_state == ARMED) printf("  ARMED  |");
                        else printf("DISARMED |");
                        fflush(stdout);
                }
                rc_usleep(1000000 / PRINTF_HZ);
        }
        return NULL;
}
/**
 * Disarm the controller and set system state to paused. If the user holds the
 * pause button for 2 seconds, exit cleanly
 */
// static void __on_pause_press(void)
// {
//         int i=0;
//         const int samples = 100;        // check for release 100 times in this period
//         const int us_wait = 2000000; // 2 seconds
//         switch(rc_get_state()){
//         // pause if running
//         case EXITING:
//                 return;
//         case RUNNING:
//                 rc_set_state(PAUSED);
//                 __disarm_controller();
//                 rc_led_set(RC_LED_RED,1);
//                 rc_led_set(RC_LED_GREEN,0);
//                 break;
//         case PAUSED:
//                 rc_set_state(RUNNING);
//                 __disarm_controller();
//                 rc_led_set(RC_LED_GREEN,1);
//                 rc_led_set(RC_LED_RED,0);
//                 break;
//         default:
//                 break;
//         }
//         // now wait to see if the user want to shut down the program
//         while(i<samples){
//                 rc_usleep(us_wait/samples);
//                 if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED){
//                         return; //user let go before time-out
//                 }
//                 i++;
//         }
//         printf("long press detected, shutting down\n");
//         //user held the button down long enough, blink and exit cleanly
//         rc_led_blink(RC_LED_RED,5,2);
//         rc_set_state(EXITING);
//         return;
// }
/**
 * toggle between position and angle modes if MiP is paused
 */
// static void __on_mode_release(void)
// {
//         // toggle between position and angle modes
//         if(setpoint.drive_mode == NOVICE){
//                 setpoint.drive_mode = ADVANCED;
//                 printf("using drive_mode = ADVANCED\n");
//         }
//         else {
//                 setpoint.drive_mode = NOVICE;
//                 printf("using drive_mode = NOVICE\n");
//         }
//         rc_led_blink(RC_LED_GREEN,5,1);
//         return;
// }