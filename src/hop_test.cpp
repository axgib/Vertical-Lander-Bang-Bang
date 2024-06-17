/**
* @example hop_test
*
* Reference hop flight for FiP rocket
**/
#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strtof()
#include <stdint.h>
#include <math.h> // for M_PI
// #include <iostream>
#include <rc/servo.h>
#include <robotcontrol.h>
#include "../include/hop_defs.h"



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
        double d2x_u;            ///< output of position controller D2x (theta_ref)
        double d2y_u;            ///< output of position controller D2y (theta_ref)
} core_state_t;
// possible modes, user selected with command line arguments (ONLY HOP IS IMPLEMENTED)
// typedef enum m_input_mode_t{
//         NONE,
//         DSM,
//         STDIN,
//         HOP
// } m_input_mode_t;

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

// global variables
core_state_t cstate;
setpoint_t setpoint;
rc_filter_t D1x = RC_FILTER_INITIALIZER;
rc_filter_t D1y = RC_FILTER_INITIALIZER;
rc_filter_t D2h = RC_FILTER_INITIALIZER;
rc_filter_t D2x = RC_FILTER_INITIALIZER;
rc_filter_t D2y = RC_FILTER_INITIALIZER;

rc_mpu_data_t mpu_data;
uint64_t tstart, ti, t, t_takeoff,t_decend,t_land;
int board_thread_id = 1;
int prop_thread_id = 2;
uint8_t rf_addr, rf_height;
const double PI = 3.141592653589793;

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

        printf("Initialization begun...\n");

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

        if (rc_i2c_init(I2C_BUS, LIDAR_ADDRESS) < 0) {
                fprintf(sterr, "Failed to initialized I2C bus \n");
                return -1;
        }

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
        mpu_config.orient = ORIENTATION_Z_DOWN;//TODO: check

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
        
        // set up D2h height controller
        double D2h_num[] = D2H_NUM;
        double D2h_den[] = D2H_DEN;
        if(rc_filter_alloc_from_arrays(&D2h, DT, D2h_num, D2H_NUM_LEN, D2h_den, D2H_DEN_LEN)){
                fprintf(stderr,"ERROR in rc_balance, failed to make filter D2h\n");
                return -1;
        }
        D2h.gain = D2H_GAIN;



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

        // set saturation limits 
        rc_filter_enable_saturation(&D1x, -PHI_REF_MAX, PHI_REF_MAX);// TODO: check saturation limits
        // rc_filter_enable_soft_start(&D1x, SOFT_START_SEC);
        rc_filter_enable_saturation(&D1y, -PHI_REF_MAX, PHI_REF_MAX);
        // rc_filter_enable_soft_start(&D1y, SOFT_START_SEC);
        rc_filter_enable_saturation(&D2h, THRUST_MIN, THRUST_MAX);
        // rc_filter_enable_soft_start(&D2h, SOFT_START_SEC);
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

        // Original battery sample code// start threads to slowly sample battery voltage
        // if(rc_pthread_create(&board_battery_thread, __battery_checker, (void*) NULL, SCHED_OTHER, 0)){
        //         fprintf(stderr, "failed to start board battery thread\n");
        //         return -1;
        // }

        // if(rc_pthread_create(&prop_battery_thread, __battery_checker, (void*) NULL, SCHED_OTHER, 0)){
        //         fprintf(stderr, "failed to start prop battery thread\n");
        //         return -1;
        // } 

        // monitor board battery voltage (powers servos)
        if(rc_pthread_create(&board_battery_thread, __battery_checker, (void*) &board_thread_id, SCHED_OTHER, 0)){
                fprintf(stderr, "failed to start board battery thread\n");
                return -1;
        }

        // monitor prop battery: not implemented, currently just uses V_NOMINAL_PROP
        if(rc_pthread_create(&prop_battery_thread, __battery_checker, (void*) &prop_thread_id, SCHED_OTHER, 0)){
                fprintf(stderr, "failed to start prop battery thread\n");
                return -1;
        }

        // wait for the battery threads to make the first read
        while(cstate.vBattBoard<1.0 && cstate.vBattProp<1.0 && rc_get_state()!=EXITING) rc_usleep(10000);
        
        // start printf_thread if running from a terminal
        // if it was started as a background process then don't bother
        if(isatty(fileno(stdout))){
                if(rc_pthread_create(&printf_thread, __printf_loop, (void*) NULL, SCHED_OTHER, 0)){
                        fprintf(stderr, "failed to start print thread\n");
                        return -1;
                }
        }

        // start mpu
        if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
                fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost!\n");
                rc_led_blink(RC_LED_RED, 5, 5);
                return -1;
        }
        // TODO: start laser randefinder

        // // start balance stack to control setpoints
        // if(rc_pthread_create(&setpoint_thread, __setpoint_manager, (void*) NULL, SCHED_OTHER, 0)){
        //         fprintf(stderr, "failed to start setpoint thread\n");
        //         return -1;
        // }

        // start balance stack to control setpoints
        tstart = rc_nanos_since_boot();
        if(rc_pthread_create(&setpoint_thread, __setpoint_manager, &tstart, SCHED_OTHER, 0)){
                fprintf(stderr, "failed to start setpoint thread\n");
                return -1;
        }

        // this should be the last step in initialization
        // to make sure other setup functions don't interfere
        rc_mpu_set_dmp_callback(&__balance_controller);
        
        printf("\n Initialization complete! All threads running...\n");

        // start in the RUNNING state
        rc_set_state(RUNNING);

        // chill until something exits the program
        while(rc_get_state()!=EXITING){
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
        rc_i2c_close(I2C_BUS);
        
        rc_led_set(RC_LED_GREEN, 0);
        rc_led_set(RC_LED_RED, 0);
        rc_led_cleanup();
        rc_servo_cleanup();
        rc_remove_pid_file();   // remove pid file LAST
        printf("Shutdown complete!\n");
        return 0;
}

/**
 * disable motors & set the setpoint.core_mode to DISARMED
 *
 * @return     { description_of_the_return_value }
 */
static int __disarm_controller(void)
{
        rc_servo_cleanup();//TODO: Does this turn off prop?
        setpoint.arm_state = DISARMED;
        return 0;
}

/**
 * This thread is in charge of adjusting the controller setpoints based on user
 * inputs from dsm radio control. Also detects pickup to control arming the
 * controller.
 *
 * @param      ptr   The pointer
 *
 * @return     { description_of_the_return_value }
 */
void* __setpoint_manager(uint64_t tstart) {
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
                        printf("\nArming controller...\n");
                        if(__wait_for_starting_condition()==0){// Once rocket has been upright for START_DELAY, 
                                __zero_out_controller();// set controller outputs and setpoints to 0
                                __arm_controller();// Set to armed
                        }
                        else continue;//keep waiting for starting condition until met
                }
                printf("\nController armed\n");

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
                else if(t < T_LAND){// decending, setpoint.height = 0
                        setpoint.height = 0;
                }
                else{// shutdown
                        rc_set_state(EXITING);
                }

                // if it has been more than 1 second since getting data
                if(stdin_timeout >= SETPOINT_MANAGER_HZ){
                        setpoint.thetax = 0;
                }
                else{
                        stdin_timeout++;
                }

                continue;

        }
        // if state becomes EXITING the above loop exists and we disarm here
        __disarm_controller();
        // return NULL;
}
/**
 * discrete-time balance controller operated off mpu interrupt Called at
 * SAMPLE_RATE_HZ
 */
static void __balance_controller(void)
{
        static int xinner_saturation_counter = 0;
        static int yinner_saturation_counter = 0;
        // static int houter_saturation_counter = 0;
        double posx, posy, dutyh; // posx and posy to servos, dutyh to prop
        /******************************************************************
        * STATE_ESTIMATION
        * read sensors and compute the state when either ARMED or DISARMED
        ******************************************************************/
        // angle theta is positive in the direction of forward tip around X axis
        cstate.thetax = mpu_data.dmp_TaitBryan[TB_PITCH_X] + BOARD_MOUNT_ANGLE_X;
        cstate.thetay = mpu_data.dmp_TaitBryan[TB_ROLL_Y] + BOARD_MOUNT_ANGLE_Y;// TODO

        /*
        LIDAR MEASUREMENT
        */
        if (rc_i2c_write_byte(I2C_BUS, 0x00, 0x04) < 0) {
                __disarm_controller();
                fprintf("Failed to send measurement command to LIDAR\n");
                return;
        }
        
        uint8_t lidarStatus;
        do {
                if (rc_i2c_read_byte(I2C_BUS, 0x01, &status) < 0) {
                        __disarm_controller();
                        fprintf("Failed to read LIDAR status register\n");
                        return;
                }

        } while (status & 0x01);

        uint8_t lidarRangeData[2];

        if (rc_i2c_read_byte(I2C_BUS, 0x0f, &lidarRangeData[0]) < 0) {
                __disarm_controller();
                fprintf("Failed to read LIDAR high byte\n");
                return;
        }

        if (rc_i2c_read_byte(I2C_BUS, 0x10, &lidarRangeData[1]) < 0) {
                __disarm_controller();
                fprintf("Failed to read LIDAR low byte\n");
                return;
        }

        int lidarDistance = (data[0] << 8) | data[1];

        /*
        HEIGHT CALCULATION::::::::::::::TODO
        */




        /*************************************************************
        * check for various exit conditions AFTER state estimate
        ***************************************************************/
        if(rc_get_state()==EXITING){
                rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,0);
                rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,0);
                rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,0);
                rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,0);
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
        if(fabs(cstate.thetay) > TIP_ANGLE || fabs(cstate.thetax) > TIP_ANGLE){
                __disarm_controller();
                printf("\ntip detected: disarmed controller\n");
                return;
        }
        // /**********************************************************
        // * OUTER LOOP HEIGHT h controller D2h
        // ***********************************************************/
        // if(fabs(setpoint.h_dot)>0.0001){
        //         setpoint.height += setpoint.h_dot * DT;
        // } 
        cstate.d2h_u = rc_filter_march(&D2h,setpoint.height - cstate.height);

        /************************************************************
        * OUTER LOOP Theta controllers D2x and D2y
        * Move the position setpoint based on theta_dot.
        * Input to the controller is theta error (setpoint-state).
        *************************************************************/
        //TODO: use updated thrust d2h_u to modify angle controllers. Currently assuming thruts const = Wg
        if(ENABLE_POSITION_HOLD){
                if(fabs(setpoint.thetax_dot) > 0.001){ //if theta setpoint is changing
                    setpoint.thetax += setpoint.thetax_dot*DT;// update theta setpoint from theta_dot
                }
                cstate.d2x_u = rc_filter_march(&D2x,setpoint.thetax-cstate.thetax); // sum block producing D2 controller input
        }
        else setpoint.thetax = 0.0;// default to try to be upright

        if(ENABLE_POSITION_HOLD){
                if(fabs(setpoint.thetay_dot) > 0.001){
                    setpoint.thetay += setpoint.thetay_dot*DT;
                }
                cstate.d2y_u = rc_filter_march(&D2y,setpoint.thetay-cstate.thetay); //TODO: Don't understand this
                setpoint.thetay = cstate.d2y_u;
        }
        else setpoint.thetay = 0.0;
        /************************************************************
        * INNER LOOP ANGLE Phi controllers D1x and D1y
        * Input to D1x and D1y is phi error (setpoint-state). Then scale the
        * output u to compensate for changing battery voltage.
        *************************************************************/
        D1x.gain = D1X_GAIN * V_NOMINAL_BOARD/cstate.vBattBoard;// scale gain based on measured voltage
        cstate.d1x_u = rc_filter_march(&D1x,(setpoint.phix-cstate.phix));

        D1y.gain = D1Y_GAIN * V_NOMINAL_BOARD/cstate.vBattBoard;
        cstate.d1y_u = rc_filter_march(&D1y,(setpoint.phiy-cstate.phiy));
        
        /*************************************************************
        * Check if the inner loop saturated. If it saturates for over
        * a second disarm the controller to prevent stalling motors.
        *************************************************************/
        if(fabs(cstate.d1x_u)>0.95) xinner_saturation_counter++;
        else xinner_saturation_counter = 0;
        if(fabs(cstate.d1y_u)>0.95) yinner_saturation_counter++;
        else yinner_saturation_counter = 0;

        // if(fabs(cstate.d2h_u)>0.95) houter_saturation_counter++;
        // else houter_saturation_counter = 0;

        // if saturate for a second, disarm for safety
        if(xinner_saturation_counter > (SAMPLE_RATE_HZ*D1_SATURATION_TIMEOUT) || yinner_saturation_counter > (SAMPLE_RATE_HZ*D1_SATURATION_TIMEOUT)){
                printf("\ninner loop controller saturated\n");
                __disarm_controller();
                xinner_saturation_counter = 0;
                yinner_saturation_counter = 0;
                return;
        }

        // /**********************************************************
        // * Send signal to motors
        // * add D1 balance control u and D3 steering control also
        // * multiply by polarity to make sure direction is correct.
        // ***********************************************************/
        
        //TODO: saturation should already be caught by rc_filter_enable_saturation but could check once more here
        // if(cstate.d2h_u < 0) cstate.d2h_u = 0;//can't apply negative thrust
        dutyh = cstate.d2h_u / (2*THRUST_MAX);// normalize thrust assuming linear throttle-thrust relationship

        posx = cstate.d1x_u * 3/PI; // normalize where pi/3 rad = 1
        posy = cstate.d1y_u * 3/PI;

        rc_servo_send_pulse_normalized(SERVO_CHANNEL_X, SERVO_POLARITY_X * posx);
        rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y, SERVO_POLARITY_Y * posy);
        rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,dutyh);
        rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,dutyh);
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
        setpoint.phix   = 0.0;
        setpoint.phiy   = 0.0;
        setpoint.phix_dot = 0.0;
        setpoint.phiy_dot = 0.0;
        setpoint.height = 0.0;
        setpoint.thetax = 0.0;
        setpoint.thetay = 0.0;
        setpoint.h_dot = 0.0;
        setpoint.thetax_dot = 0.0;
        setpoint.thetay_dot = 0.0;

        rc_motor_set(0,0.0); //TODO: make work for servos and escs
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
        setpoint.arm_state = ARMED;
        return 0;
}

/**
 * Wait for rocket to be positioned upright long enough to begin. Returns
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
        // Wait for rocket to be upright
        checks = 0;
        // exit if state becomes paused or exiting
        while(rc_get_state()==RUNNING) {
                // if within range, start counting
                if(fabs(cstate.thetax) < START_ANGLE || fabs(cstate.thetay) < START_ANGLE) checks++;
                // fell out of range, restart counter
                else checks = 0;
                // waited long enough, return
                if(checks >= checks_needed) {
                        printf("\nStarting condition met\n");
                        return 0;
                }
                rc_usleep(wait_us);
        }
        return -1;
}

/**
 * Slow loop checking battery voltages. Also changes the D1 saturation limit
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
                        new_v = V_NOMINAL_PROP;// TODO: Read from an input pin
                        cstate.vBattProp = new_v;
                        rc_usleep(1000000 / BATTERY_CHECK_HZ);
                }
                else {
                        printf("\nthread_id not found\n");
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
                        printf(" armstate |");
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
