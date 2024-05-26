/**
 * rc_balance_defs.h
 *
 * Contains the settings for configuration of rc_balance.c
 */

#ifndef RC_HOP_CONFIG
#define RC_HOP_CONFIG

#define SAMPLE_RATE_HZ		100	// main filter and control loop speed
#define DT			0.01			// 1/sample_rate

// Structural properties of eduMiP
#define BOARD_MOUNT_ANGLE_X	0.0 // increase if mip tends to roll forward
#define BOARD_MOUNT_ANGLE_Y	0.0 // increase if mip tends to roll forward
// #define GEARBOX			35.577
// #define ENCODER_RES		60
// #define WHEEL_RADIUS_M		0.034
// #define TRACK_WIDTH_M		0.035
#define V_NOMINAL_BOARD		7.44
#define V_NOMINAL_PROP      7.44// TODO: Update

// inner loop x gimbal controller 100hz
#define D1X_GAIN			1.05
#define D1X_ORDER		2
#define D1X_NUM			{-4.945, 8.862, -3.967}
#define D1X_DEN			{ 1.000, -1.481, 0.4812}
#define D1X_NUM_LEN		3
#define D1X_DEN_LEN		3

// inner loop y gimbal controller 100hz
#define D1Y_GAIN			1.05
#define D1Y_ORDER		2
#define D1Y_NUM			{-4.945, 8.862, -3.967}
#define D1Y_DEN			{ 1.000, -1.481, 0.4812}
#define D1Y_NUM_LEN		3
#define D1Y_DEN_LEN		3

#define D1_SATURATION_TIMEOUT	0.4


// outer loop height controller 100hz
#define D2H_GAIN			0.9
#define	D2H_ORDER		2
#define D2H_NUM			{0.18856,  -0.37209,  0.18354}
#define D2H_DEN			{1.00000,  -1.86046,   0.86046}
#define D2H_NUM_LEN		3
#define D2H_DEN_LEN		3

#define H_REF_MAX		0.33
#define HOVER_HEIGHT    2
#define RANGEFINDER_OFFSET 0.1
#define T_TAKEOFF       5*1e9
#define T_DECEND        20*1e9
#define T_LAND          30*1e9

// outer loop x-position controller 100hz
#define D2X_GAIN			0.9
#define	D2X_ORDER		2
#define D2X_NUM			{0.18856,  -0.37209,  0.18354}
#define D2X_DEN			{1.00000,  -1.86046,   0.86046}
#define D2X_NUM_LEN		3
#define D2X_DEN_LEN		3

// outer loop y-position controller 100hz
#define D2Y_GAIN			0.9
#define	D2Y_ORDER		2
#define D2Y_NUM			{0.18856,  -0.37209,  0.18354}
#define D2Y_DEN			{1.00000,  -1.86046,   0.86046}
#define D2Y_NUM_LEN		3
#define D2Y_DEN_LEN		3

#define THETA_REF_MAX		0.33
#define PHI_REF_MAX         0.15 // 1 corresponds to 60 deg rotation, limit movement to ~10 deg

// electrical hookups
#define PROP_CHANNEL_1      1
#define PROP_CHANNEL_2      2
#define SERVO_CHANNEL_X		3
#define SERVO_CHANNEL_Y		4
#define SERVO_POLARITY_X	1
#define SERVO_POLARITY_Y	1
#define ENCODER_CHANNEL_X	3
#define ENCODER_CHANNEL_Y	2
// #define ENCODER_POLARITY_L	1
// #define ENCODER_POLARITY_R	-1

// inputs
#define PROP_BATTERY_CHANNEL 1
#define RANGEFINDER_CHANNEL 2

// Thread Loop Rates
#define BATTERY_CHECK_HZ	5
#define SETPOINT_MANAGER_HZ	100
#define PRINTF_HZ		50

// other
#define TIP_ANGLE		0.85
#define START_ANGLE		0.3
#define START_DELAY		0.4
#define PICKUP_DETECTION_TIME	0.6
#define ENABLE_POSITION_HOLD	1
#define SOFT_START_SEC		0.7

#endif	// endif RC_BALANCE_CONFIG