//Lander Hop Test

#include <iostream>
#include <rc/time.h>
#include <rc/adc.h>
#include <rc/dsm.h>
#include <rc/servo.h>
using namespace std;

// Define Physical properties of rocket
float m = 1;
float Ibody = 1;// Moment of Inertia of full rocket about center of mass
float Ixg = 1;// Moment of Inertia of gimbal in about y-axis (in x-z plane) about gimbal pivot
float Iyg = 1;// Moment of Inertia of gimbal in about x-axis (in y-z plane) about gimbal pivot
float g = 9.81;// Gravity

//Define flight regimes
float liftoff_height = 0.1; // m
bool liftoff = false;
bool landed = false;
bool converged = false; // converged to hover height

//Define Flight Profile
// Hover to hover_height for hover_pause seconds, then land
float hover_height = 2; // m
float hover_pause = 5; // s

float AltitudeController(float height_error){
    // Assume vertical (small angle)
    // Input controller here
    float thrust;
    return thrust;
}

float AttitudeController(float attitude_error, float thrust){
    // Input controller here
    float gimbal_angle;
    return gimbal_angle;
}

float PositionController(float position_error, float thrust){
    // Input controller here
    float body_angle;
    return body_angle;
}


int main() {

    // Ensure propeller is initially off
    // Set initial servo position to phi = 0

    //Turn on power rail? -from rc_test_servos.c
    // printf("Turning On 6V Servo Power Rail\n");
    // rc_servo_power_rail_en(1);
	
    // Check battery levels
    float prop_battery;//Read battery voltage
    float board_battery;
    float low_battery;
    if (prop_battery <= low_battery || board_battery <= low_battery) {
        cout << "Battery low"; // Should specify which battery is low
        return 0;
    }

    // Begin data collection

    // Check for data collection from all sensors

	// Motors check
    //Define motor pins
    string xservo_pin;
    string yservo_pin;
    string prop_pin;

    cout << "Motor check complete!";

    // Pause between checks and execution
    rc_sleep(2000000)
    int t_start = rc_nanos_since_boot();// start timer

    
	while (!landed) {
        double t = (rc_nanos_since_boot() - t_start)/1000000000;// [s] time since loop began

		if (h > liftoff_height) {
			liftoff  = true;
        }
        if (t>5 && t<20) {
            float href = 2;
            float xref = 0;
            float yref = 0;
        } else {
            float href = 0;
            float xref = 0;
            float yref = 0;
        }
        // Altitude and Position controller in parallel
        float href = hover_height;
        float xref = 0;
        float yref = 0;

        // Sample height and position measurement
        float hout;// From laser range finder (and accelerometer integration)
        float xout;// From accelerometer integration
        float yout;// From accelerometer integration

        // Altitude controller
        float herr = href - hout;
        float thrust = AltitudeController(herr);

        // Position controller
        float xerr = xref - xout;
        float xthetaref = PositionController(xerr,thrust);

        float yerr = yref - yout;
        float ythetaref = PositionController(yerr,thrust);

		//Attitude control loops 10 times for every loop of position and thrust
        int i = 0;
        while (i <= 10) {
            i++;
            //Sample attidute measurements from inclinometer
            float xthetaout;
            float ythetaout;

            float xthetaerr = xthetaref - xthetaout;
            float xphi = AttitudeController(xthetaerr,thrust);
            
            float ythetaerr = ythetaref - ythetaout;
            float yphi = AttitudeController(ythetaerr,thrust);

            //Cap gimbal angles at min and max
            if (xphi > maxphi) {
                xphi = maxphi;
            }
            if (yphi > maxphi) {
                yphi = maxphi;
            }
            //Send gimbal angle commands to servos
            if(rc_servo_send_pulse_normalized(ch,xservo_pos)==-1) return -1;
            if(rc_servo_send_pulse_normalized(ch,yservo_pos)==-1) return -1;
            // Sleep to roughly maintain attidute frequency
            rc_usleep(1000000/frequency_hz);
        }
        //Factor battery charge into thrust command
        //Convert thrust to throttle command
        //Send throttle command to propeller

		if (h < liftoff_height && liftoff) {
			landed = true;
        }
    }
    // Ensure propeller is off

    // Return motors to neutral position
    // 
}
