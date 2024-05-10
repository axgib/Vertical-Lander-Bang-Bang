//Lander Hop Test

#include <iostream>
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

    cout << "Motor check complete!";


    // Pause between checks and execution
    
	while (!landed) {
		if (h > liftoff_height) {
			liftoff  = true;
        }
        // Altitude and Position controller in parallel
        // Read desired altitude and position from transmitter
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

            //Send gimbal angle commands to servos

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
