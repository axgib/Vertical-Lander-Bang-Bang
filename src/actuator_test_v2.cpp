#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <algorithm>
#include <cstring>
#include "../include/hop_defs.h"

//Should be all that is necessary on BB
#include <robotcontrol.h>

// // Attempt to include libraries on laptop, not quite right...
// #include "../../librobotcontrol/library/include/rc/time.h"
// #include "../../librobotcontrol/library/include/rc/servo.h"
// #include "../../librobotcontrol/library/include/rc/motor.h"
// #include "../../librobotcontrol/library/include/rc/start_stop.h"
// #include "../../librobotcontrol/library/include/rc/adc.h"


double t, t_ref;
double prop_pause = 3e9;
double prop_test_duration = 6e9;
double servo_test_duration = 5e8;
double servo_test_duration_fast = 1e8;
double servo_max_angle = 0.2;//servo at 60deg at 1, want max ~12 ---> 12/60 = 0.2
double servo_angle;
double prop1_signal = -0.1;
double prop2_signal = -0.1;
std::vector<double> servo_signal = {0.0, 0.0};
// double servo1_signal = 0;
// double servo2_signal = 0;
// int i = 0;
int loop_hz = 50;

enum TestState { 
    SINGLEPROP, 
    DUALPROP, 
    SINGLESERVO, 
    DUALSERVO,
    PARALLEL,
    FULLTEST,
    DONE 
};

TestState state;


void printHelp() {
    std::cout << "Usage: program [-h] [-t TESTS] [-a]\n"
              << "Options:\n"
              << "  -h        Show this help message\n"
              << "  -t TESTS  Specify tests to run: s, d, S, D, f\n"
              << "          s: single propeller"
              << "          d: double propeller"
              << "          S: single servo"
              << "          D: double servo"
              << "          f: full system test, all actuators simultaneously"
              << "  -a        Run all tests\n";
}
void singleprop() {
    printf("Running Single Propeller Test sp...\n");

    if(t < prop_test_duration) {                                    // Power on 1st prop
        printf("1st prop to 20%%...\n");
        prop1_signal = 0.2;
        prop2_signal = -0.1;
        servo_signal = {0.0, 0.0};
    }
    else if(t < prop_pause + prop_test_duration) {                  // Power off 1st prop
        printf("pause...\n");
        prop1_signal = -0.1;
        prop2_signal = -0.1;
    }
    else if(t < prop_pause + 2*prop_test_duration) {                // Power on 2nd prop
        printf("2nd prop to 20%%...\n");
        prop1_signal = -0.1;
        prop2_signal = 0.2;
    }
    else if(t < 2*prop_pause + 2*prop_test_duration) {             // Power off 2nd prop
        printf("pause...\n");
        prop1_signal = -0.1;
        prop2_signal = -0.1;
    }
    else {
        t_ref = t;
        state = DUALPROP;
    }
}

void dualprop() {
    printf("Running Dual Propeller Test dp...:\n");
    if(t < prop_test_duration) {                                    // Power on both props to 20%%
        printf("Both props to 20%%...\n");
        prop1_signal = 0.2;
        prop2_signal = 0.2;
        servo_signal = {0.0, 0.0};
    }
    else if(t < prop_pause + prop_test_duration) {                  // Power off props
        printf("pause...\n");
        prop1_signal = -0.1;
        prop2_signal = -0.1;
    }
    else if(t < prop_pause + 2*prop_test_duration) {                // Power on both props to 50%%
        printf("Both props to 50%%...\n");
        prop1_signal = 0.5;
        prop2_signal = 0.5;
    }
    else if(t < 2*prop_pause + 2*prop_test_duration) {             // Power off props
        printf("pause...\n");
        prop1_signal = -0.1;
        prop2_signal = -0.1;
    }
    else if(t < 2*prop_pause + 3*prop_test_duration) {                // Power on both props to 100%%
        printf("Both props to 100%%...\n");
        prop1_signal = 1.0;
        prop2_signal = 1.0;
    }
    else if(t < 3*prop_pause + 3*prop_test_duration) {             // Power off both props
        printf("pause...\n");
        prop1_signal = -0.1;
        prop2_signal = -0.1;
    }
    else {
        t_ref = t;
        state = SINGLESERVO;
    }
    // std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate test running
}
void singleservo(int servo_to_test) {
    printf("Running Single Servo Test ss..\n");
    if(t < servo_test_duration) {                                    
        prop1_signal = -0.1;
        prop2_signal = -0.1;
        servo_signal = {0.0, 0.0};
        servo_angle = servo_max_angle;
    }
    else if(t < 2*servo_test_duration) {                   
        servo_angle = -servo_max_angle;
    }
    else if(t < 3*servo_test_duration) {                   
        servo_angle = -0.8*servo_max_angle;
    }
    else if(t < 4*servo_test_duration) {                   
        servo_angle = -0.6*servo_max_angle;
    }
    else if(t < 5*servo_test_duration) {                   
        servo_angle = -0.4*servo_max_angle;
    }
    else if(t < 6*servo_test_duration) {                   
        servo_angle = -0.2*servo_max_angle;
    }
    else if(t < 7*servo_test_duration) {                   
        servo_angle = 0.0*servo_max_angle;
    }
    else if(t < 8*servo_test_duration) {                   
        servo_angle = 0.2*servo_max_angle;
    }
    else if(t < 9*servo_test_duration) {                   
        servo_angle = 0.4*servo_max_angle;
    }
    else if(t < 10*servo_test_duration) {                   
        servo_angle = 0.6*servo_max_angle;
    }
    else if(t < 11*servo_test_duration) {                   
        servo_angle = 0.8*servo_max_angle;
    }
    else if(t < 12*servo_test_duration) {                   
        servo_angle = servo_max_angle;
    }
    else if(t < 13*servo_test_duration) {                   
        servo_angle = 0.0;
    }
    else {
        if(servo_to_test == 0){
            servo_to_test = 1;
        }
        else {
            state = DUALSERVO;
        }
        t_ref = t;
    }
    std::cout << "Servo " << servo_to_test << "] to " << servo_angle << " degrees" << std::endl;
    servo_signal[servo_to_test] = servo_angle;
    }

void dualservo() {
    printf("Running Dual Servo Test ds..\n");
    if(t < servo_test_duration) {                                    
        prop1_signal = -0.1;
        prop2_signal = -0.1;
        servo_signal = {servo_max_angle, servo_max_angle};          // CCW square
    }
    else if(t < 2*servo_test_duration) {                   
        servo_signal = {-servo_max_angle, servo_max_angle};
    }
    else if(t < 3*servo_test_duration) {                   
        servo_signal = {-servo_max_angle, -servo_max_angle};
    }
    else if(t < 4*servo_test_duration) {                   
        servo_signal = {servo_max_angle, -servo_max_angle};
    }
    else if(t < 6*servo_test_duration) {                   
        servo_signal = {servo_max_angle, servo_max_angle};
    }

    else if(t < 7*servo_test_duration) {                            // CW Circle
        servo_signal = {servo_max_angle, 0.0};
    }
    else if(t < 8*servo_test_duration) {                   
        servo_signal = {0.707*servo_max_angle, -0.707*servo_max_angle};
    }
    else if(t < 9*servo_test_duration) {                   
        servo_signal = {0.0, -servo_max_angle};
    }
    else if(t < 10*servo_test_duration) {                   
        servo_signal = {-0.707*servo_max_angle, -0.707*servo_max_angle};
    }
    else if(t < 11*servo_test_duration) {                           
        servo_signal = {-servo_max_angle, 0.0};
    }
    else if(t < 12*servo_test_duration) {                   
        servo_signal = {-0.707*servo_max_angle, 0.707*servo_max_angle};
    }
    else if(t < 13*servo_test_duration) {                   
        servo_signal = {0.0, servo_max_angle};
    }
    else if(t < 14*servo_test_duration) {                   
        servo_signal = {0.707*servo_max_angle, 0.707*servo_max_angle};
    }
    else if(t < 16*servo_test_duration) {                   
        servo_signal = {0.0, 0.0};
    }
    else {
        t_ref = t;
        state = FULLTEST;
    }
}
void fulltest() {
    printf("Running All Actuator Test a..\n");

    
    state = DONE;
    // std::cout << "Running Test 3" << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate test running
}

int main(int argc, char* argv[]) {

    // Initialize
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

    
    // Read command line arguments
    std::vector<std::string> tests_to_run;
    std::vector<char> valid_tests = {'s', 'd', 'S', 'D', 'f'};
    bool run_all_tests = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-h") == 0) {
            printHelp();
            return 0;
        }
        else if (argv[i] == "-a") {
            run_all_tests = true;
            break;
        } 
        else if (strcmp(argv[i], "-t") == 0) {
            if (i + 1 < argc) {
                std::string tests(argv[++i]);
                for (char test : tests) {
                    if (std::find(valid_tests.begin(),valid_tests.end(), test) != valid_tests.end()) {
                        switch (test) {
                            case 's': tests_to_run.push_back("s");
                            case 'd': tests_to_run.push_back("d"); 
                            case 'S': tests_to_run.push_back("S"); 
                            case 'D': tests_to_run.push_back("D"); 
                            case 'f': tests_to_run.push_back("f"); 
                        }
                    } 
                    else {
                        std::cerr << "Unknown test: " << test << '\n';
                        return 1;
                    }
                }
                if (tests_to_run.empty()) {
                    std::cerr << "Error: -t requires at least one valid test\n";
                    return 1;
                }
            } 
            else {
                std::cerr << "Error: -t requires a test list\n";
                return 1;
            }
        }
        else {
            tests_to_run.push_back(argv[i]);
        }
    }

    t_ref = rc_nanos_since_boot(); // Reference time

    state = SINGLEPROP;// Always start with single prop test if specifiec in command line
    printf("Ensure vehicle is properly secured! Exit now if not...\n");
    rc_usleep(3000000);
    int servo_num = 0; // start servo test with 1st servo

    while (state != DONE) {
        t = t_ref - rc_nanos_since_boot();

        switch (state) {
            case SINGLEPROP:
                if (run_all_tests || std::find(tests_to_run.begin(), tests_to_run.end(), "s") != tests_to_run.end()) {
                    singleprop();
                }
                break;

            case DUALPROP:
                if (run_all_tests || std::find(tests_to_run.begin(), tests_to_run.end(), "d") != tests_to_run.end()) {
                    dualprop();
                }
                break;

            case SINGLESERVO:
                if (run_all_tests || std::find(tests_to_run.begin(), tests_to_run.end(), "S") != tests_to_run.end()) {
                    singleservo(servo_num);
                }
                break;

            case DUALSERVO:
                if (run_all_tests || std::find(tests_to_run.begin(), tests_to_run.end(), "D") != tests_to_run.end()) {
                    dualservo();
                }
                break;
            case FULLTEST:
                if (run_all_tests || std::find(tests_to_run.begin(), tests_to_run.end(), "f") != tests_to_run.end()) {
                    fulltest();
                }
                break;
            default:
                state = DONE;
                break;

        rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_1,prop1_signal);
        rc_servo_send_esc_pulse_normalized(PROP_CHANNEL_2,prop2_signal);
        rc_servo_send_pulse_normalized(SERVO_CHANNEL_X,servo_signal[0]);
        rc_servo_send_pulse_normalized(SERVO_CHANNEL_Y,servo_signal[1]);
        
        rc_nanosleep(1e9/loop_hz);
        }

    }
    rc_usleep(50000);
    rc_servo_power_rail_en(0);
    rc_servo_cleanup();
    rc_adc_cleanup();
    return 0;
}