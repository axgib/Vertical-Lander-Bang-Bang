#include <iostream>
#include <robotcontrol.h>
#include <csignal>

//Lidar I2C device information
#define I2C_BUS 1
#define LIDAR_ADDRESS 0x62

//State tracker
volatile bool keepRunning = true;

//Signal handler to catch ctrl+c
void signalHandler(int signum) {
    keepRunning = false;
}

int main() {

    //Register Signal Handler
    std::signal(SIGINT, signalHandler);

    //Initialize I2C bus with Robot Control Library
    if (rc_i2c_init(I2C_BUS, LIDAR_ADDRESS) < 0) {
        std::cerr << "Failed to initialized I2C bus" << std::endl;
        return -1;
    }

    //Running Loop
    while (keepRunning) {

        //Write Command to initiate measurement
        uint8_t cmd[2] = {0x00, 0x04};
        uint8_t regAddr = 0x00;
        if (rc_i2c_write_bytes(I2C_BUS, regAddr, 2, cmd) < 0) {
            std::cerr << "Failed to send measurement command" << std::endl;
            rc_i2c_close(I2C_BUS);
            return -1;
        }

        //Delay to allow measurement completion
        rc_usleep(20000);

        //Read range data
        uint8_t reg = 0x10; // register to read
        uint8_t data[2];    // buffer to hold the data

        //Check for errors
        if (rc_i2c_read_bytes(I2C_BUS, reg, 2, data) < 0) {
            std::cerr << "Failed to read range data" << std::endl;
            rc_i2c_close(I2C_BUS);
            return -1;
        }

        //Store and print range data
        int distance = (data[0] << 8) | data[1]; // Combine the high and low bytes
        std::cout << "Distance: " << distance << " cm" << std::endl;
    
        rc_usleep(500000);
    }

    // Close I2C bus
    rc_i2c_close(I2C_BUS);

    std::cout << "Program Terminated" << std::endl;

    return 0;

}
