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
        uint8_t cmd = 0x04;
        uint8_t regAddr = 0x00;
        if (rc_i2c_write_byte(I2C_BUS, 0x00, 0x04) < 0) {
            std::cerr << "Failed to send measurement command" << std::endl;
            rc_i2c_close(I2C_BUS);
            return -1;
        }

        uint8_t status;
        do {
            if (rc_i2c_read_byte(I2C_BUS, 0x01, &status) < 0) {
                std::cerr << "Failed to read status register" << std::endl;
                rc_i2c_close(I2C_BUS);
                return -1;
            }
            //std::cerr << "Waiting For Completed Measurement" << std::endl;
        } while (status & 0x01);


        //Read range data
        uint8_t regHigh = 0x0f; // register to read high data
        uint8_t regLow = 0x10; // register to read low data
        uint8_t data[2];    // buffer to hold the data

        //Read each individually and check for errors
        if (rc_i2c_read_byte(I2C_BUS, regHigh, &data[0]) < 0) {
            std::cerr << "Failed to read high byte" << std::endl;
            rc_i2c_close(I2C_BUS);
            return -1;
        }

        if (rc_i2c_read_byte(I2C_BUS, regLow, &data[1]) < 0) {
            std::cerr << "Failed to read low byte" << std::endl;
            rc_i2c_close(I2C_BUS);
            return -1;
        }

        //Store and print range data
        int distance = (data[0] << 8) | data[1]; // Combine the high and low bytes
        std::cout << "Distance: " << distance << " cm" << std::endl;
    
        rc_usleep(100000);
    }

    // Close I2C bus
    rc_i2c_close(I2C_BUS);

    std::cout << "Program Terminated" << std::endl;

    return 0;

}
