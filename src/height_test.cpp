#include <iostream>
#include <robotcontrol.h>
#include <csignal>

#define LIDAR_I2C_BUS 1
#define LIDAR_ADDRESS 0x62
#define DMP_SAMPLE_RATE_HZ 100
#define DMP_I2C_BUS 2
#define DMP_GPIO_INT_PIN_CHIP 3
#define DMP_GPIO_INT_PIN_PIN 21

volatile bool keepRunning = true;

double quat[4] = {1.0, 0.0, 0.0, 0.0};

void signalHandler(int signum) {
    keepRunning = false;
}

void power_down() {
    rc_i2c_close(LIDAR_I2C_BUS);
    rc_mpu_power_off();
}

int main() {

    /*
    Initialize signal handler, IMU, and i2c communication
    */

    std::signal(SIGINT, signalHandler);

    rc_mpu_data_t mpu_data;
    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.dmp_sample_rate = DMP_SAMPLE_RATE_HZ;
    conf.i2c_bus = DMP_I2C_BUS;
    conf.gpio_interrupt_pin_chip = DMP_GPIO_INT_PIN_CHIP;
    conf.gpio_interrupt_pin = DMP_GPIO_INT_PIN_PIN;
    conf.enable_magnetometer = 1;

    if (rc_mpu_initialize_dmp(&mpu_data, conf) < 0) {
        std::cerr << "Failed to initialize MPU" << std::endl;
        rc_i2c_close(LIDAR_I2C_BUS);
        return -1;
    }

    if (rc_i2c_init(LIDAR_I2C_BUS, LIDAR_ADDRESS) < 0) {
        std::cerr << "Failed to initialized I2C bus" << std::endl;
        return -1;
    }

    while (keepRunning) {

        /*
        READING DISTANCE FROM RANGEFINDER
        */
        if (rc_i2c_write_byte(LIDAR_I2C_BUS, 0x00, 0x04) < 0) {
            std::cerr << "Failed to send measurement command" << std::endl;
            power_down();
            return -1;
        }

        uint8_t status;
        do {
            if (rc_i2c_read_byte(LIDAR_I2C_BUS, 0x01, &status) < 0) {
                std::cerr << "Failed to read status register" << std::endl;
                power_down();
                return -1;
            }
        } while (status & 0x01);

        uint8_t data[2];
        if (rc_i2c_read_byte(LIDAR_I2C_BUS, 0x0f, &data[0]) < 0) {
            std::cerr << "Failed to read high byte" << std::endl;
            power_down();
            return -1;
        }
        if (rc_i2c_read_byte(LIDAR_I2C_BUS, 0x10, &data[1]) < 0) {
            std::cerr << "Failed to read low byte" << std::endl;
            power_down();
            return -1;
        }

        int distance = (data[0] << 8) | data[1];
        std::cout << "Distance: " << distance << "cm" << std::endl;

        /*
        READING ORIENTATION FROM IMU
        */

        for (int i = 0; i < 4; i++) {
            quat[i] = mpu_data.fused_quat[i];
            std::cout << "quat[" << i << "]: " << quat[i] << std::endl;
        }



        double roll = mpu_data.fused_TaitBryan[0]*RAD_TO_DEG; // Rotation about X
        double pitch = mpu_data.fused_TaitBryan[1]*RAD_TO_DEG;  // Rotation about Y
        double yaw = mpu_data.fused_TaitBryan[2]*RAD_TO_DEG; // Rotation about Z

        std::cout << "TB: " << roll << ", " << pitch << ", " << yaw << std::endl;


        rc_usleep(100000);
    }

    power_down();
    std::cout << "Program Terminated" << std::endl;
    return 0;

}
