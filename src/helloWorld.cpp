#include <rc/mpu.h>
#include <printf.h>

static rc_mpu_data_t mpu_data;
// static rc_mpu_data_t gyr_data;
// static rc_mpu_data_t mag_data;

rc_mpu_config_t conf = rc_mpu_default_config();


int main(void){

    conf.enable_magnetometer = 1;

    rc_mpu_initialize(mpu_data,conf);

    rc_mpu_read_accel(mpu_data);
    rc_mpu_read_gyro(mpu_data);
    rc_mpu_read_mag(mpu_data);

    // printf(" %5.2f %5.2f %5.2f |", data.accel[0],\
    //                                data.accel[1],\
    //                                data.accel[2]);
    
    rc_mpu_power_off()
    return 0;
}