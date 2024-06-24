#include <iostream>
#include <rc/mpu.h>

static rc_mpu_data_t mpu_data;
double accel_vec[3];

int main(void){

    for(int i = 0; i < 10; i++){
        for(i=0;i<3;i++){
            accel_vec[i]=mpu_data.accel[i];
            std::cout << std::format("{}", std::numbers::accel_vec[i]);
            } 
    }

    return 0;
}