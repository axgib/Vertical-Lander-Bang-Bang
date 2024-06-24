#include <stdio.h>
#include <signal.h>
#include <math.h> 
#include <rc/bmp.h>
#include <rc/mpu.h>

static rc_mpu_data_t mpu_data;
double accel_vec[3];

