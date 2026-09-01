#ifndef _IMU_H_
#define _IMU_H_

#include "common.h"

extern bool acc_calibrated;
extern float acc_x;
extern float acc_y;

void yaw_get();
void acc_get();

#endif
