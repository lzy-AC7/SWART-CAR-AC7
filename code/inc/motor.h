#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "common.h"

void motor_init(void);
void motor_run(uint8 motor_id,int32 duty);
void motor_stop();
void motor_cmd();

#endif