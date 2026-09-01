#ifndef _PID_H_
#define _PID_H_

#include "common.h"

float pid_incremental(struct PID *this ,const float target, const float feedback);// 增量式 PID

float pid_positional(struct PID *this ,const float target, const float feedback);// 位置式 PID

void speed_pid_update();//速度环 PID 更新函数

void v_pid_update();//运动速度 PID 更新函数

void pos_pid_update();//位置环 PID 更新函数

void yaw_pid_update();//航向环 PID 更新函数

#endif