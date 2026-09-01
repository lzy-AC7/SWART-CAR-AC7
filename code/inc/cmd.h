#ifndef _CMD_H_
#define _CMD_H_

#include "common.h"

extern float x_target;
extern float y_target;
extern float x_start;
extern float y_start;
extern float distance;
extern float path_length_sq;
extern float dis_t;
void car_2p_start(float x_target, float y_target, float yaw_target);

void car_2p_start_map(float x, float y, float yaw);

void car_2p();

void car_runing_path();

void car_runing_path_start(float yaw);

bool LineCheck(int x0,int y0,int x1,int y1);

#endif