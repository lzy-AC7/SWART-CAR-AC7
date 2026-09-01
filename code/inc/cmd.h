#ifndef _CMD_H_
#define _CMD_H_

#include "common.h"



void car_2p_start(float x_target, float y_target, float yaw_target);

void car_2p_start_map(float x, float y, float yaw);

void car_2p();

void car_runing_path();

void car_runing_path_start(float yaw);

bool LineCheck(int x0,int y0,int x1,int y1);

#endif