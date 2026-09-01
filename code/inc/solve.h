#ifndef _SOLVE_H_
#define _SOLVE_H_

#include "common.h"

void solve_car2wheel(float v_x_ref, float v_y_ref, float omega_ref);
void solve_world2wheel(float angle_deg, float speed, float omega_ref_deg);
void world_position_get();
void position_init();
void position_calibrate(float x, float y, float yaw);

#endif