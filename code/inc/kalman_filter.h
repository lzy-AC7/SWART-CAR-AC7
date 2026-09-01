#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#include "common.h"

/* 函数声明 */
void kalman_filter_1d_init(Kalman_Filter_1D *kf, float q, float r, float p, float x);
float kalman_filter_1d_update(Kalman_Filter_1D *kf, float z, float a, float dt);

void kalman_filter_2d_init(Kalman_Filter_2D *kf, float q, float r, float p, float vx, float vy);
void kalman_filter_2d_update(Kalman_Filter_2D *kf, float vx_measured, float vy_measured, float ax, float ay, float dt);
void kalman_filter_2d_get_velocity(Kalman_Filter_2D *kf, float *vx, float *vy);
void velocity_fusion_init(void);

#endif

