#ifndef _SOLVE_H_
#define _SOLVE_H_

#include "common.h"

// 滑动窗口结构体
typedef struct 
{
    float buf[5];
    uint8_t idx;
} Median5;


typedef struct {
    float v;           // 融合后的车体速度
    float P;           // 估计协方差
    float Q;           // 过程噪声协方差
    float R;           // 动态测量噪声协方差
} VelocityKF_t;

typedef struct {
    float x_pred;       // 世界坐标系预测X
    float y_pred;       // 世界坐标系预测Y
    float v_x_world;    // 世界坐标系X速度
    float v_y_world;    // 世界坐标系Y速度
    float P_x;          // X轴位置协方差
    float P_y;          // Y轴位置协方差
    uint32_t timestamp; // 系统时间戳(ms)
} KfHistory_t;

extern float k_cte;
extern int DELAY_TIME;
extern float Q_COV_X;
extern float Q_COV_Y;
extern float R_COV_X;
extern float R_COV_Y;

float Median5_Update(Median5 *f, float new_val);
float velocity_kf_update(VelocityKF_t *kf, float v_enc, float a_imu, float dt);
void solve_car2wheel(float v_x_ref, float v_y_ref, float omega_ref);
void solve_world2wheel(float angle_deg, float speed);
void world_position_get();
void position_init();
void position_calibrate(float x, float y, float yaw);

#endif