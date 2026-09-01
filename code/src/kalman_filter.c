/*
 * 卡尔曼滤波器实现
 * 用于融合加速度计和编码器的测速
 */

#include "kalman_filter.h"

/* 1D 卡尔曼滤波器初始化 */
void kalman_filter_1d_init(Kalman_Filter_1D *kf, float q, float r, float p, float x)
{
    kf->Q = q;              // 过程噪声方差（加速度不确定性）
    kf->R = r;              // 测量噪声方差
    kf->P = p;              // 估计误差方差
    kf->X = x;              // 初始状态（速度）
    kf->K = 0.0f;           // 卡尔曼增益
}

/* 1D 卡尔曼滤波 - 支持加速度驱动的预测
 * z: 测量值（编码器速度）
 * a: 加速度（从加速度计得到）
 * dt: 时间间隔
 */
float kalman_filter_1d_update(Kalman_Filter_1D *kf, float z, float a, float dt)
{
    // 预测阶段：线性运动模型 v(k+1) = v(k) + a*dt
    float x_predict = kf->X + a * dt;
    float P_predict = kf->P + kf->Q;
    
    // 更新阶段
    kf->K = P_predict / (P_predict + kf->R);  // 卡尔曼增益
    kf->X = x_predict + kf->K * (z - x_predict);  // 状态更新
    kf->P = (1.0f - kf->K) * P_predict;  // 误差协方差更新
    
    return kf->X;
}

/* 2D 卡尔曼滤波器初始化（用于2轴速度）*/
void kalman_filter_2d_init(Kalman_Filter_2D *kf, float q, float r, float p, float vx, float vy)
{
    kalman_filter_1d_init(&kf->kf_x, q, r, p, vx);
    kalman_filter_1d_init(&kf->kf_y, q, r, p, vy);
}

/* 2D 卡尔曼滤波更新 - 融合加速度计和编码器
 * vx_measured, vy_measured: 编码器测得的速度
 * ax, ay: 加速度计测得的加速度
 */
void kalman_filter_2d_update(Kalman_Filter_2D *kf, float vx_measured, float vy_measured, float ax, float ay, float dt)
{
    kf->vx = kalman_filter_1d_update(&kf->kf_x, vx_measured, ax, dt);
    kf->vy = kalman_filter_1d_update(&kf->kf_y, vy_measured, ay, dt);
}

/* 获取2D滤波后的速度 */
void kalman_filter_2d_get_velocity(Kalman_Filter_2D *kf, float *vx, float *vy)
{
    *vx = kf->vx;
    *vy = kf->vy;
}

void velocity_fusion_init(void)
{
    /* 初始化2D卡尔曼滤波器 
     * Q: 过程噪声 - 越大越相信加速度计（低延迟）
     * R: 测量噪声 - 越小越相信编码器（高精度）
     * P: 初始误差协方差
     */
    kalman_filter_2d_init(&velocity_filter, 
                          KALMAN_Q_VELOCITY,     // Q
                          KALMAN_R_VELOCITY,     // R
                          KALMAN_P_VELOCITY,     // P
                          0.0f, 0.0f);           // 初始速度0
    kalman_initialized = true;
}
