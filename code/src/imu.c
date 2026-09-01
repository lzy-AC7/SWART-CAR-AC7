#include "imu.h"


// --- 基础宏定义 ---
#define RAD_TO_DEG         57.29577951f     
#define DEG_TO_RAD         0.0174532925f    

// 根据IMU实际安装方向调整三轴正负号
#define IMU_X_DIR          1.0f
#define IMU_Y_DIR          1.0f
#define IMU_Z_FRONT_DIR    1.0f 

float gyro_var_sum[3] = {0.0f, 0.0f, 0.0f}; // 用于累加方差

// 全局四元数，初始化为 Yaw = -90 度的状态 (Roll=0, Pitch=0)
static float q[4] = {0.7071068f, 0.0f, 0.0f, -0.7071068f};
void yaw_get(void)
{
    // 1. 获取并转换三轴原始角速度
    imu660ra_get_gyro();
    float gyro_raw[3];
    gyro_raw[0] = IMU_X_DIR       * imu660ra_gyro_transition(imu660ra_gyro_x);
    gyro_raw[1] = IMU_Y_DIR       * imu660ra_gyro_transition(imu660ra_gyro_y);
    gyro_raw[2] = IMU_Z_FRONT_DIR * imu660ra_gyro_transition(imu660ra_gyro_z);

    // 用于二阶龙格库塔（RK2）的上一周期角速度缓存（弧度制）
    static float last_gx = 0.0f;
    static float last_gy = 0.0f;
    static float last_gz = 0.0f;

    /* ========================================================================= */
    /* 标定阶段（均方差 3-Sigma 优化版）                                         */
    /* ========================================================================= */
    if (!gyro_calibrated)
    {
        uint32_t half_epoch = GYRO_CALIB_EPOCH / 2;

        if (gyro_epoch == 0)
        {
            for (int i = 0; i < 3; i++) {
                gyro_sum[i] = 0.0f;
                gyro_var_sum[i] = 0.0f;
            }
        }

        /* 前半段：累加求平均值（获取零偏 Bias） */
        if (gyro_epoch < half_epoch)
        {
            for (int i = 0; i < 3; i++) gyro_sum[i] += gyro_raw[i];
        }
        /* 后半段：累加均方差（评估本底噪声特征） */
        else 
        {
            // 刚进入后半段时，计算确切的零偏
            if (gyro_epoch == half_epoch)
            {
                for (int i = 0; i < 3; i++) gyro_bias[i] = gyro_sum[i] / half_epoch;
            }

            // 累加方差的平方项
            for (int i = 0; i < 3; i++) {
                float tmp = gyro_raw[i] - gyro_bias[i];
                gyro_var_sum[i] += (tmp * tmp);
            }
        }

        gyro_epoch++;

        /* 标定结束，使用 3-Sigma 法则计算死区 */
        if (gyro_epoch >= GYRO_CALIB_EPOCH)
        {
            for (int i = 0; i < 3; i++) {
                // 计算标准差 StdDev = sqrt( Sum((x - mu)^2) / N )
                float std_dev = sqrtf(gyro_var_sum[i] / half_epoch);
                // 3-Sigma 原则：3倍标准差可以覆盖 99.7% 的高斯随机噪声
                // 结合调节系数 GYRO_DEADBAND_K，得到非常稳定的死区边界
                gyro_deadband[i] = GYRO_DEADBAND_K * 3.0f * std_dev;
            }

            gyro_calibrated = true;
            gyro_epoch = 0;
            
            // 标定成功时，重置四元数为 Yaw = -90 度的初始姿态
            q[0] = 0.7071068f; 
            q[1] = 0.0f; 
            q[2] = 0.0f; 
            q[3] = -0.7071068f;
            
            yaw_angle = -90.0f; 

            // 清空角速度历史缓存
            last_gx = 0.0f; last_gy = 0.0f; last_gz = 0.0f;
        }
        return;
    }

    /* ========================================================================= */
    /* 正常运行阶段：去零偏、死区抑制与 RK2 积分解算                             */
    /* ========================================================================= */
    float gyro_corr[3];
    for (int i = 0; i < 3; i++) {
        gyro_corr[i] = gyro_raw[i] - gyro_bias[i];
        if (fabsf(gyro_corr[i]) < gyro_deadband[i]) 
        {
            gyro_corr[i] = 0.0f; // 处于稳态本底噪声内，直接斩断
        }
    }

    float gx = gyro_corr[0] * DEG_TO_RAD;
    float gy = gyro_corr[1] * DEG_TO_RAD;
    float gz = gyro_corr[2] * DEG_TO_RAD;

    float gx_mid = (gx + last_gx) * 0.5f;
    float gy_mid = (gy + last_gy) * 0.5f;
    float gz_mid = (gz + last_gz) * 0.5f;

    last_gx = gx; last_gy = gy; last_gz = gz;

    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float half_dt = SENSOR_SOLVE_dt * 0.5f; // SENSOR_SOLVE_dt 需在外部宏定义，例如 0.005f (200Hz)
    
    q[0] += (-q1 * gx_mid - q2 * gy_mid - q3 * gz_mid) * half_dt;
    q[1] += ( q0 * gx_mid + q2 * gz_mid - q3 * gy_mid) * half_dt;
    q[2] += ( q0 * gy_mid - q1 * gz_mid + q3 * gx_mid) * half_dt;
    q[3] += ( q0 * gz_mid + q1 * gy_mid - q2 * gx_mid) * half_dt;

    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0.0f) {
        float inv_norm = 1.0f / norm;
        q[0] *= inv_norm; q[1] *= inv_norm; q[2] *= inv_norm; q[3] *= inv_norm;
    }

    yaw_angle = atan2f(2.0f * (q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3])) * RAD_TO_DEG;
    sin_yaw = 2.0f * (q[0]*q[3] + q[1]*q[2]);
    cos_yaw = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
}


// --- 加速度计相关参数与状态变量 ---
#define ACC_CALIB_EPOCH    1500   
#define ACC_DEADBAND_K     1.2f   

bool acc_calibrated = false;
uint32_t acc_epoch = 0;

float acc_sum[2]     = {0.0f, 0.0f};
float acc_var_sum[2] = {0.0f, 0.0f}; // 用于累加方差
float acc_bias[2]    = {0.0f, 0.0f};
float acc_deadband[2]= {0.0f, 0.0f};

// 全局输出的校准后 XY 加速度
float acc_x = 0.0f;
float acc_y = 0.0f;

void acc_get(void)
{
    // 1. 获取并转换 XY 轴原始加速度
    imu660ra_get_acc(); 
    float acc_raw[2];
    acc_raw[0] = -1.0f * imu660ra_acc_transition(imu660ra_acc_x) * 9.80665f;
    acc_raw[1] = -1.0f * imu660ra_acc_transition(imu660ra_acc_y) * 9.80665f;

    /* ========================================================================= */
    /* 标定阶段（均方差 3-Sigma 优化版）                                         */
    /* 注意：标定期间要求车体绝对静止且水平                                      */
    /* ========================================================================= */
    if (!acc_calibrated)
    {
        uint32_t half_epoch = ACC_CALIB_EPOCH / 2;

        if (acc_epoch == 0)
        {
            for (int i = 0; i < 2; i++) {
                acc_sum[i] = 0.0f;
                acc_var_sum[i] = 0.0f;
            }
        }

        /* 前半段：累加求平均值（获取零偏 Bias） */
        if (acc_epoch < half_epoch)
        {
            for (int i = 0; i < 2; i++) acc_sum[i] += acc_raw[i];
        }
        /* 后半段：累加均方差（评估机架振动噪声特征） */
        else 
        {
            if (acc_epoch == half_epoch)
            {
                for (int i = 0; i < 2; i++) acc_bias[i] = acc_sum[i] / half_epoch;
            }

            for (int i = 0; i < 2; i++) {
                float tmp = acc_raw[i] - acc_bias[i];
                acc_var_sum[i] += (tmp * tmp);
            }
        }

        acc_epoch++;

        /* 标定结束，使用 3-Sigma 法则计算死区 */
        if (acc_epoch >= ACC_CALIB_EPOCH)
        {
            for (int i = 0; i < 2; i++) {
                float std_dev = sqrtf(acc_var_sum[i] / half_epoch);
                acc_deadband[i] = ACC_DEADBAND_K * 3.0f * std_dev;
            }

            acc_calibrated = true;
            acc_epoch = 0;
        }
        return;
    }

    /* ========================================================================= */
    /* 正常运行阶段：去零偏与死区抑制                                            */
    /* ========================================================================= */
    float acc_corr[2];
    for (int i = 0; i < 2; i++) {
        acc_corr[i] = acc_raw[i] - acc_bias[i];
        
        if (fabsf(acc_corr[i]) < acc_deadband[i]) {
            acc_corr[i] = 0.0f; // 滤除电机的微小高频震颤
        }
    }

    acc_x = acc_corr[0];
    acc_y = acc_corr[1];
}