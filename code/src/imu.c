#include "imu.h"

#include <math.h>
#include <float.h>
#include <stdbool.h>

// --- 需要在外部或全局定义的变量与宏 ---
#define RAD_TO_DEG         57.29577951f     // 弧度转角度
#define DEG_TO_RAD         0.0174532925f    // 角度转弧度

// 根据IMU实际安装方向调整三轴正负号
#define IMU_X_DIR          1.0f
#define IMU_Y_DIR          1.0f
#define IMU_Z_FRONT_DIR    1.0f 

// 全局四元数，初始化为 Yaw = -90 度的状态 (Roll=0, Pitch=0)
static float q[4] = {0.7071068f, 0.0f, 0.0f, -0.7071068f};

void yaw_get(void)
{
    // 1. 获取并转换三轴原始角速度 (假设 transition 转换为 deg/s)
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
    /* 标定阶段（X, Y, Z 三轴并行标定）                                             */
    /* ========================================================================= */
    if (!gyro_calibrated)
    {
        if (gyro_epoch == 0)
        {
            for (int i = 0; i < 3; i++) {
                gyro_sum[i] = 0.0f;
                gyro_max[i] = -FLT_MAX;
                gyro_min[i] =  FLT_MAX;
            }
        }

        /* 前半段：求三轴零偏 */
        if (gyro_epoch < GYRO_CALIB_EPOCH / 2)
        {
            for (int i = 0; i < 3; i++) gyro_sum[i] += gyro_raw[i];
        }
        /* 后半段：计算噪声包络 */
        else 
        {
            if (gyro_epoch == GYRO_CALIB_EPOCH / 2)
            {
                for (int i = 0; i < 3; i++) gyro_bias[i] = gyro_sum[i] / (GYRO_CALIB_EPOCH / 2);
            }

            for (int i = 0; i < 3; i++) {
                float tmp = gyro_raw[i] - gyro_bias[i];
                if (tmp > gyro_max[i]) gyro_max[i] = tmp;
                if (tmp < gyro_min[i]) gyro_min[i] = tmp;
            }
        }

        gyro_epoch++;

        /* 标定结束，计算死区 */
        if (gyro_epoch >= GYRO_CALIB_EPOCH)
        {
            for (int i = 0; i < 3; i++) {
                float noise = fmaxf(fabsf(gyro_max[i]), fabsf(gyro_min[i]));
                gyro_deadband[i] = GYRO_DEADBAND_K * noise;
            }

            gyro_calibrated = true;
            gyro_epoch = 0;
            
            // 标定成功时，重置四元数为 Yaw = -90 度的初始姿态
            q[0] = 0.7071068f; 
            q[1] = 0.0f; 
            q[2] = 0.0f; 
            q[3] = -0.7071068f;
            
            // 同步更新初始角度变量
            yaw_angle = -90.0f; 

            // 清空角速度历史缓存，避免标定期间的脏数据污染第一次积分
            last_gx = 0.0f;
            last_gy = 0.0f;
            last_gz = 0.0f;
        }
        return;
    }

    /* ========================================================================= */
    /* 正常运行阶段                                                              */
    /* ========================================================================= */

    // 1. 三轴去零偏与死区抑制
    float gyro_corr[3];
    for (int i = 0; i < 3; i++) {
        gyro_corr[i] = gyro_raw[i] - (1.0)*gyro_bias[i];
        if (fabsf(gyro_corr[i]) < gyro_deadband[i]) {
            // 被死区抑制时，强制清零
            gyro_corr[i] = 0.0f;
        }
    }

    // 2. 融合车体 Z 轴角速度 (将互补滤波保留在车体系Z轴)
    float wheel_rot_sum = -wheel_speed[0] + wheel_speed[1] - wheel_speed[2] + wheel_speed[3];
    float omega_wheel   = wheel_rot_sum / (4.0f * (CAR_L + CAR_W));
    
    float omega_z_fused = YAW_ALPHA * gyro_corr[2] + (1.0f - YAW_ALPHA) * omega_wheel;

    // 强制卡死静止漂移优化
    if (gyro_corr[2] == 0.0f && fabsf(wheel_rot_sum) < 0.05f) 
    {
        omega_z_fused = 0.0f;
    }

    // 3. 将当前角速度转换为弧度制 (Rad/s)，构建当前周期的 3D 旋转向量
    float gx = gyro_corr[0] * DEG_TO_RAD;
    float gy = gyro_corr[1] * DEG_TO_RAD;
    float gz = omega_z_fused * DEG_TO_RAD; // Z轴使用融合了轮速的角速度

    // 4. 四元数二阶龙格库塔（RK2）更新核心
    // 计算当前周期与上一周期的中点角速度（中点轴向角速率法）
    float gx_mid = (gx + last_gx) * 0.5f;
    float gy_mid = (gy + last_gy) * 0.5f;
    float gz_mid = (gz + last_gz) * 0.5f;

    // 保存当前角速度，供下一周期作为“上一周期历史值”使用
    last_gx = gx;
    last_gy = gy;
    last_gz = gz;

    // 缓存当前的四元数状态
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float half_dt = SENSOR_SOLVE_dt * 0.5f;
    
    // 使用中点角速度进行微分方程步进
    q[0] += (-q1 * gx_mid - q2 * gy_mid - q3 * gz_mid) * half_dt;
    q[1] += ( q0 * gx_mid + q2 * gz_mid - q3 * gy_mid) * half_dt;
    q[2] += ( q0 * gy_mid - q1 * gz_mid + q3 * gx_mid) * half_dt;
    q[3] += ( q0 * gz_mid + q1 * gy_mid - q2 * gx_mid) * half_dt;

    // 5. 四元数归一化（防止浮点数累积误差导致的数学解算失真）
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0.0f) {
        float inv_norm = 1.0f / norm;
        q[0] *= inv_norm; q[1] *= inv_norm; q[2] *= inv_norm; q[3] *= inv_norm;
    }

    // 6. 从 3D 姿态中解算出抗空间倾斜的真实 Yaw 轴角度
    // atan2f 自动输出 [-pi, pi]，乘上 RAD_TO_DEG 后天然映射到 [-180, 180]
    yaw_angle = atan2f(2.0f * (q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3])) * RAD_TO_DEG;
}