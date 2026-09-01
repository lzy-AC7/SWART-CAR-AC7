#include "solve.h"

void solve_car2wheel(float v_x_ref, float v_y_ref, float omega_ref_rad)
{
    float k = (CAR_L + CAR_W);

    wheel_speed_target[0] = v_x_ref - v_y_ref - k * omega_ref_rad;  // 左前
    wheel_speed_target[1] = v_x_ref + v_y_ref + k * omega_ref_rad;  // 右前
    wheel_speed_target[2] = v_x_ref + v_y_ref - k * omega_ref_rad;  // 左后
    wheel_speed_target[3] = v_x_ref - v_y_ref + k * omega_ref_rad;  // 右后
}

// 换向瞬间前馈系数
#define K_FF_YAW    (0.003f)  // 侧向加速度对自旋的前馈系数，需根据甩头严重程度微调
// 越接近 1.0，前馈力矩持续时间越长；越接近 0.0，越像原始脉冲
#define LPF_ALPHA_FF 0.4f

static float last_v_y_ref = 0.0f;
static float filtered_omega_ff = 0.0f; //记录上次的前馈值

void solve_world2wheel(float angle_deg, float speed, float omega_ref_deg)
{
    // 无前馈
    // float angle_rad = (angle_deg-yaw_angle) * PI_F / 180.0f;
    // float omega_ref_rad = omega_ref_deg * PI_F / 180.0f;
    // solve_car2wheel(speed * cosf(angle_rad), speed * sinf(angle_rad), omega_ref_rad);

    // 有前馈
    float angle_rad = (angle_deg - yaw_angle) * PI_F / 180.0f;
    float v_x_ref = speed * cosf(angle_rad);
    float v_y_ref = speed * sinf(angle_rad);

    // 计算目标速度的瞬时变化量
    float delta_v_y = v_y_ref - last_v_y_ref;
    last_v_y_ref = v_y_ref;

    // 计算原始前馈目标 (注意这里吸收了 dt 的概念，K_FF_YAW 取值要重新调)
    float raw_omega_ff = delta_v_y * K_FF_YAW;

    // 对前馈输出进行低通滤波，将 5ms 的脉冲“拉长”为一个平滑衰减的力矩
    filtered_omega_ff = (LPF_ALPHA_FF * filtered_omega_ff) + ((1.0f - LPF_ALPHA_FF) * raw_omega_ff);

    // 融合
    float omega_ref_rad = (omega_ref_deg * PI_F / 180.0f) + filtered_omega_ff;

    solve_car2wheel(v_x_ref, v_y_ref, omega_ref_rad);
}

void world_position_get()
{
    encoder_get();
    yaw_get();
    
    if(!gyro_calibrated || !acc_calibrated)
        return;

    /* 第一次初始化卡尔曼滤波器 */
    if(!kalman_initialized)
    {
        velocity_fusion_init();
        v_x_car = v_x_encoder;
        v_y_car = v_y_encoder;
        // v_x_car = v_x_imu;
        // v_y_car = v_y_imu;
        return;
    }

    /* 如果只使用编码器输出，则跳过融合 */
    if(kalman_filter_enable == false)
    {
        v_x_car = v_x_encoder;
        v_y_car = v_y_encoder;
        // v_x_car = v_x_imu;
        // v_y_car = v_y_imu;
    }
    else
    {
        /* 使用卡尔曼滤波器融合编码器速度（测量）和加速度（预测） */
        kalman_filter_2d_update(&velocity_filter, 
                                v_x_encoder,        // 编码器提供的速度测量
                                v_y_encoder,        // 编码器提供的速度测量
                                acc_x,              // 加速度计提供的加速度
                                acc_y,              // 加速度计提供的加速度
                                SENSOR_SOLVE_dt);   // 时间间隔

        /* 获取卡尔曼滤波后的速度估计 */
        kalman_filter_2d_get_velocity(&velocity_filter, &v_x_car, &v_y_car);
    }

    /* 车体速度旋转到世界坐标系，进行距离积分 */
    float cos_yaw = cosf(yaw_angle*PI_F/180.0f);
    float sin_yaw = sinf(yaw_angle*PI_F/180.0f);


    // x_world += (v_x_car * cos_yaw - v_y_car * sin_yaw) * SENSOR_SOLVE_dt;
    // y_world += (v_x_car * sin_yaw + v_y_car * cos_yaw) * SENSOR_SOLVE_dt;
    
}

void position_init()
{
    gyro_z_raw = 0.0f;
    acc_x_raw = 0.0f; 
    acc_y_raw = 0.0f;
    v_x_encoder = 0.0f;
    v_y_encoder = 0.0f;
    v_x_car = v_y_car = omega_car = 0.0f;
    yaw_angle_target = yaw_angle = yaw_init;
    speed_target_angle = 0.0f;
    speed_target_value = 0.0f;
}

void position_calibrate(float x, float y, float yaw)
{

    yaw_angle = yaw;
    x_world = x;
    y_world = y;
    //printf("%f,%f,%f\n",art_x,art_y,art_yaw);
}
