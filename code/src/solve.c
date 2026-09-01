#include "solve.h"

#include <stdint.h>

// 5 中值
float median5(float a, float b, float c, float d, float e) {
    float arr[5] = {a, b, c, d, e};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4 - i; j++) {
            if (arr[j] > arr[j + 1]) {
                float tmp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = tmp;
            }
        }
    }
    return arr[2];
}

// 更新滤波器
float Median5_Update(Median5 *f, float new_val) {
    f->buf[f->idx] = new_val;
    f->idx = (f->idx + 1) % 5;

    return median5(f->buf[0], f->buf[1], f->buf[2], f->buf[3], f->buf[4]);
}

static VelocityKF_t kf_vx = { .v = 0.0f, .P = 1.0f, .Q = 0.01f, .R = 0.15f};
static VelocityKF_t kf_vy = { .v = 0.0f, .P = 1.0f, .Q = 0.01f, .R = 0.15f};

//卡尔曼融合

float velocity_kf_update(VelocityKF_t *kf, float v_enc, float a_imu, float dt)
{
    //利用 IMU 加速度推算速度
    float v_pred = kf->v + a_imu * dt;
    kf->P = kf->P + kf->Q;
    //更新
    float K = kf->P / (kf->P + kf->R);
    kf->v = v_pred + K * (v_enc - v_pred);
    kf->P = (1.0f - K) * kf->P;
    return kf->v;
}

void solve_car2wheel(float v_x_ref, float v_y_ref, float omega_ref_deg)
{
    float k = (CAR_L + CAR_W);
    float omega_ref_rad = omega_ref_deg * PI_F / 180.0f;
    wheel_speed_target[0] = v_x_ref - v_y_ref - k * omega_ref_rad;  // 左前
    wheel_speed_target[1] = v_x_ref + v_y_ref + k * omega_ref_rad;  // 右前
    wheel_speed_target[2] = v_x_ref + v_y_ref - k * omega_ref_rad;  // 左后
    wheel_speed_target[3] = v_x_ref - v_y_ref + k * omega_ref_rad;  // 右后
}

// 直线移动修正系数
float k_cte = 25.0f;

void solve_world2wheel(float angle_deg, float speed)
{
    // ================= 1. 先算：保留你原来的基础极坐标逻辑 =================
    float angle_rad = (angle_deg - yaw_angle) * PI_F / 180.0f;
    v_x_car_target = speed * cosf(angle_rad);
    v_y_car_target = speed * sinf(angle_rad);
    if(!car_2p_runing_flag)return;
    // ================= 2. 计算横向偏差补偿（向量投影法） =================
    float ab_x = x_target - x_start;
    float ab_y = y_target - y_start;

    if (path_length_sq > 10.0f) // 防止起点和终点重合除以0
    {
        float ap_x = x_world - x_start;
        float ap_y = y_world - y_start;
        
        // 步骤 A：算点积求投影比例 t
        // t 的物理意义：车子在 AB 直线方向上，走过了全长的百分之几
        float t = (ap_x * ab_x + ap_y * ab_y) / path_length_sq;

        if(distance < 500)return;

        // 步骤 B：求出理想直线上的最近点（投影点坐标）
        float proj_x = x_start + t * ab_x;
        float proj_y = y_start + t * ab_y;
        
        // 步骤 C：直接算出从“车子当前位置”指向“直线的最近点”的向量！
        float err_vec_x = proj_x - x_world;
        float err_vec_y = proj_y - y_world;
        
        // 步骤 D：乘上纠偏系数 (建议从 1.0f 到 3.0f 之间调)
        float comp_x_world = k_cte * err_vec_x;
        float comp_y_world = k_cte * err_vec_y;

        // ================= 3. 再转换：将补偿速度转到车体坐标系 =================
        float yaw_rad = yaw_angle * PI_F / 180.0f;
        float comp_x_car =  comp_x_world * cosf(yaw_rad) + comp_y_world * sinf(yaw_rad);
        float comp_y_car = -comp_x_world * sinf(yaw_rad) + comp_y_world * cosf(yaw_rad);

        // ================= 4. 再叠加：将补偿速度直接加给底盘 =================
        v_x_car_target += comp_x_car;
        v_y_car_target += comp_y_car;
    }
}

//参数
#define HISTORY_LEN 260        // 历史缓存长度
int DELAY_TIME  = 251;         // 固定延时 (ms)
float Q_COV_X  = 1.0f;          // 编码器噪声
float Q_COV_Y  = 1.0f;          
float R_COV_X  = 100.0f;          // 视觉噪声
float R_COV_Y  = 100.0f;           

KfHistory_t kf_buffer[HISTORY_LEN];
int current_idx = 0;    

float current_P_x = 1.0f; 
float current_P_y = 1.0f; 

void world_position_get()
{
    //数据获取
    encoder_get();
    yaw_get();
    // acc_get();

    if(!gyro_calibrated)return;
        
    // 获取车体系速度并转换到世界系
    v_x_car = v_x_car*0.9+0.1*v_x_encoder; //velocity_kf_update(&kf_vx, v_x_encoder, acc_x * 1000.0f, SENSOR_SOLVE_dt);
    v_y_car = v_y_car*0.9+0.1*v_y_encoder; //velocity_kf_update(&kf_vy, v_y_encoder, acc_y * 1000.0f, SENSOR_SOLVE_dt);

    float v_x_w = v_x_car * cos_yaw - v_y_car * sin_yaw;
    float v_y_w = v_x_car * sin_yaw + v_y_car * cos_yaw;

    // 当前步积分
    x_world += v_x_w * SENSOR_SOLVE_dt;
    y_world += v_y_w * SENSOR_SOLVE_dt;
    current_P_x += Q_COV_X; 
    current_P_y += Q_COV_Y;

    // 压入历史队列
    kf_buffer[current_idx].x_pred    = x_world;
    kf_buffer[current_idx].y_pred    = y_world;
    kf_buffer[current_idx].v_x_world = v_x_w;
    kf_buffer[current_idx].v_y_world = v_y_w;
    kf_buffer[current_idx].P_x       = current_P_x;
    kf_buffer[current_idx].P_y       = current_P_y;
    kf_buffer[current_idx].timestamp = sys_time;

    // 更新最新写入的索引
    int most_recent_idx = current_idx;
    current_idx = (current_idx + 1) % HISTORY_LEN;

    //视觉延迟补偿
    if (ready) //视觉数据接收标志位
    {
        int hist_idx = -1;
        int min_time_diff = 1e9;

        // 回溯寻找时间戳
        for(int i = 0; i < HISTORY_LEN; i++) 
        {
            int diff = abs((int32_t)(sys_time - DELAY_TIME - kf_buffer[i].timestamp));
            if(diff < min_time_diff) 
            {
                min_time_diff = diff;
                hist_idx = i;
            }
        }
        
        // 如果找到了合理的历史节点
        if(hist_idx != -1 && min_time_diff <= HISTORY_LEN) 
        {
            float h_Px = kf_buffer[hist_idx].P_x;
            float h_Py = kf_buffer[hist_idx].P_y;

            // 计算卡尔曼增益
            float K_x = h_Px / (h_Px + R_COV_X);
            float K_y = h_Py / (h_Py + R_COV_Y);
            // 修正历史点
            float sim_x = kf_buffer[hist_idx].x_pred + K_x * (art_x - kf_buffer[hist_idx].x_pred);
            float sim_y = kf_buffer[hist_idx].y_pred + K_y * (art_y - kf_buffer[hist_idx].y_pred);
            float sim_Px = (1.0f - K_x) * h_Px;
            float sim_Py = (1.0f - K_y) * h_Py;

            if(sim_Px < 0.001f) sim_Px = 0.001f;
            if(sim_Py < 0.001f) sim_Py = 0.001f;
            
            // 快进推演
            int steps = (most_recent_idx - hist_idx + HISTORY_LEN) % HISTORY_LEN;
            
            for (int step = 1; step <= steps; step++) 
            {
                int idx = (hist_idx + step) % HISTORY_LEN;
                
                // 纯线性速度积分
                sim_x += kf_buffer[idx].v_x_world * SENSOR_SOLVE_dt;
                sim_y += kf_buffer[idx].v_y_world * SENSOR_SOLVE_dt;
                sim_Px += Q_COV_X;
                sim_Py += Q_COV_Y;
                
                // 刷新队列中的旧数据，保证下次修正路径连贯
                kf_buffer[idx].x_pred = sim_x;
                kf_buffer[idx].y_pred = sim_y;
                kf_buffer[idx].P_x = sim_Px;
                kf_buffer[idx].P_y = sim_Py;
            }

            // 覆盖
            x_world = sim_x;
            y_world = sim_y;
            current_P_x = sim_Px;
            current_P_y = sim_Py;
        }

        ready = 0; 
    }
    player = GET_ID((int)x_world / 200 - 1, (int)(-y_world) / 200 - 1);
}

void position_init()
{
    gyro_z_raw = 0.0f;
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
    //printf("%.2f,%.2f,%.2f\n",x,y,yaw);
}
