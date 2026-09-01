#include "common.h"
#include "data.h"

/* 变量定义 */	
float GYRO_DEADBAND_K =  1.0f;
uint32 sys_time = 0;

/* debug uart*/
uint8 debug_data[128];

/* 识别分类 uart*/
uint8 uart_rec_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo_rec_get_data[64];                                                        // fifo 输出读出缓冲区
uint8 get_rec_data = 0;                                                             // 接收数据变量
uint32 fifo_rec_data_count = 0;                                                     // fifo 数据个数
fifo_struct uart_rec_data_fifo;

/* 识别全局 uart*/
uint8 uart_map_get_data[256];                                                        // 串口接收数据缓冲区
uint8 fifo_map_get_data[256];                                                        // fifo 输出读出缓冲区
uint8 get_map_data = 0;                                                             // 接收数据变量
uint32 fifo_map_data_count = 0;                                                     // fifo 数据个数
fifo_struct uart_map_data_fifo;

bool maping = 0;
float art_x;
float art_y; 
float art_yaw;
uint8 proc_pos_timer_count = 0;
int art_yaw_count = 0;
int art_yaw_cou = 0;
bool ready = 0;
/* WIFI*/
bool wifi_en = false;
uint8 wifi_data[256];

/* 编码器初始值 */
int16 encoder[4];

float wheel_speed[4];            

/* IMU数据 */

float gyro_z_raw = 0.0f;       // 原始转换后的角速度 (deg/s or rad/s)
float acc_x_raw = 0.0f;        // 原始转换后的加速度 (m/s^2)
float acc_y_raw = 0.0f;        // 原始转换后的加速度 (m/s^2)

float gyro_z = 0.0f;           // 最终输出（去零漂+死区）
float acc_x = 0.0f;            // 最终输出（去零漂）
float acc_y = 0.0f;            // 最终输出（去零漂）

/* ===== 加速度标定相关 ===== */
bool  acc_calibrated = true;
uint16 acc_epoch = 0;

float acc_bias_x = 0.0f;
float acc_bias_y = 0.0f;
float acc_sum_x = 0.0f;
float acc_sum_y = 0.0f;

float acc_max_x = -FLT_MAX;
float acc_min_x =  FLT_MAX;
float acc_max_y = -FLT_MAX;
float acc_min_y =  FLT_MAX;
float acc_deadband_x = 0.0f;
float acc_deadband_y = 0.0f;

/* 加速度积分出的速度（仅由加速度积分得到，用于互补滤波） */
float v_x_imu = 0.0f;
float v_y_imu = 0.0f;

/* ===== 角速度标定相关 ===== */
bool  gyro_calibrated = false;
uint16 gyro_epoch = 0;

// float gyro_bias = 0.0f;        // 零偏
// float gyro_sum = 0.0f;         // 累加和

// float gyro_max = -FLT_MAX;     // 噪声极值
// float gyro_min =  FLT_MAX;
// float gyro_deadband = 0.0f;

float gyro_bias[3];
float gyro_deadband[3];
float gyro_max[3], gyro_min[3], gyro_sum[3];

float yaw_angle = YAW_ANGLE_INIT;         // 航向角

/* 小车状态 */
float x_world = 0.0f;        // 世界坐标 X
float y_world = 0.0f;        // 世界坐标 Y

/* 小车运动学输出 */
float v_x_car = 0.0f;      // 车体坐标系前向速度
float v_y_car = 0.0f;      // 车体坐标系侧向速度
float omega_car = 0.0f;    // 车体角速度（Z轴）计算自轮速

/* 卡尔曼滤波器 - 速度融合 */
Kalman_Filter_2D velocity_filter;  // 速度卡尔曼滤波器实例
/* 卡尔曼滤波器初始化标志 */
bool kalman_initialized = false;

float v_x_encoder = 0.0f;   // 编码器测得的前向速度
float v_y_encoder = 0.0f;   // 编码器测得的侧向速度

/* 调试控制 */
bool kalman_filter_enable = KALMAN_EN;  // 默认不使用卡尔曼融合

/* PID 控制器 */

PID pid_speed[4] = 
{
    {0.25f, 0.05f, 0.0f, 300.0f, 100.0f , 60.0f},
    {0.25f, 0.05f, 0.0f, 300.0f, 100.0f , 60.0f},
    {0.25f, 0.05f, 0.0f, 300.0f, 100.0f , 60.0f},
    {0.25f, 0.05f, 0.0f, 300.0f, 100.0f , 60.0f}
};

PID pid_yaw =
{
    7.0f, 0.00f, 0.0f, 300.0f, 100.0f,0.1f
};

float wheel_speed_target[4];

float yaw_init = -90.0;
float x_init = 150;
float y_init = -1200;

float yaw_angle_target = 0.0f;
float speed_target_angle = 0.0f;//坐标系角度
float speed_target_value = 0.0f;

/* 坐标移动 */
bool car_2p_runing_flag = 0;
uint16 car_2p_timer_count = 0;
bool car_2p_timer_flag = 0;

/* 路径移动 */
bool car_runing_path_flag = 0;
uint16 car_runing_path_timer_count = 0;
bool car_runing_path_timer_flag = 0;

float run_speed = 2000;//运行速度
bool over = 0;
bool fast_flag = 1;

//运行路径
int player;
int A_path_x[251*10],A_path_y[251*10];
int A_path_size;

bool OPTIMAL = 0;//炸弹最优路径  0为最优模式
int checkpoint = 0;
