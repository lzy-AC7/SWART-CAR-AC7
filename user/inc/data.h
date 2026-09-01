#ifndef _DATA_H_
#define _DATA_H_

/* 变量类型定义 */

extern float GYRO_DEADBAND_K;        // 死区放大系数

typedef struct PID
{
	/* PID 参数 */
	float Kp;
	float Ki;
	float Kd;
	float output_limit;
	float i_limit;
    float output_deadband;
	
	/* PID 变量 */  
	float now_err;    
	float last_err;    
	float last_last_err;
	float sigma_err;
	float value;
    float value_delta;
}PID;

/* 1维卡尔曼滤波器结构体 */
typedef struct 
{
    float Q;    // 过程噪声方差（加速度不确定性）
    float R;    // 测量噪声方差
    float P;    // 估计误差方差
    float X;    // 状态估计值（速度）
    float K;    // 卡尔曼增益
} Kalman_Filter_1D;

/* 2维卡尔曼滤波器结构体（分别处理X和Y方向）*/
typedef struct 
{
    Kalman_Filter_1D kf_x;
    Kalman_Filter_1D kf_y;
    float vx;           // 当前X方向速度估计
    float vy;           // 当前Y方向速度估计
} Kalman_Filter_2D;


/* 数据变量 */ 

extern uint32 sys_time;

/* debug uart*/
extern uint8 debug_data[128];

/* 识别分类 uart*/
#define UART_REC_INDEX              (UART_4      )
#define UART_REC_BAUDRATE           (115200)      
#define UART_REC_TX_PIN             (UART4_TX_C16)
#define UART_REC_RX_PIN             (UART4_RX_C17)
#define UART_REC_PRIORITY           (LPUART4_IRQn)

extern uint8 uart_rec_get_data[64];                                                        // 串口接收数据缓冲区
extern uint8 fifo_rec_get_data[64];                                                        // fifo 输出读出缓冲区
extern uint8 get_rec_data;                                                             // 接收数据变量
extern uint32 fifo_rec_data_count;                                                     // fifo 数据个数
extern fifo_struct uart_rec_data_fifo;

/* 识别全局 uart*/
#define UART_MAP_INDEX              (UART_2      )                              
#define UART_MAP_BAUDRATE           (115200)                                 
#define UART_MAP_TX_PIN             (UART2_TX_B18)                         
#define UART_MAP_RX_PIN             (UART2_RX_B19)    
#define UART_MAP_PRIORITY           (LPUART2_IRQn)  

extern uint8 uart_map_get_data[256];                                                        // 串口接收数据缓冲区
extern uint8 fifo_map_get_data[256];                                                        // fifo 输出读出缓冲区
extern uint8 get_map_data;                                                             // 接收数据变量
extern uint32 fifo_map_data_count;                                                     // fifo 数据个数
extern fifo_struct uart_map_data_fifo;

extern bool maping;
extern uint32 art_time;
extern float v_x_art;
extern float v_y_art;
extern float v_art;
extern float art_x;
extern float art_y; 
extern float art_yaw;
extern uint8 proc_pos_timer_count;

extern int art_yaw_count;
extern int art_yaw_cou;

extern bool ready;

/* WIFI */
extern bool wifi_en;
extern uint8 wifi_data[256];

/*编码器*/
extern int16 encoder[4];
extern float wheel_speed[4];

/* IMU数据 */

extern float gyro_z_raw;                        // 原始转换后的角速度 (deg/s or rad/s)
extern float gyro_z;                            // 最终输出（去零漂+死区）

/* 加速度原始与处理值 */
extern float acc_x_raw;                         // 原始转换后的加速度 (m/s^2)
extern float acc_y_raw;
extern float acc_x;                             // 最终输出（去零漂）
extern float acc_y;

/* 加速度标定相关 */
extern bool  acc_calibrated;
extern uint16 acc_epoch;
extern float acc_bias_x;
extern float acc_bias_y;
extern float acc_sum_x;
extern float acc_sum_y;
extern float acc_max_x;
extern float acc_min_x;
extern float acc_max_y;
extern float acc_min_y;
extern float acc_deadband_x;
extern float acc_deadband_y;
extern float v_x_imu;    /* 由加速度积分得到的速度（车体系） */
extern float v_y_imu;

/* ===== 角速度标定相关 ===== */
#define FLT_MAX 3.402823466e+38F
extern bool  gyro_calibrated;
extern uint16 gyro_epoch;

// extern float gyro_bias;                         // 零偏
// extern float gyro_sum;                          // 累加和

// extern float gyro_max;                          // 噪声极值
// extern float gyro_min;
// extern float gyro_deadband;

extern float gyro_bias[3];
extern float gyro_deadband[3];
extern float gyro_max[3], gyro_min[3], gyro_sum[3];

extern float gyro_z_lpf;
extern float yaw_angle;                         // 航向角

extern float x_world;        // 世界坐标 X
extern float y_world;        // 世界坐标 Y
extern float v_x_car;      // 车体坐标系前向速度
extern float v_y_car;      // 车体坐标系侧向速度
extern float omega_car;    // 车体角速度（Z轴）计算自轮速

/* 卡尔曼滤波器 - 速度融合 */
extern  Kalman_Filter_2D velocity_filter;  // 速度卡尔曼滤波器
extern bool kalman_initialized;
extern float v_x_encoder;   // 编码器测得的前向速度
extern float v_y_encoder;   // 编码器测得的侧向速度
/* 调试控制 */
extern bool kalman_filter_enable;  // 如果为 true, 则直接使用编码器速度而不融合
/* PID 控制器 */

extern PID pid_speed[4];
extern PID pid_yaw;
extern float wheel_speed_target[4];

extern float yaw_init;
extern float yaw_init;
extern float x_init;
extern float y_init;

extern float yaw_angle_target;
extern float speed_target_angle;
extern float speed_target_value;

/* 坐标移动 */
extern bool car_2p_runing_flag;
extern uint16 car_2p_timer_count;
extern bool car_2p_timer_flag;

/* 路径移动 */
extern bool car_runing_path_flag;
extern uint16 car_runing_path_timer_count;
extern bool car_runing_path_timer_flag;

extern float run_speed;
extern bool over;
extern bool fast_flag;

extern int player;
extern int A_path_x[251*10],A_path_y[251*10];
extern int A_path_size;

extern bool OPTIMAL;//炸弹最优路径
extern int checkpoint;

/* 底盘参数*/
#define ENCODER_LINE_NUM		(float)(1024)			// 编码器线数
#define GEAR_RATIO 				(float)(2.333)		    // 齿轮比（7/3）
#define WHEEL_CIRCUMFERENCE 	(float)(197.920)		// 轮周长（mm）
#define CAR_L                   (float)(180.0)          // 车体前后轴半长
#define CAR_W                   (float)(198.0)          // 车体左右轴半宽
/* 轮速计算因子 */
#define WHEEL_SPEED_FACTOR      (float)(WHEEL_CIRCUMFERENCE / (1.0f * ENCODER_LINE_NUM * GEAR_RATIO))

/* 逐飞库头文件 */
#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "isr.h"

/* 用户头文件 */
#include "common.h"

#endif
