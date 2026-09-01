#ifndef _COMMON_H_
#define _COMMON_H_

/****************************** 硬件设置 ******************************/

/* WIFI */
#define WIFI_SSID_TEST          "holo"//"TP-LINK_F84B"// wifi名称
#define WIFI_PASSWORD_TEST      "12468910"//"yishe123456789"// 如果需要连接的WIFI 没有密码则需要将 "12345678" 替换为 NULL


/*------------------------------- 编码器 -------------------------------*/

/* 系统时钟 */
#define SYS_PIT_CH 						( PIT_CH2 )				// 系统时钟中断号
#define SYS_PIT_TIME 					( 1 )					// 系统时钟中断周期 ms

/* 传感器时钟 */
#define SENSOR_SOLVE_PIT_CH 			( PIT_CH0 ) 			// 传感器值/解算获取中断号
#define SENSOR_SOLVE_PIT_TIME 			( 5 ) 					// 传感器值/解算获取中断周期 ms
#define SENSOR_SOLVE_dt                 ( (float)SENSOR_SOLVE_PIT_TIME / 1000 )

/* 按键时钟 */
#define KEY_PIT_CH 						( PIT_CH3 )				// 按键扫描中断号
#define KEY_PIT_TIME 					( 5 )					// 按键扫描中断周期 ms

/*控制时钟*/

#define CONTROL_PIT_CH 					( PIT_CH1 )				// 电机控制中断号
#define CONTROL_PIT_TIME 				( 5 )					// 电机控制中断周期 ms

/* 编码器引脚 */
#define ENCODER_1                       (QTIMER1_ENCODER1)
#define ENCODER_1_LSB                   (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_DIR                   (QTIMER1_ENCODER1_CH2_C1)
                                        
#define ENCODER_2                       (QTIMER1_ENCODER2)
#define ENCODER_2_LSB                   (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_DIR                   (QTIMER1_ENCODER2_CH2_C24)
                                        
#define ENCODER_3                       (QTIMER2_ENCODER1)
#define ENCODER_3_LSB                   (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_3_DIR                   (QTIMER2_ENCODER1_CH2_C4)
                                        
#define ENCODER_4                       (QTIMER2_ENCODER2)
#define ENCODER_4_LSB                   (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_4_DIR                   (QTIMER2_ENCODER2_CH2_C25)


/* 
编码器方向标定
如果电机逆时针旋转时编码器数值为正，则标定值为 1
如果电机逆时针旋转时编码器数值为负，则标定值为 -1
*/
#define ENCODER_1_FRONT_DIR ( -1 )
#define ENCODER_2_FRONT_DIR ( 1 )
#define ENCODER_3_FRONT_DIR ( -1 )
#define ENCODER_4_FRONT_DIR ( 1 )

/* 
陀螺仪方向标定
如果车右转时陀螺仪Z轴度数为正，则标定值为 1
如果车右转时陀螺仪Z轴度数为负，则标定值为 -1
*/
#define YAW_ANGLE_INIT  ( -90.0f )
#define GYRO_CALIB_EPOCH   1500          // 标定采样点数（建议 500~2000）
#define GYRO_DEADBAND_K    2.0f         // 死区放大系数

#define YAW_ALPHA          1.0f         // 互补滤波计算航向角系数

/* ============ 卡尔曼滤波器参数配置 ============ */
#define KALMAN_EN               false

#define KALMAN_Q_VELOCITY       0.02f   // 过程噪声方差（加速度不确定性，越大越相信测量）
#define KALMAN_R_VELOCITY       5.0f   // 测量噪声方差（越大越不信任单次测量）
#define KALMAN_P_VELOCITY       0.5f    // 初始估计误差方差

/* 电机 */
#define MAX_DUTY                 (50 )

#define MOTOR1_DIR               (D3 )
#define MOTOR1_PWM               (PWM2_MODULE3_CHA_D2)

#define MOTOR2_DIR               (C11 )
#define MOTOR2_PWM               (PWM2_MODULE2_CHA_C10)

#define MOTOR3_DIR               (C7 )
#define MOTOR3_PWM               (PWM2_MODULE0_CHA_C6)

#define MOTOR4_DIR               (C9 )
#define MOTOR4_PWM               (PWM2_MODULE1_CHA_C8)


#define MOTOR_DUTY_MAX   1000   // 逻辑占空比最大值
#define MOTOR_DUTY_LIMIT 1000   // 逻辑占空比限制值

/* 
电机方向标定
如果给DIR脚为高电平时电机顺时针旋转，则标定值为 1
如果给DIR脚为高电平时电机逆时针旋转，则标定值为 -1
*/
#define MOTOR_1_FRONT_DIR 	( 1 )
#define MOTOR_2_FRONT_DIR 	( -1 )
#define MOTOR_3_FRONT_DIR 	( 1 )
#define MOTOR_4_FRONT_DIR 	( -1 )

#define PI_F 3.1415926f

/* C标准库 */
#include "math.h"
#include "string.h"

/* 逐飞库头文件 */
#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "isr.h"

/* 用户头文件 */
#include "data.h"
#include "menu.h"
#include "encoder.h"
#include "imu.h"
#include "user_debug.h"
#include "motor.h"
#include "pid.h"
#include "solve.h"
#include "cmd.h"
#include "play.h"
#include "uart.h"

#endif
