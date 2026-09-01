#include "encoder.h"

#define STOP_STANDARD 10

void encoder_init()
{
    encoder_dir_init(ENCODER_1, ENCODER_1_LSB, ENCODER_1_DIR),
    encoder_dir_init(ENCODER_2, ENCODER_2_LSB, ENCODER_2_DIR),
    encoder_dir_init(ENCODER_3, ENCODER_3_LSB, ENCODER_3_DIR),
    encoder_dir_init(ENCODER_4, ENCODER_4_LSB, ENCODER_4_DIR);
}

void encoder_get()
{
    encoder[0] = ENCODER_1_FRONT_DIR*encoder_get_count(ENCODER_1);
    encoder[1] = ENCODER_2_FRONT_DIR*encoder_get_count(ENCODER_2);
    encoder[2] = ENCODER_3_FRONT_DIR*encoder_get_count(ENCODER_3);
    encoder[3] = ENCODER_4_FRONT_DIR*encoder_get_count(ENCODER_4);

    wheel_speed[0] = encoder[0] * WHEEL_SPEED_FACTOR / SENSOR_SOLVE_dt;
    wheel_speed[1] = encoder[1] * WHEEL_SPEED_FACTOR / SENSOR_SOLVE_dt;
    wheel_speed[2] = encoder[2] * WHEEL_SPEED_FACTOR / SENSOR_SOLVE_dt;
    wheel_speed[3] = encoder[3] * WHEEL_SPEED_FACTOR / SENSOR_SOLVE_dt;

    /* 从编码器计算小车体坐标系速度 */
    v_x_encoder = 1.0 * (wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) / 4.0f;
    v_y_encoder = 1.0 *(-wheel_speed[0] + wheel_speed[1] + wheel_speed[2] - wheel_speed[3]) / 4.0f;
    
	encoder_clear_count(ENCODER_1);
    encoder_clear_count(ENCODER_2);
    encoder_clear_count(ENCODER_3);
    encoder_clear_count(ENCODER_4);
}

bool judge_stop()
{
    return (encoder[0] < STOP_STANDARD && encoder[1] < STOP_STANDARD && encoder[2] < STOP_STANDARD && encoder[3] < STOP_STANDARD);
}