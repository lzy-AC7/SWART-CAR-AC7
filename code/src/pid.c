#include "pid.h"

// 增量式 PID
float pid_incremental(struct PID *this,const float target,const float feedback)
{
    this->now_err = target - feedback;

    // 死区
    if(target == 0 && fabs(this->now_err) < this->output_deadband)
    {
        this->value=0;
        this->last_err=0;
        this->last_last_err=0;
        return this->value;
    }

    float delta = 0;

    // 增量式 Kp
    delta += (this->Kp) * (this->now_err - this->last_err);
    // 增量式 Ki
    delta += (this->Ki) * (this->now_err);
    // 增量式 Kd
    delta += (this->Kd) * (this->now_err - 2 * this->last_err + this->last_last_err);

    // 变化率限幅
    if(delta > this->delta_limit)
        delta = this->delta_limit;
    if(delta < -this->delta_limit)
        delta = -this->delta_limit;

    // 输出更新
    this->value += delta;

    // 输出限幅
    if(this->value > this->output_limit)
        this->value = this->output_limit;

    if(this->value < -this->output_limit)
        this->value = -this->output_limit;

    // 更新参数
    this->last_last_err = this->last_err;
    this->last_err = this->now_err;

    return this->value;
}

// 位置式 PID
float pid_positional(struct PID *this,const float target,const float feedback)
{
    this->now_err = target - feedback;

    // 死区
    if(fabs(this->now_err) < this->output_deadband)
    {
        this->value=0;
        this->last_err=0;
        this->last_last_err=0;
        return this->value;
    }

    float output = 0;

    // 位置式 Kp
    output += (this->Kp) * this->now_err;

    // 位置式 Ki
    if(fabs(output) < this->output_limit)
        this->sigma_err += this->now_err;

    // 积分限幅
    if(this->sigma_err > this->i_limit)
        this->sigma_err = this->i_limit;

    if(this->sigma_err < -this->i_limit)
        this->sigma_err = -this->i_limit;

    output += (this->Ki) * this->sigma_err;

    // 位置式 Kd
    output += (this->Kd) * (this->now_err - this->last_err);

    // 输出变化率限制
    this->value_delta = output - this->value;

    if(this->value_delta > this->delta_limit)
        output = this->value + this->delta_limit;

    if(this->value_delta < -this->delta_limit)
        output = this->value - this->delta_limit;

    // 更新输出
    this->value = output;

    // 更新参数
    this->last_err = this->now_err;

    // 输出限幅
    if(this->value > this->output_limit)
        this->value = this->output_limit;

    if(this->value < -this->output_limit)
        this->value = -this->output_limit;

    return this->value;
}

float K_x_ff = 1.1,K_y_ff = 1.25;
void speed_pid_update()
{
    //前馈
    solve_car2wheel(v_x_car_target*K_x_ff + pid_v_x_car.value,v_y_car_target*K_y_ff + pid_v_y_car.value,pid_yaw.value);
    pid_incremental(pid_speed  , wheel_speed_target[0], wheel_speed[0]);
    pid_incremental(pid_speed+1, wheel_speed_target[1], wheel_speed[1]);
    pid_incremental(pid_speed+2, wheel_speed_target[2], wheel_speed[2]);
    pid_incremental(pid_speed+3, wheel_speed_target[3], wheel_speed[3]);
}

void v_pid_update()
{
    solve_world2wheel(speed_target_angle,speed_target_value);
    pid_incremental(&pid_v_x_car, v_x_car_target, v_x_car);
    pid_incremental(&pid_v_y_car, v_y_car_target, v_y_car);
}

void pos_pid_update()
{
    float dx = x_target - x_world;
    float dy = y_target - y_world;
    distance = distance*0.6+0.4*sqrtf(dx * dx + dy * dy);
    speed_target_angle = atan2f(dy, dx) / PI_F * 180.0f;
    if(distance*distance <= path_length_sq/4)pid_pos_speed.Kp = pid_pos_speed_kp;
    else pid_pos_speed.Kp = pid_pos_speed_Kp;
    pid_positional(&pid_pos_speed, distance, 0);
    speed_target_value = pid_pos_speed.value;
}

void yaw_pid_update()
{
    // 处理180度跳变问题
    float err = yaw_angle_target - yaw_angle;
    if (err > 180.0f) {
        err -= 360.0f;
    } else if (err < -180.0f) {
        err += 360.0f;
    }
    float target_norm = yaw_angle + err;
    
    pid_positional(&pid_yaw, target_norm, yaw_angle);
}