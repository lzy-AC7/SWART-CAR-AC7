#include "pid.h"

// 增量式 PID
float pid_incremental(struct PID *this,const float target,const float feedback)
{
    this->now_err = target - feedback;
    // 增量式 Kp
    this->value += (this->Kp) * (this->now_err - this->last_err);
    // 增量式 Ki
    this->value += (this->Ki) * (this->now_err);
    // 增量式 Kd
    this->value += (this->Kd) * (this->now_err - 2 * this->last_err + this->last_last_err);

    // 更新参数
    this->last_err = this->now_err;
    this->last_last_err = this->last_err;

    // 输出限幅
    if (this->value > this->output_limit)
    {
        this->value = this->output_limit;
    }
    if (this->value < -this->output_limit)
    {
        this->value = -this->output_limit;
    }
    //死区
    if(target == 0 && fabs(this->now_err) < this->output_deadband)this->value = 0.0f;

    return this->value;
}

// 位置式 PID
float pid_positional(struct PID *this, const float target, const float feedback)
{
    float value_buffer = 0;
    this->now_err = target - feedback;
    // 位置式 Kp
    value_buffer += (this->Kp) * (this->now_err);
    // 位置式 Ki
    value_buffer += (this->Ki) * this->sigma_err;
    // 位置式 Kd
    value_buffer += (this->Kd) * (this->now_err - this->last_err);

    // 更新参数
    this->sigma_err += this->now_err;
    this->last_err = this->now_err;

    // 输出限幅
    if (value_buffer > this->output_limit)
    {
        value_buffer = this->output_limit;
    }
    if (value_buffer < -this->output_limit)
    {
        value_buffer = -this->output_limit;
    }
    // 积分限幅
    if (this->sigma_err > this->i_limit)
    {
        this->sigma_err = this->i_limit;
    }
    if (this->sigma_err < -this->i_limit)
    {
        this->sigma_err = -this->i_limit;
    }

    this->value = value_buffer;
    value_buffer = 0;

    return this->value;
}

void speed_pid_update()
{
    pid_incremental(pid_speed  , wheel_speed_target[0], wheel_speed[0]);
    pid_incremental(pid_speed+1, wheel_speed_target[1], wheel_speed[1]);
    pid_incremental(pid_speed+2, wheel_speed_target[2], wheel_speed[2]);
    pid_incremental(pid_speed+3, wheel_speed_target[3], wheel_speed[3]);
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