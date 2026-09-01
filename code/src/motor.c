#include "motor.h"

void motor_init()
{
    gpio_init (MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);      //GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR1_PWM, 17000, 0) ;                            //PWM 通道初始化频率17KHz占空比初始为0

    gpio_init (MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);      //GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0) ;                            //PWM 通道初始化频率17KHz占空比初始为0

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);       //GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR3_PWM, 17000, 0) ;                            //PWM 通道初始化频率17KHz占空比初始为0

    gpio_init (MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);      //GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR4_PWM, 17000, 0) ;                            //PWM 通道初始化频率17KHz占空比初始为0

    interrupt_global_enable(0);
}

void motor_run(uint8 motor_id,int32 duty)
{
    if(abs(duty) > MOTOR_DUTY_LIMIT)duty = (duty > 0) ? MOTOR_DUTY_LIMIT : -MOTOR_DUTY_LIMIT;
	if(motor_id == 1)
    {
        duty *= MOTOR_1_FRONT_DIR;
		if(duty <= 0)
            gpio_set_level(MOTOR1_DIR, GPIO_LOW),
            pwm_set_duty(MOTOR1_PWM, -duty * (PWM_DUTY_MAX / MOTOR_DUTY_MAX));
        else
			gpio_set_level(MOTOR1_DIR, GPIO_HIGH),
            pwm_set_duty(MOTOR1_PWM, duty * (PWM_DUTY_MAX / MOTOR_DUTY_MAX));
		
	}
    else if(motor_id == 2)
    {
        duty *= MOTOR_2_FRONT_DIR;
        if(duty <= 0)
            gpio_set_level(MOTOR2_DIR, GPIO_LOW),
            pwm_set_duty(MOTOR2_PWM, -duty * (PWM_DUTY_MAX / MOTOR_DUTY_MAX));
        else
            gpio_set_level(MOTOR2_DIR, GPIO_HIGH),
            pwm_set_duty(MOTOR2_PWM, duty * (PWM_DUTY_MAX / MOTOR_DUTY_MAX));
    }
    else if(motor_id == 3)
    {
        duty *= MOTOR_3_FRONT_DIR;
        if(duty <= 0)
            gpio_set_level(MOTOR3_DIR, GPIO_LOW),
            pwm_set_duty(MOTOR3_PWM, -duty * (PWM_DUTY_MAX / MOTOR_DUTY_MAX));
        else
            gpio_set_level(MOTOR3_DIR, GPIO_HIGH),
            pwm_set_duty(MOTOR3_PWM, duty * (PWM_DUTY_MAX / MOTOR_DUTY_MAX));
    }
    else if(motor_id == 4)
    {
        duty *= MOTOR_4_FRONT_DIR;
        if(duty <= 0)
            gpio_set_level(MOTOR4_DIR, GPIO_LOW),
            pwm_set_duty(MOTOR4_PWM, -duty * (PWM_DUTY_MAX / MOTOR_DUTY_MAX));
        else
            gpio_set_level(MOTOR4_DIR, GPIO_HIGH),
            pwm_set_duty(MOTOR4_PWM, duty * (PWM_DUTY_MAX / MOTOR_DUTY_MAX));
    }
}

void motor_stop()
{
    //gpio_set_level(MOTOR1_DIR, GPIO_LOW),
    pwm_set_duty(MOTOR1_PWM, 0);
    //gpio_set_level(MOTOR2_DIR, GPIO_LOW),
    pwm_set_duty(MOTOR2_PWM, 0);
    //gpio_set_level(MOTOR3_DIR, GPIO_LOW),
    pwm_set_duty(MOTOR3_PWM, 0);
    //gpio_set_level(MOTOR4_DIR, GPIO_LOW),
    pwm_set_duty(MOTOR4_PWM, 0);
}

void motor_cmd()
{
    motor_run(1, pid_speed[0].value);
    motor_run(2, pid_speed[1].value);
    motor_run(3, pid_speed[2].value);
    motor_run(4, pid_speed[3].value);
}