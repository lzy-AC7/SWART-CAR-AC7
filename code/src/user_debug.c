#include "user_debug.h"

static float justfloat[MAX_DATA_BUFFER_SIZE][4]; // JUSTFLOAT数据
static uint8 justfloat_data_num;                 // JUSTFLOAT数据量

// JUSTFLOAT数据添加
void justfloat_add(const uint32 data_num, ...)
{
    va_list args;
    uint8 data_add[4] = {0};

    va_start(args, data_num); // 初始化可变参数列表
    if (justfloat_data_num <= MAX_DATA_BUFFER_SIZE)
    {
        for (uint32_t i = 0; i < data_num; i++)
        {
            double data_double = va_arg(args, double); // 逐个读取 int 类型参数
            float data_float = (float)data_double;

            *(float *)data_add = data_float;

            justfloat[justfloat_data_num][0] = data_add[0];
            justfloat[justfloat_data_num][1] = data_add[1];
            justfloat[justfloat_data_num][2] = data_add[2];
            justfloat[justfloat_data_num][3] = data_add[3];

            justfloat_data_num++;

            if (justfloat_data_num > 50)
                break;
        }
    }
    va_end(args);
}

static uint8 buffer[256];
// JUSTFLOAT数据发送
void justfloat_send()
{
    for (int i = 0; i < justfloat_data_num; i++)
    {
        // uart_write_byte(uart_idx,justfloat[i][0]);
        // uart_write_byte(uart_idx,justfloat[i][1]);
        // uart_write_byte(uart_idx,justfloat[i][2]);
        // uart_write_byte(this -> uart_idx,this -> justfloat[i][3]);

        buffer[i * 4 + 0] = justfloat[i][0],
        buffer[i * 4 + 1] = justfloat[i][1],
        buffer[i * 4 + 2] = justfloat[i][2],
        buffer[i * 4 + 3] = justfloat[i][3];
    }
    buffer[justfloat_data_num * 4] = 0x00,
    buffer[justfloat_data_num * 4 + 1] = 0x00,
    buffer[justfloat_data_num * 4 + 2] = 0x80,
    buffer[justfloat_data_num * 4 + 3] = 0x7f;

    if(!wifi_spi_send_buffer(buffer, justfloat_data_num * 4 + 4))
				wifi_spi_udp_send_now();

    // uint8 frame_end[4] = {0x00, 0x00, 0x80, 0x7f};
    // uart_write_buffer(this -> uart_idx, frame_end, 4);
    justfloat_data_num = 0;
}

uint8 buff[128],len = 0;
bool buff_ready = false;
void stop()
{
    if(debug_data[0] == '!')
    {
        speed_target_angle = 0.0f,
        speed_target_value = 0.0f,
        yaw_angle_target = -90.0f;
        car_runing_path_flag = 0;
        car_2p_runing_flag = 0;
        fsm_flag = 0,played_flag = 0,backed_flag = 0,map_init_flag = 0;
        motor_stop();
    }
}

float debug_t1,debug_t2;
void debug()
{
    uint32 t = debug_read_ring_buffer(debug_data,128);
    float x,y,x1,y1;
    debug_data[t] = '\0';
    int a;
    if(t)
    {
        stop();
        
        // printf("debug_data:%s\r\n", debug_data);
        //蓝牙数据接收处理
        if(debug_data[0] == '[')
        {
            len = 0,buff_ready = false;
            for(uint8 i = 1;i < t;i++)
                if(debug_data[i] == ']')buff[len] = '\0',buff_ready = true;
                else if(debug_data[i] == '[')len = 0,buff_ready = false;
                else buff[len++] = debug_data[i];
        }
        else if (debug_data[0] == ']')
        {
            buff[len] = '\0',buff_ready = true;
        }
        else 
        {
            for(uint8 i = 0;i < t;i++)
                if(debug_data[i] == ']')buff[len] = '\0',buff_ready = true;
                else if(debug_data[i] == '[')len = 0,buff_ready = false;
                else buff[len++] = debug_data[i];
        }

        if(buff_ready)
        {
            // printf("buff:%s\r\n",buff);
            if(buff[0] == 'r'&&buff[1] == 'e'&&buff[2] == 'c')
            {
                if(process_rec_data(0))
                    printf("rec success:%s\n",fifo_rec_get_data);
            }
            else if(buff[0] == 'R'&&buff[1] == 'e'&&buff[2] == 'c')
            {
                if(process_rec_data(1))
                    printf("rec success:%s\n",fifo_rec_get_data);
            }
            else if(buff[0] == 'm' && buff[1] == 'a' && buff[2] == 'p')
            {
               process_map_data();
            }
            else if(buff[0] == 'f')
            {
                sscanf((char*)buff, "f,%d",&a);
                checkpoint_set(a);
            }
            else if(buff[0] == 'p')
            {
                sscanf((char*)buff, "p,%f,%f",&x1,&y1);
                car_2p_start_map(x1, y1, yaw_angle_target);
            }
            else if(buff[0] == 'u')
            {
                sscanf((char*)buff, "u,%f",&x1);
                car_2p_start_map(GET_X(player), GET_Y(player)-x1, yaw_angle_target);
            }
            else if(buff[0] == 'd')
            {
                sscanf((char*)buff, "d,%f",&x1);
                car_2p_start_map(GET_X(player), GET_Y(player)+x1, yaw_angle_target);
            }else if(buff[0] == 'l')
            {
                sscanf((char*)buff, "l,%f",&x1);
                car_2p_start_map(GET_X(player)-x1, GET_Y(player), yaw_angle_target);
            }else if(buff[0] == 'r')
            {
                sscanf((char*)buff, "r,%f",&x1);
                car_2p_start_map(GET_X(player)+x1, GET_Y(player), yaw_angle_target);
            }
            else if(buff[0] == 'j')
            {
                sscanf((char*)buff, "j,%f,%f,%f,%f", &x,&y,&x1,&y1);
                if(x != 0.0f || y != 0.0f)
                    yaw_angle_target = atan2f(y, x)/PI_F*180.0f;
                speed_target_angle = atan2f(y1, x1)/PI_F*180.0f,
                speed_target_value = sqrtf(x1*x1+y1*y1);
            }
        }

        // if(debug_data[0] == 'x')solve_car2wheel(0,0,0),motor_stop();
        // else if(debug_data[0] == 'd')sscanf((char*)debug_data, "d%f", &yaw_angle_target);
        // else if(debug_data[0] == 's')sscanf((char*)debug_data, "s%f,%f", &speed_target_angle,&speed_target_value);
    }

    if(!wifi_en)return;
    t = wifi_spi_read_buffer(wifi_data, sizeof(wifi_data));
    wifi_data[t] = '\0';
    if(t)
    {
        // printf("wifi_data:%s\r\n", wifi_data);
        // wifi_spi_send_string(wifi_data);

        //wifi控制移动
        if(wifi_data[0] == 'a')sscanf((char*)wifi_data, "a:%f", &yaw_angle_target);
        else if(wifi_data[0] == 's')sscanf((char*)wifi_data, "s:%f,%f", &x,&y),
                                speed_target_angle = atan2f(y, x)/PI_F*180.0f,
                                speed_target_value = sqrtf(x*x+y*y);
        // if(wifi_data[0] == 'p')sscanf((char*)wifi_data, "p:%f", &pid_speed[2].Kp);
        // if(wifi_data[0] == 'i')sscanf((char*)wifi_data, "i:%f", &pid_speed[2].Ki);
        // if(wifi_data[0] == 'd')sscanf((char*)wifi_data, "d:%f", &pid_speed[2].Kd);
        if(wifi_data[0] == 't')sscanf((char*)wifi_data, "t:%f", &yaw_angle_target);
    }

    // justfloat_add(1,wheel_speed_target[0]);
    // justfloat_add(4,wheel_speed_target[0],wheel_speed_target[1],wheel_speed_target[2],wheel_speed_target[3]);
    // justfloat_add(4,wheel_speed[0],wheel_speed[1],wheel_speed[2],wheel_speed[3]);
    // justfloat_add(2,pid_speed[0].now_err,pid_speed[0].value);
    // justfloat_add(2,yaw_angle_target,yaw_angle);
    // justfloat_add(2,pid_pos.now_err,pid_pos.value);
    // justfloat_add(2,x_target,y_target);
    // justfloat_add(2,v_x_encoder,v_y_encoder);
    // justfloat_add(2,v_x_car_target,v_y_car_target);
    // justfloat_add(2,v_x_car,v_y_car);
    // justfloat_add(2,acc_x,acc_y);
    // justfloat_add(4,speed_target_value,v_y_car,pid_v_y_car.now_err,pid_v_y_car.value);
    justfloat_add(1,speed_target_value);
    justfloat_add(2,x_target,y_target);
    justfloat_add(2,x_world,y_world);
    justfloat_add(2,art_x,art_y);
    // justfloat_add(2,debug_t1,debug_t2);
    justfloat_send();
}