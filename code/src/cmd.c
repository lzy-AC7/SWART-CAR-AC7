#include "cmd.h"

#define V_MAX 2500.0f       // 最大速度
#define V_MIN 200.0f         // 最小启动/停止速度
float ACCEL = 50.0f;         // 加速度
#define DEACCEL_K 0.5f      // 减速加速度增益
#define OMEGA_ACC 1.0f      // 角加速度      
float STOP_ERROR  = 50.0f;

static float x_target = 0, y_target = 0, speed = 0, max_v = 0, distance_acc_need = 0, yaw_target = 0;
static uint8 mode = 0;
static bool turn_flag = 0;

float dis = 0;

float one_k;

static float calculate_smooth_velocity(float distance, float current_v)
{
    // 1. 到达判定
    if (distance < STOP_ERROR) return 0.0f;
    
    if (fast_flag)
    {
        if(dis <= 1.0)
        {
            STOP_ERROR = 100,ACCEL = 50;
            one_k = 100;
        }
        else if(dis <= 2.0)
        {
            STOP_ERROR = 80,ACCEL = 110;
            if (distance < 100) return 100.0f;
            if (distance < 150) return 150.0f;
            if (distance < 200) return 200.0f;
            if (distance < 250) return 250.0f;
        }
        else if(dis <= 3.0)
        {
            STOP_ERROR = 100,ACCEL = 130;
            if (distance < 150) return 150.0f;
            if (distance < 200) return 200.0f;
            if (distance < 250) return 250.0f;
            if (distance < 300) return 300.0f;
            // if (distance < 500) return 400.0f;
        }
        else
        {
            STOP_ERROR = 100,ACCEL = 150;
        
            if (distance < 150) return 200.0f;
            if (distance < 200) return 300.0f;
            if (distance < 300) return 350.0f;
            if (distance < 500) return 400.0f;
        }
    }
    else ACCEL = 50,STOP_ERROR = 50;
    if (distance < 200)
    {
        float close_ratio = distance / 200.0f;
        return one_k + (300.0f - one_k) * close_ratio;
    }
    
    // 2. 动态计算当前最大减速能力（考虑减速增益）
    float deaccel_a = ACCEL * DEACCEL_K;

    // 3. 【核心优化】利用物理公式 v^2 = 2_a_s 反推当前距离允许的最大安全速度 limit_v
    // 实际物理距离需要转换，除以 WHEEL_SPEED_FACTOR
    float physical_dist = distance / WHEEL_SPEED_FACTOR;
    
    // 理论上能刹住的最大速度：V = sqrt(V_MIN^2 + 2 * a * s)
    float limit_v = sqrtf((V_MIN * V_MIN) + (2.0f * deaccel_a * physical_dist));
    
    // 限制最大速度不能超过设定的 max_v
    if (limit_v > max_v) limit_v = max_v;

    float target_v = 0.0f;

    // 4. 速度规划决策
    // 如果当前速度加上一个周期的加速度后，依然小于安全限速，说明处于【加速段/匀速段】
    if (current_v + ACCEL < limit_v)
    {
        target_v = current_v + ACCEL;
    }
    else
    {
        // 否则，强制进入【减速段】。直接紧贴安全限速曲线下滑，保证最高的减速效率
        // 这样长距离时会在末端完美平滑减速；短距离时会因为 limit_v 限制，直接快起快停
        target_v = limit_v;

        // 限制减速斜率，防止超出电机的物理减速极限（防止抖动或过载）
        if (target_v < current_v - deaccel_a) target_v = current_v - deaccel_a;
    }

    // 5. 最小速度保护与平滑
    if (target_v < V_MIN) target_v = V_MIN;

    // 用于给 mode == 1 做转向判定的动态距离更新
    // 逆向计算：当前车速刹车到 V_MIN 到底需要多少距离
    if (current_v > V_MIN)
    {
        distance_acc_need = ((current_v * current_v) - (V_MIN * V_MIN)) / (2.0f * deaccel_a) * WHEEL_SPEED_FACTOR;
    }
    else
    {
        distance_acc_need = 0.0f;
    }

    return target_v;
}

static float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

static float approach_angle(float current, float target, float step)
{
    float diff = normalize_angle(target - current);
    if (fabsf(diff) <= step) return normalize_angle(target);
    return normalize_angle(current + copysignf(step, diff));
}

void car_2p()
{
    if (!car_2p_runing_flag) return;
    if (!car_2p_timer_flag) return;
    else car_2p_timer_flag = 0;
        
    float dx = x_target - x_world;
    float dy = y_target - y_world;
    float distance = sqrtf(dx * dx + dy * dy);
    float target_angle = atan2f(dy, dx) / PI_F * 180.0f;    

    // 到达及原位旋转判定
    if (distance < STOP_ERROR || turn_flag)
    {
        turn_flag = 1; 
        GYRO_DEADBAND_K = 2.0;
        yaw_angle_target = approach_angle(yaw_angle_target, yaw_target, OMEGA_ACC);
        speed_target_value = 0.0f;
        if(yaw_angle_target == yaw_target&& encoder[0] + encoder[1] + encoder[2] + encoder[3] < 10)
        {
            speed_target_angle = yaw_angle_target;
            car_2p_runing_flag = car_2p_timer_count = car_2p_timer_flag = turn_flag = 0;
            GYRO_DEADBAND_K = 1.0;
        }
        return;
    }

    // 计算动态目标速度
    float target_speed = calculate_smooth_velocity(distance, speed_target_value);
    speed_target_angle = target_angle;
    speed_target_value = target_speed;
}

void car_2p_start(float x, float y, float yaw)
{
    x_target = x;
    y_target = y;
    yaw_target = yaw;
    speed_target_value = 0.0f; // 每次启动从 0 开始
    car_2p_timer_count = car_2p_timer_flag = 0;
    car_2p_runing_flag = 1;

    max_v = (run_speed > V_MAX) ? V_MAX : run_speed; 
    if (max_v < V_MIN) max_v = V_MIN;

    distance_acc_need = 0.0f; // 初始设为 0，交由循环动态计算
}

void car_2p_start_map(float x, float y, float yaw)
{
    if(x > 13 || x < 0 || y > 9 || y < 0)return;
    dis = sqrtf((GET_X(player)-x)*(GET_X(player)-x)+(GET_Y(player)-y)*(GET_Y(player)-y));
    float world_x = 200 + 100 + x * 200, world_y = -(200 + 100 + y * 200);
    car_2p_start(world_x, world_y, yaw);
}

int k = 0;
float yaw_target_t = 0;

void car_runing_path_start(float yaw)
{
    car_runing_path_flag = 1;
    yaw_target_t = yaw;
    k = 0;
}

void car_runing_path()
{
    if (!car_runing_path_flag) return;

    if (!car_runing_path_timer_flag) return;
    car_runing_path_timer_flag = 0;

    if (car_2p_runing_flag) return;

    if (k < A_path_size)
    {   
        if (k > 0)
        {
           while (1) 
            {
                // 计算向量 u (k-1 -> k)
                float ux = A_path_x[k] - A_path_x[k-1];
                float uy = A_path_y[k] - A_path_y[k-1];
                
                // 计算向量 v (k -> k+1)
                float vx = A_path_x[k+1] - A_path_x[k];
                float vy = A_path_y[k+1] - A_path_y[k];

                // 向量叉积：ux * vy - uy * vx (等于 0 代表平行)
                // 向量点积：ux * vx + uy * vy (大于 0 代表同向)
                if ((ux * vy - uy * vx == 0.0f) && (ux * vx + uy * vy > 0.0f)) {
                    k++;
                } else {
                    break; // 方向不一致或发生转折，停止跳过
                }
            }
        }

        if (k == A_path_size - 1)
            car_2p_start_map(A_path_x[k], A_path_y[k], yaw_target_t);
        else
            car_2p_start_map(A_path_x[k], A_path_y[k], yaw_angle_target);

        // printf("%d %d %f %f\n",A_path_x[k], A_path_y[k], yaw_angle_target,yaw_target_t);
    }
    else if (k == A_path_size)
    {
        if (A_path_x[k] == 666) // 识别
        {
            printf("look start!\n");
            system_delay_ms(200);
            if (process_rec_data(min_type))
            {
                id[min_type][min_p] = (fifo_rec_get_data[0] - '0');
                if (!min_type) 
                {
                    map[box_init[min_p]] = 10 + id[min_type][min_p];
                    
                    printf("box:%d %d:%d\n", GET_X(box_init[min_p]), GET_Y(box_init[min_p]), id[min_type][min_p]);
                }
                else 
                {
                    map[target_init[min_p]] = 20 + id[min_type][min_p];
                    printf("target:%d %d:%d\n", GET_X(target_init[min_p]), GET_Y(target_init[min_p]), id[min_type][min_p]);
                }
                id_c[min_type][id[min_type][min_p]]++;
                id_c[min_type][10]++;
                id_count++;
            }
        }
        else if (A_path_x[k] == 999) // 等待炸弹爆炸
        {
            system_delay_ms(1500);
        }
        
        car_runing_path_flag = 0;
        car_runing_path_timer_flag = 0; 
        car_runing_path_timer_count = 0;
        k = 0;
        A_path_size = 0;
        return;
    }
    
    k++;
}

#define SQRT2 1.41421356237f

typedef struct
{
    float x;
    float y;
}POINT;

/* 点到线段距离 */
static float PointToSegmentDist(
    float px,float py,
    float x1,float y1,
    float x2,float y2)
{
    float vx=x2-x1;
    float vy=y2-y1;

    float wx=px-x1;
    float wy=py-y1;

    float len2=vx*vx+vy*vy;

    if(len2<1e-6f)
        return sqrtf(wx*wx+wy*wy);

    float t=(wx*vx+wy*vy)/len2;

    if(t<0) t=0;
    if(t>1) t=1;

    float cx=x1+t*vx;
    float cy=y1+t*vy;

    float dx=px-cx;
    float dy=py-cy;

    return sqrtf(dx*dx+dy*dy);
}
bool check_t(int x,int y)
{
    if(x>=0 && x<COL && y>=0 && y<ROW && (map[GET_ID(x,y)] != 1) && (map[GET_ID(x,y)] != 2) && (map[GET_ID(x,y)]%100 != 4) && (map[GET_ID(x,y)]%100/10 != 1)) return true;
    return false;
}
/* 判断是否可直达 */
bool LineCheck(int x0,int y0,int x1,int y1)
{
    if(!check_t(x1,y1))return 0;
    float sx=x0+0.5f;
    float sy=y0+0.5f;

    float ex=x1+0.5f;
    float ey=y1+0.5f;

    float dx=ex-sx;
    float dy=ey-sy;

    float len=sqrtf(dx*dx+dy*dy);

    if(len<1e-6f)
        return 1;

    /* 单位法向量 */
    float nx=-dy/len;
    float ny= dx/len;

    /* 四个角 */
    POINT s_corner[4]={
        {x0,y0},
        {x0+1,y0},
        {x0,y0+1},
        {x0+1,y0+1}
    };

    POINT e_corner[4]={
        {x1,y1},
        {x1+1,y1},
        {x1,y1+1},
        {x1+1,y1+1}
    };

    int sMax=0,sMin=0;
    int eMax=0,eMin=0;

    float maxProj=-1e9f;
    float minProj= 1e9f;

    /* 起点最远两个角 */
    for(int i=0;i<4;i++)
    {
        float proj=
        (s_corner[i].x-sx)*nx+
        (s_corner[i].y-sy)*ny;

        if(proj>maxProj)
        {
            maxProj=proj;
            sMax=i;
        }

        if(proj<minProj)
        {
            minProj=proj;
            sMin=i;
        }
    }

    maxProj=-1e9f;
    minProj= 1e9f;

    /* 终点最远两个角 */
    for(int i=0;i<4;i++)
    {
        float proj=
        (e_corner[i].x-ex)*nx+
        (e_corner[i].y-ey)*ny;

        if(proj>maxProj)
        {
            maxProj=proj;
            eMax=i;
        }

        if(proj<minProj)
        {
            minProj=proj;
            eMin=i;
        }
    }

    /* 外包矩形 */
    int xmin=x0<x1?x0:x1;
    int xmax=x0>x1?x0:x1;

    int ymin=y0<y1?y0:y1;
    int ymax=y0>y1?y0:y1;

    for(int y=ymin;y<=ymax;y++)
    {
        for(int x=xmin;x<=xmax;x++)
        {
            float cx=x+0.5f;
            float cy=y+0.5f;

            float d1=PointToSegmentDist(
                cx,cy,
                s_corner[sMax].x,s_corner[sMax].y,
                e_corner[eMax].x,e_corner[eMax].y);

            float d2=PointToSegmentDist(
                cx,cy,
                s_corner[sMin].x,s_corner[sMin].y,
                e_corner[eMin].x,e_corner[eMin].y);

            if((d1<SQRT2/2 || d2<SQRT2/2) && check_t(x,y)==0)
                return 0;
        }
    }

    return 1;
}
