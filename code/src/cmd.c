#include "cmd.h"

float STOP_ERROR  = 25.0f;
#define OMEGA_ACC 2.0f      // 角加速度

float x_target = 0.0f;
float y_target = 0.0f;
float x_start = 0.0f;
float y_start = 0.0f;
float path_length_sq = 0.0f;
float distance = 0.0f;
static float yaw_target = 0.0f;

static bool turn_flag = 0;

// 角度规范化
static float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

// 角度逼近
static float approach_angle(float current, float target, float step)
{
    float diff = normalize_angle(target - current);
    if (fabsf(diff) <= step) return normalize_angle(target);
    return normalize_angle(current + copysignf(step, diff));
}

// 2P运动逻辑
void car_2p()
{
    if (!car_2p_runing_flag) return;
    if (!car_2p_timer_flag) return;
    car_2p_timer_flag = 0;

    // 到达及原位旋转判定
    if (distance < STOP_ERROR || turn_flag)
    {
        turn_flag = 1; 
        
        // 到达后平移速度清零
        speed_target_value = 0.0f; 
        
        // 偏航角逼近逻辑
        yaw_angle_target = approach_angle(yaw_angle_target, yaw_target, OMEGA_ACC);
        
        // 判定旋转是否结束 (角度到位且编码器脉冲数极小证明车身稳定)
        if (yaw_angle_target == yaw_target)
        {
            speed_target_angle = yaw_angle_target;
            car_2p_runing_flag = 0;
            car_2p_timer_count = 0;
            x_start = x_world;
            y_start = y_world;
            turn_flag = 0;
        }
    }
}

// 2P运动触发初始化
void car_2p_start(float x, float y, float yaw)
{
    x_start = x_world;
    y_start = y_world;
    x_target = x;
    y_target = y;
    float ab_x = x_target - x_start;
    float ab_y = y_target - y_start;
    path_length_sq = ab_x * ab_x + ab_y * ab_y;
    yaw_target = yaw;
    
    // 状态机重置
    speed_target_value = 0.0f; 
    turn_flag = 0;
    car_2p_timer_count = 0;
    car_2p_timer_flag = 0;
    car_2p_runing_flag = 1;
}

float dis_t;

// 栅格地图坐标系启动接口
void car_2p_start_map(float x, float y, float yaw)
{
    if(x > 13 || x < 0 || y > 9 || y < 0) return;
    dis_t = sqrtf((x-GET_X(player))*(x-GET_X(player))+(y-GET_Y(player))*(y-GET_Y(player)));
         if(dis_t <= 1 )MECANUM_KX = 0.770f,MECANUM_KY = 0.770f,pid_pos_speed_Kp = 4.5;
    else if(dis_t <= 2 )MECANUM_KX = 0.800f,MECANUM_KY = 0.820f,pid_pos_speed_Kp = 4.0;
    else if(dis_t <= 3 )MECANUM_KX = 0.900f,MECANUM_KY = 0.900f,pid_pos_speed_Kp = 4.0;
    else if(dis_t <= 4 )MECANUM_KX = 0.940f,MECANUM_KY = 0.940f,pid_pos_speed_Kp = 4.0;
    else if(dis_t <= 5 )MECANUM_KX = 0.940f,MECANUM_KY = 0.940f,pid_pos_speed_Kp = 3.5;
    else if(dis_t <= 6 )MECANUM_KX = 0.940f,MECANUM_KY = 0.940f,pid_pos_speed_Kp = 3.5;
    else if(dis_t <= 7 )MECANUM_KX = 0.940f,MECANUM_KY = 0.960f,pid_pos_speed_Kp = 3.5;
    else if(dis_t <= 8 )MECANUM_KX = 0.960f,MECANUM_KY = 0.980f,pid_pos_speed_Kp = 3.5;
    else if(dis_t <= 9 )MECANUM_KX = 1.000f,MECANUM_KY = 1.000f,pid_pos_speed_Kp = 3.5;
    else if(dis_t <= 10)MECANUM_KX = 1.100f,MECANUM_KY = 1.000f,pid_pos_speed_Kp = 3.5;
    else if(dis_t <= 11)MECANUM_KX = 1.300f,MECANUM_KY = 1.100f,pid_pos_speed_Kp = 3.5;
    else if(dis_t <= 12)MECANUM_KX = 1.300f,MECANUM_KY = 1.100f,pid_pos_speed_Kp = 3.5;
    else                MECANUM_KX = 1.300f,MECANUM_KY = 1.100f,pid_pos_speed_Kp = 3.5;

    car_2p_start(200 + 100 + x * 200, -(200 + 100 + y * 200), yaw);
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

        // printf("%d %d %f\n",A_path_x[k], A_path_y[k], yaw_target_t);
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
