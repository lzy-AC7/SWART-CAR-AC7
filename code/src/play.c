#include "play.h"

bool PLAN = 1;
int BOM_PLAN = 1;// 0 1 2 3
int BOM_PLAN_count,BOM_PLAN_init[MAX_SIZE],BOM_PLAN_size;
int i,j;

//下 右 上 左
static const int dx[8] = {0, 1, 0, -1, 1, 1,-1,-1};
static const int dy[8] = {1, 0, -1, 0, 1,-1,-1, 1};

int arr_A[2][MAX_SIZE],arr_C[2][MAX_SIZE];
void swap(int *a,int *b){int tem = *a;*a = *b,*b = tem;}
void reverse(int k, int start, int end) 
{
    while (start < end) 
        swap(&arr_A[k][start],&arr_A[k][end]),start++,end--;
}
void arr_A_init(int k,int n)
{
    for(i = 0;i < n;i++)
        arr_A[k][i] = i;
}

bool nextPermutation(int k,int n) 
{
    i = n - 2;
    while (i >= 0 && arr_A[k][i] >= arr_A[k][i + 1])i--;
    if (i < 0)return 0; 
    j = n - 1;
    while (arr_A[k][j] <= arr_A[k][i])j--;
    swap(&arr_A[k][i], &arr_A[k][j]);
    reverse(k,i + 1, n - 1);

    return 1; // 成功生成下一个排列
}

void arr_C_init(int k,int n)
{
    for(i = 0;i < n;i++)
        arr_C[k][i] = i;
}

bool nextCombination(int k,int n,int x) 
{

    int i = x - 1;
    while (i >= 0 && arr_C[k][i] == n - x + i) 
        i--;
    if (i < 0)
        return 0;
    arr_C[k][i]++;
    for (int j = i + 1; j < x; j++)
        arr_C[k][j] = arr_C[k][j - 1] + 1;
    return 1;// 成功生成下一个组合
}

char *map_txt[ROW] =
{
    "-#---#--#----.",
    ".-$#-#--####--",
    "#-##-#--#-----",
    "--#--#$##--#--",
    "--#-#---------",
    "--#-------##--",
    "---------*-#--",
    "----#-----$*--",
    "-*--########--",
    "----#.--------",
};

//地图
// char map_txt[ROW][COL];
int map[ROW*COL],map_t[ROW*COL];//0:空 1:墙 2:箱 3:未识别目的地 4:炸弹 已识别目的地为10-19
int box_init[MAX_SIZE], target_init[MAX_SIZE],bom_init[MAX_SIZE],size,bom_size;

void map_init()
{
    int t2 = 0,t3 = 0,t4 = 0;
    do
    {
        t2 = 0,t3 = 0,t4 = 0;
        system_delay_ms(300);
        for(i = 0;i < ROW*COL;i++)
            map[i] = map_t[i] = 0;
        process_map_data();

        for (j = 0; j < 10; j++)
            for (i = 0; i < 14; i++)
            {
                if (map_txt[j][i] == '#')
                    map[GET_ID(i,j)] = map_t[GET_ID(i,j)] = 1;
                else if (map_txt[j][i] == '$')
                    map[GET_ID(i,j)] = map_t[GET_ID(i,j)] = 2,box_init[t2++] = GET_ID(i, j);
                else if (map_txt[j][i] == '.')
                    map[GET_ID(i,j)] = map_t[GET_ID(i,j)] = 3,target_init[t3++] = GET_ID(i, j);    
                else if (map_txt[j][i] == '*')
                    map[GET_ID(i,j)] = map_t[GET_ID(i,j)] = 4,bom_init[t4++] = GET_ID(i, j);
            }
    } while (t2 != t3);
    
    size = t2;
    bom_size = t4;
    if(GET_X(bom_init[0]) > GET_X(bom_init[1]))swap(&bom_init[0],&bom_init[1]);
}

int T_path_x[251*MAX_SIZE],T_path_y[251*MAX_SIZE],T_path_size;
void opt()
{
    T_path_size = 0;
    T_path_x[T_path_size] = A_path_x[0],T_path_y[T_path_size++] = A_path_y[0];
    i = 0;
    while(i < A_path_size)
    {
        int p = i+1;
        while(p<A_path_size && LineCheck(A_path_x[i],A_path_y[i],A_path_x[p],A_path_y[p]))p++;
        p--;
        if(p<A_path_size-1 && p == i && ((map[GET_ID(A_path_x[p+1],A_path_y[p+1])] == 2) || (map[GET_ID(A_path_x[p+1],A_path_y[p+1])]%100/10 == 1)))
        {
            p++;
            if((map[GET_ID(2*A_path_x[p]-A_path_x[i],2*A_path_y[p]-A_path_y[i])] == 3) 
            || (map[GET_ID(2*A_path_x[p]-A_path_x[i],2*A_path_y[p]-A_path_y[i])]-10 == map[GET_ID(A_path_x[p],A_path_y[p])]))
                map[GET_ID(2*A_path_x[p]-A_path_x[i],2*A_path_y[p]-A_path_y[i])] = 0;
            else 
                map[GET_ID(2*A_path_x[p]-A_path_x[i],2*A_path_y[p]-A_path_y[i])] = map[GET_ID(2*A_path_x[p]-A_path_x[i],2*A_path_y[p]-A_path_y[i])]*100+map[GET_ID(A_path_x[p],A_path_y[p])]%100;
            map[GET_ID(A_path_x[p],A_path_y[p])] /= 100;

            // for(int i = 0;i < ROW;i++)
            // {
            //     for(int j = 0;j < COL;j++)
            //         printf("%d ",map[GET_ID(j,i)]);
            //     putchar('\n');
            // }
            // putchar('\n');
        }
        if(p<A_path_size-1 && p == i && ((map[GET_ID(A_path_x[p+1],A_path_y[p+1])] == 4) || (map[GET_ID(A_path_x[p+1],A_path_y[p+1])]%100 == 4)))
        {
            p++;
            if((map[GET_ID(2*A_path_x[p]-A_path_x[i],2*A_path_y[p]-A_path_y[i])] == 1))
                map[GET_ID(2*A_path_x[p]-A_path_x[i],2*A_path_y[p]-A_path_y[i])] = 0;
            else
                map[GET_ID(2*A_path_x[p]-A_path_x[i],2*A_path_y[p]-A_path_y[i])] = map[GET_ID(2*A_path_x[p]-A_path_x[i],2*A_path_y[p]-A_path_y[i])]*100+map[GET_ID(A_path_x[p],A_path_y[p])]%100;
            map[GET_ID(A_path_x[p],A_path_y[p])] /= 100;
            // for(int i = 0;i < ROW;i++)
            // {
            //     for(int j = 0;j < COL;j++)
            //         printf("%d ",map[GET_ID(j,i)]);
            //     putchar('\n');
            // }
            // putchar('\n');
        }
        T_path_x[T_path_size] = A_path_x[p],T_path_y[T_path_size++] = A_path_y[p];
        i = p;
        if(i == A_path_size-1)
        {
            T_path_x[T_path_size] = A_path_x[p+1],T_path_y[T_path_size] = A_path_y[p+1];
            break;
        }
    }
    for(i = 0;i <= T_path_size;i++)
        A_path_x[i] = T_path_x[i],A_path_y[i] = T_path_y[i];
    
    A_path_size = T_path_size;
}

typedef struct
{
    int x;      // 坐标 / ID
    int dir;    // 到达该节点时的移动方向
    int g;      // 实际代价
    int h;      // 启发代价
    int f;      // g + h
    int pre;    // 父节点
    int turn;   // 从起点到当前节点累计转弯次数
} Node;

Node ns_player[ROW * COL*4],ns_box[ROW * COL*4];
int ns_size_player = 0, ns_size_box = 0;
bool visited_player[ROW * COL];
//小根堆
int heap_player[ROW * COL*4],heap_box[ROW * COL*4];
int heap_size_player = 0, heap_size_box = 0;

int curr_p;

// 规则：
// 1. f 越小越优先
// 2. f 相同，turn 越少越优先
static inline int player_better(int a, int b)
{
    if(ns_player[a].f != ns_player[b].f)
        return ns_player[a].f < ns_player[b].f;

    return ns_player[a].turn < ns_player[b].turn;
}

void push_player(int idx)
{
    heap_player[heap_size_player] = idx;
    curr_p = heap_size_player++;
    while (curr_p > 0)
    {
        int parent = (curr_p - 1) / 2;
        if (player_better(heap_player[curr_p],heap_player[parent]))
        {
            swap(&heap_player[curr_p], &heap_player[parent]);
            curr_p = parent;
        }
        else
            break;
    }
}

int pop_player()
{
    int top = heap_player[0];
    heap_player[0] = heap_player[--heap_size_player];
    curr_p = 0;
    while (curr_p * 2 + 1 < heap_size_player)
    {
        int left = curr_p * 2 + 1;
        int right = curr_p * 2 + 2;
        int smallest = curr_p;
        if (left < heap_size_player && player_better(heap_player[left],heap_player[smallest]))
            smallest = left;
        if (right < heap_size_player && player_better(heap_player[right],heap_player[smallest]))
            smallest = right;
        if (smallest != curr_p) 
        {
            swap(&heap_player[curr_p], &heap_player[smallest]);
            curr_p = smallest;
        } 
        else break;
    }
    return top;
}

void push_box(int idx)
{
    heap_box[heap_size_box] = idx;
    curr_p = heap_size_box++;
    while (curr_p > 0)
    {
        int parent = (curr_p - 1) / 2;
        if (ns_box[heap_box[curr_p]].f < ns_box[heap_box[parent]].f)
        {
            swap(&heap_box[curr_p], &heap_box[parent]);
            curr_p = parent;
        }
        else
            break;
    }
}

int pop_box()
{
    int top = heap_box[0];
    heap_box[0] = heap_box[--heap_size_box];
    curr_p = 0;
    while (curr_p * 2 + 1 < heap_size_box)
    {
        int left = curr_p * 2 + 1;
        int right = curr_p * 2 + 2;
        int smallest = curr_p;
        if (left < heap_size_box && ns_box[heap_box[left]].f < ns_box[heap_box[smallest]].f)
            smallest = left;
        if (right < heap_size_box && ns_box[heap_box[right]].f < ns_box[heap_box[smallest]].f)
            smallest = right;
        if (smallest != curr_p) 
        {
            swap(&heap_box[curr_p], &heap_box[smallest]);
            curr_p = smallest;
        } 
        else break;
    }
    return top;
}

bool check_0(int x,int y)//玩家位置合法检查（界内非墙非箱非炸弹）
{
    if(x>=0 && x<COL && y>=0 && y<ROW && (map_t[GET_ID(x,y)] != 1) && (map_t[GET_ID(x,y)] != 2) && (map_t[GET_ID(x,y)] != 4) && (map_t[GET_ID(x,y)]/10 != 1)) return true;
    return false;
}

bool check(int x,int y,int target)//箱子位置合法检查
{
    if(x == GET_X(target) && y == GET_Y(target))return true;//目的地

    //合法
    if(!check_0(x,y))return false;

    //非未识别目的地  非相同目的地
    if(map_t[GET_ID(x,y)] == map_t[target])return false;

    if((DIS_MANHATTAN(x,y,GET_X(target),GET_Y(target)) == 1) && (map_t[target] == 1))return true;//推炸弹特判

    //死角
    if((!check_0(x,y-1)) && (!check_0(x+1,y))) return false;
    if((!check_0(x+1,y)) && (!check_0(x,y+1))) return false;
    if((!check_0(x,y+1)) && (!check_0(x-1,y))) return false;
    if((!check_0(x-1,y)) && (!check_0(x,y-1))) return false;

    //贴墙
    if(x == 0 && GET_X(target) != 0) return false;
    if(x == COL-1 && GET_X(target) != COL-1) return false;
    if(y == 0 && GET_Y(target) != 0) return false;
    if(y == ROW-1 && GET_Y(target) != ROW-1) return false;

    return true;
}

int t_player,curr_player;
int nx_payer,ny_player,ng,nh,nt,next_player;
//玩家A* 当不存在路径将返回0，存在路径返回最后一步节点标号（路径长度为0时返回1）
int a_star_player(int ax,int ex)
{
    if(!check_0(GET_X(ax), GET_Y(ax)) || !check_0(GET_X(ex), GET_Y(ex))) return 0;

    memset(visited_player,0,sizeof(visited_player));
    ns_size_player = heap_size_player = 0;

    t_player = DIS_MANHATTAN(GET_X(ax), GET_Y(ax), GET_X(ex), GET_Y(ex));
    ns_player[++ns_size_player] = (Node){ax,-1,0,t_player,t_player,0,0};
    push_player(ns_size_player);

    while (heap_size_player > 0) 
    {
        curr_player = pop_player();

        if(visited_player[ns_player[curr_player].x])continue;
        visited_player[ns_player[curr_player].x] = true;

        if (ns_player[curr_player].x == ex) 
            return curr_player;

        for (int dir = 0; dir < 4; dir++) 
        {
            nx_payer = GET_X(ns_player[curr_player].x) + dx[dir];
            ny_player = GET_Y(ns_player[curr_player].x) + dy[dir];
            next_player = GET_ID(nx_payer, ny_player);
            if (check_0(nx_payer,ny_player) && !visited_player[next_player])//玩家移动合法检查+去重
            {
                ng = ns_player[curr_player].g + 1;
                nh = DIS_MANHATTAN(nx_payer, ny_player, GET_X(ex), GET_Y(ex));
                if(ns_player[curr_player].dir == -1 || ns_player[curr_player].dir == dir)
                    nt = ns_player[curr_player].turn;
                else nt = ns_player[curr_player].turn + 1;
                ns_player[++ns_size_player] = (Node){next_player,dir, ng, nh, ng + nh, curr_player,nt};
                push_player(ns_size_player);
            }
        }
    }
    return 0;
}

bool visited_box[4][ROW * COL];
int nx_box,ny_box,t_box,bx,by,player_to_box,curr_box,next_box,id_pre,idd_pre;
//箱子A*(玩家初始位置为player_t)当不存在路径将返回0，存在路径返回最后一步节点标号（无路径长度为0的情况）
int a_star_box(int player_t,int box,int target)
{
    id_pre = map_t[box];
    map_t[box] = 0;
    memset(visited_box,0,sizeof(visited_box));
    ns_size_box = heap_size_box = 0;
    bx = GET_X(box), by = GET_Y(box);
    t_box = DIS_MANHATTAN(bx,by,GET_X(target),GET_Y(target));
    //下 右 上 左(箱子移动方向)
    for (int dir = 0; dir < 4; dir++) 
    {
        nx_box = bx + dx[dir];
        ny_box = by + dy[dir];
        next_box = GET_ID(nx_box, ny_box);
        if(check_0(bx-dx[dir], by-dy[dir]) && check(nx_box, ny_box,target))
        {
            
            map_t[box] = id_pre;
            player_to_box = a_star_player(player_t,GET_ID(bx-dx[dir], by-dy[dir]));
            map_t[box] = 0;
            if(player_to_box)
            {
                ns_box[++ns_size_box] = (Node){next_box,dir,ns_player[player_to_box].g+1, t_box,t_box+ns_player[player_to_box].g+1, 0},
                push_box(ns_size_box);
            }
        }
    }

    while (heap_size_box > 0)
    {
        curr_box = pop_box();
        bx = GET_X(ns_box[curr_box].x), by = GET_Y(ns_box[curr_box].x);

        if(visited_box[ns_box[curr_box].dir][ns_box[curr_box].x])continue;
        visited_box[ns_box[curr_box].dir][ns_box[curr_box].x] = true;

        if (ns_box[curr_box].x == target)
        {
            map_t[box] = id_pre;
            return curr_box;
        }
        
        
        for (int dir = 0; dir < 4; dir++) //下 右 上 左(箱子移动方向)
        {
            nx_box = bx + dx[dir];
            ny_box = by + dy[dir];
            
            next_box = GET_ID(nx_box, ny_box);
            if (!visited_box[dir][next_box] && check(nx_box,ny_box,target))//检查箱子移动合法检查+去重
            {
                t_box = DIS_MANHATTAN(nx_box,ny_box,GET_X(target),GET_Y(target));
                if(dir == ns_box[curr_box].dir)//与上次移动方向相同
                {
                    ns_box[++ns_size_box] = (Node){next_box,dir,ns_box[curr_box].g+1, t_box,ns_box[curr_box].g+1 + t_box, curr_box},
                    push_box(ns_size_box);
                }
                else //与上次移动方向不同
                {
                    idd_pre = map_t[ns_box[curr_box].x],map_t[ns_box[curr_box].x] = id_pre;
                    player_to_box = a_star_player(GET_ID(bx - dx[ns_box[curr_box].dir], by - dy[ns_box[curr_box].dir]), GET_ID(bx-dx[dir], by-dy[dir]));
                    map_t[ns_box[curr_box].x] = idd_pre;
                    if(player_to_box)
                        ns_box[++ns_size_box] = (Node){next_box,dir,ns_box[curr_box].g+ns_player[player_to_box].g+1, t_box ,ns_box[curr_box].g+ns_player[player_to_box].g+1 + t_box,curr_box},
                        push_box(ns_size_box);
                }
            }
        }
    }
    map_t[box] = id_pre;
    return 0;
}


int path_x[ROW * COL*MAX_SIZE],path_y[ROW * COL*MAX_SIZE];
int path_size;
int curr_player_path, curr_box_path;
//建造路径(倒序存在path)
int make_path(int playerr,int box, int target)
{
    path_size = 0;
    id_pre = map_t[box];
    curr_box_path = a_star_box(playerr,box, target);
    if (curr_box_path == 0) return 0;
    while (curr_box_path != 0) 
    {
        if (ns_box[curr_box_path].pre != 0)
        {
            //直线和转弯
            if (ns_box[ns_box[curr_box_path].pre].dir == (ns_box[curr_box_path].dir)) 
                // 直线推行：路径为上一个箱子位置
                path_x[path_size] = GET_X(ns_box[ns_box[curr_box_path].pre].x),
                path_y[path_size++] = GET_Y(ns_box[ns_box[curr_box_path].pre].x);
            else 
            {
                // 转弯
                //上一箱子位置
                path_x[path_size] = GET_X(ns_box[ns_box[curr_box_path].pre].x), path_y[path_size++] = GET_Y(ns_box[ns_box[curr_box_path].pre].x);
                // 上一箱子位置设为障碍
                map_t[box] = 0,map_t[GET_ID(GET_X(ns_box[ns_box[curr_box_path].pre].x), GET_Y(ns_box[ns_box[curr_box_path].pre].x))] = id_pre; 
                curr_player_path = a_star_player(GET_ID(GET_X(ns_box[ns_box[curr_box_path].pre].x) - dx[ns_box[ns_box[curr_box_path].pre].dir], GET_Y(ns_box[ns_box[curr_box_path].pre].x) - dy[ns_box[ns_box[curr_box_path].pre].dir]),
                                            GET_ID(GET_X(ns_box[curr_box_path].x) - 2*dx[ns_box[curr_box_path].dir], GET_Y(ns_box[curr_box_path].x) - 2*dy[ns_box[curr_box_path].dir]));
                map_t[box] = id_pre,map_t[GET_ID(GET_X(ns_box[ns_box[curr_box_path].pre].x), GET_Y(ns_box[ns_box[curr_box_path].pre].x))] = 0;

                while (curr_player_path != 0) 
                    path_x[path_size] = GET_X(ns_player[curr_player_path].x),path_y[path_size++] = GET_Y(ns_player[curr_player_path].x),
                    curr_player_path = ns_player[curr_player_path].pre;
                path_size--;//推上一箱子路径已经记录
            }
        }
        else if (ns_box[curr_box_path].pre == 0) // 玩家抵达箱子的第一段路径
        {
            path_x[path_size] = GET_X(box), path_y[path_size++] = GET_Y(box);
            
            curr_player_path = a_star_player(playerr, GET_ID(GET_X(box) - dx[ns_box[curr_box_path].dir], GET_Y(box) - dy[ns_box[curr_box_path].dir]));

            while (curr_player_path != 0)
                path_x[path_size] = GET_X(ns_player[curr_player_path].x), path_y[path_size++] = GET_Y(ns_player[curr_player_path].x),
                curr_player_path = ns_player[curr_player_path].pre;

            break;
        }
        curr_box_path = ns_box[curr_box_path].pre;
    }
    return path_size;
}

int sovled_count = 0;

int t_G,min_G,min_arr[2][MAX_SIZE],min_type,min_p,min_dir;
int t;
bool timeout;

bool checkpoint1_running;
void checkpoint1()
{
    if((!checkpoint1_running) || car_runing_path_flag)return;
    if(sovled_count == size)
    {
        checkpoint1_running = 0;
        return;
    }
    //初始化地图标记
    for(i = 0;i < ROW*COL;i++)
        map_t[i] = map[i];
    int player_t = player;
    min_G = 1e9;
    for(i = 0;i < size;i++)//推箱子
    {
        if((map[box_init[i]] == 0))continue;
        for(j = 0;j < size;j++)//找匹配
        {
            if((map[target_init[j]] == 0))continue;
            int curr = a_star_box(player_t,box_init[i],target_init[j]);
            if(curr && ns_box[curr].g < min_G)
            {
                min_G = ns_box[curr].g;//代价
                min_type = 2;//推箱子
                min_p = i;//箱子下标
                min_dir = j;//目的地下标
            }
        }
    }

    if(min_G < 1e9)
    {
        printf("**%d\n",min_G);
        make_path(player_t,box_init[min_p],target_init[min_dir]);
        for(i = path_size-1;i >= 0;i--)
            A_path_x[A_path_size] = path_x[i],A_path_y[A_path_size++] = path_y[i];
        A_path_x[A_path_size] = 333,A_path_y[A_path_size] = 333;
        opt();
        car_runing_path_start(yaw_angle_target);
        map[box_init[min_p]] = map[target_init[min_dir]] = 0;
        sovled_count++;
    }
}

int id[2][MAX_SIZE],id_count;//0:箱子  1:目的地
bool checkpoint2_running;
bool boming;
int wall[ROW*COL],wall_n;
int bom_waitting[2][MAX_SIZE],bom_waitting_n;


// 全局记录最小代价和最佳策略
int min_sum = 1e9;
int best_bom_t[2][MAX_SIZE];
int best_k = 0;

// 【安全剪枝】：只记录在初始地图下，首发(depth==0)去炸毫无意义的墙。
// 避免因为排列顺序不同而产生误杀。
bool invalid_first_wall[MAX_SIZE][ROW * COL] = {0};

/*
 * 【核心深搜函数 - 全排列版】
 * @param depth        : 当前已经是第几颗炸弹了（深度限制，不超过 bom_size）
 * @param used_bomb    : 布尔数组，记录哪些炸弹在当前的探索路径里已经被用掉了
 * 返回值：当前分支所能达到的【最大任务完成数量】
 */
int dfs_bombs(int current_player, int current_cost, 
              int* current_map, bool current_sloved[3][MAX_SIZE], int current_completed,
              int current_path_bom[2][MAX_SIZE], int depth, bool used_bomb[MAX_SIZE]) 
{
    // =========================================================
    // 阶段 1：推箱子跑任务 (贪心推进当前地图的所有可做任务)
    // =========================================================
    int added_score = 0;
    bool push_fail = 0;
    while(1)
    {
        memcpy(map_t,current_map,sizeof(map_t));
        int min_G = 1e9, min_type = -1, min_p = -1, min_dir = -1;
        bool stuck = true;

        // (此处的两段 for 循环与之前完全相同：识别箱子和目标点)
        for(int i = 0; i < size; i++) {
            if(id[0][i] == -1 && !current_sloved[0][i]) {
                int x = GET_X(box_init[i]), y = GET_Y(box_init[i]);
                for(int j = 0; j < 4; j++) {
                    if(check_0(x+dx[j], y+dy[j])) {
                        int curr = a_star_player(current_player, GET_ID(x+dx[j], y+dy[j]));
                        if(curr && ns_player[curr].g < min_G) { min_G = ns_player[curr].g; min_type = 0; min_p = i; min_dir = j; stuck = false; }
                    }
                }
            }
            if(id[1][i] == -1 && !current_sloved[1][i]) {
                int x = GET_X(target_init[i]), y = GET_Y(target_init[i]);
                for(int j = 0; j < 4; j++) {
                    if(check_0(x+dx[j], y+dy[j])) {
                        int curr = a_star_player(current_player, GET_ID(x+dx[j], y+dy[j]));
                        if(curr && ns_player[curr].g < min_G) { min_G = ns_player[curr].g; min_type = 1; min_p = i; min_dir = j; stuck = false; }
                    }
                }
            }
        }
        
        for(int i = 0; i < size; i++) {
            int s = 0;
            if((id[0][i] == -1) || (current_map[box_init[i]] == 0) || current_sloved[0][i]) continue;
            for(int j = 0; j < size; j++) {
                if((id[0][i] != id[1][j]) || (current_map[target_init[j]] == 0)) continue;
                s++;
                int curr = a_star_box(current_player, box_init[i], target_init[j]);
                if(curr)s+=100;
                if(curr && ns_box[curr].g < min_G) { min_G = ns_box[curr].g; min_type = 2; min_p = i; min_dir = j; stuck = false; }
            }
            if(s>0 && s<100)push_fail = 1;
        }
        if(stuck || min_G >= 1e9) break; // 没任务可做或者被卡死，跳出
        added_score++;
        current_sloved[min_type][min_p] = 1;
        current_cost += min_G;
        
        if(min_type < 2) {
            if(!min_type) current_player = GET_ID(GET_X(box_init[min_p])+dx[min_dir], GET_Y(box_init[min_p])+dy[min_dir]);
            else          current_player = GET_ID(GET_X(target_init[min_p])+dx[min_dir], GET_Y(target_init[min_p])+dy[min_dir]);
        } else {
            current_player = target_init[min_dir];
            current_map[box_init[min_p]] = current_map[target_init[min_dir]] = 0;
        }
        
    }
    
    current_completed += added_score;
    int max_achieved = current_completed; // 记录本分支及子树所能探到的最大效益
    // =========================================================
    // 阶段 2：通关校验
    // =========================================================
    if (current_cost >= min_sum) return max_achieved; // 代价剪枝

    bool all_done = true;
    for(int i = 0; i < size; i++) 
    {
        if(id[0][i] == -1 && !current_sloved[0][i]) { all_done = false; break; }
        if(id[1][i] == -1 && !current_sloved[1][i]) { all_done = false; break; } 
        
    }
    if(push_fail)all_done = false;
    if (all_done) 
    {
        if (current_cost < min_sum) {
            min_sum = current_cost;
            best_k = depth;
            for(int i = 0; i < depth; i++) {
                best_bom_t[0][i] = current_path_bom[0][i];
                best_bom_t[1][i] = current_path_bom[1][i];
            }
        }
        return max_achieved; 
    }

    if (depth >= bom_size) return max_achieved; // 炸弹名额用尽，还没通关，强制回溯

    // =========================================================
    // 阶段 3：深搜拓展（遍历【所有未使用的炸弹】和【墙】的组合）
    // =========================================================
    for(int b = 0; b < bom_size; b++) 
    {
        // 如果这颗炸弹在当前路线里已经炸过了，跳过
        if (used_bomb[b]) continue;

        for(int w = 0; w < wall_n; w++) 
        {
            int target_wall = wall[w];
            
            if (current_map[target_wall] == 0) continue;
            
            // 安全剪枝：如果是第一手推炸弹，并且全局记录为绝对废墙，跳过
            if (invalid_first_wall[b][target_wall] == 1) continue;
            memcpy(map_t,current_map,sizeof(map_t));
            int t = a_star_box(current_player, bom_init[b], target_wall);
            if (!t) continue;
            
            int push_cost = ns_box[t].g;
            if (current_cost + push_cost >= min_sum) continue;

            // --- 备份当前环境 ---
            int next_map[ROW * COL];
            memcpy(next_map,current_map,sizeof(map_t));
            bool next_sloved[3][MAX_SIZE];
            for(int h = 0; h < 3; h++) 
                for(int v = 0; v < MAX_SIZE; v++) 
                    next_sloved[h][v] = current_sloved[h][v];
            
            // --- 模拟引爆 ---
            next_map[bom_init[b]] = 0;
            next_map[target_wall] = 0;
            int x0 = GET_X(target_wall), y0 = GET_Y(target_wall);
            for(int dir = 0; dir < 8; dir++) {
                if(x0+dx[dir]>=0 && x0+dx[dir]<COL && y0+dy[dir]>=0 && y0+dy[dir]<ROW) {
                    if(next_map[GET_ID(x0+dx[dir],y0+dy[dir])] == 1)
                        next_map[GET_ID(x0+dx[dir],y0+dy[dir])] = 0;
                }
            }
                                                                    
            int next_player = GET_ID(GET_X(ns_box[t].x) - dx[ns_box[t].dir], GET_Y(ns_box[t].x) - dy[ns_box[t].dir]);
            current_path_bom[0][depth] = b;
            current_path_bom[1][depth] = target_wall;
            
            // 标记这颗炸弹被征用了
            used_bomb[b] = true;
            
            // --- 往深处搜（深度+1） ---
            int branch_max = dfs_bombs(next_player, current_cost + push_cost, 
                                       next_map, next_sloved, current_completed, 
                                       current_path_bom, depth + 1, used_bomb);
            
            // 回溯时，把炸弹状态还回来
            used_bomb[b] = false;
            
            if (branch_max > max_achieved) {
                max_achieved = branch_max;
            }

            // 【真伪废墙判定】：
            // 如果我们穷尽了从这步往下所有的炸弹配合组合，
            // 带回来的最高任务量都没有超过炸这堵墙之前的进度，
            // 说明这个选项绝对是废的。如果碰巧它还是 depth == 0，记入全局黑名单！
            if (branch_max <= current_completed) {
                // invalid_first_wall[b][target_wall] = 1;
                                                            // for(int i = 0;i < bom_size;i++)
                                                            // {
                                                            //     putchar('\n');
                                                            //     printf("%d:\n",i);
                                                            //     for(int i = 0;i < ROW;i++)
                                                            //     {
                                                            //         for(int j = 0;j < COL;j++)
                                                            //             printf("%d ",invalid_first_wall[i][GET_ID(j,i)]);
                                                            //         putchar('\n');
                                                            //     }
                                                            //     putchar('\n');                                                                
                                                            // }
            }
        }
    }
    
    return max_achieved;
}


float dir_to_angle(float t)
{
    if(t == 0)return 90.0f;
    if(t == 1)return 180.0f;
    if(t == 2)return -90.0f;
    if(t == 3)return 0.0f;
    return 0;
}

bool c_min(int a,int b)
{
    a = dir_to_angle(a);
    b = dir_to_angle(b);
    return fabs(a-yaw_angle) < fabs(b-yaw_angle);
}
int id_c[2][11];

void checkpoint2()
{
    if((!checkpoint2_running) || car_runing_path_flag)return;
    if(sovled_count == size)
    {
        checkpoint2_running = 0;
        return;
    }
    
    if(id_count < size*2 && (id_c[0][10] == size || id_c[1][10] == size))
    {
        int full = (id_c[0][10] == size) ? 0 : 1;
        int other = 1 - full;

        int remain_id = -1;
        int remain_num = 0;

        for(i = 0; i < 10; i++)
        {
            int n = id_c[full][i] - id_c[other][i];

            if(n > 0)
            {
                if(remain_id == -1)
                {
                    remain_id = i;
                    remain_num = n;
                }
                else
                {
                    // 剩余存在多个不同 ID，不能推
                    remain_id = -2;
                    break;
                }
            }
        }

        // 只有一个剩余 ID，且数量刚好等于未知数量时才推
        if(remain_id >= 0 &&
        remain_num == size - id_c[other][10])
        {
            for(i = 0; i < size; i++)
            {
                if(id[other][i] == -1)
                {
                    id[other][i] = remain_id;

                    if(other == 0)
                        map[box_init[i]] = 10 + remain_id;
                    else
                        map[target_init[i]] = 20 + remain_id;

                    id_c[other][remain_id]++;
                    id_c[other][10]++;
                    id_count++;
                }
            }
        }
    }
    if(boming && bom_size && (id_count == size*2))
    {
        for(int i = 0;i < ROW*COL;i++)
            if(map[i] == 1)
                wall[wall_n++] = i;
        
        memset(invalid_first_wall, 0, sizeof(invalid_first_wall));
        min_sum = 1e9;
        best_k = 0;

        int initial_map[ROW * COL];
        for(int i = 0; i < ROW * COL; i++) initial_map[i] = map[i];

        bool initial_sloved[3][MAX_SIZE] = {0};
        int current_path_bom[2][MAX_SIZE] = {0};
        bool used_bomb[MAX_SIZE] = {0}; // 记录炸弹占用情况

        // 启动深搜：初始 depth=0，传入未使用过的 used_bomb
        dfs_bombs(player, 0, initial_map, initial_sloved, 0, current_path_bom, 0, used_bomb);
        // --- 处理 min_sum 并气泡排序写入队列的代码同前，略 ---

        // 3. 处理最优解
        if (min_sum < 1e9) 
        {
            //降序排列
            for(int m = 0; m < best_k - 1; m++) 
                for(int n = 0; n < best_k - 1 - m; n++) 
                    if(best_bom_t[0][n] < best_bom_t[0][n+1]) {
                        swap(&best_bom_t[0][n], &best_bom_t[0][n+1]);
                        swap(&best_bom_t[1][n], &best_bom_t[1][n+1]);
                    }
                    
            for(int h = 0; h < best_k; h++) 
            {
                bom_waitting[0][bom_waitting_n] = best_bom_t[0][h];
                bom_waitting[1][bom_waitting_n++] = best_bom_t[1][h];
                for(int g = best_bom_t[0][h] + 1; g < bom_size; g++) 
                    bom_init[g-1] = bom_init[g];
                bom_size--;
            }
        }
    }

    //初始化地图标记
    for(i = 0;i < ROW*COL;i++)
        map_t[i] = map[i];
    int player_t = player;

    min_G = 1e9;
    for(i = 0;i < size;i++)//识别
    {
        bool box_ok = 0,target_ok = 0;
        if(id[0][i] == -1)
        {
            int x = GET_X(box_init[i]),y = GET_Y(box_init[i]);
            for(int j = 0;j < 4;j++)
            {
                if(check_0(x+dx[j],y+dy[j]))
                {
                    int curr = a_star_player(player_t,GET_ID(x+dx[j],y+dy[j]));
                    if(curr)box_ok = 1;
                    if(curr && ((ns_player[curr].g < min_G || (ns_player[curr].g == min_G && c_min(j,min_dir)))||
                                dir_to_angle(j) == yaw_angle_target && dir_to_angle(min_dir) != yaw_angle_target && ns_player[curr].g <= min_G+2))
                    {
                        min_G = ns_player[curr].g;//代价
                        min_type = 0;//箱子
                        min_p = i;//下标
                        min_dir = j;//方向
                    }
                }
            }
        }
        if(id[1][i] == -1)
        {
            int x = GET_X(target_init[i]),y = GET_Y(target_init[i]);
            for(int j = 0;j < 4;j++)//找最近
            {
                if(check_0(x+dx[j],y+dy[j]))
                {
                    int curr = a_star_player(player_t,GET_ID(x+dx[j],y+dy[j]));
                    if(curr)target_ok = 1;
                    if(curr && ((ns_player[curr].g < min_G || (ns_player[curr].g == min_G && c_min(j,min_dir)))||
                                dir_to_angle(j) == yaw_angle_target && dir_to_angle(min_dir) != yaw_angle_target && ns_player[curr].g <= min_G+2))
                    {
                        min_G = ns_player[curr].g;//代价
                        min_type = 1;//目的地
                        min_p = i;//下标
                        min_dir = j;//方向
                    }
                }
            }
        }
    }

    for(i = 0;i < size;i++)//推箱子
    {
        if((id[0][i] == -1) || (map[box_init[i]] == 0))continue;
        for(j = 0;j < size;j++)//找匹配
        {
            if((id[0][i] != id[1][j]) || (map[target_init[j]] == 0))continue;
            
            int curr = a_star_box(player_t,box_init[i],target_init[j]);
            if(curr && ns_box[curr].g < min_G)
            {
                min_G = ns_box[curr].g;//代价
                min_type = 2;//推箱子
                min_p = i;//箱子下标
                min_dir = j;//目的地下标
            }
        }
    }
    if(BOM_PLAN && BOM_PLAN_count)
    {
        int min_gg = min_G,min_wp,min_ww;
        for(int b = 0;b < BOM_PLAN_size;b++)
        {
            if(map[BOM_PLAN_init[b]] != 4)continue;
            for(int i = 0;i < ROW*COL;i++)
                if(map[i] == 1)
                {
                    int curr = a_star_box(player_t,BOM_PLAN_init[b],i);
                    if(curr && ns_box[curr].g < min_gg)
                    {
                        min_gg = ns_box[curr].g;
                        min_wp = b,min_ww = i;
                    }
                }
        }
        if(min_gg<min_G)
        {
            bom_init[bom_size] = BOM_PLAN_init[min_wp];
            bom_waitting[0][bom_waitting_n] = bom_size;
            bom_waitting[1][bom_waitting_n++] = min_ww;
            BOM_PLAN_count--;
        }
    }

    if(bom_waitting_n)
    {
        for(i = 0;i < bom_waitting_n;i++)
        {
            int curr = a_star_box(player_t,bom_init[bom_waitting[0][i]],bom_waitting[1][i]);
            if(curr && ns_box[curr].g < min_G)
            {
                min_G = ns_player[curr].g;//代价
                min_type = 3;//推炸弹
                min_p = i;//炸弹等待下标
                min_dir = 0;
            }
        }
    }
    if(min_G < 1e9)
    {
        if(min_type < 2)//识别
        {
            int x,y;
            if(min_type == 0)x = GET_X(box_init[min_p]),y = GET_Y(box_init[min_p]);
            else x = GET_X(target_init[min_p]),y = GET_Y(target_init[min_p]);
            int curr = a_star_player(player_t,GET_ID(x+dx[min_dir],y+dy[min_dir]));
            path_size = 0;
            while (curr != 0)
                path_x[path_size] = GET_X(ns_player[curr].x), path_y[path_size++] = GET_Y(ns_player[curr].x),
                curr = ns_player[curr].pre;
            for(i = path_size-1;i >= 0;i--)
                A_path_x[A_path_size] = path_x[i],A_path_y[A_path_size++] = path_y[i];
            A_path_x[A_path_size] = 666,A_path_y[A_path_size] = 666;//加入识别标识
            opt();
            if(min_dir == 0)car_runing_path_start(90.0f);
            if(min_dir == 1)car_runing_path_start(180.0f);
            if(min_dir == 2)car_runing_path_start(-90.0f);
            if(min_dir == 3)car_runing_path_start(0.0f);
        }
        else if(min_type == 2)//推箱子
        {
            make_path(player_t,box_init[min_p],target_init[min_dir]);
            for(i = path_size-1;i >= 0;i--)
                A_path_x[A_path_size] = path_x[i],A_path_y[A_path_size++] = path_y[i];
            A_path_x[A_path_size] = 333,A_path_y[A_path_size] = 333;
            opt();
            car_runing_path_start(yaw_angle_target);
            map[box_init[min_p]] = map[target_init[min_dir]] = 0;
            sovled_count++;
        }
        else//推炸弹
        {
            make_path(player_t,bom_init[bom_waitting[0][min_p]],bom_waitting[1][min_p]);
            for(i = path_size-1;i >= 0;i--)
                A_path_x[A_path_size] = path_x[i],A_path_y[A_path_size++] = path_y[i];
            A_path_x[A_path_size] = 999,A_path_y[A_path_size] = 999;
            opt();
            int x0 = GET_X(bom_waitting[1][min_p]),y0 = GET_Y(bom_waitting[1][min_p]);
            for(int dir = 0;dir < 8;dir++)
                if(map[GET_ID(x0+dx[dir],y0+dy[dir])] == 1)
                    map[GET_ID(x0+dx[dir],y0+dy[dir])] = 0;
            car_runing_path_start(yaw_angle_target);
            for(int g = min_p+1;g < bom_waitting_n;g++)
                bom_waitting[0][g-1] = bom_waitting[0][g],
                bom_waitting[1][g-1] = bom_waitting[1][g];
            bom_waitting_n--;
        }
    }
    else if(bom_size)//if(boming && ((min_G == 1e9 && id_count < size*2) || id_count == size*2))//尝试推炸弹且可执行任务(look/push)都最优
    {
        for(int i = 0;i < ROW*COL;i++)
            if(map[i] == 1)
                wall[wall_n++] = i;
        
        memset(invalid_first_wall, 0, sizeof(invalid_first_wall));
        min_sum = 1e9;
        best_k = 0;

        int initial_map[ROW * COL];
        for(int i = 0; i < ROW * COL; i++) initial_map[i] = map[i];

        bool initial_sloved[3][MAX_SIZE] = {0};
        int current_path_bom[2][MAX_SIZE] = {0};
        bool used_bomb[MAX_SIZE] = {0}; // 记录炸弹占用情况

        // 启动深搜：初始 depth=0，传入未使用过的 used_bomb
        dfs_bombs(player, 0, initial_map, initial_sloved, 0, current_path_bom, 0, used_bomb);
        // --- 处理 min_sum 并气泡排序写入队列的代码同前，略 ---

        // 3. 处理最优解
        if (min_sum < 1e9) 
        {
            //降序排列
            for(int m = 0; m < best_k - 1; m++) 
                for(int n = 0; n < best_k - 1 - m; n++) 
                    if(best_bom_t[0][n] < best_bom_t[0][n+1]) {
                        swap(&best_bom_t[0][n], &best_bom_t[0][n+1]);
                        swap(&best_bom_t[1][n], &best_bom_t[1][n+1]);
                    }
                    
            for(int h = 0; h < best_k; h++) 
            {
                bom_waitting[0][bom_waitting_n] = best_bom_t[0][h];
                bom_waitting[1][bom_waitting_n++] = best_bom_t[1][h];
                for(int g = best_bom_t[0][h] + 1; g < bom_size; g++) 
                    bom_init[g-1] = bom_init[g];
                bom_size--;
            }
        }
    }
}

void bubbleSort(int arr[], int n) 
{
    for (int i = 0; i < n - 1; i++) 
    {
        bool swapped = false;
        for (int j = 0; j < n - 1 - i; j++)
            //根据到小车的距离排升序
            if (DIS_MANHATTAN(0,5,GET_X(arr[j]),GET_Y(arr[j])) > DIS_MANHATTAN(0,5,GET_X(arr[j+1]),GET_Y(arr[j+1])))
                swap(arr+j,arr+j+1),
                swapped = true;
        if (!swapped)break;
    }
}


void play(int mode)
{
    A_path_size = 0;
    uint32 time = sys_time;
    if (mode == 0)
    {
        if(!PLAN)
        {
            min_G = 1e9,timeout = 0;
            bubbleSort(box_init,size);
            bubbleSort(target_init,size);
            arr_A_init(0,size);
            do//匹配
            {
                arr_A_init(1,size);
                do//顺序
                {
                    //初始化地图标记
                    for(i = 0;i < ROW*COL;i++)
                        map_t[i] = map[i];
                    t_G = 0;
                    int player_t = player;
                    for(i = 0;i < size;i++)
                    {
                        t = a_star_box(player_t,box_init[arr_A[1][i]],target_init[arr_A[0][arr_A[1][i]]]);
                        if(t)t_G += ns_box[t].g,player_t = GET_ID(GET_X(ns_box[t].x) - dx[ns_box[t].dir], GET_Y(ns_box[t].x) - dy[ns_box[t].dir]);
                        else break;
                        if(t_G >= min_G)break;
                        map_t[box_init[arr_A[1][i]]] = map_t[target_init[arr_A[0][arr_A[1][i]]]] = 0;
                    }
                    if(i == size && t_G < min_G)
                    {
                        min_G = t_G;
                        for(i = 0;i < size;i++)
                            min_arr[0][i] = arr_A[0][i],min_arr[1][i] = arr_A[1][i];
                    }
                    if(sys_time-time > size*251 && min_G < 1e9)timeout = 1;
                    if(timeout)break;
                }while(nextPermutation(1,size));
                if(timeout)break;
            }while(nextPermutation(0,size));
            //路径
            int player_t = player;
            for(i = 0;i < ROW*COL;i++)
                map_t[i] = map[i];
            for(i = 0;i < size;i++)
            {
                make_path(player_t,box_init[min_arr[1][i]],target_init[min_arr[0][min_arr[1][i]]]);
                map_t[box_init[min_arr[1][i]]] = map_t[target_init[min_arr[0][min_arr[1][i]]]] = 0;
                for(j = path_size-1;j >= 0;j--)
                    A_path_x[A_path_size] = path_x[j],A_path_y[A_path_size++] = path_y[j];
                player_t = GET_ID(path_x[0],path_y[0]);
            }
            A_path_x[A_path_size] = 333,A_path_y[A_path_size] = 333;
            // for(int i = 0;i <= A_path_size;i++)
            //     printf("[%d,%d]\n",A_path_x[i],A_path_y[i]);
            opt();
            car_runing_path_start(yaw_angle_target);
        }
        else
        {
            checkpoint1_running = 1;
        }
    }
    else if (mode == 1 || mode == 2)
    {
        memset(id,-1,sizeof(id));
        id_count = 0;
        checkpoint2_running = 1;
        if(mode == 2)boming = 1;
        if(BOM_PLAN)
        {
            if(BOM_PLAN == 1)
            {
                BOM_PLAN_count = BOM_PLAN_size = 1;
                BOM_PLAN_init[0] = bom_init[0];
                swap(&bom_init[0],&bom_init[1]);
                bom_size--;
            }else if (BOM_PLAN == 2)
            {
                BOM_PLAN_count = BOM_PLAN_size = 1;
                BOM_PLAN_init[0] = bom_init[1];
                bom_size--;
            }else if (BOM_PLAN == 3)
            {
                BOM_PLAN_count = BOM_PLAN_size = 2;
                BOM_PLAN_init[0] = bom_init[0];
                BOM_PLAN_init[1] = bom_init[1];
                bom_size = 0;
            }
        }
    }
}

bool fsm_flag;
bool played_flag = 0,backed_flag = 0,map_init_flag = 0;

void checkpoint_set(int t)
{
    checkpoint = t;
    printf("start:%d\n",checkpoint);
    car_2p_start_map(1,5,yaw_angle_target);
    played_flag = 0,backed_flag = 0,map_init_flag = 0,boming = 0,checkpoint1_running = 0,checkpoint2_running = 0,sovled_count = 0;
    fsm_flag = 1,over = 0;
}
float yaw_T;
void back()
{
    if(checkpoint == 2)
    {
        yaw_T = yaw_angle_target;
        for(int i = 0;i <= 370;i++)
        {
            yaw_angle_target = (int)(yaw_T+i)%360;
            system_delay_ms(5);
        }
        yaw_angle_target = (int)(yaw_T+10)%360;
    }
    car_2p_start_map(0,5,yaw_angle_target);
    over = 1;
}

void fsm()
{
    if(!fsm_flag)return;
    if(car_runing_path_flag || car_2p_runing_flag || checkpoint1_running || checkpoint2_running)return;
    if(!map_init_flag)map_init(),map_init_flag = 1,printf("map_init--\n");

    if(!played_flag)
    {
        play(checkpoint),played_flag = 1;
        printf("play--\n");
        return;
    }
    if(!backed_flag)
    {
        back(),backed_flag = 1;
        printf("back--\n");
        fsm_flag = 0;
        if(checkpoint<2)checkpoint++;
        else checkpoint = 0;
        return;
    }
}
