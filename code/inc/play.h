#ifndef _PLAY_H_
#define _PLAY_H_

#include "common.h"

#define ROW 10
#define COL 14
#define GET_ID(x, y) ((y) * COL + (x))
#define GET_Y(id) ((id) / COL)
#define GET_X(id) ((id) - GET_Y(id) * COL)
#define DIS_MANHATTAN(x0, y0, x1, y1) (((x0 - x1) > 0 ? x0 - x1 : x1 - x0) + ((y0 - y1) > 0 ? y0 - y1 : y1 - y0))
#define MAX_SIZE 20

// extern char map_txt[ROW][COL];
extern char *map_txt[ROW];
extern int map[ROW*COL];
extern int box_init[MAX_SIZE], target_init[MAX_SIZE],bom_init[MAX_SIZE];
extern int min_type,min_p,id[2][MAX_SIZE],id_count,id_c[2][11];//0:箱子  1:目的地;
extern bool fsm_flag;
extern bool played_flag,backed_flag,map_init_flag;
extern bool PLAN;
void map_init();
void checkpoint1();
void checkpoint2();
void play(int mode);
void checkpoint_set(int t);
void fsm();
#endif // !_PLAY_H
