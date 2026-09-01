#ifndef _USER_DEBUG_H_
#define _USER_DEBUG_H_

#include "common.h"

#define MAX_DATA_BUFFER_SIZE ( 20 )		// 最大数据存储量

extern float debug_t1,debug_t2;

// JUSTFLOAT数据添加
void justfloat_add(const uint32 data_num, ...);
// JUSTFLOAT数据发送
void justfloat_send();

void debug();

#endif