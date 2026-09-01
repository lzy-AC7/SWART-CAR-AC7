#ifndef _DOG_MENU_H_
#define _DOG_MENU_H_

#include "common.h"

#define MENU_ROW_HEIGHT				(20)	// 行高
#define MENU_ROW_WIDTH				(320)	// 行宽
#define MENU_PAGE_HEIGHT			(240)	// 页高
#define MENU_PAGE_WIDTH				(320)	// 页宽
#define MENU_PAGE_MOST_ROW			(12)	// 单页最多行
/****************************** 菜单组件 ******************************/

typedef enum 
{
	PAGE,
	DIGITAL,
	FUNCTION
} _MENU_KIND_;

typedef enum 
{
	BOOL,	//0
	UINT8,
	UINT16,
	UINT32,
	UINT64,
	INT8,	//5
	INT16,
	INT32,
	INT64,
	FLOAT,	//9
	DOUBLE
} _MENU_DATA_KIND_;

typedef union
{	
	uint8 son;
	bool *p_bool;
	uint8 *p_uint8;
	uint16 *p_uint16;
	uint32 *p_uint32;
	uint64 *p_uint64;
	int8 *p_int8;
	int16 *p_int16;
	int32 *p_int32;
	int64 *p_int64;
	float *p_float;
	double *p_double;
	void (*fp)(bool);
	
}_PTR_;

/* 菜单单元 */
typedef struct
{
	char* name;			//单元名称
	_MENU_KIND_ kind;	//单元类型
	uint8 fa;			//上一级单元
	uint8 next; 		//同级单元
	_PTR_ data;			//单元数据指针
	_MENU_DATA_KIND_ data_kind;
}MENU_UNION;



void menu_init(void);						//菜单初始化 
void menu_runing(void);						//菜单运行
void menu_back(bool en);					//返回上一级菜单
void speed_change(bool en);					//关卡切换
void plan_change(bool en);
void OPTIMAL_SET(bool en);					//炸弹策略选择
void wifi_init(bool en);					//WIFI初始化
void calibrate(bool en);					//陀螺仪标定
void speed_target_change(bool en);			//速度目标设置
void Kp1_change(bool en);					//速度环Kp设置
void Ki1_change(bool en);					//速度环Ki设置
void Kd1_change(bool en);					//速度环Kd设置
void Kp2_change(bool en);					//速度环Kp设置
void Ki2_change(bool en);					//速度环Ki设置
void Kd2_change(bool en);					//速度环Kd设置
void Kp3_change(bool en);					//速度环Kp设置
void Ki3_change(bool en);					//速度环Ki设置
void Kd3_change(bool en);					//速度环Kd设置
void Kp4_change(bool en);					//速度环Kp设置
void Ki4_change(bool en);					//速度环Ki设置
void Kd4_change(bool en);					//速度环Kd设置
void Kp_yaw_change(bool en);				//速度环Kp设置
void Ki_yaw_change(bool en);				//速度环Ki设置
void Kd_yaw_change(bool en);				//速度环Kd设置
void kalman_turn(bool en);					//卡尔曼滤波开关
/**********************************************************************/

/****************************** 菜单页面 ******************************/


#endif