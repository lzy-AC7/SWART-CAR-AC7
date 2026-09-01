#include "menu.h"

#define BACD_ID 					( 56 )					// 返回功能ID 

/* 页面列表 */
static MENU_UNION menu_union[] = 
{
	/* 页面名称                 类型        上一级  下一个同级  数据                            */
	{"ROOT"                 	,PAGE       ,0      ,0      ,1      ,       },      //0
		{"FAST"		   		    ,FUNCTION   ,0      ,2      ,NULL   ,       },      //1
		{"PLAN"        	    	,FUNCTION   ,0      ,3      ,NULL   ,       },      //2 
		{"WIFI"                 ,FUNCTION   ,0      ,4      ,NULL   ,       },      //3
		{"ENCODER"              ,PAGE       ,0      ,13     ,5      ,       },      //4
			{"encoder_1"        ,DIGITAL    ,4      ,6      ,NULL   ,INT16  },      //5
			{"encoder_2"        ,DIGITAL    ,4      ,7      ,NULL   ,INT16  },      //6
			{"encoder_3"        ,DIGITAL    ,4      ,8      ,NULL   ,INT16  },      //7
			{"encoder_4"        ,DIGITAL    ,4      ,9      ,NULL   ,INT16  },      //8
			{"wheel_speed_1"    ,DIGITAL    ,4      ,10     ,NULL   ,FLOAT  },      //9
			{"wheel_speed_2"    ,DIGITAL    ,4      ,11     ,NULL   ,FLOAT  },      //10
			{"wheel_speed_3"    ,DIGITAL    ,4      ,12     ,NULL   ,FLOAT  },      //11
			{"wheel_speed_4"    ,DIGITAL    ,4      ,BACD_ID,NULL   ,FLOAT  },      //12
		{"POSITION"             ,PAGE       ,0      ,22     ,14     ,       },      //13
			{"omega_car"        ,DIGITAL    ,13     ,15     ,NULL   ,FLOAT  },      //14
			{"gyro_z"           ,DIGITAL    ,13     ,16     ,NULL   ,FLOAT  },      //15
			{"yaw_angle"        ,DIGITAL    ,13     ,17     ,NULL   ,FLOAT  },      //16
			{"v_x_car"          ,DIGITAL    ,13     ,18     ,NULL   ,FLOAT  },      //17
			{"v_y_car"          ,DIGITAL    ,13     ,19     ,NULL   ,FLOAT  },      //18
			{"x_world"          ,DIGITAL    ,13     ,20     ,NULL   ,FLOAT  },      //19
			{"y_world"          ,DIGITAL    ,13     ,21     ,NULL   ,FLOAT  },      //20
			{"calibrate"        ,FUNCTION   ,13     ,BACD_ID,NULL   ,       },      //21
		{"PID_CONTROL"          ,PAGE       ,0      ,54     ,23     ,       },      //22
			{"speed_target"     ,FUNCTION   ,22     ,24     ,NULL   ,       },      //23
			{"pid_speed_1"      ,PAGE       ,22     ,30     ,25     ,       },      //24
				{"kp_1"         ,FUNCTION   ,24     ,26     ,NULL   ,       },      //25
				{"ki_1"         ,FUNCTION   ,24     ,27     ,NULL   ,       },      //26
				{"kd_1"         ,FUNCTION   ,24     ,28     ,NULL   ,       },      //27
				{"ol_1"         ,DIGITAL    ,24     ,29     ,NULL   ,FLOAT  },      //28
				{"il_1"         ,DIGITAL    ,24     ,BACD_ID,NULL   ,FLOAT  },      //29
			{"pid_speed_2"      ,PAGE       ,22     ,36     ,31     ,       },      //30
				{"kp_2"         ,FUNCTION   ,30     ,32     ,NULL   ,       },      //31
				{"ki_2"         ,FUNCTION   ,30     ,33     ,NULL   ,       },      //32
				{"kd_2"         ,FUNCTION   ,30     ,34     ,NULL   ,       },      //33
				{"ol_2"         ,DIGITAL    ,30     ,35     ,NULL   ,FLOAT  },      //34
				{"il_2"         ,DIGITAL    ,30     ,BACD_ID,NULL   ,FLOAT  },      //35
			{"pid_speed_3"      ,PAGE       ,22     ,42     ,37     ,       },      //36
				{"kp_3"         ,FUNCTION   ,36     ,38     ,NULL   ,       },      //37
				{"ki_3"         ,FUNCTION   ,36     ,39     ,NULL   ,       },      //38
				{"kd_3"         ,FUNCTION   ,36     ,40     ,NULL   ,       },      //39
				{"ol_3"         ,DIGITAL    ,36     ,41     ,NULL   ,FLOAT  },      //40
				{"il_3"         ,DIGITAL    ,36     ,BACD_ID,NULL   ,FLOAT  },      //41
			{"pid_speed_4"      ,PAGE       ,22     ,48     ,43     ,       },      //42
				{"kp_4"         ,FUNCTION   ,42     ,44     ,NULL   ,       },      //43
				{"ki_4"         ,FUNCTION   ,42     ,45     ,NULL   ,       },      //44
				{"kd_4"         ,FUNCTION   ,42     ,46     ,NULL   ,       },      //45
				{"ol_4"         ,DIGITAL    ,42     ,47     ,NULL   ,FLOAT  },      //46
				{"il_4"         ,DIGITAL    ,42     ,BACD_ID,NULL   ,FLOAT  },      //47
			{"pid_yaw"          ,PAGE       ,22     ,BACD_ID,49     ,       },      //48
				{"kp_yaw"       ,FUNCTION   ,48     ,50     ,NULL   ,       },      //49
				{"ki_yaw"       ,FUNCTION   ,48     ,51     ,NULL   ,       },      //50
				{"kd_yaw"       ,FUNCTION   ,48     ,52     ,NULL   ,       },      //51
				{"ol_yaw"       ,DIGITAL    ,48     ,53     ,NULL   ,FLOAT  },      //52
				{"il_yaw"       ,DIGITAL    ,48     ,BACD_ID,NULL   ,FLOAT  },      //53
		{"kalman_turn"          ,FUNCTION   ,0      ,BACD_ID,NULL   ,       },      //54
		{"BACK"                 ,FUNCTION   ,0      ,BACD_ID,NULL   ,       },      //55
    {"BACK"                     ,FUNCTION   ,0      ,0      ,NULL   ,       },      //56 (BACD_ID)
};

static uint8 menu_page_p = 0;		//页面指针
static uint8 menu_p = 0;			//光标
static int8 menu_p_move = -1,pre = 0;

/* 菜单初始化 */
void menu_init(void)
{
	menu_union[1].data.fp = speed_change;
	menu_union[2].data.fp = plan_change;
	menu_union[3].data.fp = wifi_init;

	/* encoder */
	menu_union[5].data.p_int16 = &encoder[0];
	menu_union[6].data.p_int16 = &encoder[1];
	menu_union[7].data.p_int16 = &encoder[2];
	menu_union[8].data.p_int16 = &encoder[3];
	menu_union[9].data.p_float = &wheel_speed[0];
	menu_union[10].data.p_float = &wheel_speed[1];
	menu_union[11].data.p_float = &wheel_speed[2];
	menu_union[12].data.p_float = &wheel_speed[3];

	/* imu */
	menu_union[14].data.p_float = &omega_car;
	menu_union[15].data.p_float = &gyro_z;
	menu_union[16].data.p_float = &yaw_angle;
	menu_union[17].data.p_float = &v_x_car;
	menu_union[18].data.p_float = &v_y_car;
	menu_union[19].data.p_float = &x_world;
	menu_union[20].data.p_float = &y_world;
	menu_union[21].data.fp = calibrate;

	/* pid */
	menu_union[23].data.fp = speed_target_change;
	menu_union[25].data.fp = Kp1_change;
	menu_union[26].data.fp = Ki1_change;
	menu_union[27].data.fp = Kd1_change;
	menu_union[28].data.p_float = &pid_speed[0].output_limit;
	menu_union[29].data.p_float = &pid_speed[0].i_limit;
	menu_union[31].data.fp = Kp2_change;
	menu_union[32].data.fp = Ki2_change;
	menu_union[33].data.fp = Kd2_change;
	menu_union[34].data.p_float = &pid_speed[1].output_limit;
	menu_union[35].data.p_float = &pid_speed[1].i_limit;
	menu_union[37].data.fp = Kp3_change;
	menu_union[38].data.fp = Ki3_change;
	menu_union[39].data.fp = Kd3_change;
	menu_union[40].data.p_float = &pid_speed[2].output_limit;
	menu_union[41].data.p_float = &pid_speed[2].i_limit;
	menu_union[43].data.fp = Kp4_change;
	menu_union[44].data.fp = Ki4_change;
	menu_union[45].data.fp = Kd4_change;
	menu_union[46].data.p_float = &pid_speed[3].output_limit;
	menu_union[47].data.p_float = &pid_speed[3].i_limit;
	menu_union[49].data.fp = Kp_yaw_change;
	menu_union[50].data.fp = Ki_yaw_change;
	menu_union[51].data.fp = Kd_yaw_change;
	menu_union[52].data.p_float = &pid_yaw.output_limit;
	menu_union[53].data.p_float = &pid_yaw.i_limit;
	menu_union[54].data.fp = kalman_turn;
	menu_union[55].data.fp = menu_back;

	/* back */
	menu_union[BACD_ID].data.fp = menu_back;
	ips200_init(IPS200_TYPE_SPI);
}

/* 菜单运行 */
void menu_runing(void)
{
	ips200_clear();
	ips200_full(RGB565_WHITE);
	position_calibrate(art_x,art_y,yaw_init);
	x_start = x_target = art_x,y_start = y_target = art_y;
	//ips200_show_rgb565_image(0, 0, (const uint16 *)gImage_holo, 320, 240, 320, 240, 0);
	while(!gyro_calibrated);
	ips200_clear();
	// wifi();
	SYS_READY = 1;
	printf("READY!!\n");
	while(1)
	{
		if(key_get_state(KEY_START) == KEY_SHORT_PRESS)
			key_clear_state(KEY_START),system_delay_ms(100),checkpoint_set(0);
		if(over && checkpoint != 0 && !car_runing_path_flag && !car_2p_runing_flag)
			    checkpoint_set(checkpoint);
		fsm();
		car_2p();
		car_runing_path();
		checkpoint1();
		checkpoint2();
		if(fsm_flag)continue;
		/* 指针移动 */
		if(key_get_state(KEY_UP) == KEY_SHORT_PRESS)menu_p_move = 1,key_clear_state(KEY_UP);
		if(key_get_state(KEY_DOWN) == KEY_SHORT_PRESS)menu_p_move = 2,key_clear_state(KEY_DOWN);
		/* 显示菜单 */
		uint8 p = menu_union[menu_page_p].data.son;
		for(uint8 i = 0;i < menu_p/MENU_PAGE_MOST_ROW*MENU_PAGE_MOST_ROW;i++)
			p = menu_union[p].next;

		for(uint8 i = 0;p;p = menu_union[p].next,i++)
		{
			//展示菜单项
			ips200_show_string(0,MENU_ROW_HEIGHT*i,menu_union[p].name);
			if(menu_union[p].kind == DIGITAL)
				switch (menu_union[p].data_kind)
				{
					case BOOL	:	ips200_show_uint (MENU_ROW_WIDTH/2,MENU_ROW_HEIGHT*i,*menu_union[p].data.p_bool  ,4  );break;
					case UINT8	:	ips200_show_uint (MENU_ROW_WIDTH/2,MENU_ROW_HEIGHT*i,*menu_union[p].data.p_uint8 ,4  );break;
					case UINT16	:	ips200_show_uint (MENU_ROW_WIDTH/2,MENU_ROW_HEIGHT*i,*menu_union[p].data.p_uint16,4  );break;
					case UINT32	:	ips200_show_uint (MENU_ROW_WIDTH/2,MENU_ROW_HEIGHT*i,*menu_union[p].data.p_uint32,4  );break;
					case UINT64	:	ips200_show_uint (MENU_ROW_WIDTH/2,MENU_ROW_HEIGHT*i,*menu_union[p].data.p_uint64,4  );break;
					case INT8	:	ips200_show_int  (MENU_ROW_WIDTH/2,MENU_ROW_HEIGHT*i,*menu_union[p].data.p_int8  ,4  );break;
					case INT16	:	ips200_show_int  (MENU_ROW_WIDTH/2,MENU_ROW_HEIGHT*i,*menu_union[p].data.p_int16 ,4  );break;
					case INT32	:	ips200_show_int  (MENU_ROW_WIDTH/2,MENU_ROW_HEIGHT*i,*menu_union[p].data.p_int32 ,4  );break;
					case INT64	:	ips200_show_int  (MENU_ROW_WIDTH/2,MENU_ROW_HEIGHT*i,*menu_union[p].data.p_int64 ,4  );break;
					case FLOAT	:	ips200_show_float(MENU_ROW_WIDTH/2,MENU_ROW_HEIGHT*i,*menu_union[p].data.p_float ,4,4);break;
					case DOUBLE	:	ips200_show_float(MENU_ROW_WIDTH/2,MENU_ROW_HEIGHT*i,*menu_union[p].data.p_double,4,4);break;
					default:	break;
				}
			
			/* 指针移动 */
			if(menu_union[p].kind == FUNCTION || menu_union[p].kind == PAGE)
			{
				if(menu_p_move == -1)ips200_draw_line(0,(menu_p+1)*MENU_ROW_HEIGHT-1,MENU_ROW_WIDTH-1,(menu_p+1)*MENU_ROW_HEIGHT-1,RGB565_BLACK),menu_p_move = 0,menu_p = i,ips200_draw_line(0,(menu_p+1)*MENU_ROW_HEIGHT-1,MENU_ROW_WIDTH-1,(menu_p+1)*MENU_ROW_HEIGHT-1,RGB565_RED);
				else if(menu_p_move == 1 && i == menu_p)ips200_draw_line(0,(menu_p+1)*MENU_ROW_HEIGHT-1,MENU_ROW_WIDTH-1,(menu_p+1)*MENU_ROW_HEIGHT-1,RGB565_BLACK),menu_p = pre,menu_p_move = 0,ips200_draw_line(0,(menu_p+1)*MENU_ROW_HEIGHT-1,MENU_ROW_WIDTH-1,(menu_p+1)*MENU_ROW_HEIGHT-1,RGB565_RED);
				else if(menu_p_move == 2 && i == menu_p)menu_p_move = 3;
				else if(menu_p_move == 3)ips200_draw_line(0,(menu_p+1)*MENU_ROW_HEIGHT-1,MENU_ROW_WIDTH-1,(menu_p+1)*MENU_ROW_HEIGHT-1,RGB565_BLACK),menu_p = i,menu_p_move = 0,ips200_draw_line(0,(menu_p+1)*MENU_ROW_HEIGHT-1,MENU_ROW_WIDTH-1,(menu_p+1)*MENU_ROW_HEIGHT-1,RGB565_RED);
				pre = i;
			}

			//进入下一级菜单
			if(menu_p == i && menu_union[p].kind == PAGE)
			{
				if(key_get_state(KEY_PRESS) == KEY_SHORT_PRESS)
				{
					key_clear_state(KEY_PRESS);
					ips200_clear();
					menu_p_move = -1,pre = 0,menu_p = 0,\
					menu_page_p = p;
					break;
				}
			}

			//执行函数
			if(menu_union[p].kind == FUNCTION && menu_union[p].data.fp != NULL)
				menu_union[p].data.fp(menu_p == i);
		}
		debug();
	}
}

/* 菜单返回函数 */
void menu_back(bool en)
{
	if(!en)return;
	if(key_get_state(KEY_PRESS) == KEY_SHORT_PRESS)
		key_clear_state(KEY_PRESS),
		ips200_clear(),
		menu_p_move = -1,
		pre = 0,
		menu_p = 0,
		menu_page_p = menu_union[menu_page_p].fa;
}

void speed_change(bool en)
{
	ips200_show_int(MENU_ROW_WIDTH/2,0*MENU_ROW_HEIGHT,fast_flag,1);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS)key_clear_state(KEY_LEFT),fast_flag^=1;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),fast_flag^=1;
}

void plan_change(bool en)
{
	ips200_show_int(MENU_ROW_WIDTH/2,1*MENU_ROW_HEIGHT,PLAN,1);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS)key_clear_state(KEY_LEFT),PLAN^=1;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),PLAN^=1;
}

void OPTIMAL_SET(bool en)
{
	if(!OPTIMAL){ips200_show_char(MENU_ROW_WIDTH/2,1*MENU_ROW_HEIGHT,'*');}
	else ips200_show_char(MENU_ROW_WIDTH/2,1*MENU_ROW_HEIGHT,'X');
	if(!en)return;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),OPTIMAL^=1;
}

void wifi()
{
	uint8 t = 0;
	while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
	{
		printf("\r\n connect wifi failed. \r\n");
		system_delay_ms(100);                                                   // 初始化失败等待 100ms
		if(++t>3)return;
	}
	
	printf("\r\n module version:%s", wifi_spi_version);      					// 模块固件版本
	printf("\r\n module mac    :%s", wifi_spi_mac_addr);     					// 模块MAC信息
	printf("\r\n module ip     :%s", wifi_spi_ip_addr_port); 					// 模块IP地址

	// zf_device_wifi_spi.h 文件内的宏定义可以修改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接
	if(0 == WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
	{
		uint8 t = 0;
		while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
			"UDP",                                                              // 指定使用TCP方式通讯
			WIFI_SPI_TARGET_IP,                                                 // 指定远端的IP地址，填写上位机的IP地址
			WIFI_SPI_TARGET_PORT,                                               // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
			WIFI_SPI_LOCAL_PORT))                                               // 指定本机的端口号
		{
			// 如果一直连接失败，考虑一下是不是没有接模块复位
			printf("\r\n Connect UDP Servers error, try again.");
			system_delay_ms(100);                                               // 连接失败等待 100ms
			if(++t>3)return;
		}
	}
	wifi_spi_send_string("Hello WIFI !\r\n");
	wifi_en = true;
}

/* WIFI初始化函数 */
void wifi_init(bool en)
{
	if(wifi_en){ips200_show_char(MENU_ROW_WIDTH/2,2*MENU_ROW_HEIGHT,'*');}
	else ips200_show_char(MENU_ROW_WIDTH/2,2*MENU_ROW_HEIGHT,'X');
	if(!en)return;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)
	{
		key_clear_state(KEY_RIGHT);
		ips200_show_char(MENU_ROW_WIDTH/2,2*MENU_ROW_HEIGHT,'-');
		wifi();
	}
	
}

void calibrate(bool en)
{
	if(gyro_calibrated){ips200_show_char(MENU_ROW_WIDTH/2,7*MENU_ROW_HEIGHT,'*');}
	else ips200_show_char(MENU_ROW_WIDTH/2,7*MENU_ROW_HEIGHT,'X');
	if(!en)return;
	if(key_get_state(KEY_PRESS) == KEY_SHORT_PRESS)
	{
		key_clear_state(KEY_PRESS);
		/* reset gyro calibration data */
		gyro_calibrated = false;
		gyro_epoch = 0;
		// gyro_bias = 0.0f;
		// gyro_sum = 0.0f;
		// gyro_max = -FLT_MAX;
		// gyro_min =  FLT_MAX;
		// gyro_deadband = 0.0f;
		gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0;
		gyro_deadband[0] = gyro_deadband[1] = gyro_deadband[2] = 0;
		gyro_max[0] = gyro_max[1] = gyro_max[2] = 0;
		gyro_min[0] = gyro_min[1] = gyro_min[2] = 0;
		gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0;
		/* reset accel calibration data */

		/* reset navigation state */
		position_calibrate(150,-1200,90);
	}
}

void speed_target_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,0*MENU_ROW_HEIGHT,wheel_speed_target[0],4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS)
	{
		key_clear_state(KEY_LEFT);
		wheel_speed_target[0] -= 10;
		wheel_speed_target[1] -= 10;
		wheel_speed_target[2] -= 10;
		wheel_speed_target[3] -= 10;
	}
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS)
	{
		key_clear_state(KEY_LEFT);
		wheel_speed_target[0] -= 10;
		wheel_speed_target[1] -= 10;
		wheel_speed_target[2] -= 10;
		wheel_speed_target[3] -= 10;
	}
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)
	{
		key_clear_state(KEY_RIGHT);
		wheel_speed_target[0] += 10;
		wheel_speed_target[1] += 10;
		wheel_speed_target[2] += 10;
		wheel_speed_target[3] += 10;
	}
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)
	{
		key_clear_state(KEY_RIGHT);
		wheel_speed_target[0] += 10;
		wheel_speed_target[1] += 10;
		wheel_speed_target[2] += 10;
		wheel_speed_target[3] += 10;
	}
}

void Kp1_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,0*MENU_ROW_HEIGHT,pid_speed[0].Kp,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_speed[0].Kp > 0.0f)key_clear_state(KEY_LEFT),pid_speed[0].Kp -= 0.1f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_speed[0].Kp > 0.0f)key_clear_state(KEY_LEFT),pid_speed[0].Kp -= 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_speed[0].Kp += 0.05f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_speed[0].Kp += 0.05f;
}

void Ki1_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,1*MENU_ROW_HEIGHT,pid_speed[0].Ki,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_speed[0].Ki > 0.0f)key_clear_state(KEY_LEFT),pid_speed[0].Ki -= 0.01f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_speed[0].Ki > 0.0f)key_clear_state(KEY_LEFT),pid_speed[0].Ki -= 0.01f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_speed[0].Ki += 0.005f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_speed[0].Ki += 0.005f;
}

void Kd1_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,2*MENU_ROW_HEIGHT,pid_speed[0].Kd,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_speed[0].Kd > 0.0f)key_clear_state(KEY_LEFT),pid_speed[0].Kd -= 0.1f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_speed[0].Kd > 0.0f)key_clear_state(KEY_LEFT),pid_speed[0].Kd -= 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_speed[0].Kd += 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_speed[0].Kd += 0.1f;
}

void Kp2_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,0*MENU_ROW_HEIGHT,pid_speed[1].Kp,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_speed[1].Kp > 0.0f)key_clear_state(KEY_LEFT),pid_speed[1].Kp -= 0.1f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_speed[1].Kp > 0.0f)key_clear_state(KEY_LEFT),pid_speed[1].Kp -= 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_speed[1].Kp += 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_speed[1].Kp += 0.1f;
}

void Ki2_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,1*MENU_ROW_HEIGHT,pid_speed[1].Ki,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_speed[1].Ki > 0.0f)key_clear_state(KEY_LEFT),pid_speed[1].Ki -= 0.01f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_speed[1].Ki > 0.0f)key_clear_state(KEY_LEFT),pid_speed[1].Ki -= 0.01f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_speed[1].Ki += 0.01f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_speed[1].Ki += 0.01f;
}

void Kd2_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,2*MENU_ROW_HEIGHT,pid_speed[1].Kd,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_speed[1].Kd > 0.0f)key_clear_state(KEY_LEFT),pid_speed[1].Kd -= 0.1f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_speed[1].Kd > 0.0f)key_clear_state(KEY_LEFT),pid_speed[1].Kd -= 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_speed[1].Kd += 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_speed[1].Kd += 0.1f;
}

void Kp3_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,0*MENU_ROW_HEIGHT,pid_speed[2].Kp,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_speed[2].Kp > 0.0f)key_clear_state(KEY_LEFT),pid_speed[2].Kp -= 0.1f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_speed[2].Kp > 0.0f)key_clear_state(KEY_LEFT),pid_speed[2].Kp -= 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_speed[2].Kp += 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_speed[2].Kp += 0.1f;
}

void Ki3_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,1*MENU_ROW_HEIGHT,pid_speed[2].Ki,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_speed[2].Ki > 0.0f)key_clear_state(KEY_LEFT),pid_speed[2].Ki -= 0.01f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_speed[2].Ki > 0.0f)key_clear_state(KEY_LEFT),pid_speed[2].Ki -= 0.01f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_speed[2].Ki += 0.01f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_speed[2].Ki += 0.01f;
}

void Kd3_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,2*MENU_ROW_HEIGHT,pid_speed[2].Kd,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_speed[2].Kd > 0.0f)key_clear_state(KEY_LEFT),pid_speed[2].Kd -= 0.1f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_speed[2].Kd > 0.0f)key_clear_state(KEY_LEFT),pid_speed[2].Kd -= 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_speed[2].Kd += 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_speed[2].Kd += 0.1f;
}

void Kp4_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,0*MENU_ROW_HEIGHT,pid_speed[3].Kp,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_speed[3].Kp > 0.0f)key_clear_state(KEY_LEFT),pid_speed[3].Kp -= 0.1f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_speed[3].Kp > 0.0f)key_clear_state(KEY_LEFT),pid_speed[3].Kp -= 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_speed[3].Kp += 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_speed[3].Kp += 0.1f;
}

void Ki4_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,1*MENU_ROW_HEIGHT,pid_speed[3].Ki,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_speed[3].Ki > 0.0f)key_clear_state(KEY_LEFT),pid_speed[3].Ki -= 0.01f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_speed[3].Ki > 0.0f)key_clear_state(KEY_LEFT),pid_speed[3].Ki -= 0.01f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_speed[3].Ki += 0.01f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_speed[3].Ki += 0.01f;
}

void Kd4_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,2*MENU_ROW_HEIGHT,pid_speed[3].Kd,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_speed[3].Kd > 0.0f)key_clear_state(KEY_LEFT),pid_speed[3].Kd -= 0.1f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_speed[3].Kd > 0.0f)key_clear_state(KEY_LEFT),pid_speed[3].Kd -= 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_speed[3].Kd += 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_speed[3].Kd += 0.1f;
}

void Kp_yaw_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,0*MENU_ROW_HEIGHT,pid_yaw.Kp,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_yaw.Kp > 0.0f)key_clear_state(KEY_LEFT),pid_yaw.Kp -= 0.1f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_yaw.Kp > 0.0f)key_clear_state(KEY_LEFT),pid_yaw.Kp -= 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_yaw.Kp += 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_yaw.Kp += 0.1f;
}

void Ki_yaw_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,1*MENU_ROW_HEIGHT,pid_yaw.Ki,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_yaw.Ki > 0.0f)key_clear_state(KEY_LEFT),pid_yaw.Ki -= 0.01f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_yaw.Ki > 0.0f)key_clear_state(KEY_LEFT),pid_yaw.Ki -= 0.01f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_yaw.Ki += 0.01f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_yaw.Ki += 0.01f;
}

void Kd_yaw_change(bool en)
{
	ips200_show_float(MENU_ROW_WIDTH/2,2*MENU_ROW_HEIGHT,pid_yaw.Kd,4,4);
	if(!en)return;
	if(key_get_state(KEY_LEFT) == KEY_SHORT_PRESS && pid_yaw.Kd > 0.0f)key_clear_state(KEY_LEFT),pid_yaw.Kd -= 0.1f;
	if(key_get_state(KEY_LEFT) == KEY_LONG_PRESS && pid_yaw.Kd > 0.0f)key_clear_state(KEY_LEFT),pid_yaw.Kd -= 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)key_clear_state(KEY_RIGHT),pid_yaw.Kd += 0.1f;
	if(key_get_state(KEY_RIGHT) == KEY_LONG_PRESS)key_clear_state(KEY_RIGHT),pid_yaw.Kd += 0.1f;
}

void kalman_turn(bool en)
{
	if(kalman_filter_enable)ips200_show_char(MENU_ROW_WIDTH/2,6*MENU_ROW_HEIGHT,'*');
	else ips200_show_char(MENU_ROW_WIDTH/2,6*MENU_ROW_HEIGHT,'X');
	if(!en)return;
	if(key_get_state(KEY_RIGHT) == KEY_SHORT_PRESS)
	{
		key_clear_state(KEY_RIGHT);
		kalman_filter_enable = !kalman_filter_enable;
	}
}
