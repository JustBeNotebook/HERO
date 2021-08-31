#ifndef RM__MY__JUDGE
#define RM__MY__JUDGE

#include "sys.h"

#define JUDGE_FRAME_HEADER 0xa5

#define Client_mid_position_x 960
#define Client_mid_position_y 540

#define AIM_X 950

#define AIM_Y 540

//***********裁判系统的分类信息***************/
enum
{ 
	ID_game_state       						= 0x0001,//比赛状态数据，1Hz
	ID_game_result 	   							= 0x0002,//比赛结果数据，比赛结束发送
	ID_game_robot_HP       					= 0x0003,//比赛机器人血量数据，1Hz发送
	ID_dart_status									= 0x0004,//飞镖发射状态，飞镖发射时发送
	ID_ICRA_buff_debuff_zone_status = 0x0005,//人工智能挑战赛加成与惩罚区状态，1Hz
	ID_event_data  									= 0x0101,//场地事件数据，事件 ——改变后—— 发送
	ID_supply_projectile_action   	= 0x0102,//场地补给站动作标识数据，动作 ——改变后—— 发送
	ID_supply_projectile_booking    = 0x0103,//请求补给站补弹数据 ——参赛队发送——（RM 对抗赛尚未开）10Hz
	ID_referee_warning					 		= 0x0104,//裁判警告数据，——警告后——发送
	ID_dart_remaining_time					= 0x0105,//飞镖发射口倒计时，1Hz
	ID_game_robot_state    					= 0x0201,//机器人状态数据，10Hz
	ID_power_heat_data    					= 0x0202,//实时功率热量数据，50Hz
	ID_game_robot_pos        				= 0x0203,//机器人位置数据，10Hz
	ID_buff_musk										= 0x0204,//机器人增益数据，1Hz
	ID_aerial_robot_energy					= 0x0205,//空中机器人能量状态数据，10Hz，只有——空中机器人主控——发送
	ID_robot_hurt										= 0x0206,//伤害状态数据，伤害发生后发送
	ID_shoot_data										= 0x0207,//实时射击数据，子弹发射后发送
	ID_bullet_remaining							= 0x0208,//弹丸剩余发送数，仅——空中机器人，哨兵机器人——以及ICRA机器人发送，1Hz
	ID_rfid_status									= 0x0209,//机器人RFID状态，1Hz
	
	ID_dart_client_directive        = 0x020A,//飞镖机器人客户端指令书, 10Hz
	
	ID_robot_interactive_header_data			= 0x0301,//机器人交互数据，——发送方触发——发送 10Hz
	ID_controller_interactive_header_data = 0x0302,//自定义控制器交互数据接口，通过——客户端触发——发送 30Hz
	ID_map_interactive_header_data        = 0x0303,//客户端小地图交互数据，——触发发送——
	ID_keyboard_information               = 0x0304//键盘、鼠标信息，通过——图传串口——发送
};
//命令码枚举CMD_ID
//**************裁判系统各个信息的长度（单位/字节）**************************/
enum judge_data_length_t {
	/* Std */
	LEN_FRAME_HEAD 	                 = 5,	// 帧头长度
	LEN_CMD_ID 		                   = 2,	// 命令码长度
	LEN_FRAME_TAIL 	                 = 2,	// 帧尾CRC16
	/* Ext */
	// 0x000x
	LEN_GAME_STATUS 				         = 11,
	LEN_GAME_RESULT 				         = 1,
	LEN_GAME_ROBOT_HP 			         = 28,
	LEN_DART_STATUS					         = 3,
	LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS = 11,//0x0005
	// 0x010x
	LEN_EVENT_DATA					         = 4,
	LEN_SUPPLY_PROJECTILE_ACTION	   = 4,//！！！！！！！！！！！！！！！！！
	LEN_SUPPLY_PROJECTILE_BOOKING	   = 3,//对抗赛未开启
	LEN_REFEREE_WARNING				       = 2,
	LEN_DART_REMAINING_TIME		     	 = 1,//0x0105
	// 0x020x
	LEN_GAME_ROBOT_STATUS			       = 27,//15!!!!!!!!!!!!!!!!!!!!!!!!!!!
	LEN_POWER_HEAT_DATA 			       = 16,//！！！！！！！！！！
	LEN_GAME_ROBOT_POS				       = 16,
	LEN_BUFF_MASK		 				         = 1,
	LEN_AERIAL_ROBOT_ENERGY 	     	 = 1,//！！！！！
	LEN_ROBOT_HURT				         	 = 1,
	LEN_SHOOT_DATA					         = 7,//！！！！
	LEN_BULLET_REMAINING	 		       = 6,//！！！！
	LEN_RFID_STATUS					         = 4,
	LEN_DART_CLIENT_DIRECTIVE        = 12,//0x020A
	// 0x030x
	//LEN_robot_interactive_header_data      = n,
	//LEN_controller_interactive_header_data = n,
	LEN_MAP_INTERACTIVE_HEADERDATA           = 15,
	LEN_KEYBOARD_INFORMATION                 = 12,//0x0304
};//表2-4
/*******************裁判系统信息内容排序**************************/


/* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;						
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
}FrameHeader;


typedef __packed struct
{
	uint8_t  sof;
	uint16_t data_length;
	uint8_t  seq;
	uint8_t  crc8;
} std_frame_header_t;//LEN_FRAME_HEAD

/* ID: 0x0001  Byte:  11    比赛状态数据 */

typedef __packed struct 
{ 
	uint8_t game_type : 4;					//比赛模式
	uint8_t game_progress : 4;			//当前比赛阶段
	uint16_t stage_remain_time;			//当前阶段剩余时间 单位s
	uint64_t SyncTimeStamp;
} ext_game_state_t; 

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 

/* ID: 0x0003  Byte:  32    比赛机器人血量数据 */
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP;   
	uint16_t red_2_robot_HP;  
	uint16_t red_3_robot_HP;  
	uint16_t red_4_robot_HP;  
	uint16_t red_5_robot_HP;  
	uint16_t red_7_robot_HP;  
	uint16_t red_outpost_HP; 
  uint16_t red_base_HP; 
	
	uint16_t blue_1_robot_HP;   
	uint16_t blue_2_robot_HP;   
	uint16_t blue_3_robot_HP;   
	uint16_t blue_4_robot_HP;   
	uint16_t blue_5_robot_HP;     
	uint16_t blue_7_robot_HP; 

	uint16_t blue_outpost_HP; 
  uint16_t blue_base_HP;   
}  ext_game_robot_HP_t; 

/* ID: 0x0004  Byte:  3    飞镖发射状态 */
typedef __packed struct 
{  
	uint8_t dart_belong; 							//发射飞镖的队伍
	uint16_t stage_remaining_time; 		//发射时剩余的比赛时间
} ext_dart_status_t;

/* ID: 0x0005  Byte:  11   人工智能挑战赛 buff and debuff */
typedef __packed struct
{ 
	uint8_t F1_zone_status:1;  							//地区是否开启的标志
	uint8_t F1_zone_buff_debuff_status:3;   //红方回血
	
	uint8_t F2_zone_status:1;  
	uint8_t F2_zone_buff_debuff_status:3;   //红方弹药
	
	uint8_t F3_zone_status:1;  
	uint8_t F3_zone_buff_debuff_status:3;		//蓝方回血  
	
	uint8_t F4_zone_status:1;  
	uint8_t F4_zone_buff_debuff_status:3;  	//蓝方弹药
	
	uint8_t F5_zone_status:1;  
	uint8_t F5_zone_buff_debuff_status:3;  	//禁止射击区
	
	uint8_t F6_zone_status:1;  
	uint8_t F6_zone_buff_debuff_status:3;  	//禁止移动区
	
	uint16_t red1_bullet_left;							//红蓝方剩余弹量
	uint16_t red2_bullet_left;
	uint16_t blue1_bullet_left;
	uint16_t blue2_bullet_left;
	
} ext_ICRA_buff_debuff_zone_status_t; 

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct 
{ 
	uint32_t event_type;										//0~4bit在用，5~31保留
	
} ext_event_data_t; 


/* ID: 0x0102  Byte:  4    场地补给站动作标识数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;						//补给站口ID
	uint8_t supply_robot_id;								//当前补弹机器人ID
	uint8_t supply_projectile_step;					//补给站出弹口状态
	
	uint8_t supply_projectile_num;					//补弹数目
} ext_supply_projectile_action_t; 

/* ID: 0x0104  Byte: 2   裁判系统警告信息 */
typedef __packed struct 
{ 
  uint8_t level; 													//警告等级
	uint8_t foul_robot_id;									//犯规机器人ID
}  ext_referee_warning_t;  

/* ID: 0x0105  Byte:1  飞镖发射口倒计时 */
typedef __packed struct 
{ 
	uint8_t dart_remaining_time; 						//15s倒计时
}  ext_dart_remaining_time_t;  


/* ID: 0X0201  Byte: 27    机器人状态数据 */
//0代表是17mm枪管，1代表42mm枪管
typedef __packed struct 			
{ 
	uint8_t robot_id;   													//机器人ID，可用来校验发送
	uint8_t robot_level;  												//机器人等级 1一级，2二级，3三级
	uint16_t remain_HP; 													//机器人剩余血量
	uint16_t max_HP; 															//机器人上限血量
	uint16_t shooter_heat0_cooling_rate;  				//机器人 17mm 1子弹热量冷却速度 单位 /s
	uint16_t shooter_heat0_cooling_limit;   			// 机器人 17mm 1子弹热量上限
	uint16_t shooter_heat0_speed_limit;						//机器人17mm 1枪口上限速度
	uint16_t shooter_heat1_cooling_rate;   				//机器人17mm 2枪口每秒冷却值
	uint16_t shooter_heat1_cooling_limit;  				//机器人17mm 2枪口热量上限
	uint16_t shooter_heat1_speed_limit;						//机器人17mm 2枪口上限速度
	uint16_t shooter_42mm_cooling_rate;   				//机器人42mm 枪口每秒冷却值
	uint16_t shooter_42mm_cooling_limit;  				//机器人42mm 枪口热量上限
	uint16_t shooter_42mm_speed_limit;						//机器人42mm 枪口上限速度
	uint16_t max_chassis_power; 										//机器人底盘功率上限
	uint8_t mains_power_gimbal_output : 1;  			//主控电源云台口输出状态
	uint8_t mains_power_chassis_output : 1;  			//主控电源底盘口输出状态
	uint8_t mains_power_shooter_output : 1; 			//主控电源枪管口输出状态
} ext_game_robot_state_t;   

/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   											//底盘输出电压 mV
	uint16_t chassis_current;    									//底盘输出电流 mA
	float chassis_power;   												//底盘输出功率 
	uint16_t chassis_power_buffer;								//底盘缓冲功率剩余
	uint16_t shooter_heat0;												//17mm枪口1目前热量
	uint16_t shooter_heat1;  											//17mm枪口2目前热量
	uint16_t shooter_heat2_42mm; 								//42mm枪口目前热量
} ext_power_heat_data_t; 

/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct 
{   
	float x;   																		//机器人位置x
	float y;   																		//机器人位置y
	float z;   																		//机器人位置z
	float yaw; 																		//机器人偏航角
} ext_game_robot_pos_t; 

/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct 
{ 
	uint8_t power_rune_buff;											// 
} ext_buff_musk_t; 

/* ID: 0x0205  Byte:  1    空中机器人能量状态数据 */
typedef __packed struct 
{ 
	uint8_t attack_time; 													//可攻击时间
} aerial_robot_energy_t; 

/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 												//受伤装甲板ID
	uint8_t hurt_type : 4; 												//血量扣除类型
} ext_robot_hurt_t; 

/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef __packed struct 
{ 
	uint8_t bullet_type;   												//子弹类型
	uint8_t shooter_id;   												//发射机构id
	uint8_t bullet_freq;   												//子弹射频 HZ
	float bullet_speed;  													//子弹射速 m/s
} ext_shoot_data_t; 


/* ID: 0x0208  Byte:  6    子弹剩余数量 */
typedef __packed struct 
{ 
	uint16_t bullet_remaining_num_17mm;   							//子弹剩余发射数
	uint16_t bullet_remaining_num_42mm;   							//子弹剩余发射数
	uint16_t coin_remain;   														//剩余金币
}  ext_bullet_remaining_t; 

/* ID: 0x0209  Byte:  4    FRID状态 */
typedef __packed struct 
{ 
	uint32_t rfid_status ;												//RFID卡状态
}  ext_rfid_status_t; 


/*飞镖机器人客户端指令数据*/
/* ID: 0x020a  Byte:  4    FRID状态 */
typedef __packed struct {  
	uint8_t dart_launch_opening_status;  
	uint8_t dart_attack_target;  
	uint16_t target_change_time; 
	uint16_t operate_launch_cmd_time; 
} ext_dart_client_cmd_t; 
	
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 






	/*******************************************************************************/
/*
	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	9，雷达（红）
	101，英雄(蓝)；
	102，工程(蓝)；
	103/104/105，步兵(蓝)；
	106，空中(蓝)；
	107，哨兵(蓝)；
	109，雷达（蓝）

	客户端 ID： 
	0x0101 为英雄操作手客户端(红) ；
	0x0102 为工程操作手客户端( 红 )；
	0x0103/0x0104/0x0105 为步兵操作手客户端(红)；
	0x0106 为空中操作手客户端((红)； 

	0x0165，英雄操作手客户端(蓝)；
	0x0166，工程操作手客户端(蓝)；
	0x0167/0x0168/0x0169，步兵操作手客户端(蓝)；
	0x016A，空中操作手客户端(蓝)。 
*/
enum judge_robot_ID{
	hero_red       = 1,
	engineer_red   = 2,
	infantry3_red  = 3,
	infantry4_red  = 4,
	infantry5_red  = 5,
	plane_red      = 6,
	
	hero_blue      = 101,
	engineer_blue  = 102,
	infantry3_blue = 103,
	infantry4_blue = 104,
	infantry5_blue = 105,
	plane_blue     = 106,
};
typedef __packed struct{
	uint16_t teammate_hero;
	uint16_t teammate_engineer;
	uint16_t teammate_infantry3;
	uint16_t teammate_infantry4;
	uint16_t teammate_infantry5;
	uint16_t teammate_plane;
	uint16_t teammate_sentry;
	
	uint16_t client_hero;
	uint16_t client_engineer;
	uint16_t client_infantry3;
	uint16_t client_infantry4;
	uint16_t client_infantry5;
	uint16_t client_plane;
} ext_interact_id_t;

typedef __packed struct JUDGE_MODULE_DATA
{
	FrameHeader all_head;							//帧头
	
	//////////////////////// 内容
	ext_game_state_t game_state;							//游戏状态
	ext_game_robot_HP_t robot_HP;							//机器人血量
	ext_dart_status_t dart_state;							//飞镖状态
	
	ext_ICRA_buff_debuff_zone_status_t zone;	//地图状态
	ext_event_data_t event;										//事件状态
	
	ext_supply_projectile_action_t supply_status;	//补给站状态
	ext_referee_warning_t offline_warning;				//犯规警告
	ext_dart_remaining_time_t dart_cnt;						//飞镖发射倒计时
	
	
	ext_game_robot_state_t robot_status;						//机器人状态
	ext_power_heat_data_t out_and_heat; 					//功率热量状态
	ext_game_robot_pos_t  robot_pos;							//机器人位置
	ext_buff_musk_t 			buff;										//机器人buff状态
	ext_robot_hurt_t     robot_hurt;							//机器人受伤警告
	ext_shoot_data_t			shoot_data;							//实时射击数据
	ext_bullet_remaining_t bullet_remain;					//子弹余量
	
	ext_rfid_status_t		rfid_status;							//rfid卡状态
	
	ext_dart_client_cmd_t           dart_client;        // 0x020A
	
	ext_interact_id_t								ids;								//与本机交互的机器人id
	uint16_t                        self_client;        //本机客户端
	
	u8 state_flag;
	u8 out_heat_flag;
	u8 buff_flag;
	u8 hurt_flag;
	u8 shoot_flag;
	u8 bullet_flag;
	u8 rfid_flag;
	
	uint8_t	 		data_valid;	// 数据有效性
	uint8_t			err_cnt;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;	
	
} JUDGE_MODULE_DATA;
/* 
	学生机器人间通信 cmd_id 0x0301，内容 data_ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：数据上下行合计带宽不超过 5000 Byte。 上下行发送频率分别不超过30Hz。
 * +------+------+-------------+------------------------------------+
 * | byte | size |    breif    |            note                    |
 * |offset|      |             |                                    |
 * +------+------+-------------+------------------------------------+
 * |  0   |  2   | 	 data_ID   | 0x0200~0x02FF,可以在这些 ID 段选取 |
 * |      |      |             | 具体ID含义由参赛队自定义           |
 * +------|------|-------------|------------------------------------|
 * |  2   |  2   | 	sender_ID  | 需要校验发送者的 ID 正确性					|
 * +------|------|-------------|------------------------------------|
 * |  4   |  2   | receiver_ID | 需要校验接收者的 ID 正确性					|
 * |      |      |             | 例如不能发送到敌对机器人的ID				| 
 * +------|------|-------------|------------------------------------|
 * |  6   |  n   | 		Data     | n 需要小于 113 										|
 * +------+------+-------------+------------------------------------+
*/
/******************************客户端交互数据**************************************/
#define INTERACT_DATA_LEN	113
typedef __packed struct //数据段内容格式
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_client_data_header_t; 
enum
{
	//0x200-0x02ff 	队伍自定义命令 格式  INTERACT_ID_XXXX
	INTERACT_ID_delete_graphic 			= 0x0100,	/*客户端删除图形*/
	INTERACT_ID_draw_one_graphic 		= 0x0101,	/*客户端绘制一个图形*/
	INTERACT_ID_draw_two_graphic 		= 0x0102,	/*客户端绘制2个图形*/
	INTERACT_ID_draw_five_graphic 	= 0x0103,	/*客户端绘制5个图形*/
	INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*客户端绘制7个图形*/
	INTERACT_ID_draw_char_graphic 	= 0x0110,	/*客户端绘制字符图形*/
	INTERACT_ID_bigbome_num					= 0x02ff
};
typedef __packed struct 
{ 
	uint8_t data[INTERACT_DATA_LEN]; //数据段,n需要小于113
} robot_interactive_data_t;
//单位（字节）
enum
{
	LEN_INTERACT_delete_graphic     = 8,  //删除图层 2(数据内容ID)+2(发送者ID)+2（接收者ID）+2（数据内容）  
	LEN_INTERACT_draw_one_graphic   = 21, // 以上2+2+2+15
	LEN_INTERACT_draw_two_graphic   = 36, //6+15*2
	LEN_INTERACT_draw_five_graphic  = 81, //6+15*5
	LEN_INTERACT_draw_seven_graphic = 111,//6+15*7
	LEN_INTERACT_draw_char_graphic  = 51, //6+15+30（字符串内容）
};
//****************************绘图的数据段内容****************************/
typedef __packed struct//图形
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; //直线  矩形  正圆  椭圆  圆弧  浮点  整型  字符
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  //空    空    空    空    角度  大小  大小  大小
	uint32_t end_angle:9;    //空    空    空    空          位数  空    长度
	uint32_t width:10;       
	uint32_t start_x:11;     //起点  起点  圆心  圆心  圆心  起点  起点  起点
	uint32_t start_y:11;     //
	uint32_t radius:10;      //空    空    半径  空    空    、    、    空
	uint32_t end_x:11;       //终点  对顶  空    半轴  半轴  、    、    空
	uint32_t end_y:11;       //                              数    数    空
} graphic_data_struct_t;
typedef __packed struct//浮点数
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  
	uint32_t end_angle:9;    
	uint32_t width:10;       
	uint32_t start_x:11;    
	uint32_t start_y:11;     
  int number;       
} Float_data_struct_t;
typedef __packed struct//整型数
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  
	uint32_t end_angle:9;    
	uint32_t width:10;       
	uint32_t start_x:11;    
	uint32_t start_y:11;     
  int number;       
} Int_data_struct_t;
/* data_ID: 0X0100  Byte:  2	    客户端删除图形*/
typedef __packed struct
{
	uint8_t operate_type; 
	uint8_t layer;//图层数：0~9
}ext_client_custom_graphic_delete_t;
typedef enum
{
	NONE_delete    = 0,
	GRAPHIC_delete = 1,
	ALL_delete     = 2
}delete_Graphic_Operate;//ext_client_custom_graphic_delete_t：uint8_t operate_type
/*图层删除操作*/

//bit 0-2
typedef enum
{
	NONE   = 0,/*空操作*/
	ADD    = 1,/*增加图层*/
	MODIFY = 2,/*修改图层*/
	DELETE = 3,/*删除图层*/
}Graphic_Operate;//graphic_data_struct_t：uint32_t operate_tpye
/*图层操作*/
//bit3-5
typedef enum
{
	LINE      = 0,//直线
	RECTANGLE = 1,//矩形
	CIRCLE    = 2,//正圆
	OVAL      = 3,//椭圆
	ARC       = 4,//圆弧
	INT    	  = 5,//浮点数
	FLOAT     = 6,//整型数
	CHAR      = 7 //字符
}Graphic_Type;
/*图层类型*/
//bit 6-9图层数 最大为9，最小0
//bit 10-13颜色
typedef enum
{
	RED_BLUE  = 0,//红蓝主色	
	YELLOW    = 1,
	GREEN     = 2,
	ORANGE    = 3,
	FUCHSIA   = 4,	/*紫红色*/
	PINK      = 5,
	CYAN_BLUE = 6,	/*青色*/
	BLACK     = 7,
	WHITE     = 8
}Graphic_Color;
/*图层颜色类型*/
//bit 14-31 角度 [0,360]
/**********************************客户端绘图************************************************/
//删除图层
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	ext_client_custom_graphic_delete_t clientData;		
	uint16_t	FrameTail;								
}ext_deleteLayer_data_t;

//绘字符串
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_string_t;

typedef __packed struct
{
	std_frame_header_t txFrameHeader;			//帧头
	uint16_t  CmdID;										//命令码
	ext_client_data_header_t   dataFrameHeader;//数据段头结构
	ext_client_string_t clientData;//数据段
	uint16_t	FrameTail;								//帧尾
}ext_charstring_data_t;
//绘象形图
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			//帧头
	uint16_t  CmdID;										//命令码
	ext_client_data_header_t   dataFrameHeader;//数据段头结构
	graphic_data_struct_t clientData;		//数据段
	uint16_t	FrameTail;								//帧尾
}ext_graphic_one_data_t;
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	graphic_data_struct_t clientData[2];		
	uint16_t	FrameTail;
	
}ext_graphic_two_data_t;
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	graphic_data_struct_t clientData[5];		
	uint16_t	FrameTail;								
}ext_graphic_five_data_t;
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	graphic_data_struct_t clientData[7];		
	uint16_t	FrameTail;								
}ext_graphic_seven_data_t;
//绘制浮点型
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	Float_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_float_two_data_t;

typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	Float_data_struct_t clientData[7];		
	uint16_t	FrameTail;								
}ext_float_seven_data_t;
//绘制整型
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	Int_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_int_two_data_t;
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	Int_data_struct_t clientData[7];		
	uint16_t	FrameTail;								
}ext_int_seven_data_t;
 
typedef __packed struct Client_Slave_Flag
{
	uint8_t global_fiction;
	uint8_t global_clip;
	uint8_t global_spin;
	uint8_t global_auto_aim;
	uint8_t global_twist;
	uint8_t global_anti_top;
	uint8_t shift_rush;
	uint8_t user1;
}Client_Slave_Flag;



extern uint8_t update_figure_flag,update_aim_flag,update_float_flag,update_supercapacitor_flag,update_int_flag;//客户端更新flag
extern JUDGE_MODULE_DATA Judge_Hero;
extern Client_Slave_Flag Slaver_flag;

void Determine_ID(void);

void DJI_Judge_Init(void);
void Client_graphic_Init(void);
void Client_graphic_delete_update(uint8_t delete_layer);//删除图层信息
void Client_graphic_Info_update(void);
void Client_aim_update(void);//准心
void Client_supercapacitor_update(void);//超级电容
void Client_gimbal_angle_update(void);//吊射角度
void Client_bullet_int_update(void);//弹丸信息
void Client_aim_line(void);//英雄

void Client_flag_update(void);//标识符更新
void PITCH_YAW_Analyze(u8 *buf);//pitch，yaw更新
// 裁判系统的分析函数
void JUDGE_Analyze(volatile u8 *databuff, u8 lenth);
void Client_task(void);


#endif
