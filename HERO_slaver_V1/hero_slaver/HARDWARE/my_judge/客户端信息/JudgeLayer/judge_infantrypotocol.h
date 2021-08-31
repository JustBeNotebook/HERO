#ifndef  __judge_infantrypotocol_h
#define  __judge_infantrypotocol_h
#include "rp_config.h"
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

typedef __packed struct
{
	uint8_t  sof;
	uint16_t data_length;
	uint8_t  seq;
	uint8_t  crc8;
} std_frame_header_t;//LEN_FRAME_HEAD

//LEN_CMD_ID
//LEN_FRAME_TAIL
/* ID: 0x0001	Byte: 	11	比赛状态数据 */
typedef __packed struct 
{ 
	uint8_t game_type     : 4;		// 比赛类型
	uint8_t game_progress : 4;		// 比赛阶段
	uint16_t stage_remain_time;		// 当前阶段剩余时间(单位:s)
	uint64_t SyncTimeStamp;       //机器人接收到该指令的精确 Unix 时间，当机载端收到有效的 NTP 服务器授时后生效
} ext_game_status_t; //LEN_GAME_STATUS   表3-1
/* ID: 0x0002	Byte:	1	比赛结果数据 */
typedef __packed struct
{
	uint8_t winner;
} ext_game_result_t; //LEN_GAME_RESULT   表3-2
/* ID: 0x0003	Byte:	28	机器人血量数据数据 */
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP;	// 红1英雄机器人血量(未上场及罚下血量为0)
	uint16_t red_2_robot_HP;	// 红2工程机器人血量
	uint16_t red_3_robot_HP;	// 红3步兵机器人血量
	uint16_t red_4_robot_HP;	// 红4步兵机器人血量
	uint16_t red_5_robot_HP;	// 红5步兵机器人血量
	uint16_t red_7_robot_HP;	// 红7哨兵机器人血量
	uint16_t red_outpost_HP;	// 红方前哨站血量
	uint16_t red_base_HP;			// 红方基地血量
	uint16_t blue_1_robot_HP;	// 蓝1英雄机器人血量
	uint16_t blue_2_robot_HP;	// 蓝2工程机器人血量
	uint16_t blue_3_robot_HP;	// 蓝3步兵机器人血量
	uint16_t blue_4_robot_HP;	// 蓝4步兵机器人血量
	uint16_t blue_5_robot_HP;	// 蓝5步兵机器人血量
	uint16_t blue_7_robot_HP;	// 蓝7哨兵机器人血量
	uint16_t blue_outpost_HP;	// 蓝方前哨站血量
	uint16_t blue_base_HP;		// 蓝方基地血量	
} ext_game_robot_HP_t; //LEN_GAME_ROBOT_HP  表3-3
/* ID: 0x0004 	Byte:	3	飞镖发射状态 */
typedef __packed struct
{
	uint8_t dart_belong;
	uint16_t stage_remaining_time;
} ext_dart_status_t;//LEN_DART_STATUS   表3-4
/* ID: 0x0005 	Byte:	11	人工智能挑战赛加成与惩罚区状态 */
typedef __packed struct
{
	uint8_t F1_zone_status:1;            //激活状态
	uint8_t F1_zone_buff_debuff_status:3;//红方回血区
	uint8_t F2_zone_status:1;
	uint8_t F2_zone_buff_debuff_status:3;//红方弹药补给区
	uint8_t F3_zone_status:1;
	uint8_t F3_zone_buff_debuff_status:3;//蓝方回血区
	uint8_t F4_zone_status:1;
	uint8_t F4_zone_buff_debuff_status:3;//蓝方弹药补给区
	uint8_t F5_zone_status:1;
	uint8_t F5_zone_buff_debuff_status:3;//禁止射击区
	uint8_t F6_zone_status:1;
	uint8_t F6_zone_buff_debuff_status:3;//禁止移动区
	uint16_t red1_bullet_left;           //红方 1 号剩余弹量
	uint16_t red2_bullet_left;           //红方 2 号剩余弹量
	uint16_t blue1_bullet_left;          //蓝方 1 号剩余弹量
	uint16_t blue2_bullet_left;          //蓝方 2 号剩余弹量
} ext_ICRA_buff_debuff_zone_status_t;//LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS  表3-5
/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct 
{ 
	uint32_t event_type;//己方补给站补血点占领状态,能量机关状态, 环形高地占领状态,己方基地护盾状态,己方前哨战状态
} ext_event_data_t; //LEN_EVENT_DATA   表3-6
/* ID: 0x0102  Byte:  4    补给站动作标识数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;  //补给站口 ID
	uint8_t supply_robot_id;       //补弹机器人 ID
	uint8_t supply_projectile_step;//出弹口开闭状态
	uint8_t supply_projectile_num; //补弹数量
} ext_supply_projectile_action_t;//LEN_SUPPLY_PROJECTILE_ACTION  表3-7
//LEN_SUPPLY_PROJECTILE_BOOKING
/* ID: 0X0104  Byte:  2	   裁判警告信息 */
typedef __packed struct
{
	uint8_t level;        //警告等级
	uint8_t foul_robot_id;//犯规机器人 ID
} ext_referee_warning_t;//LEN_REFEREE_WARNING   表3-8
/* ID: 0X0105  Byte:  1	   飞镖发射口倒计时 */
typedef __packed struct
{
	uint8_t dart_remaining_time;//15s 倒计时
} ext_dart_remaining_time_t;//LEN_DART_REMAINING_TIME
/* ID: 0X0201  Byte: 27    机器人状态数据 */
typedef __packed struct 
{ 
	uint8_t robot_id;                       //本机器人 ID
	uint8_t robot_level;                    //机器人等级
	uint16_t remain_HP;                     //机器人剩余血量
	uint16_t max_HP;                        //机器人上限血量
	uint16_t shooter_id1_17mm_cooling_rate; //机器人 1 号 17mm 枪口每秒冷却值
	uint16_t shooter_id1_17mm_cooling_limit;//机器人 1 号 17mm 枪口热量上限
	uint16_t shooter_id1_17mm_speed_limit;  //机器人 1 号 17mm 枪口上限速度 单位 m/s
	uint16_t shooter_id2_17mm_cooling_rate; //机器人 2 号 17mm 枪口每秒冷却值
	uint16_t shooter_id2_17mm_cooling_limit;//机器人 2 号 17mm 枪口热量上限
	uint16_t shooter_id2_17mm_speed_limit;  //机器人 2 号 17mm 枪口上限速度 单位 m/s
	uint16_t shooter_id1_42mm_cooling_rate; //机器人 42mm 枪口每秒冷却值
	uint16_t shooter_id1_42mm_cooling_limit;//机器人 42mm 枪口热量上限
	uint16_t shooter_id1_42mm_speed_limit;  //机器人 42mm 枪口上限速度 单位 m/s
	uint16_t chassis_power_limit;           //机器人底盘功率限制上限
	//主控电源输出情况
	uint8_t mains_power_gimbal_output  : 1; //gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出
	uint8_t mains_power_chassis_output : 1; //chassis 口输出：1 为有 24V 输出，0 为无 24v 输出
	uint8_t mains_power_shooter_output : 1; //shooter 口输出：1 为有 24V 输出，0 为无 24v 输出
} ext_game_robot_status_t; //LEN_GAME_ROBOT_STATUS  表3-10
/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   				// 底盘输出电压，单位：mV
	uint16_t chassis_current;				// 底盘输出电流，单位：mA
	float chassis_power;   					// 底盘瞬时功率，单位：W
	uint16_t chassis_power_buffer;	// 底盘功率缓冲，单位：60J焦耳缓冲能量(飞坡根据规则增加至250J)
	uint16_t shooter_id1_17mm_cooling_heat; //1 号 17mm 枪口热量
	uint16_t shooter_id2_17mm_cooling_heat; //2 号 17mm 枪口热量
	uint16_t shooter_id1_42mm_cooling_heat;	//42mm 枪口热量
} ext_power_heat_data_t; //LEN_POWER_HEAT_DATA   表3-11
/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; //位置枪口,单位度数
} ext_game_robot_pos_t; //LEN_GAME_ROBOT_POS   表3-12
/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_t; //LEN_BUFF_MASK  表3-13
/* ID: 0x0205  Byte:  1    空中机器人能量状态数据 */
typedef __packed struct 
{ 
	uint8_t attack_time;//可攻击时间 单位 s。30s 递减至 0
} ext_aerial_robot_energy_t; //LEN_AERIAL_ROBOT_ENERGY  表3-14
/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 	// 装甲伤害时代表装甲ID
	uint8_t hurt_type : 4; 	// 0x0装甲伤害 0x1模块掉线 0x2超射速 0x3超热量 0x4超功率 0x5撞击
} ext_robot_hurt_t; //LEN_ROBOT_HURT  表3-15
/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef __packed struct 
{ 
	uint8_t bullet_type; 	// 子弹类型(1-17mm, 2-42mm)
	uint8_t shooter_id;   // 发射机构 ID（1：1 号 17mm 发射机构，2：2 号 17mm 发射机构，3：42mm 发射机构）
	uint8_t bullet_freq;  // 子弹射频(Hz)
	float bullet_speed;		// 子弹射速(m/s)
} ext_shoot_data_t; //LEN_SHOOT_DATA  表3-16
/* ID: 0x0208  Byte:  6    子弹剩余发射数数据 */
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm;//17mm 子弹剩余发射数目
	uint16_t bullet_remaining_num_42mm;//42mm 子弹剩余发射数目
	uint16_t coin_remaining_num;       //剩余金币数量
} ext_bullet_remaining_t; //LEN_BULLET_REMAINING   表3-17
/* ID: 0x0209  Byte:  4 	机器人RFID状态 */
typedef __packed struct
{
	uint32_t rfid_status;
} ext_rfid_status_t;//LEN_RFID_STATUS  表3-18
/*ID：0x020A   Byte：12  飞镖机器人客户端指令数据*/
typedef struct{
	uint8_t dart_launch_opening_status;//当前飞镖发射口的状态
	uint8_t dart_attack_target;        //飞镖的打击目标，默认为前哨站（1：前哨站，2：基地）
	uint16_t target_change_time;       //切换打击目标时的比赛剩余时间
	uint8_t first_dart_speed;          //检测到的第一枚飞镖速度，单位 0.1m/s/LSB
	uint8_t second_dart_speed;         //检测到的第二枚飞镖速度，单位 0.1m/s/LSB
	uint8_t third_dart_speed;          //检测到的第三枚飞镖速度，单位 0.1m/s/LSB
	uint8_t fourth_dart_speed;         //检测到的第四枚飞镖速度，单位 0.1m/s/LSB
	uint16_t last_dart_launch_time;    //最近一次的发射飞镖的比赛剩余时间，单位秒
	uint16_t operate_launch_cmd_time;  //最近一次操作手确定发射指令时的比赛剩余时间，单位秒
} ext_dart_client_cmd_t; //LEN_DART_CLIENT_DIRECTIVE  表3-19
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
typedef struct{
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


typedef struct judge_info_struct {
	std_frame_header_t							fream_header;				// 帧头信息
	
	ext_game_status_t 							game_status;				// 0x0001
	ext_game_result_t 							game_result;				// 0x0002
	ext_game_robot_HP_t 						game_robot_HP;			// 0x0003
	ext_dart_status_t								dart_status;				// 0x0004
	ext_ICRA_buff_debuff_zone_status_t	ICRA_buff;
	
	ext_event_data_t								event_data;					// 0x0101
	ext_supply_projectile_action_t	supply_projectile_action;		// 0x0102
	//ext_supply_projectile_booking_t supply_projectile_booking;// 0x0103
	ext_referee_warning_t						referee_warning;		// 0x0104
	ext_dart_remaining_time_t				dart_remaining_time;// 0x0105
	
	ext_game_robot_status_t					game_robot_status;	// 0x0201
	ext_power_heat_data_t						power_heat_data;		// 0x0202
	ext_game_robot_pos_t						game_robot_pos;			// 0x0203
	ext_buff_t											buff;								// 0x0204
	ext_aerial_robot_energy_t				aerial_robot_energy;// 0x0205
	ext_robot_hurt_t								robot_hurt;					// 0x0206
	ext_shoot_data_t								shoot_data;					// 0x0207
	ext_bullet_remaining_t					bullet_remaining;		// 0x0208	
	ext_rfid_status_t								rfid_status;				// 0x0209	
	ext_dart_client_cmd_t           dart_client;        // 0x020A
	
	ext_interact_id_t								ids;								//与本机交互的机器人id
	uint16_t                        self_client;        //本机客户端
	bool	 		data_valid;	// 数据有效性
	bool			err_cnt;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;	
} judge_info_t;
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
  float number;       
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
	FLOAT     = 5,//浮点数
	INT       = 6,//整型数
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
//
typedef struct judge_sensor_struct {
	judge_info_t		*info;
	drv_uart_t			*driver;
	void		(*init)(struct judge_sensor_struct *self);
	void		(*update)(struct judge_sensor_struct *self, uint8_t *rxBuf);
	void		(*check)(struct judge_sensor_struct *self);	
	void		(*heart_beat)(struct judge_sensor_struct *self);
	void		(*sendclient)(struct judge_sensor_struct *self);
	void		(*sendteammate)(struct judge_sensor_struct *self);
	dev_work_state_t	work_state;
	dev_errno_t				errno;
	dev_id_t					id;
} judge_sensor_t;

void Client_graphic_Init(void);
void Client_graphic_delete_update(uint8_t delete_layer);//删除图层信息
void Client_graphic_Info_update(void);


void Client_aim_update(void);//准心
void Client_supercapacitor_update(void);//超级电容
void Client_gimbal_angle_update(void);//吊射角度
void Client_bullet_int_update(void);//弹丸信息

void Client_aim_line(void);//英雄
#endif
