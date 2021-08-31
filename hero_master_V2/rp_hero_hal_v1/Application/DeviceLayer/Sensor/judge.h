#ifndef RM__MY__JUDGE
#define RM__MY__JUDGE

#include "main.h"

#define Judge_SOF 0xa5;


/* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;						
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
}FrameHeader;

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
	uint16_t red_6_robot_HP;  
	uint16_t red_7_robot_HP;  
	uint16_t red_outpost_HP; 
  uint16_t red_base_HP; 
	
	uint16_t blue_1_robot_HP;   
	uint16_t blue_2_robot_HP;   
	uint16_t blue_3_robot_HP;   
	uint16_t blue_4_robot_HP;   
	uint16_t blue_5_robot_HP;   
	uint16_t blue_6_robot_HP;   
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
	uint16_t shooter_42mm_cooling_rate;   				//机器人17mm 2枪口每秒冷却值
	uint16_t shooter_42mm_cooling_limit;  				//机器人17mm 2枪口热量上限
	uint16_t shooter_42mm_speed_limit;						//机器人17mm 2枪口上限速度
	
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
typedef __packed struct {  
	uint8_t dart_launch_opening_status;  
	uint8_t dart_attack_target;  
	uint16_t target_change_time; 
  uint8_t first_dart_speed;   
	uint8_t second_dart_speed;  
  uint8_t third_dart_speed;   
	uint8_t fourth_dart_speed; 
	uint16_t last_dart_launch_time; 
	uint16_t operate_launch_cmd_time; 
} ext_dart_client_cmd_t; 
	
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 




typedef __packed struct 
{ 
	uint8_t data[10]; //数据段,n需要小于113
} robot_interactive_data_t;

typedef struct JUDGE_MODULE_DATA
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
	
	
	ext_game_robot_state_t robot_state;						//机器人状态
	ext_power_heat_data_t out_and_heat; 					//功率热量状态
	ext_game_robot_pos_t  robot_pos;							//机器人位置
	ext_buff_musk_t 			buff;										//机器人buff状态
	ext_robot_hurt_t     robot_hurt;							//机器人受伤警告
	ext_shoot_data_t			shoot_data;							//实时射击数据
	ext_bullet_remaining_t bullet_remain;					//子弹余量
	
	ext_rfid_status_t		rfid_status;							//rfid卡状态
	
	uint8_t state_flag;
	uint8_t out_heat_flag;
	uint8_t buff_flag;
	uint8_t hurt_flag;
	uint8_t shoot_flag;
	uint8_t rfid_flag;
	
} JUDGE_MODULE_DATA;

extern JUDGE_MODULE_DATA Judge_Hero;
extern int shoot_big_num;
extern uint32_t judge_offline_cnt;
extern uint8_t Judge_Offline_flag;



void DJI_Judge_Init(void);

// 裁判系统的分析函数
void JUDGE_Analyze(volatile uint8_t *databuff, uint8_t lenth);
void CAN_Judge_Analyze(uint16_t id, uint8_t *databuff);


#endif
