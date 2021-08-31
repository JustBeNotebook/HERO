#include "judge_infantrypotocol.h"

#include "judge_sensor.h"
#include "crc.h"
#include "string.h"
#include "drv_uart.h"
#include "usart.h"
#define JUDGE_FRAME_HEADER	0xA5  //此为帧头的SOF,帧头分为 SOF,length,Seq,CRC8
void USART5_rxDataHandler(uint8_t *rxBuf)
{	
	judge_sensor.update(&judge_sensor, rxBuf);
	judge_sensor.check(&judge_sensor);
}
/**********偏移位置**********/
//帧字节（单位/字节）
enum judge_frame_offset_t {//帧头占五个字节，命令码占两个字节，从第七个开始为数据帧
	FRAME_HEADER	= 0,
	CMD_ID			= 5,
	DATA_SEG		= 7
};
//帧头字节（单位/bit）
enum judge_frame_header_offset_t {//帧头的详细位置
	SOF			= 0,
	DATA_LENGTH	= 1,
	SEQ			= 3,
	CRC8		= 4
};//FRAME_HEADER
void judge_sensor_init(judge_sensor_t *judge_sen)
{
	judge_sen->info->offline_cnt = judge_sen->info->offline_max_cnt+1;
	judge_sen->work_state= DEV_OFFLINE;
	
	if(judge_sen->id == DEV_ID_JUDGE) {
		judge_sen->errno = NONE_ERR;
	}
	else {
		judge_sen->errno = DEV_ID_ERR;
	}
}
void Determine_ID(void);
void judge_sensor_update(judge_sensor_t *judge_sen, uint8_t *rxBuf)
{
	uint8_t  res = false;
	uint16_t frame_length;
	uint16_t cmd_id;	
	judge_info_t *judge_info = judge_sen->info;
	
	if( rxBuf == NULL )
	{
		judge_info->data_valid = false;
		return;
	}
	
	memcpy(&judge_info->fream_header, rxBuf, LEN_FRAME_HEAD);//5个字节
	
	/* 帧首字节是否为0xA5 */
	if(rxBuf[SOF] == JUDGE_FRAME_HEADER) 
	{
		/* 帧头CRC8校验 */
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true) 
		{
			/* 统计一帧的总数据长度，用于CRC16校验 */     // 长度两个字节
			frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + judge_info->fream_header.data_length + LEN_FRAME_TAIL;
			
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == true)
			{
				res = true;
				
				cmd_id = (rxBuf[CMD_ID+1] << 8 | rxBuf[CMD_ID]);
				
				switch(cmd_id)
				{
					case ID_game_state: 
						memcpy(&judge_info->game_status, (rxBuf+DATA_SEG), LEN_GAME_STATUS);
						break;
					
					case ID_game_result: 
						memcpy(&judge_info->game_result, (rxBuf+DATA_SEG), LEN_GAME_RESULT);
						break;
					
					case ID_game_robot_HP: 
						memcpy(&judge_info->game_robot_HP, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_HP);
						break;//所有机器人的血量都可以获得
					
					case ID_dart_status: 
						memcpy(&judge_info->dart_status, (rxBuf+DATA_SEG), LEN_DART_STATUS);
						break;
					
					case ID_ICRA_buff_debuff_zone_status:
						memcpy(&judge_info->ICRA_buff,(rxBuf+DATA_SEG),LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS);
						break;
					
					case ID_event_data: 
						memcpy(&judge_info->event_data, (rxBuf+DATA_SEG), LEN_EVENT_DATA);
						break;
					
					case ID_supply_projectile_action: 
						memcpy(&judge_info->supply_projectile_action, (rxBuf+DATA_SEG), LEN_SUPPLY_PROJECTILE_ACTION);
						break;
					
					case ID_referee_warning: 
						memcpy(&judge_info->referee_warning, (rxBuf+DATA_SEG), LEN_REFEREE_WARNING);
						break;
					
					case ID_dart_remaining_time: 
						memcpy(&judge_info->dart_remaining_time, (rxBuf+DATA_SEG), LEN_DART_REMAINING_TIME);
						break;
						
					case ID_game_robot_state: 
						memcpy(&judge_info->game_robot_status, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_STATUS);
						Determine_ID();
						break;
					
					case ID_power_heat_data: 
						memcpy(&judge_info->power_heat_data, (rxBuf+DATA_SEG), LEN_POWER_HEAT_DATA);
						break;
					
					case ID_game_robot_pos: 
						memcpy(&judge_info->game_robot_pos, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_POS);
						break;
					
					case ID_buff_musk: 
						memcpy(&judge_info->buff, (rxBuf+DATA_SEG), LEN_BUFF_MASK);
						break;
					
					case ID_aerial_robot_energy: 
						memcpy(&judge_info->aerial_robot_energy, (rxBuf+DATA_SEG), LEN_AERIAL_ROBOT_ENERGY);
						break;
					
					case ID_robot_hurt: 
						memcpy(&judge_info->robot_hurt, (rxBuf+DATA_SEG), LEN_ROBOT_HURT);
						break;
					
					case ID_shoot_data: 
						memcpy(&judge_info->shoot_data, (rxBuf+DATA_SEG), LEN_SHOOT_DATA);
						break;
					
					case ID_bullet_remaining: 
						memcpy(&judge_info->bullet_remaining, (rxBuf+DATA_SEG), LEN_BULLET_REMAINING);
						break;
					
					case ID_rfid_status: 
						memcpy(&judge_info->rfid_status, (rxBuf+DATA_SEG), LEN_RFID_STATUS);
						break;
						
					case ID_dart_client_directive:
						memcpy(&judge_info->dart_client,(rxBuf+DATA_SEG),LEN_DART_CLIENT_DIRECTIVE);
						break;
					
//					case ID_COMMUNICATION: 
//						//JUDGE_ReadFromCom();
//						break;
				}
			}
		}
		
		/* 帧尾CRC16下一字节是否为0xA5 */
		if(rxBuf[frame_length] == JUDGE_FRAME_HEADER)
		{
			/* 如果一个数据包出现了多帧数据就再次读取 */
			judge_sensor_update( judge_sen, &rxBuf[frame_length] );
		}
	}
	
	judge_info->data_valid = res;
	if(judge_info->data_valid != true)
		judge_info->err_cnt++;
	
	// 接收到数据表示在线
	judge_info->offline_cnt = 0;
}
void Determine_ID(void)//判断自己是哪个队伍
{
	if(judge_info.game_robot_status.robot_id < 10)//本机器人的ID，红方
	{ 
		judge_info.ids.teammate_hero 		 	= 1;
		judge_info.ids.teammate_engineer  = 2;
		judge_info.ids.teammate_infantry3 = 3;
		judge_info.ids.teammate_infantry4 = 4;
		judge_info.ids.teammate_infantry5 = 5;
		judge_info.ids.teammate_plane		 	= 6;
		judge_info.ids.teammate_sentry		= 7;
		
		judge_info.ids.client_hero 		 	= 0x0101;
		judge_info.ids.client_engineer  = 0x0102;
		judge_info.ids.client_infantry3 = 0x0103;
		judge_info.ids.client_infantry4 = 0x0104;
		judge_info.ids.client_infantry5 = 0x0105;
		judge_info.ids.client_plane			= 0x0106;
		
		if     (judge_info.game_robot_status.robot_id == hero_red)//不断刷新放置在比赛中更改颜色
			judge_info.self_client = judge_info.ids.client_hero;
		else if(judge_info.game_robot_status.robot_id == engineer_red)
			judge_info.self_client = judge_info.ids.client_engineer;
		else if(judge_info.game_robot_status.robot_id == infantry3_red)
			judge_info.self_client = judge_info.ids.client_infantry3;
		else if(judge_info.game_robot_status.robot_id == infantry4_red)
			judge_info.self_client = judge_info.ids.client_infantry4;
		else if(judge_info.game_robot_status.robot_id == infantry5_red)
			judge_info.self_client = judge_info.ids.client_infantry5;
		else if(judge_info.game_robot_status.robot_id == plane_red)
			judge_info.self_client = judge_info.ids.client_plane;
	}
	else //蓝方
	{
		judge_info.ids.teammate_hero 		 	= 101;
		judge_info.ids.teammate_engineer  = 102;
		judge_info.ids.teammate_infantry3 = 103;
		judge_info.ids.teammate_infantry4 = 104;
		judge_info.ids.teammate_infantry5 = 105;
		judge_info.ids.teammate_plane		 	= 106;
		judge_info.ids.teammate_sentry		= 107;
		
		judge_info.ids.client_hero 		 	= 0x0165;
		judge_info.ids.client_engineer  = 0x0166;
		judge_info.ids.client_infantry3 = 0x0167;
		judge_info.ids.client_infantry4 = 0x0168;
		judge_info.ids.client_infantry5 = 0x0169;
		judge_info.ids.client_plane			= 0x016A;
		
		if     (judge_info.game_robot_status.robot_id == hero_blue)
			judge_info.self_client = judge_info.ids.client_hero;
		else if(judge_info.game_robot_status.robot_id == engineer_blue)
			judge_info.self_client = judge_info.ids.client_engineer;
		else if(judge_info.game_robot_status.robot_id == infantry3_blue)
			judge_info.self_client = judge_info.ids.client_infantry3;
		else if(judge_info.game_robot_status.robot_id == infantry4_blue)
			judge_info.self_client = judge_info.ids.client_infantry4;
		else if(judge_info.game_robot_status.robot_id == infantry5_blue)
			judge_info.self_client = judge_info.ids.client_infantry5;
		else if(judge_info.game_robot_status.robot_id == plane_blue)
			judge_info.self_client = judge_info.ids.client_plane;
		
	}

}
/*********************************************/
uint8_t CliendTxBuffer[200];
uint8_t TeammateTxBuffer[200];
char first_line[30]  = {"readyfire:"};//是否可以射击,最多放30个字符串，bool
char second_line[30] = {"      top:"};//小陀螺
char third_line[30]  = {" auto_aim:"};//自瞄
char fourth_line[30] = {"    twist:"};//扭腰
char fifth_line[30]  = {":bullet_sum"};//发弹量int
char sixth_line[30]  = {":bullet_speed"};//射速int
char seventh_line[30]= {":supercapacitor"};//超级电容剩余量,float
//*******************************绘字符串******************************/
ext_charstring_data_t tx_client_char;
uint8_t state_first_graphic;//0~7循环
void Char_Graphic(ext_client_string_t* graphic,//最终要发出去的数组中的数据段内容
									const char* name,
									uint32_t operate_tpye,
									
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t length,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,

									
									
									const char *character)//外部放入的数组
{
	graphic_data_struct_t *data_struct = &graphic->grapic_data_struct;
	for(char i=0;i<3;i++)
		data_struct->graphic_name[i] = name[i];	//字符索引
	data_struct->operate_tpye = operate_tpye; //图层操作
	data_struct->graphic_tpye = CHAR;         //Char型
	data_struct->layer = layer;//都在第零层
	data_struct->color = color;//都是白色
	data_struct->start_angle = size;
	data_struct->end_angle = length;	
	data_struct->width = width;
	data_struct->start_x = start_x;
	data_struct->start_y = start_y;	
	
	data_struct->radius = 0;
	data_struct->end_x = 0;
	data_struct->end_y = 0;
	memcpy(graphic->data,character,length);
}
static void Draw_char()
{
	if(state_first_graphic == 0)//不知道什么时候进入客户端所以要不断更新
	{
		Char_Graphic(&tx_client_char.clientData,"CL1",ADD,0,WHITE,10,strlen(first_line),1,(50),(1080*9/12),first_line);//x1920/18
		state_first_graphic = 1;
	}
	else if(state_first_graphic == 1)
	{
		Char_Graphic(&tx_client_char.clientData,"CL2",ADD,0,WHITE,10,strlen(second_line),1,(50),(1080*8/12),second_line);
		state_first_graphic = 2;
	}
	else if(state_first_graphic == 2)
	{
		Char_Graphic(&tx_client_char.clientData,"CL3",ADD,0,WHITE,10,strlen(third_line),1,(50),(1080*7/12),third_line);
		state_first_graphic = 3;
	}
	else if(state_first_graphic == 3)
	{
		Char_Graphic(&tx_client_char.clientData,"CL4",ADD,0,WHITE,10,strlen(fourth_line),1,(50),(1080*6/12),fourth_line);
		state_first_graphic = 4;
	}
	else if(state_first_graphic == 4)
	{
		Char_Graphic(&tx_client_char.clientData,"CL5",ADD,0,WHITE,10,strlen(fifth_line),1,(1920-200),(1080*9/12),fifth_line);
		state_first_graphic = 5;
	}
	else if(state_first_graphic == 5)
	{
		Char_Graphic(&tx_client_char.clientData,"CL6",ADD,0,WHITE,10,strlen(sixth_line),1,(1920-200),(1080*8/12),sixth_line);
		state_first_graphic = 6;
	}
	else if(state_first_graphic == 6)
	{
		Char_Graphic(&tx_client_char.clientData,"CL7",ADD,0,WHITE,10,strlen(seventh_line),1,(1920-200),(1080*7/12),seventh_line);
		state_first_graphic = 7;
	}
}
void Client_graphic_Init()
{
	if(state_first_graphic>=7)
	{
		state_first_graphic = 0;
	}
		//帧头
		tx_client_char.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_client_char.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(ext_client_string_t);
		tx_client_char.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_client_char.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验
	
		//命令码
		tx_client_char.CmdID = ID_robot_interactive_header_data;
		
		//数据段头结构
		tx_client_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
		tx_client_char.dataFrameHeader.send_ID     = judge_info.game_robot_status.robot_id;
		tx_client_char.dataFrameHeader.receiver_ID = judge_info.self_client;
		
		//数据段
		Draw_char();
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_char.CmdID, LEN_CMD_ID+tx_client_char.txFrameHeader.data_length);//加上命令码长度2
		
		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_client_char));
		
		HAL_UART_Transmit_DMA(&huart5, CliendTxBuffer, sizeof(tx_client_char));
}
//************************************绘制象形*******************************/
ext_graphic_five_data_t tx_client_graphic_figure;
void Figure_Graphic(graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
									uint32_t layer,
									uint32_t color,
									uint32_t start_angle,
									uint32_t end_angle,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									uint32_t radius,
									uint32_t end_x,
									uint32_t end_y)							
{
	for(char i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];	//字符索引
	graphic->operate_tpye = operate_tpye; //图层操作
	graphic->graphic_tpye = graphic_tpye;         //Char型
	graphic->layer        = layer;//都在第一层
	graphic->color        = color;//变色
	graphic->start_angle  = start_angle;
	graphic->end_angle    = end_angle;	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->radius = radius;
	graphic->end_x  = end_x;
	graphic->end_y  = end_y;
}
int update_figure_flag;
static void readyfire_first_figure(bool fiction,bool clip)//摩擦轮打开为true，舵机关上为true，
{
	if(fiction == true && clip == true)//可准备射击为绿色
		Figure_Graphic(&tx_client_graphic_figure.clientData[0],"GL1",update_figure_flag,CIRCLE,1,GREEN,0,0,5,  200,1080*9/12, 20,0,0);
	else if(fiction == true && clip == false)//舵机打开了，但是摩擦轮已开启，橙色危险
		Figure_Graphic(&tx_client_graphic_figure.clientData[0],"GL1",update_figure_flag,CIRCLE,1,ORANGE,0,0,5,  200,1080*9/12, 20,0,0);
	else if(fiction == false && clip == true)//舵机关上了，但是摩擦轮没打开，黄色无法发弹
		Figure_Graphic(&tx_client_graphic_figure.clientData[0],"GL1",update_figure_flag,CIRCLE,1,YELLOW,0,0,5,  200,1080*9/12, 20,0,0);
	else if(fiction == false && clip == false)//舵机打开，摩擦轮没有开启，紫红色装弹
		Figure_Graphic(&tx_client_graphic_figure.clientData[0],"GL1",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,5,  200,1080*9/12, 20,0,0);
}

static void spin_second_figure(bool spin)//小陀螺打开为true
{
	if(spin == true)//打开小陀螺为绿色
		Figure_Graphic(&tx_client_graphic_figure.clientData[1],"GL2",update_figure_flag,CIRCLE,1,GREEN,0,0,5,  200,1080*8/12, 20,0,0);
	else if(spin == false)//没开小陀螺为紫红色
		Figure_Graphic(&tx_client_graphic_figure.clientData[1],"GL2",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,5,  200,1080*8/12, 20,0,0);
}
static void auto_aim_third_figure(bool auto_aim)//自瞄打开为true
{
	if(auto_aim == true)//打开自瞄为绿色
		Figure_Graphic(&tx_client_graphic_figure.clientData[2],"GL3",update_figure_flag,CIRCLE,1,GREEN,0,0,5,  200,1080*7/12, 20,0,0);
	else if(auto_aim == false)//没开自瞄为紫红色
		Figure_Graphic(&tx_client_graphic_figure.clientData[2],"GL3",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,5,  200,1080*7/12, 20,0,0);
}
static void twist_fourth_figure(bool twist)//打开扭腰为true
{
	if(twist == true)
		Figure_Graphic(&tx_client_graphic_figure.clientData[3],"GL4",update_figure_flag,CIRCLE,1,GREEN,0,0,5,  200,1080*6/12, 20,0,0);
	else if(twist == false)
		Figure_Graphic(&tx_client_graphic_figure.clientData[3],"GL4",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,5,  200,1080*6/12, 20,0,0);	
}
//static void sight_bead_fifsix_figrue(uint32_t x,uint32_t y)//可移动准心，请强制转换成uint32_t1920*1080有部分地区无法画出
//{
//	Figure_Graphic(&tx_client_graphic_figure.clientData[4],"GL5",update_figure_flag,LINE,1,FUCHSIA,0,0,3,  x-20,y+20  ,0,  x+20,y-20);
//	Figure_Graphic(&tx_client_graphic_figure.clientData[5],"GL6",update_figure_flag,LINE,1,FUCHSIA,0,0,3,  x-20,y-20  ,0,  x+20,y+20);
//}

//static void supercapacitor_seventh_figure(float remain_energy,uint32_t turning_point)//剩余超级电容（单位百分比），低于某百分比变红色
//{
//	uint32_t remaining = (uint32_t)remain_energy;//强制转换
//	if(remaining >= turning_point)//直线长度为3
//		Figure_Graphic(&tx_client_graphic_figure.clientData[6],"GL7",update_figure_flag,LINE,1,GREEN,0,0,10,(1920-350),585  ,0,  (1920-350)+remaining*3,585);
//	else if(remaining < turning_point)
//		Figure_Graphic(&tx_client_graphic_figure.clientData[6],"GL7",update_figure_flag,LINE,1,FUCHSIA,0,0,10,(1920-350),585  ,0,  (1920-350)+remaining*3,585);		
//}

static void anti_top_fifth_figure(bool anti_top)//反陀螺
{
	if(anti_top == true)
		Figure_Graphic(&tx_client_graphic_figure.clientData[4],"GL5",update_figure_flag,RECTANGLE,1,GREEN,0,0,5,  (1920-200),540, 0,1750,510);
	else if(anti_top == false)
		Figure_Graphic(&tx_client_graphic_figure.clientData[4],"GL5",update_figure_flag,RECTANGLE,1,FUCHSIA,0,0,5,  (1920-200),540, 0,1750,510);
}
//!!!!!!!!!!!!!!!!!!!全局变量！！！！！
bool global_fiction,global_clip,global_spin,global_auto_aim,global_twist,global_anti_top;
uint32_t global_sight_bead_x = 960,global_sight_bead_y = 720,global_supercapacitor_point;//[0,100]
float global_supercapacitor_remain = 77.3,//[0,100]
	    global_gimble_pitch,global_gimble_yaw;
int   global_bullet_speed,global_bullet_sum;
static void Draw_Figure_bool()
{
	readyfire_first_figure(global_fiction,global_clip);
	spin_second_figure    (global_spin);
	auto_aim_third_figure (global_auto_aim);
	twist_fourth_figure   (global_twist);
	anti_top_fifth_figure (global_anti_top);
//	sight_bead_fifsix_figrue(global_sight_bead_x,global_sight_bead_y);
//	supercapacitor_seventh_figure(global_supercapacitor_remain,global_supercapacitor_point);
}
//删除图层信息
ext_deleteLayer_data_t tx_client_delete;
void Client_graphic_delete_update(uint8_t delete_layer)//删除图层信息
{
		//帧头
		tx_client_delete.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_client_delete.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(ext_client_custom_graphic_delete_t);
		tx_client_delete.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_client_delete.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_client_delete.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_client_delete.dataFrameHeader.data_cmd_id = INTERACT_ID_delete_graphic;
		tx_client_delete.dataFrameHeader.send_ID     = judge_info.game_robot_status.robot_id;
		tx_client_delete.dataFrameHeader.receiver_ID = judge_info.self_client;
		
		//数据段
		tx_client_delete.clientData.operate_type = ALL_delete;
		tx_client_delete.clientData.layer = delete_layer;
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_delete.CmdID, LEN_CMD_ID+tx_client_delete.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_client_delete));
		
		HAL_UART_Transmit_DMA(&huart5, CliendTxBuffer, sizeof(tx_client_delete));
}
void Client_graphic_Info_update()//七个图像一起更新
{
		//帧头
		tx_client_graphic_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_client_graphic_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*5;
		tx_client_graphic_figure.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_client_graphic_figure.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_client_graphic_figure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_client_graphic_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;
		tx_client_graphic_figure.dataFrameHeader.send_ID     = judge_info.game_robot_status.robot_id;
		tx_client_graphic_figure.dataFrameHeader.receiver_ID = judge_info.self_client;
	
		//数据段
		Draw_Figure_bool();
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_graphic_figure.CmdID, LEN_CMD_ID+tx_client_graphic_figure.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_client_graphic_figure));
		
		HAL_UART_Transmit_DMA(&huart5, CliendTxBuffer, sizeof(tx_client_graphic_figure));

}
//*************************绘制变化图像*****************************************/

//void Client_Data_Info_update()
//{
//		
//}
ext_graphic_two_data_t tx_aim_figure;//第三层放准心
int update_aim_flag;//1-add,3删除
static void sight_bead_figrue(uint32_t x,uint32_t y)//可移动准心，请强制转换成uint32_t1920*1080有部分地区无法画出
{
	Figure_Graphic(&tx_aim_figure.clientData[0],"GR1",update_aim_flag,LINE,2,FUCHSIA,0,0,3,  x-20,y+20  ,0,  x+20,y-20);//graphic_Remove
	Figure_Graphic(&tx_aim_figure.clientData[1],"GR2",update_aim_flag,LINE,2,FUCHSIA,0,0,3,  x-20,y-20  ,0,  x+20,y+20);
}
void Client_aim_update()//两个个图像一起更新
{
		//帧头
		tx_aim_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_aim_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*2;
		tx_aim_figure.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_aim_figure.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_aim_figure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_aim_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
		tx_aim_figure.dataFrameHeader.send_ID     = judge_info.game_robot_status.robot_id;
		tx_aim_figure.dataFrameHeader.receiver_ID = judge_info.self_client;
	
		//数据段
		sight_bead_figrue(global_sight_bead_x,global_sight_bead_y);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_aim_figure.CmdID, LEN_CMD_ID+tx_aim_figure.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_aim_figure));
		
		HAL_UART_Transmit_DMA(&huart5, CliendTxBuffer, sizeof(tx_aim_figure));
}
//剩余电容只有一个图层
ext_graphic_one_data_t tx_supercapacitor_figure;
int update_supercapacitor_flag;
static void supercapacitor_figure(float remain_energy,uint32_t turning_point)//剩余超级电容（单位百分比），低于某百分比变红色
{
	uint32_t remaining = (uint32_t)remain_energy;//强制转换
	if(remaining >= turning_point)//直线长度为3
		Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR1",update_supercapacitor_flag,LINE,3,GREEN,0,0,10,(1920-350),585  ,0,  (1920-350)+remaining*3,585);
	else if(remaining < turning_point)
		Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR2",update_supercapacitor_flag,LINE,3,FUCHSIA,0,0,10,(1920-350),585  ,0,  (1920-350)+remaining*3,585);		
}
void Client_supercapacitor_update()//一个图像更新
{
		//帧头
		tx_supercapacitor_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_supercapacitor_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t);
		tx_supercapacitor_figure.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_supercapacitor_figure.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_supercapacitor_figure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_supercapacitor_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_one_graphic;
		tx_supercapacitor_figure.dataFrameHeader.send_ID     = judge_info.game_robot_status.robot_id;
		tx_supercapacitor_figure.dataFrameHeader.receiver_ID = judge_info.self_client;
	
		//数据段
		supercapacitor_figure(global_supercapacitor_remain,global_supercapacitor_point);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_supercapacitor_figure.CmdID, LEN_CMD_ID+tx_supercapacitor_figure.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_supercapacitor_figure));
		
		HAL_UART_Transmit_DMA(&huart5, CliendTxBuffer, sizeof(tx_supercapacitor_figure));
}
//******************绘制浮点数*************************/
//第五层图层
ext_float_two_data_t tx_gimbal_angle;
int update_float_flag;
void Float_Graphic(Float_data_struct_t* graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t decimal,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									float number)							
{
	for(char i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];	//字符索引
	graphic->operate_tpye = operate_tpye; //图层操作
	graphic->graphic_tpye = graphic_tpye;  
	graphic->layer        = layer;//
	graphic->color        = color;//变色
	graphic->start_angle  = size;
	graphic->end_angle    = decimal;//小数有效位	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->number       = number;
}
static void gimbal_angle_float(float gimble_pitch,float gimble_yaw)//当前云台角度
{
	//青色pitch第一行，黄色yaw第二行
		Float_Graphic(&tx_gimbal_angle.clientData[0],"FR1",update_float_flag,FLOAT,4,CYAN_BLUE,30,2,3,(1920*4/6),810  ,(float)gimble_pitch);
		Float_Graphic(&tx_gimbal_angle.clientData[1],"FR2",update_float_flag,FLOAT,4,YELLOW,   30,2,3,(1920*4/6),760  ,(float)gimble_yaw);		
}
void Client_gimbal_angle_update()//两个图像更新
{
		//帧头
		tx_gimbal_angle.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_gimbal_angle.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*2;
		tx_gimbal_angle.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_gimbal_angle.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_gimbal_angle.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_gimbal_angle.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
		tx_gimbal_angle.dataFrameHeader.send_ID     = judge_info.game_robot_status.robot_id;
		tx_gimbal_angle.dataFrameHeader.receiver_ID = judge_info.self_client;
	
		//数据段
		gimbal_angle_float(global_gimble_pitch,global_gimble_yaw);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_gimbal_angle.CmdID, LEN_CMD_ID+tx_gimbal_angle.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_gimbal_angle));
		
		HAL_UART_Transmit_DMA(&huart5, CliendTxBuffer, sizeof(tx_gimbal_angle));
}
//**********************绘制int类型***************************/
ext_int_two_data_t tx_bullet_int;
int update_int_flag;
void Int_Graphic(Int_data_struct_t* graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t zero,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									int number)							
{
	for(char i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];	//字符索引
	graphic->operate_tpye = operate_tpye; //图层操作
	graphic->graphic_tpye = graphic_tpye;        
	graphic->layer        = layer;//都在第一层
	graphic->color        = color;//变色
	graphic->start_angle  = size;
	graphic->end_angle    = zero;	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->number       = number;
}
static void bullet_int(int bullet_speed,int bullet_sum)//子弹射速和发弹量
{
	//总数量第一行，射速第二行
		Int_Graphic(&tx_bullet_int.clientData[0],"IR1",update_int_flag,INT,5,WHITE,30,0,3,(1920-280),(820)  ,(int)bullet_sum);
		Int_Graphic(&tx_bullet_int.clientData[1],"IR2",update_int_flag,INT,5,WHITE,30,0,3,(1920-280),(730)  ,(int)bullet_speed);		
}
void Client_bullet_int_update()//两个图像更新
{
		//帧头
		tx_bullet_int.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_bullet_int.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*2;
		tx_bullet_int.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_bullet_int.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_bullet_int.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_bullet_int.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
		tx_bullet_int.dataFrameHeader.send_ID     = judge_info.game_robot_status.robot_id;
		tx_bullet_int.dataFrameHeader.receiver_ID = judge_info.self_client;
	
		//数据段
		bullet_int(global_bullet_speed,global_bullet_sum);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_bullet_int.CmdID, LEN_CMD_ID+tx_bullet_int.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_bullet_int));
		
		HAL_UART_Transmit_DMA(&huart5, CliendTxBuffer, sizeof(tx_bullet_int));
}
//*****************************英雄需求*************************************/
ext_graphic_five_data_t aim_line_graphic;
uint32_t global_vertical_1,global_vertical_2,global_horizontal_1,global_horizontal_2,global_horizontal_3;
//绘图
static void Draw_Figure_aimline(uint32_t vertical_1,uint32_t vertical_2,//竖线范围在0~1920
	                              uint32_t horizontal_1,uint32_t horizontal_2,uint32_t horizontal_3)//水平线范围在0~1080
{
	//竖线
		Figure_Graphic(&aim_line_graphic.clientData[0],"LL1",ADD,LINE,6,WHITE,0,0,3,  vertical_1,1080, 0,vertical_1,0);
		Figure_Graphic(&aim_line_graphic.clientData[1],"LL2",ADD,LINE,6,WHITE,0,0,3,  vertical_2,1080, 0,vertical_2,0);
	//水平线
		Figure_Graphic(&aim_line_graphic.clientData[2],"LL3",ADD,LINE,6,WHITE,0,0,3,  0,horizontal_1, 0,1920,horizontal_1);
		Figure_Graphic(&aim_line_graphic.clientData[3],"LL4",ADD,LINE,6,WHITE,0,0,3,  0,horizontal_2, 0,1920,horizontal_2);
		Figure_Graphic(&aim_line_graphic.clientData[4],"LL5",ADD,LINE,6,WHITE,0,0,3,  0,horizontal_3, 0,1920,horizontal_3);
		

}
void Client_aim_line()//五个图像不更新
{
		//帧头
		aim_line_graphic.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		aim_line_graphic.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*5;
		aim_line_graphic.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&aim_line_graphic.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		aim_line_graphic.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		aim_line_graphic.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;
		aim_line_graphic.dataFrameHeader.send_ID     = judge_info.game_robot_status.robot_id;
		aim_line_graphic.dataFrameHeader.receiver_ID = judge_info.self_client;
	
		//数据段
    Draw_Figure_aimline(global_vertical_1,global_vertical_2,global_horizontal_1,global_horizontal_2,global_horizontal_3);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&aim_line_graphic.CmdID, LEN_CMD_ID+aim_line_graphic.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(aim_line_graphic));
		
		HAL_UART_Transmit_DMA(&huart5, CliendTxBuffer, sizeof(aim_line_graphic));

}



