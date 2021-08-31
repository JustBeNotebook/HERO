#include "link.h"
#include "usart.h"

u8 flag = 99;

////////////////////////////////////////////////////////////////////////////////////
extern u8 pass_flag;
static u32 mpu6050_outtime;
void System_Init(void)
{
	u32 remote_time;

	
	timestamp_init();			//时间结构体初始化，记录开机时间
	systick_init(1000);		//systick初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	LED_Init();		        //初始化LED端口`
	//1M BPS
	CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_11tq, 3, CAN_Mode_Normal);//can初始化
	CAN2_Mode_Init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_11tq, 3, CAN_Mode_Normal);
	RC_Init(100000);
	Usart1_Init();
	Usart3_Init();
	//IMUsensor_Init();
	
	//Chassis_Config();			//底盘初始化
	//Gimbal_Config();
	//Launcher_Init();
	ALL_Judge_Config();

	DJI_Judge_Init();//串口5
	
	SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;  //开启中断              
	
	TASK_Init();					//任务系统初始化
}

/////////////////////////////////////////////////////////////////////////



/////////////                  judge                       /////////////////
void ALL_Judge_Flash(void)
{
	Chassis_State_Update();
	Gimbal_State_Update();
	Launcher_state_update();
	
	RC_JUDGE();
}

void ALL_Judge_Config(void)
{
	Judge_Set(&(DJI_RC1.Judge), 14*5);
	DJI_RC1.Judge.func.deal_reconnecting = RC_Reconnecting_func;
	
}
////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////
//CAN 全部ID发送函数
u8 CAN1_0X1ff_BUF[8] = {0};
u8 CAN1_0X200_BUF[8] = {0};
u8 CAN1_0X2ff_BUF[8] = {0};

u8 CAN2_0X1ff_BUF[8] = {0};
u8 CAN2_0X200_BUF[8] = {0};
u8 CAN2_0X2ff_BUF[8] = {0};



//rfid
u8 CAN1_0X1a8_BUF[8] = {0};
u8 CAN1_0X1a9_BUF[8] = {0};

//bullet
u8 CAN1_0X1aa_BUF[8] = {0};

//shoot
u8 CAN1_0X1ab_BUF[8] = {0};

//robot_status
u8 CAN1_0X1ac_BUF[8] = {0};
u8 CAN1_0X1ad_BUF[8] = {0};

//out_heat + hurt + buff
u8 CAN1_0X1ae_BUF[8] = {0};
u8 CAN1_0X1af_BUF[8] = {0};

u8 CAN2_0X501_BUF[8] = {0};

void Can_Total_sent(void)
{
	CAN1_Send_Msg2(CAN1_0X1ff_BUF, 8, 0x1ff);
	CAN1_Send_Msg2(CAN1_0X200_BUF, 8, 0x200);
	CAN1_Send_Msg2(CAN1_0X2ff_BUF, 8, 0x2ff);
	
	CAN2_Send_Msg2(CAN2_0X1ff_BUF, 8, 0x1ff);
	CAN2_Send_Msg2(CAN2_0X200_BUF, 8, 0x200);
	CAN2_Send_Msg2(CAN2_0X2ff_BUF, 8, 0x2ff);
	
	Can_Sent_Clear();//每一轮发送完成后将缓冲区清零,以免进入奇怪的状态里面
}

void Can_Sent_Clear(void)
{
	u8 i;
	for(i = 0; i<8; i++)
	{
		CAN1_0X1ff_BUF[i] = 0;
		CAN1_0X200_BUF[i] = 0;
		CAN1_0X2ff_BUF[i] = 0;
		
		CAN2_0X1ff_BUF[i] = 0;
		CAN2_0X200_BUF[i] = 0;
		CAN2_0X2ff_BUF[i] = 0;
	}
}


///////////////////////////////////////////////////////////////////////////

Motor_speed max = 0;
Motor_speed Max(Motor_speed *pS1)
{
	u8 i = 0;
	Motor_speed max = 0;
	Motor_speed temp = 0; 
	
	for(i = 0; i<4; i++)
	{
		temp = pS1[i]>0 ? pS1[i] : (-pS1[i]);//求绝对值
		
		if(temp>max)
		{
			max = temp;
		}
	}	
	
	if(max<8000)
	{
		max = 8000;
	}
	
	
	return max;
	
}


u8 switch_flag = 1;
Motor_speed speed_buf[4] = {0};
float k_temp;
void DR16_C_Chassis(void)
{
	short int vx, vy;
	short int omega;
	
	RC_Module RCtemp;
	RCtemp = DJI_RC1;
	
	if(RCtemp.Judge.state == WORKING)
	{
//		if(switch_flag)
//		{
//			start_pid = 1;
//		}
		
		//获取数据
		vx = RCtemp.rece_Data.ch2 *8000/660;
		vy = RCtemp.rece_Data.ch3 *8000/660;
		omega = RCtemp.rece_Data.ch0 * 8000/660;
		

		//
		speed_buf[0] = +vx + vy + omega;		//
		speed_buf[1] = +vx - vy + omega;		//
		speed_buf[2] = -vx + vy + omega;		//
		speed_buf[3] = -vx - vy + omega;		//
		
		//越界比例处理
		
		k_temp = (float)(Max(speed_buf));
		k_temp = k_temp/8000;
		
		speed2[0] = (short int)(speed_buf[0]/k_temp);		//
		speed2[1] = (short int)(speed_buf[1]/k_temp);		//
		speed2[2] = (short int)(speed_buf[2]/k_temp);		//
		speed2[3] = (short int)(speed_buf[3]/k_temp);		//
		
		Chassis_set_speed(speed2[0], speed2[1], speed2[2], speed2[3]);
		Chassis_Speed_Update();//计算各自的速度，最后通过CAN发送出去
	}
	else
	{
		//start_pid = 0;
		Chassis_Set_Output(0, 0, 0, 0);
	}
	
}






//////////////////////////////////////////////////
//判断失联相关函数

void RC_JUDGE(void)
{
	u8 rx_error_flag = 0;
	
	Judge_State_Flash(&(DJI_RC1.Judge));
	rx_error_flag = DR_data_check(&(DJI_RC1.rece_Data));
	if(DJI_RC1.Judge.state == DISCONNECTING || rx_error_flag)
	{
		RC_data_Init(&(DJI_RC1.rece_Data));
	}
}



u32 gyro_reset_cnt = 0;

void anoc_SendChar(u8 chr)
{
	USART3->DR = chr;
	
	while((USART3->SR&0x40)==0);
}

void RP_SendToPc(float yaw, float pitch, float roll, float rateYaw, float ratePitch, float rateRoll)
{
	static uint16_t send_cnt = 1;
	uint8_t i;
	uint16_t check_sum = 0;
	uint8_t data_to_pc[29];
	
	data_to_pc[0] = 0xAA;
	data_to_pc[1] = 0xAA;
	data_to_pc[2] = 0x01;
	data_to_pc[3] = 24;
	
	/* 以默认的小端模式发送数据 */
	memcpy(&data_to_pc[4], (uint8_t*)&yaw, 4);
	memcpy(&data_to_pc[8], (uint8_t*)&pitch, 4);
	memcpy(&data_to_pc[12], (uint8_t*)&roll, 4);
	memcpy(&data_to_pc[16], (uint8_t*)&rateYaw, 4);
	memcpy(&data_to_pc[20], (uint8_t*)&ratePitch, 4);
	memcpy(&data_to_pc[24], (uint8_t*)&rateRoll, 4);
	
//	send_cnt++;
//	以下操作会将数据转化成大端模式发送出去
//	data_to_pc[16] = (rateYaw & 0xff00) >> 8;
//	data_to_pc[17] = (rateYaw & 0xff);
//	data_to_pc[18] = (ratePitch & 0xff00) >> 8;
//	data_to_pc[19] = (ratePitch & 0xff);
//	data_to_pc[20] = (rateRoll & 0xff00) >> 8;
//	data_to_pc[21] = (rateRoll & 0xff);
	
	for(i = 0; i < 28; i++) {
		check_sum += data_to_pc[i];
	}
	data_to_pc[28] = check_sum & 0xff;
	
//	USART1_DMA_SendBuf(data_to_pc, 23);
	for(i = 0; i < 29; i++) {
		anoc_SendChar(data_to_pc[i]);
	}
}





