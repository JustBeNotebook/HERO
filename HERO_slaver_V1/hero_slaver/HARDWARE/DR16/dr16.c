#include "dr16.h"

DR16_Module DR16_Data_struct;
RC_Module DJI_RC1;

volatile u8 DR16_DataBuf[2][18];

void RC_Init(u32 bound)
{
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE); //使能DMA1时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
 
	//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
	
	//USART2端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //无上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA3

   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;//偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//接收模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口2

	
  USART_Cmd(USART2, ENABLE);  //使能串口2
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE); //使能串口的DMA模式
	
		
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;//DMA1流5 中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;													//选择通道4
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART2->DR);				//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&DR16_DataBuf[0][0];			//存储器地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;									//外设到存储器模式
	DMA_InitStructure.DMA_BufferSize = 18;																	//单次接收到的帧的数目
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//外设非增量模式							
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;									//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设发送单个数据长度
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//存储器单个接收数据长度
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;													//存储器循环接收模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;									//优先级非常高
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;									//FIFO模式禁止
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;						//FIFO模式阈值
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;							//存储器突发单个传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;			//外设突发单个传输
	
	//根据当前缓冲区状态进行一个切换，然后将另外一个缓冲区地址写入
	//DMA_DoubleBufferModeConfig(DMA1_Stream5,(uint32_t)(&DR16_DataBuf[1][0]), DMA_Memory_0);	//双缓冲区
	//DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE); 													//双缓冲区模式开启
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);															//DMA1初始化
	

	
	DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	
}

u32 cnt2 = 0;


void DMA1_Stream5_IRQHandler(void)
{
	
	cnt2++;
	if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5)!=RESET)//标志在被清零前一直有效
	{
		
		cnt2++;
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);

		if(DMA_GetCurrentMemoryTarget(DMA1_Stream5) == 0)  //存储器0
		{
			get_dr16_data(&(DJI_RC1.rece_Data), DR16_DataBuf[0]);
			DJI_RC1.Judge.cycle_cnt = 0 ;
		}
	}

}



void get_dr16_data(DR16_Module *dr16, volatile unsigned char *pData)
{
	
	short int temp;
	
	temp = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	dr16->ch0 = temp-1024;
	
	temp = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	dr16->ch1 = temp-1024;
	
	temp = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
	dr16->ch2 = temp-1024;
	
	temp = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
  dr16->ch3 = temp-1024;    
	
	
	dr16->sw1 = ((pData[5] >> 4) & 0x000C) >> 2;
	dr16->sw2	= ((pData[5] >> 4) & 0x0003); 
  dr16->mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	dr16->mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8); 
	dr16->mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);     
  dr16->mouse.l = pData[12];
	dr16->mouse.r = pData[13];
	dr16->kb.key_code = ((int16_t)pData[14]);
	
	temp = ((int16_t)pData[16] | ((int16_t)pData[17] << 8)) & 0x07FF;
	dr16->wheel = temp-1024; 
	
}


u8 DR_data_check(DR16_Module *dr16)
{
	if(dr16->ch0 < -660 || dr16->ch0 > 660)
		return 1;
	if(dr16->ch1 < -660 || dr16->ch1 > 660)
		return 1;
	if(dr16->ch2 < -660 || dr16->ch2 > 660)
		return 1;
	if(dr16->ch3 < -660 || dr16->ch3 > 660)
		return 1;
	
	if(dr16->sw1 == 0 || dr16->sw1>3)
		return 1;
	if(dr16->sw2 == 0 || dr16->sw2>3)
		return 1;
	
	return 0;
}

void RC_data_Init(DR16_Module *dr16)
{
	
	dr16->ch0 = 0;
	dr16->ch1 = 0;
	dr16->ch2 = 0;
	dr16->ch3 = 0;
	
	dr16->sw1 = 0;
	dr16->sw2 = 0;
	
	dr16->mouse.x = 0;
	dr16->mouse.y = 0;
	dr16->mouse.z = 0;
	dr16->mouse.r = 0;
	dr16->mouse.l = 0;
	
	dr16->kb.key_code = 0;
	
}

u8 RC_Reconnecting_func(void)
{
//	PID_Set(&(Gimbal.Yaw_Motor.speed_PID), 10, 6, 0, 60, 0, 28000);//速度环PID要配合陀螺仪速度使用
//	PID_Set(&(Gimbal.Yaw_Motor.angle_PID), 8, 0, 0, 0, 0, 1000);
//	PID_Set(&(Gimbal.Pitch_Motor.speed_PID), 11, 1, 0, 350, 3, 28000);
//	PID_Set(&(Gimbal.Pitch_Motor.angle_PID), 10, 1, 0, 60, 5, 1000);
	
	/////////////////////////////////  云台缓慢回中
	
	Gimbal.Yaw_Motor.angle_PID.output_limit = 1000;
	Gimbal.Pitch_Motor.angle_PID.output_limit = 1000;
	
	Gimbal.Yaw_Motor.angle_PID.startfalg = PID_ENABLE;
	Gimbal.Pitch_Motor.angle_PID.startfalg = PID_ENABLE;
	
	Motor_set_angle(&(Gimbal.Pitch_Motor), 2000);
	Motor_set_angle(&(Gimbal.Yaw_Motor), 2500);
	
	Gimbal_PID_Cal(0); //PITCH轴
	Gimbal_PID_Cal(1);//YAW轴电机
	Gimbal_Set_Output(Gimbal.Yaw_Motor.output, Gimbal.Pitch_Motor.output);
	
	CAN1_Send_Msg2(CAN1_0X2ff_BUF, 8, 0x2ff);
	
	Hero_Launcher.Friction.Target_Speed = 1000; // 摩擦轮设置目标为就绪区
	
	if(Gimbal.Yaw_Motor.angle_PID.err <= 20 && Gimbal.Pitch_Motor.angle_PID.err <= 20
		&& Gimbal.Yaw_Motor.angle_PID.err >= -20 && Gimbal.Pitch_Motor.angle_PID.err >= -20)//重新设置
	{
		
		PID_Set(&(Gimbal.Yaw_Motor.speed_PID), 10, 6, 0, 60, 0, 28000);
		PID_Set(&(Gimbal.Yaw_Motor.angle_PID), 9, 1, 0, 60, 0, 8000);
		PID_Set(&(Gimbal.Pitch_Motor.speed_PID), 11, 1, 0, 350, 3, 28000);
		PID_Set(&(Gimbal.Pitch_Motor.angle_PID), 10, 1, 0, 60, 5, 8000);
		
		Gimbal.Pitch_Motor.output = 0;
		Gimbal.Yaw_Motor.output = 0;
		
		Gimbal_Set_Output(Gimbal.Yaw_Motor.output, Gimbal.Pitch_Motor.output);
		
		return 1;
	}
	
	return 0;
}


