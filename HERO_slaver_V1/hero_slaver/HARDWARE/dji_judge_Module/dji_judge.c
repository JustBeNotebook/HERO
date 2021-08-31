#include "dji_judge.h"



/* TX */
#define    GPIO_TX                   GPIOC
#define    GPIO_PIN_TX               GPIO_Pin_12
#define    GPIO_PINSOURCE_TX         GPIO_PinSource12
#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOC

/* RX */
#define    GPIO_RX                   GPIOD
#define    GPIO_PIN_RX               GPIO_Pin_2
#define    GPIO_PINSOURCE_RX         GPIO_PinSource2
#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOD

/* if use DMA */
#define    DMA1_Stream_RX            DMA1_Stream0

#define    COM5_PACKAGE_HEADER       JUDGE_FRAME_HEADER


/*****************系统数据定义**********************/
ext_game_state_t       				GameState;					//0x0001
ext_game_result_t            		GameResult;					//0x0002
ext_game_robot_HP_t          GameRobotSurvivors;			//0x0003
ext_dart_status_t          GameRobotmissile;			//0x0004
ext_ICRA_buff_debuff_zone_status_t  Game_ICRA_buff ; //0x0005
 
ext_event_data_t        			EventData;					//0x0101
ext_supply_projectile_action_t		SupplyProjectileAction;		//0x0102
ext_referee_warning_t		Supplywarming;	//0x0104
ext_dart_remaining_time_t  dart_remaining_time_t ;// 0x0105

ext_game_robot_state_t			  	GameRobotStat;				//0x0201    
ext_power_heat_data_t		  		PowerHeatData;				//0x0202
ext_game_robot_pos_t				GameRobotPos;				//0x0203
ext_buff_musk_t						BuffMusk;					//0x0204
aerial_robot_energy_t				AerialRobotEnergy;			//0x0205
ext_robot_hurt_t					RobotHurt;					//0x0206
ext_shoot_data_t					ShootData;					//0x0207
ext_bullet_remaining_t   bullet_remaining_t; //0x0208

xFrameHeader              FrameHeader;		//发送帧头信息

bool Judge_Data_TF = FALSE;//裁判数据是否可用,辅助函数调用
uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID

/**************裁判系统数据辅助****************/
uint16_t ShootNum;//统计发弹量,0x0003触发一次则认为发射了一颗
bool Hurt_Data_Update = FALSE;//装甲板伤害数据是否更新,每受一次伤害置TRUE,然后立即置FALSE,给底盘闪避用


/**
  * @brief  读取裁判数据,loop中循环调用此函数来读取数据
  * @param  缓存数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验,无误再写入数据
  */
bool Judege_read_data(u8 *ReadFromUsart )
{
	bool retval_tf = FALSE;//数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误
	
	uint16_t judge_length;//统计一帧数据长度 
	
	int CmdID = 0;//数据命令码解析	
	
	if(ReadFromUsart == 0)
	{
		return -1;
	}
	
	memcpy(&FrameHeader,ReadFromUsart,LEN_HEADER);   //储存帧头数据
	
	if(ReadFromUsart[SOF] == JUDGE_FRAME_HEADER)                   //判断帧头是否为0xa5
	{
		if(Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)  //帧头CRC校验
		{
			judge_length = ReadFromUsart[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;	//统计一帧数据长度,用于CR16校验
			
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)//帧尾CRC16校验
			{
				retval_tf = TRUE;//数据可用
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				
				switch(CmdID)
				{
					case ID_game_state:     //0x0001
							 memcpy(&GameState, (ReadFromUsart + DATA), LEN_game_state);
							 break;
					
					case ID_game_result:    //0x0002
							 memcpy(&GameResult, (ReadFromUsart + DATA), LEN_game_result);
							 break;
					
					case ID_game_robot_survivors:    //0x0003
							 memcpy(&GameRobotSurvivors, (ReadFromUsart + DATA), LEN_game_robot_survivors);
							 break;
					
					case ID_game_missile_state:    //0x0004
							 memcpy(&GameRobotmissile, (ReadFromUsart + DATA), LED_game_missile_state);
							 break;
					
					case ID_game_buff:    //0x0005
							 memcpy(&Game_ICRA_buff, (ReadFromUsart + DATA), LED_game_buff);
							 break;
					
					case ID_event_data:    //0x0101
							 memcpy(&EventData, (ReadFromUsart + DATA), LEN_event_data);
							 break;
					
					case ID_supply_projectile_action:    //0x0102
							 memcpy(&SupplyProjectileAction, (ReadFromUsart + DATA), LEN_supply_projectile_action);
							 break;
					
					case ID_supply_warm:    //0x0104
							 memcpy(&Supplywarming, (ReadFromUsart + DATA), LEN_supply_warm);
							 break;
					
					
					case ID_missile_shoot_time:    //0x0105
							 memcpy(&dart_remaining_time_t, (ReadFromUsart + DATA), LEN_missile_shoot_time);
							 break;
					
					case ID_game_robot_state:    //0x0201
							 memcpy(&GameRobotStat, (ReadFromUsart + DATA), LEN_game_robot_state);
							 break;
				
					case ID_power_heat_data:    //0x0202
							 memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
							 break;
					
					case ID_game_robot_pos:    //0x0203
							 memcpy(&GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
							 break;
					
					case ID_buff_musk:    //0x0204
							 memcpy(&BuffMusk, (ReadFromUsart + DATA), LEN_buff_musk);
							 break;
					
					case ID_aerial_robot_energy:    //0x0205
							 memcpy(&AerialRobotEnergy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
							 break;
					
					case ID_robot_hurt:      			//0x0206
							memcpy(&RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
							if(RobotHurt.hurt_type == 0)//非装甲板离线造成伤害
							{	
								Hurt_Data_Update = TRUE;
							}//装甲数据每更新一次则判定为受到一次伤害
							break;
					case ID_shoot_data:      			//0x0207
							 memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
							 
					     break;	
					
					case ID_shoot_num:    //0x0208
							 memcpy(&bullet_remaining_t, (ReadFromUsart + DATA), LEN_shoot_num);
							 break;
					
					
				}
					
			}
		}
		//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
		if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//如果一个数据包出现了多帧数据,则再次读取
			Judege_read_data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//辅助函数用
	}
	else		//只要CRC16校验不通过就为FALSE
	{
		Judge_Data_TF = FALSE;//辅助函数用
	}	
	return retval_tf;
}

//裁判系统发过来的数据暂存在这里
uint8_t  Judge_Buffer[ JUDGE_BUFFER_LEN ] = {0};

int Usart5_Clean_IDLE_Flag = 0;
DMA_InitTypeDef xCom5DMAInit;
/***************************裁判系统串口初始化***********************************/
void UART5_Init( void )
{
	USART_InitTypeDef  xUsartInit;
	GPIO_InitTypeDef   xGpioInit;
	NVIC_InitTypeDef   xNvicInit;

	RCC_AHB1PeriphClockCmd( RCC_AHB1PERIPH_GPIO_TX | RCC_AHB1PERIPH_GPIO_RX, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART5, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA1, ENABLE );
	
	GPIO_PinAFConfig( GPIO_TX, GPIO_PINSOURCE_TX, GPIO_AF_UART5 );
	GPIO_PinAFConfig( GPIO_RX, GPIO_PINSOURCE_RX, GPIO_AF_UART5 ); 

	xGpioInit.GPIO_Pin   = GPIO_PIN_TX;
	xGpioInit.GPIO_Mode  = GPIO_Mode_AF;
	xGpioInit.GPIO_OType = GPIO_OType_PP;
	xGpioInit.GPIO_Speed = GPIO_Speed_100MHz;
	xGpioInit.GPIO_PuPd  = GPIO_PuPd_UP;

	GPIO_Init( GPIO_TX, &xGpioInit );

	xGpioInit.GPIO_Pin = GPIO_PIN_RX;
	GPIO_Init( GPIO_RX, &xGpioInit );

	xUsartInit.USART_BaudRate            = 115200;   
	xUsartInit.USART_WordLength          = USART_WordLength_8b;
	xUsartInit.USART_StopBits            = USART_StopBits_1;
	xUsartInit.USART_Parity              = USART_Parity_No;
	xUsartInit.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
	xUsartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_Init( UART5, &xUsartInit );
	USART_Cmd( UART5, ENABLE );
	
	USART_ITConfig( UART5, USART_IT_IDLE, ENABLE  ); //注意要配置成串口空闲中断 

	USART_DMACmd( UART5, USART_DMAReq_Rx, ENABLE );
	USART_DMACmd( UART5, USART_DMAReq_Tx, ENABLE );
	
	UART5_DMA_Init( );//初始化usart5的DMA
	
	xNvicInit.NVIC_IRQChannel                    = UART5_IRQn;
	xNvicInit.NVIC_IRQChannelPreemptionPriority  = 0;
	xNvicInit.NVIC_IRQChannelSubPriority         = 0;
	xNvicInit.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init( &xNvicInit );
}

//DMA初始化
void UART5_DMA_Init( void )
{		
	DMA_DeInit( DMA1_Stream_RX );
	xCom5DMAInit.DMA_Channel = DMA_Channel_4;

	xCom5DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;//方向外设到存储器

	xCom5DMAInit.DMA_PeripheralBaseAddr  = (uint32_t)&(UART5->DR);
	xCom5DMAInit.DMA_Memory0BaseAddr     = (uint32_t)Judge_Buffer;
	xCom5DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
	xCom5DMAInit.DMA_BufferSize = 100;
	xCom5DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	xCom5DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
	xCom5DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	xCom5DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	xCom5DMAInit.DMA_Mode = DMA_Mode_Circular;
	xCom5DMAInit.DMA_Priority = DMA_Priority_VeryHigh;
	xCom5DMAInit.DMA_FIFOMode = DMA_FIFOMode_Enable;
	xCom5DMAInit.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	xCom5DMAInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	xCom5DMAInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init( DMA1_Stream_RX, &xCom5DMAInit );	
	DMA_Cmd( DMA1_Stream_RX, ENABLE);  //stream0   GameRobotStat.robot_id  GameRobotStat.
}


void UART5_IRQHandler( void )
{
	if(USART_GetITStatus(UART5,USART_IT_IDLE)!=RESET)//检测到空闲线路
	{		
		//根据ST官方手册,读一下SR和DR寄存器,IDLE才能再次使用,否则会一直进入中断,就会跟串口接收中断没区别
		Usart5_Clean_IDLE_Flag = UART5->SR ;
		Usart5_Clean_IDLE_Flag = UART5->DR ;
		
		DMA_Cmd(DMA1_Stream0,DISABLE);
		
		Usart5_Clean_IDLE_Flag = JUDGE_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);

		Judege_read_data(Judge_Buffer);		//读取裁判系统数据
		
		memset(Judge_Buffer, 0, 200);
		
		DMA_ClearFlag(DMA1_Stream0, DMA_IT_TCIF0);
		DMA1_Stream0->NDTR = 200;	
		DMA_Cmd(DMA1_Stream0,ENABLE);
	}
}




/**
  * @brief  串口一次发送一个字节数据
  * @param  自己打包好的要发给裁判的数据
  * @retval void
  * @attention  串口移位发送
  */
void UART5_SendChar(uint8_t cData)
{
	while (USART_GetFlagStatus( UART5, USART_FLAG_TC ) == RESET);
	
	USART_SendData( UART5, cData );   
}



