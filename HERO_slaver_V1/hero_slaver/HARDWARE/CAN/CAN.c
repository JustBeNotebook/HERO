#include "CAN.h"



//CAN初始化  初始化顺序：IO配置→CAN寄存器配置→过滤器配置→中断配置
//tsjw 重新同步跳跃单元， 1~4tq
//tbs2 时间段2的时间单元  1~8tq
//tbs1 时间段1的时间单元  1~16tq
//brp  波特率分频器 1~1024（config配置里面已经包含减一操作，所以此处不需要减一）
//42/（1+4+9）/3 = 1M
//mode CAN_Mode_Normal 普通模式 CAN_Mode_Silent_LoopBack静默回环模式
u8 CAN1_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode)// CAN模式初始化
{
	GPIO_InitTypeDef CAN_Handle;
	CAN_InitTypeDef CAN_Init_Handle;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_Initure;
	
//旧主控CAN引脚	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
//	
//	
//	CAN_Handle.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
//	CAN_Handle.GPIO_Mode = GPIO_Mode_AF;//复用模式
//	CAN_Handle.GPIO_OType = GPIO_OType_PP;//开漏输出
//	CAN_Handle.GPIO_PuPd = GPIO_PuPd_UP;//上拉电阻
//	CAN_Handle.GPIO_Speed = GPIO_Speed_100MHz;//
//	GPIO_Init(GPIOA, &CAN_Handle);
//	
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);//复用CAN1输出
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);//复用CAN1输出
	
	
//新主控CAN引脚
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	
	
	CAN_Handle.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	CAN_Handle.GPIO_Mode = GPIO_Mode_AF;//复用模式
	CAN_Handle.GPIO_OType = GPIO_OType_PP;//开漏输出
	CAN_Handle.GPIO_PuPd = GPIO_PuPd_UP;//上拉电阻
	CAN_Handle.GPIO_Speed = GPIO_Speed_100MHz;//
	GPIO_Init(GPIOD, &CAN_Handle);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);//复用CAN1输出
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);//复用CAN1输出
	
	//CAN单元设置
	CAN_Init_Handle.CAN_TTCM = DISABLE; //非时间触发通信模式
	CAN_Init_Handle.CAN_ABOM = ENABLE;	//自动离线恢复
	CAN_Init_Handle.CAN_AWUM = DISABLE; //睡眠模式通过软件唤醒（清除CAN->MCR的标志位）
	CAN_Init_Handle.CAN_NART = DISABLE;  //禁止报文自动传送
	CAN_Init_Handle.CAN_RFLM = DISABLE; //报文不锁定
	CAN_Init_Handle.CAN_TXFP = ENABLE; //优先级由报文标识符表示
	CAN_Init_Handle.CAN_Mode = mode;		//模式设置为初始化时的模式
	//CAN波特率设置
	CAN_Init_Handle.CAN_SJW = tsjw;   //设置重新同步时间长度4
	CAN_Init_Handle.CAN_BS2 = tbs2;		//设置时间段2时间长度9
	CAN_Init_Handle.CAN_BS1 = tbs1;		//设置时间段1时间长度1
	CAN_Init_Handle.CAN_Prescaler = brp;//设置波特率分频器3
	CAN_Init(CAN1, &CAN_Init_Handle);			//配置CAN的寄存器
	
	//配置过滤器
	CAN_FilterInitStructure.CAN_FilterNumber = 0;												//过滤器0
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;			//屏蔽位模式/掩码模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;		//设置过滤器为32位模式
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0X0000;									//过滤器高十六位设置为0
	CAN_FilterInitStructure.CAN_FilterIdLow = 0X0000;										//过滤器低十六位设置为0
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0X0000;							//过滤器掩码高十六位为0
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0X0000;								//过滤器掩码低十六位为0
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//过滤器关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;							//设置CAN启动
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

	NVIC_Initure.NVIC_IRQChannel = CAN1_RX0_IRQn;				//设置中断向量
	NVIC_Initure.NVIC_IRQChannelPreemptionPriority = 0;	//设置主优先级为1
	NVIC_Initure.NVIC_IRQChannelSubPriority = 1;				//设置从优先级为0
	NVIC_Initure.NVIC_IRQChannelCmd = ENABLE;						//设置中断状态
	NVIC_Init(&NVIC_Initure);

	return 1;
}

//CAN初始化  初始化顺序：IO配置→CAN寄存器配置→过滤器配置→中断配置
//tsjw 重新同步跳跃单元， 1~4tq
//tbs2 时间段2的时间单元  1~8tq
//tbs1 时间段1的时间单元  1~16tq
//brp  波特率分频器 1~1024（config配置里面已经包含减一操作，所以此处不需要减一）
//42/（1+4+9）/3 = 1M
//mode CAN_Mode_Normal 普通模式 CAN_Mode_Silent_LoopBack静默回环模式
u8 CAN2_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode)// CAN模式初始化
{
	GPIO_InitTypeDef CAN_Handle;
	CAN_InitTypeDef CAN_Init_Handle;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_Initure;

	
//旧主控CAN2接口
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);
//	
//	CAN_Handle.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
//	CAN_Handle.GPIO_Mode = GPIO_Mode_AF;//复用模式
//	CAN_Handle.GPIO_OType = GPIO_OType_PP;//开漏输出
//	CAN_Handle.GPIO_PuPd = GPIO_PuPd_UP;//上拉电阻
//	CAN_Handle.GPIO_Speed = GPIO_Speed_100MHz;//
//	GPIO_Init(GPIOB, &CAN_Handle);
//	
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);//复用CAN2输出
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);//复用CAN2输出
	
//新主控CAN2接口
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);
	
	
	CAN_Handle.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5;
	CAN_Handle.GPIO_Mode = GPIO_Mode_AF;//复用模式
	CAN_Handle.GPIO_OType = GPIO_OType_PP;//开漏输出
	CAN_Handle.GPIO_PuPd = GPIO_PuPd_UP;//上拉电阻
	CAN_Handle.GPIO_Speed = GPIO_Speed_100MHz;//
	GPIO_Init(GPIOB, &CAN_Handle);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);//复用CAN2输出
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);//复用CAN2输出
	
	
	//CAN单元设置
	CAN_Init_Handle.CAN_TTCM = DISABLE; //非时间触发通信模式
	CAN_Init_Handle.CAN_ABOM = ENABLE;	//自动离线恢复
	CAN_Init_Handle.CAN_AWUM = DISABLE; //睡眠模式通过软件唤醒（清除CAN->MCR的标志位）
	CAN_Init_Handle.CAN_NART = DISABLE;  //禁止报文自动传送
	CAN_Init_Handle.CAN_RFLM = DISABLE; //报文不锁定
	CAN_Init_Handle.CAN_TXFP = ENABLE; //优先级由报文标识符表示
	CAN_Init_Handle.CAN_Mode = mode;		//模式设置为初始化时的模式
	//CAN波特率设置
	CAN_Init_Handle.CAN_SJW = tsjw;   //设置重新同步时间长度4
	CAN_Init_Handle.CAN_BS2 = tbs2;		//设置时间段2时间长度9
	CAN_Init_Handle.CAN_BS1 = tbs1;		//设置时间段1时间长度1
	CAN_Init_Handle.CAN_Prescaler = brp;//设置波特率分频器3
	CAN_Init(CAN2, &CAN_Init_Handle);			//配置CAN的寄存器
	
	//配置过滤器
	CAN_FilterInitStructure.CAN_FilterNumber = 14;												//过滤器0
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;			//屏蔽位模式/掩码模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;		//设置过滤器为32位模式
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0X0000;									//过滤器高十六位设置为0
	CAN_FilterInitStructure.CAN_FilterIdLow = 0X0000;										//过滤器低十六位设置为0
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0X0000;							//过滤器掩码高十六位为0
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0X0000;								//过滤器掩码低十六位为0
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//过滤器关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;							//设置CAN启动
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);

	NVIC_Initure.NVIC_IRQChannel = CAN2_RX0_IRQn;				//设置中断向量
	NVIC_Initure.NVIC_IRQChannelPreemptionPriority = 0;	//设置主优先级为1
	NVIC_Initure.NVIC_IRQChannelSubPriority = 0;				//设置从优先级为0
	NVIC_Initure.NVIC_IRQChannelCmd = ENABLE;						//设置中断状态
	NVIC_Init(&NVIC_Initure);

	return 1;
}

int vola;
CanRxMsg RxMessage1;
u8 len = 0;
u32 cnt = 0;
//CAN1 信箱0接收中断
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg Rxbuf;//CAN通信可能出现的毛病
	
	if(CAN_GetITStatus(CAN1, CAN_IT_FMP0)!=RESET)
	{
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage1);		//接收can数据
		
		Rxbuf = RxMessage1;
		
		//////////////////////////1st begin 在这里对接收数据的ID进行判断
		if((Rxbuf.StdId&0x200) != 0)
		{
			cnt++;
			speed_analyze(Rxbuf.StdId, Rxbuf.Data);
		}

		if(Rxbuf.StdId == 0x150)
		{
			PITCH_YAW_Analyze(Rxbuf.Data);
		}
		if(Rxbuf.StdId == 0x151)
		{
			Slaver_flag.global_clip = Rxbuf.Data[0];
			Slaver_flag.global_fiction = Rxbuf.Data[1];
			Slaver_flag.global_auto_aim = Rxbuf.Data[2];
			Slaver_flag.global_spin = Rxbuf.Data[3];
			Slaver_flag.global_anti_top = Rxbuf.Data[4];
			Slaver_flag.global_twist = Rxbuf.Data[5];
			Slaver_flag.user1 = Rxbuf.Data[6];
			Slaver_flag.shift_rush = Rxbuf.Data[7];
		}
		/////////////////////////////////1st end 在这里对接收数据的ID进行判断
		
		CAN1->RF0R |= (0x01<<5);//置1释放邮箱
	}
}


int can2_cnt = 0;
CanRxMsg RxMessage2;
void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg Rxbuf;//CAN通信可能出现的毛病
	
	if(CAN_GetITStatus(CAN2, CAN_IT_FMP0)!=RESET)
	{
		CAN_Receive(CAN2, CAN_FIFO0, &RxMessage2);		//接收can数据
		
		Rxbuf = RxMessage2;
		
		//////////////////////////2sc begin 在这里对接收数据的ID进行判断
		if((Rxbuf.StdId&0x200) != 0)
		{
			can2_cnt++;
			Motor_analyze2(Rxbuf.StdId, Rxbuf.Data);
		}
		if(Rxbuf.StdId == 0x502)
		{
			memcpy(&SC_R_DATA, Rxbuf.Data, 8);
		}
		//////////////////////////2sc end 在这里对接收数据的ID进行判断
		
		CAN2->RF0R |= (0x01<<5);//置1释放邮箱
	}


}



//CAN 标准帧发送数据函数
//msg 待发送数据的首地址
//len 待发送数据的长度
//stdid 发送标识符 
//return 0, 发送成功
//return 1, 发送失败
u8 CAN1_Send_Msg2(u8 *msg, u8 len, u16 stdid)
{
	u8 mbox;
	u16 i=0;
	
	CanTxMsg TxMessage;								//CAN发送数据结构体，可用于日后写协议参考用
	TxMessage.StdId = stdid;						//标准ID/标识符 虽然没标出来，
	TxMessage.ExtId = 0x00;						//设置扩展标识符
	TxMessage.IDE = CAN_Id_Standard;	//标准帧
	TxMessage.RTR = CAN_RTR_Data;			//数据帧
	TxMessage.DLC = len;							//要发送的数据长度
	for(i=0; i<len; i++)							//将待发送数据收到缓冲区里
	{
		TxMessage.Data[i] = msg[i];
	}
	
	mbox = CAN_Transmit(CAN1, &TxMessage);//CAN 发送函数
	
	i = 0;
	while((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i<0xfff))
	{
		i++;
	}
	if(i == 0xfff)
		return 1;
	else 
		return 0;
	
}

u8 CAN2_Send_Msg2(u8 *msg, u8 len, u16 stdid)
{
	u8 mbox;
	u16 i=0;
	
	CanTxMsg TxMessage;								//CAN发送数据结构体，可用于日后写协议参考用
	TxMessage.StdId = stdid;						//标准ID/标识符 虽然没标出来，
	TxMessage.ExtId = 0x00;						//设置扩展标识符
	TxMessage.IDE = CAN_Id_Standard;	//标准帧
	TxMessage.RTR = CAN_RTR_Data;			//数据帧
	TxMessage.DLC = len;							//要发送的数据长度
	for(i=0; i<len; i++)							//将待发送数据收到缓冲区里
	{
		TxMessage.Data[i] = msg[i];
	}
	
	mbox = CAN_Transmit(CAN2, &TxMessage);//CAN 发送函数
	
	i = 0;
	while((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i<0xfff))
	{
		i++;
	}
	if(i == 0xfff)
		return 1;
	else 
		return 0;
}



