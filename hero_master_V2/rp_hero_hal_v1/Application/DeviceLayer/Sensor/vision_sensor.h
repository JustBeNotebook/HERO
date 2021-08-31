#ifndef RM__VISION__H
#define RM__VISION__H

#include "rp_config.h"

//起始字节，协议固定尾0xA5
#define 	VISION_SOF					(0xA5)

//长度根据协议定义
#define		VISION_LEN_HEADER		3			//帧头长
#define 	VISION_LEN_DATA 		18    //数据段长度，可自定义
#define   VISION_LEN_TAIL			2			//频尾CRC16
#define		VISION_LEN_PACKED		23		//数据包长度，可自定义

#define 	VISION_LEN_TX_PACKED 8

#define    VISION_OFF           (0x00)
#define    VISION_RED_ATTACK    (0x01)
#define    VISION_BLUE_ATTACK   (0x02)
#define    VISION_RED_BASE      (0x03)
#define    VISION_BLUE_BASE     (0x04)
#define    VISION_RED_GETBOMB   (0x05)
#define    VISION_BLUE_GETBOMB  (0x06)


//利用收与发的指令码进行比较，当收发一致时数据可用

//帧头加CRC8校验,保证发送的指令是正确的

//PC收发与STM32收发成镜像关系,以下结构体适用于STM32,PC需稍作修改

typedef __packed struct
{
    //头
    uint8_t		SOF;			//帧头起始位,暂定0xA5
    uint8_t 	CmdID;		//指令
    uint8_t 	CRC8;			//帧头CRC校验,保证发送的指令是正确的
} extVisionSendHeader_t;


/*****************视觉接收包格式********************/
typedef __packed struct
{
    /*头*/
    uint8_t 	SOF;			//帧头起始位,0xA5
    uint8_t 	CmdID;		//指令
    uint8_t 	CRC8;			//帧头CRC校验,保证发送的指令是正确的

    /*数据*/
    float pitch_angle;
    float yaw_angle;
    float distance;
    uint8_t identify_target;// 是否识别到目标	单位：0/1
    uint8_t buff_change_armor_four;	// 是否切换到第四块装甲板		单位：0/1
    uint8_t identify_buff;	// 是否识别到Buff	单位：0/1
    uint8_t identify_too_close;	// 目标距离过近	单位：0/1
    uint8_t anti_gyro;	// 是否识别到小陀螺	单位：0/1
    uint8_t	anti_gyro_change_armor;	// 是否在反陀螺状态下切换装甲板	单位：0/1

    /*尾*/
    uint16_t CRC16;
    uint8_t flag;
} extVisionRecvData_t;

typedef __packed struct vision_sensor_info_struct {
	/*头*/
    uint8_t 	SOF;			//帧头起始位,0xA5
    uint8_t 	CmdID;		//指令
    uint8_t 	CRC8;			//帧头CRC校验,保证发送的指令是正确的

    /*数据*/
    float pitch_angle;
    float yaw_angle;
    float distance;
    uint8_t identify_target;// 是否识别到目标	单位：0/1
    uint8_t buff_change_armor_four;	// 是否切换到第四块装甲板		单位：0/1
    uint8_t identify_buff;	// 是否识别到Buff	单位：0/1
    uint8_t identify_too_close;	// 目标距离过近	单位：0/1
    uint8_t anti_gyro;	// 是否识别到小陀螺	单位：0/1
    uint8_t	anti_gyro_change_armor;	// 是否在反陀螺状态下切换装甲板	单位：0/1

    /*尾*/
    uint16_t CRC16;
    uint8_t flag;
		uint16_t zhen;
	
		int16_t		offline_cnt;
		int16_t		offline_max_cnt;
} vision_sensor_info_t;


/**
  * @brief  发送视觉指令
  * @param  CmdID
  * @retval void
  * @attention  按协议打包好数据发送
  *				CmdID   0x00   关闭视觉
  *				CmdID   0x01   识别红色装甲
  *				CmdID   0x02   识别蓝色装甲
  *				CmdID   0x03   小符
  *				CmdID   0x04   大符
  */
typedef __packed struct vision_sent_info_struct {
	/*头*/
    uint8_t 	SOF;			//帧头起始位,0xA5
    uint8_t 	CmdID;		//指令
    uint8_t 	CRC8;			//帧头CRC校验,保证发送的指令是正确的

    /*数据*/
    uint8_t    sentry_mode;   		//击打哨兵模式
    uint8_t		 base_far_mode;     //远程吊射基地模式
    uint8_t    base_near_mode;    //近程吊射基地模式(也包括吊前哨站)
    uint8_t    fric_speed; 				//子弹射速(直接分几档)
		uint8_t    user5;   		//击打哨兵模式
    uint8_t		 user6;     //远程吊射基地模式
    uint8_t    user7;    //近程吊射基地模式(也包括吊前哨站)
    uint8_t    user8; 				//子弹射速(直接分几档)
		uint8_t    user9;   		//击打哨兵模式
    uint8_t		 user10;     //远程吊射基地模式
    uint8_t    user11;    //近程吊射基地模式(也包括吊前哨站)
    uint8_t    user12; 				//子弹射速(直接分几档)
		uint8_t    user13;   		//击打哨兵模式
    uint8_t		 user14;     //远程吊射基地模式
    uint8_t    user15;    //近程吊射基地模式(也包括吊前哨站)
    uint8_t    user16; 				//子弹射速(直接分几档)
		uint8_t    user17;   		//击打哨兵模式
    uint8_t		 user18;     //远程吊射基地模式



    /*尾*/
    uint16_t CRC16;

} vision_sent_info_t;


typedef struct vision_sensor_struct {
	vision_sensor_info_t	*info;
	vision_sent_info_t		*sent_info;
	uint8_t  sent[VISION_LEN_PACKED];
	drv_uart_t			*driver;
	void				(*init)(struct vision_sensor_struct *self);
	void				(*update)(struct vision_sensor_struct *self, uint8_t *rxBuf);
	void				(*check)(struct vision_sensor_struct *self);	
	void				(*heart_beat)(struct vision_sensor_struct *self);
	void 				(*sent_self)(struct vision_sensor_struct *self, uint8_t *txBuf);
	dev_work_state_t	work_state;
	dev_errno_t			errno;
	dev_id_t			id;
} vision_sensor_t;
/*****************视觉发送包格式********************/


extern float theta_absolute;
extern vision_sensor_t vision_sensor;

//关于如何写入CRC校验值
//我们可以直接利用官方给的CRC代码

//注意,CRC8和CRC16所占字节不一样,8为一个字节,16为2个字节

//写入    CRC8 调用    Append_CRC8_Check_Sum( param1, param2)
//其中 param1代表写好了帧头数据的数组(帧头后的数据还没写没有关系),
//     param2代表CRC8写入后数据长度,我们定义的是头的最后一位,也就是3

//写入    CRC16 调用   Append_CRC16_Check_Sum( param3, param4)
//其中 param3代表写好了   帧头 + 数据  的数组(跟上面是同一个数组)
//     param4代表CRC16写入后数据长度,我们定义的整个数据长度是22,所以是22

/*----------------------------------------------------------*/


extern uint8_t Vision_SentNum[50];
extern uint8_t Vision_attack_Color;

void Vision_Read_Data(uint8_t *ReadFormUsart);
void Vision_Send_Data(uint8_t CmdID);

void Usart1_Sent_Byte(uint8_t ch);
void Vision_Sent(void);
void Vision_Init(void);

/******************各种辅助小函数*******************/
void Vision_Ctrl(void);
void Vision_Auto_Attack_Off(void);
void Vision_Auto_Attack_Ctrl(void);

void Vision_Error_Yaw(float *error);
void Vision_Error_Pitch(float *error);
void Vision_Error_Angle_Yaw(float *error);
void Vision_Error_Angle_Pitch(float *error);
void Vision_Get_Distance(float *distance);

bool Vision_IF_Updata(void);
void Vision_Clean_Updata_Flag(void);
int16_t Get_Attack_attention_Mode(void);

float auto_aim(void);
float auto_get_speed(float angle);
float auto_get_accel(float speed);

#endif


