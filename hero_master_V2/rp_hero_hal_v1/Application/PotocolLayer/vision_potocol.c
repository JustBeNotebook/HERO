
/**
 * @file        rc_potocol.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-September-2020
 * @brief       DT7&DR16 Rc Potocol.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "vision_potocol.h"
#include "rc_sensor.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include "string.h"
#include "usart.h"
#include "rc_potocol.h"
#include "crc.h"
#include "judge.h"
#include "vision_sensor.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void vision_sensor_init(vision_sensor_t *rc_sen)
{
	// 初始化为离线状态
	rc_sen->info->offline_cnt = rc_sen->info->offline_max_cnt + 1;
	rc_sen->work_state = DEV_OFFLINE;
	
	if(rc_sen->id == DEV_ID_RC)
		rc_sen->errno = NONE_ERR;
	else
		rc_sen->errno = DEV_ID_ERR;
}

uint8_t anti_sentry_mode_flag = 0;
void Anti_sentry_mode(void)
{
	static uint8_t press_flag = 0;
	
	if(IF_KEY_PRESSED_B && press_flag == 0)
	{
		if(anti_sentry_mode_flag == 1)
		{
			anti_sentry_mode_flag = 0;
		}
		else 
		{
			anti_sentry_mode_flag = 1;
		}
		press_flag = 1;
	}
	else if(IF_KEY_PRESSED_B == 0)
	{
		if(press_flag == 1)
		{
			press_flag = 0;
		}
	}
}

/**
 *	@brief	视觉数据解析协议
 */
uint32_t zhen_usart = 0;
uint32_t zhen_usart_temp = 0;
uint32_t zhen_tick = 0;
uint32_t last_time_now = 0;
int vision_interval = 0;
extern moving_Average_Filter zhen_time_ms;
void vision_sensor_update(vision_sensor_t *vision_sen, uint8_t *rxBuf)
{
	uint8_t pot = 0;
	static uint16_t zhen_temp = 0;

	uint32_t time_now = osKernelSysTick();
	
	vision_interval = time_now-last_time_now;
	average_add(&zhen_time_ms, vision_interval);
	
	if(zhen_tick<time_now)
	{
		zhen_tick = time_now+1000;
		vision_sen->info->zhen = zhen_temp;
		zhen_usart = zhen_usart_temp;
		zhen_temp = 0;
		zhen_usart_temp = 0;
	}
	zhen_usart_temp++;
	
	if(rxBuf[pot] == 0xa5)
	{
			if(Verify_CRC8_Check_Sum(&rxBuf[pot], VISION_LEN_HEADER))
			{
					if(Verify_CRC16_Check_Sum(&rxBuf[pot], VISION_LEN_PACKED))
					{
							memcpy((void*)(vision_sen->info), (const void*)(&rxBuf[pot]), VISION_LEN_PACKED);
							
							if(vision_sen->info->distance <= 1 && 0)
							{
								vision_sen->info->flag = 0;
								vision_sen->info->identify_target = 0;
							}
							else
							{
								vision_sen->info->flag = vision_sen->info->identify_target;
								if(vision_sen->info->flag == 1)
								{
									zhen_temp++;
								}
							}
							//last_y_angle = Gimbal.yaw;
					}
			}
	}
	
	last_time_now = time_now;
}

void Usart1_Sent_Byte(uint8_t ch)
{
    USART1->DR = ch;
    while((USART1->SR&0x40)==0);
}


uint8_t Vision_attack_Color = 0;
void Vision_Sent(void)
{
	uint8_t i = 0;
	
	Vision_SentNum[0] = 0xa5;
	
	if(Vision_SentNum[1] == 0 || Vision_SentNum[1]>2)//初始化时选为蓝色，其余保留上一次
	{
		Vision_SentNum[1] = 1;
	}
	else
	{
		Vision_SentNum[1] = Vision_SentNum[1];
	}
	
	Vision_SentNum[3] = VISION_BLUE_ATTACK;
	
	//调试用
	if(sys.remote_mode == KEY)
	{
		if(RC_SW1_VALUE == 1)
		{
			Vision_SentNum[3] = VISION_RED_ATTACK;
			Vision_attack_Color = VISION_RED_ATTACK;
		}
		else if(RC_SW1_VALUE == 2)
		{
			Vision_SentNum[3] = VISION_BLUE_ATTACK;
			Vision_attack_Color = VISION_BLUE_ATTACK;
		}
		else if(RC_SW1_VALUE == 3)
		{
			if(Judge_Hero.robot_state.robot_id>50)
			{
				Vision_SentNum[3] = VISION_RED_ATTACK;
				Vision_attack_Color = VISION_RED_ATTACK;
			}
			else 
			{
				Vision_SentNum[3] = VISION_BLUE_ATTACK;
				Vision_attack_Color = VISION_BLUE_ATTACK;
			}
			
			//禁用哨兵模式
			//Vision_SentNum[4] = anti_sentry_mode_flag;
		}
	}


	
	
	Append_CRC8_Check_Sum(Vision_SentNum, 3);
	Append_CRC16_Check_Sum(Vision_SentNum,VISION_LEN_TX_PACKED);
	
	HAL_UART_Transmit(&huart1, Vision_SentNum, VISION_LEN_TX_PACKED, 1);
}

/**
 *	@brief	在串口1中解析遥控数据协议
 */
void USART1_rxDataHandler(uint8_t *rxBuf)
{
	// 更新遥控数据
	vision_sensor.update(&vision_sensor, rxBuf);
	vision_sensor.check(&vision_sensor);
}

void USART1_rxDataHandler(uint8_t *rxBuf);

