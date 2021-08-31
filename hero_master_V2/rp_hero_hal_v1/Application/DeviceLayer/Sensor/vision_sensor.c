/**
 * @file        vision_sensor.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        24-Feb-2021
 * @brief       Device Vision.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "vision_sensor.h"

#include "rp_math.h"
#include "device.h"
#include "vision_potocol.h"

extern void vision_sensor_init(vision_sensor_t *vision_sen);
extern void vision_sensor_update(vision_sensor_t *vision_sen, uint8_t *rxBuf);

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void vision_sensor_check(vision_sensor_t *vision_sen);
static void vision_sensor_heart_beat(vision_sensor_t *vision_sen);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 视觉驱动
drv_uart_t	vision_sensor_driver = {
	.type = DRV_UART1,
	.tx_byte = NULL,
};

// 视觉信息
vision_sensor_info_t 	vision_sensor_info = {
	.offline_max_cnt = 60,
};

// 视觉传感器
vision_sensor_t vision_sensor = {
	.info = &vision_sensor_info,								//数据结构体
	.init = vision_sensor_init,									//传感器初始化
	.update = vision_sensor_update,							//数据更新
	.check = vision_sensor_check,								//数据合理性判断
	.heart_beat = vision_sensor_heart_beat,			//状态更新
	.work_state = DEV_OFFLINE,							//状态查询
	.id = DEV_ID_VISION,
};

uint8_t Vision_SentNum[50];

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	视觉数据检查
 */
static void vision_sensor_check(vision_sensor_t *vision_sen)
{
	
	vision_sen->info->offline_cnt = 0;
}

/**
 *	@brief	视觉心跳包
 */
static void vision_sensor_heart_beat(vision_sensor_t *vision_sen)//毫秒任务
{
	vision_sensor_info_t *vision_info = vision_sen->info;

	vision_info->offline_cnt++;
	if(vision_info->offline_cnt > vision_info->offline_max_cnt)//每次等待一段时间后自动离线
	{
		vision_info->offline_cnt = vision_info->offline_max_cnt;
		vision_sen->work_state = DEV_OFFLINE;
	} 
	else //每次接收成功就清空计数
	{
		/* 离线->在线 */
		if(vision_sen->work_state == DEV_OFFLINE)
			vision_sen->work_state = DEV_ONLINE;
	}
}

/* Exported functions --------------------------------------------------------*/


void VISION_ResetData(vision_sensor_t *vision)//视觉数据清空，一般失联时使用
{

}
