/**
 * @file        judge_sensor.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        5-November-2020
 * @brief       Device Judge.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "judge_sensor.h"
#include "device.h"
#include "drv_uart.h"

/* Exported functions --------------------------------------------------------*/
extern void judge_sensor_init(judge_sensor_t *judge_sen);
extern void judge_sensor_update(judge_sensor_t *judge_sen, uint8_t *rxBuf);

/* Private macro -------------------------------------------------------------*/
#define JUDGE_OFFLINE_MAX_CNT	100
/* Private function prototypes -----------------------------------------------*/
static void judge_sensor_check(judge_sensor_t *judge_sen);
static void judge_sensor_heart_beat(judge_sensor_t *judge_sen);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// 裁判系统信息
judge_info_t 	judge_info = {
	.offline_max_cnt = JUDGE_OFFLINE_MAX_CNT,
};

// 裁判系统
judge_sensor_t	judge_sensor = {
	.info = &judge_info,
	.init = judge_sensor_init,
	.update = judge_sensor_update,
	.check = judge_sensor_check,
	.heart_beat = judge_sensor_heart_beat,
	.work_state = DEV_OFFLINE,
	.errno = NONE_ERR,
	.id = DEV_ID_JUDGE,
};
/* Exported variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	裁判系统数据检查
 */
static void judge_sensor_check(judge_sensor_t *judge_sen)
{
	judge_info_t *judge_info = judge_sen->info;
}

/**
 *	@brief	裁判系统心跳包
 */
static void judge_sensor_heart_beat(judge_sensor_t *judge_sen)
{
	judge_info_t *judge_info = judge_sen->info;

	judge_info->offline_cnt++;
	if(judge_info->offline_cnt > judge_info->offline_max_cnt) {
		judge_info->offline_cnt = judge_info->offline_max_cnt;
		judge_sen->work_state = DEV_OFFLINE;
	} 
	else {
		/* 离线->在线 */
		if(judge_sen->work_state == DEV_OFFLINE)
			judge_sen->work_state = DEV_ONLINE;
	}
}

