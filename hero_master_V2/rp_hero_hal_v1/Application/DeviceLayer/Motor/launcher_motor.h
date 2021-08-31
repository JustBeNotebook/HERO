#ifndef __LAUNCHER_MOTOR_H
#define __LAUNCHER_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

//1:1 31130
#define DIAL_STEP -31130

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct launcher_motor_info_struct {
	uint16_t	angle;
	int16_t		speed;
	int16_t		current;
	uint16_t	angle_prev;
	int32_t		angle_sum;
	uint8_t		init_flag;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;	
} launcher_motor_info_t;

typedef struct launcher_motor_struct {
	launcher_motor_info_t 	*info;
	drv_can_t				*driver;
	void					(*init)(struct launcher_motor_struct *self);
	void					(*update)(struct launcher_motor_struct *self, uint8_t *rxBuf);
	void					(*check)(struct launcher_motor_struct *self);	
	void					(*heart_beat)(struct launcher_motor_struct *self);
	dev_work_state_t		work_state;
	dev_errno_t				errno;
	dev_id_t				id;
} launcher_motor_t;

extern launcher_motor_t	launcher_motor[LAUNCHER_MOTOR_CNT];

/* Exported functions --------------------------------------------------------*/


#endif
