#ifndef __CHASSIS_MOTOR_H
#define __CHASSIS_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct chassis_motor_info_struct {
	uint16_t	angle;
	int16_t		speed;
	int16_t		current;
	uint16_t	angle_prev;
	int32_t		angle_sum;
	uint8_t		init_flag;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;	
} chassis_motor_info_t;

typedef struct chassis_motor_struct {
	chassis_motor_info_t 	*info;
	drv_can_t				*driver;
	void					(*init)(struct chassis_motor_struct *self);
	void					(*update)(struct chassis_motor_struct *self, uint8_t *rxBuf);
	void					(*check)(struct chassis_motor_struct *self);	
	void					(*heart_beat)(struct chassis_motor_struct *self);
	dev_work_state_t		work_state;
	dev_errno_t				errno;
	dev_id_t				id;
} chassis_motor_t;

extern chassis_motor_t	chassis_motor[CHAS_MOTOR_CNT];

/* Exported functions --------------------------------------------------------*/


#endif
