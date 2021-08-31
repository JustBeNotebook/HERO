#ifndef __GIMBAL_MOTOR_H
#define __GIMBAL_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
#define GIMBAL_PITCH_MID 3037
#define GIMBAL_PITCH_MAX 3600
#define GIMBAL_PITCH_MIN 1993
//最低角度2025，最高角度3700

//云台转一周对应的电机角度
#define GIMBAL_YAW_CIRCULAR_STEP 12288


/* Exported types ------------------------------------------------------------*/
typedef struct gimbal_motor_info_struct {
	uint16_t	angle;
	int16_t		speed;
	int16_t		current;
	uint16_t	angle_prev;
	int32_t		angle_sum;
	uint8_t		init_flag;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;	
} gimbal_motor_info_t;

typedef struct gimbal_motor_struct {
	gimbal_motor_info_t 	*info;
	drv_can_t				*driver;
	void					(*init)(struct gimbal_motor_struct *self);
	void					(*update)(struct gimbal_motor_struct *self, uint8_t *rxBuf);
	void					(*check)(struct gimbal_motor_struct *self);	
	void					(*heart_beat)(struct gimbal_motor_struct *self);
	dev_work_state_t		work_state;
	dev_errno_t				errno;
	dev_id_t				id;
} gimbal_motor_t;

extern gimbal_motor_t	gimbal_motor[GIMBAL_MOTOR_CNT];
extern gimbal_motor_info_t gimbal_yaw, gimbal_m_pitch, gimbal_s_pitch;
/* Exported functions --------------------------------------------------------*/


#endif
