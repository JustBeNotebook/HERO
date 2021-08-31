#ifndef __GIMBAL_H
#define __GIMBAL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "rp_math.h"
#include "chassis_motor.h"
#include "gimbal_motor.h"
#include "imu_sensor.h"
#include "rc_sensor.h"
#include "drv_io.h"
#include "predict.h"

#define GIMBAL_MECH_ANGLE_POS_MID CO_MECH_ANGLE_POS_MID
#define GIMBAL_MECH_ANGLE_NEG_MID CO_MECH_ANGLE_NEG_MID

//ANTI_TOP_TEST_FLAG = 
//1 时默认反陀螺，主动普通预测
//0 时默认普通预测，主动反陀螺
#define ANTI_TOP_TEST_FLAG 0


/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
  * @brief   云台本地模式分类 
  * @note    根据系统模式的不同，可以设置不同的云台本地模式
  * @author  RobotPilots
  */
typedef enum {
	GIMBAL_MODE_NORMAL		 			= 0, // 正常模式
	GIMBAL_MODE_AUTO    				= 1,
	GIMBAL_MODE_LONGSHOOT   		= 2, // 远射模式
	GIMBAL_MODE_RELOAD_BULLET		= 3, // 底盘低速补弹模式
	GIMBAL_MODE_SZUPUP					= 4, // SZU爬坡模式
} gimbal_mode_t;


/**
  * @brief 云台电机pid中的target设置
  * @note 
  * @author  RobotPilots  
*/
typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	float		out;
} gimbal_motor_pid_t;

typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	float 		out;
} gimbal_imu_pid_t;

/**
  * @brief 		gimbal_ctrl_t 云台控制样例
  * @note 		结构体内是指向底盘单个电机的PID的指针
							以及底盘运动时z方向旋转的PID指针
	* @program	gimbal_motor_pid_t  	指向电机速度环和角度环以及输出机构体的指针
							gimbal_imu_pid_t				指向Yaw方向角度的Yaw指针
  * @author	  RobotPilots  
*/
typedef struct {
	gimbal_motor_pid_t		(*motor)[GIMBAL_MOTOR_CNT];
} gimbal_ctrl_t;

typedef struct {
	float yaw;
	float pitch;
	float roll;
	short rate_yaw;
	short rate_pitch;
	short rate_roll;
} gimbal_imu_t;

/**
  * @brief 		gimbal_dev_t 底盘驱动样例
  * @note 		可以获得控制时所需要的各类传感信息
	* @program	gim_motor  	指向单个电机数据的指针
							imu_sensor		指向IMU传感器的指针
							rc_sensor			指向遥控器传感器的指针	
							gimbal_imu_sensor		imu传感器经过变换后变成这边的数据
  * @author 	RobotPilots  
*/
typedef struct {
	gimbal_motor_t	*gimbal_motor[GIMBAL_MOTOR_CNT];
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
	gimbal_imu_t  *gimbal_imu_sensor;
} gimbal_dev_t;


/**
  * @brief		gimbal_info_t  底盘信息样例
  * @note			记录了当前底盘的工作模式以及逻辑定位
  * @program	remote_mode			遥控模式样例，确定控制数据来源
							co_mode					联合模式样例，确定与云台联合的模式
							local_mode			底盘模式样例，确定底盘运动模式
							co_angle_logic	云台正方向标志
							top_gyro				小陀螺模式标志位
  * @author  RobotPilots  
*/
typedef struct {
	remote_mode_t		remote_mode;
	co_mode_t		co_mode;
	gimbal_mode_t		local_mode;
	co_angle_logic_t	co_angle_logic; //设置头朝向的位置
	bool				top_gyro;
	bool 				if_fire_ready;
}gimbal_info_t;


/**
  * @brief		gimbal_t  底盘样例
  * @note			除了传感信息和控制信息外，还集成了函数指针用于执行响应程序
  * @program	（仅函数部分，信息部分看上面）
							.init 										初始化
							.update 									底盘更新函数
							.test											测试函数，仅当test_open开启时有用
							.ctrl											控制函数
							.output										电机输出函数
							.self_protect							失联保护
							.if_back_to_mid_angle			是否归中查询
							.if_top_gyro_open					是否开启陀螺仪查询

	* @author  RobotPilots  
*/
typedef struct gimbal{
	gimbal_ctrl_t	*controller;		//指向“chassis_ctrl_t 底盘控制样例” 类型的指针
	gimbal_dev_t	*dev;							//底盘控制相关的数据信息
	gimbal_info_t	*info;					//从系统模式拷贝下来的本地模式
	bool			test_open;						//测试用函数
	void			(*init)(void);
	void 			(*reset)(void);
	void			(*update)(void);
	void			(*test)(void);
	void			(*ctrl)(void);
	void			(*output)(void);
	void			(*self_protect)(void);
	bool			(*gimbal_imu_if_back_mid)(void);
}gimbal_t;

extern gimbal_t gimbal;
extern gimbal_imu_t gimbal_imu;
extern uint8_t anti_top_flag;
/* Exported functions --------------------------------------------------------*/
void VISION_PREDICT(void);
float GIMBAL_GET_IMU_MID_ERR(void);
/* 信息层 --------------------------------------------------------------------*/
//bool CHASSIS_IfBackToMiddleAngle(void);


void gimbal_now_angle_ms_update(void);
/* provide functions --------------------------------------------------------*/
	

#endif
