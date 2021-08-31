#ifndef __CHASSIS_H
#define __CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "chassis_motor.h"
#include "gimbal.h"
#include "imu_sensor.h"
#include "rc_sensor.h"

#define CHAS_MECH_ANGLE_POS_MID CO_MECH_ANGLE_POS_MID
#define CHAS_MECH_ANGLE_NEG_MID CO_MECH_ANGLE_NEG_MID

#define CHAS_MAX_TOTAL_OUT 50000

#define KEY_Twist_FLAG 1

//#define Top_Speed 3000

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
  * @brief   底盘本地模式分类 
  * @note    根据系统模式的不同，可以设置不同的底盘本地模式
  * @author  RobotPilots
  */
typedef enum {
	CHASSIS_MODE_NORMAL 				= 0, // 正常模式
	CHASSIS_MODE_AUTO     			= 1,
	CHASSIS_MODE_LONGSHOOT   		= 2, // 远射模式
	CHASSIS_MODE_RELOAD_BULLET	= 3, // 底盘低速补弹模式
	CHASSIS_MODE_SZUPUP					= 4, // SZU爬坡模式
	
} chassis_mode_t;


/**
  * @brief 底盘电机pid中的target设置
  * @note 
  * @author  RobotPilots  
*/
typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	float		out;
} chassis_motor_pid_t;

typedef struct {
	pid_ctrl_t	angle;
	pid_ctrl_t	speed;
	float 		out;
	uint8_t back_flag;
} chassis_z_pid_t;

/**
  * @brief 		chassis_ctrl_t 底盘控制样例
  * @note 		结构体内是指向底盘单个电机的PID的指针
							以及底盘运动时z方向旋转的PID指针
	* @program	chassis_motor_pid_t  	指向电机速度环和角度环以及输出机构体的指针
							chassis_z_pid_t				指向z方向角度和输出结构体的指针
  * @author	  RobotPilots  
*/
typedef struct {
	chassis_motor_pid_t		(*motor)[CHAS_MOTOR_CNT];
	chassis_z_pid_t			*z_atti;	// z方向姿态
} chassis_ctrl_t;

/**
  * @brief 		chassis_dev_t 底盘驱动样例
  * @note 		可以获得控制时所需要的各类传感信息
	* @program	chas_motor  	指向单个电机数据的指针
							yaw_motor			指向云台电机数据的指针
							imu_sensor		指向IMU传感器的指针
							rc_sensor			指向遥控器传感器的指针	
  * @author 	RobotPilots  
*/
typedef struct {
	chassis_motor_t	*chas_motor[CHAS_MOTOR_CNT];
	gimbal_motor_t	*yaw_motor;
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
} chassis_dev_t;


/**
  * @brief		chassis_info_t  底盘信息样例
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
	chassis_mode_t		local_mode;
	co_angle_logic_t	co_angle_logic; //设置头朝向的位置
	bool				top_gyro;
	bool				twist;
}chassis_info_t;


/**
  * @brief		chassis_t  底盘样例
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
typedef struct chassis{
	chassis_ctrl_t	*controller;		//指向“chassis_ctrl_t 底盘控制样例” 类型的指针
	chassis_dev_t	*dev;							//底盘控制相关的数据信息
	chassis_info_t	*info;					//从系统模式拷贝下来的本地模式
	bool			test_open;						//测试用函数
	void			(*init)(void);
	void			(*update)(void);
	void			(*test)(void);
	void			(*ctrl)(void);
	void			(*output)(void);
	void			(*self_protect)(void);
	bool			(*if_back_to_mid_angle)(void);
	bool			(*if_top_gyro_open)(void);
}chassis_t;

extern chassis_t chassis;
extern bool N2O_Trigger;
/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/
bool CHASSIS_IfBackToMiddleAngle(void);

#endif
