#ifndef __LAUNCHER_H
#define __LAUNCHER_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "launcher_motor.h"
#include "rc_sensor.h"
#include "driver.h"

#define re_dial_times 0 
#define MAGEZINE_OPEM_TIME 1000

#define SAFE_SPIN_TIME (120)
#define	SAFE_SPIN_SPEED (-12000)
#define DIAL_WARNING_OUTPUT (8000)

#define FIRE_FASTER_ABILITY 0
 
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
  * @brief   发射本地模式分类 
  * @note    根据系统模式的不同，可以设置不同的发射本地模式
  * @author  RobotPilots
  */
typedef enum {
	LAUNCHER_MODE_NORMAL 		= 0, // 正常模式
	LAUNCHER_MODE_COUNT   		= 1, // 模式计数
} launcher_mode_t;

typedef enum{
	MGZ_OFF,
	MGZ_ON,
} launcher_mgz_flag_t;

typedef enum{
	Friction_OFF,
	Friction_ON,
} launcher_friction_flag_t;

/**
  * @brief 发射电机pid中的target设置
  * @note 
  * @author  RobotPilots  
*/
typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	float		out;
} launcher_motor_pid_t;


/**
  * @brief 		launcher_ctrl_t 底盘控制样例
  * @note 		结构体内是指向底盘单个电机的PID的指针
							以及底盘运动时z方向旋转的PID指针
	* @program	launcher_motor_pid_t  	指向电机速度环和角度环以及输出机构体的指针
  * @author	  RobotPilots  
*/
typedef struct {
	launcher_motor_pid_t		(*motor)[LAUNCHER_MOTOR_CNT];
} launcher_ctrl_t;

/**
  * @brief 		launcher_dev_t 底盘驱动样例
  * @note 		可以获得控制时所需要的各类传感信息
	* @program	lucher_motor  	指向单个电机数据的指针
							yaw_motor			指向云台电机数据的指针
							imu_sensor		指向IMU传感器的指针
							rc_sensor			指向遥控器传感器的指针	
  * @author 	RobotPilots  
*/
typedef struct {
	launcher_motor_t	*launcher_motor[LAUNCHER_MOTOR_CNT];
	rc_sensor_t		*rc_sensor;
} launcher_dev_t;


/**
  * @brief		launcher_info_t  底盘信息样例
  * @note			记录了当前底盘的工作模式以及逻辑定位
  * @program	remote_mode			遥控模式样例，确定控制数据来源
							local_mode			底盘模式样例，确定底盘运动模式
							co_angle_logic	云台正方向标志
							top_gyro				小陀螺模式标志位
  * @author  RobotPilots  
*/
typedef struct {
	remote_mode_t		remote_mode;
	co_mode_t		co_mode;
	launcher_mode_t		local_mode;
	launcher_mgz_flag_t launcher_mgz_flag;
	launcher_friction_flag_t launcher_friction_flag;
	bool fire_permit_flag;
}launcher_info_t;


/**
  * @brief		launcher_t  底盘样例
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
typedef struct launcher{
	launcher_ctrl_t	*controller;		//指向“chassis_ctrl_t 底盘控制样例” 类型的指针
	launcher_dev_t	*dev;							//底盘控制相关的数据信息
	launcher_info_t	*info;					//从系统模式拷贝下来的本地模式
	bool			test_open;						//测试用函数
	void			(*init)(void);
	void			(*update)(void);
	void			(*test)(void);
	void			(*ctrl)(void);
	void			(*output)(void);
	void			(*self_protect)(void);
	bool			(*if_friction_on)(void);
	bool			(*if_dud)(void);
}launcher_t;

extern launcher_t launcher;
extern uint8_t slaver_photogate, slaver_photogate_flag;
extern int Friction_speed;
extern int Fric_3508_speed[6];
extern uint8_t dial_empty_flag;

/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/
bool LAUNCHER_IfFriction_On(void);

#endif
