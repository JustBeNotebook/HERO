/**
 * @file        gimbal.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        23-October-2020
 * @brief       gimbal Module.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "gimbal.h"
#include "vision_sensor.h"
#include "cmsis_os.h"
#include "launcher.h"
#include "control.h"
#include "can_potocol.h"
#include "vision_sensor.h"
#include "rp_math.h"
#include "kalman.h"
#include "kalman_filter.h"

/* Private macro -------------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/


void GIMBAL_Init(void);
void GIMBAL_Ctrl(void);
void GIMBAL_Test(void);
void GIMBAL_Reset(void);
void GIMBAL_SelfProtect(void);
void GIMBAL_PidCtrl(void);
bool GIMBAL_IMU_IF_BACK_MID(void);
void VISION_PREDICT(void);
void vision_anti_top_analyze(void);

//获取系统信息
void GIMBAL_GetSysInfo(void);
void GIMBAL_GetJudgeInfo(void);
void GIMBAL_GetRcInfo(void);
void GIMBAL_GetTopGyroInfo(void);
void GIMBAL_GetSelfAttitude(void);
void VISION_PREDICT(void);//获取视觉信息

//初始化PID内部数据
static void GIMBAL_PidParamsInit(gimbal_motor_pid_t *pid, uint8_t motor_cnt);

//停止PID输出
static void GIMBAL_Stop(gimbal_motor_pid_t *pid);

//pid输出函数
static void GIMBAL_PidOut(gimbal_motor_pid_t *pid);

//根据ID计算PID输出
static void GIMBAL_Angle_PidCalc(gimbal_motor_pid_t *pid, gimbal_motor_cnt_t MOTORx);
static void GIMBAL_Speed_PidCalc(gimbal_motor_pid_t *pid, gimbal_motor_cnt_t MOTORx);


/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// 云台电机本地驱动
drv_can_t				*gimb_drv[GIMBAL_MOTOR_CNT];
gimbal_motor_t			*gimb_motor[GIMBAL_MOTOR_CNT];
gimbal_motor_info_t	*gimb_motor_info[GIMBAL_MOTOR_CNT];


//云台运动两变量
float TargetPitch = 0, TargetYaw = 0;

float gimbal_mouse_vy = 0.15f;
float gimbal_mouse_vp = 0.1f;

//因为机械装配对云台有一定的倾斜，所以需要有这个参数，根据角度进行仰角补偿
float gimbal_top_offset = 0;

float auto_binary;
//RPAA RobotPilotsAutoAim
//RP自瞄距离补偿
float RPAA_Distance_offset = 0;

// 底盘PID输出最大值
uint16_t GIMBAL_SPEED_MAX	= 8000;
uint16_t GIMBAL_PID_OUT_MAX = 9000;



// 遥控按键信息
static uint8_t key_lock_G = false;
static uint8_t key_lock_Q = false;
static uint8_t key_lock_E = false;
static uint8_t key_lock_V = false;

static uint8_t key_lock_W = false;
static uint8_t key_lock_S = false;
static uint8_t key_lock_A = false;
static uint8_t key_lock_D = false;


static uint8_t key_lock_C = false;
/* 斜坡 */


/* 键盘模式下的前后左后以及旋转斜坡变量 */


/* 期望速度 */


/* 摇杆灵敏度 */
//机械模式下云台比例系数,控制摇杆响应速度,如果过小也会限制最高转速,max = 此系数 *660
float kRc_Mech_Gimbal_Pitch, kRc_Mech_Gimbal_Yaw;//平移，旋转,

//陀螺仪模式下云台比例系数,控制摇杆响应速度,如果过小也会限制最高转速,max = 此系数 *660
float kRc_Gyro_Gimbal_Pitch, kRc_Gyro_Gimbal_Yaw;//平移，旋转

//机械模式下云台比例系数,控制键盘斜坡变化率
float kKey_Mech_Gimbal_Pitch, kKey_Mech_Gimbal_Yaw;//平移，旋转

//陀螺仪模式下云台比例系数,控制键盘斜坡变化率
float kKey_Gyro_Gimbal_Pitch, kKey_Gyro_Gimbal_Yaw;//平移，旋转



/* Exported variables --------------------------------------------------------*/
gimbal_imu_t gimbal_imu = {
	.yaw = 0,
	.pitch = 0,
	.roll = 0,
	.rate_yaw = 0,
	.rate_pitch = 0,
	.rate_roll = 0,
};

// 云台电机PID控制器
gimbal_motor_pid_t 	gimbal_motor_pid[GIMBAL_MOTOR_CNT] = {
	[GIMBAL_YAW] = {
		.speed.kp = 40.0f, //35.0f,
		.speed.ki = 1.0f,
		.speed.kd = 0,
		.speed.integral_max = 20000,
		.speed.out_max = 26000,
		.angle.kp = 12.0f, //30.f,
		.angle.ki = 0.0f,
		.angle.kd = 0,
		.angle.integral_max = 1000,
		.angle.out_max = 8000,
	},
	[GIMBAL_M_PITCH] = {
		.speed.kp = -22.0f, //22.0f,
		.speed.ki = -0.4f,
		.speed.kd = 0,
		.speed.integral_max = 20000,
		.speed.integral_bias = 18000,
		.speed.out_max = 28000,
		.angle.kp = -15.0f,//22.0f,
		.angle.ki = 0.f,
		.angle.kd = 0.f,
		.angle.integral_max = 2000,
		.angle.out_max = 8000,
	},
	[GIMBAL_IMU_YAW] = {
		.speed.kp = 40.0f, 
		.speed.ki = 1.0f,
		.speed.kd = 0,
		.speed.integral_max = 16000,
		.speed.out_max = 26000,
		.angle.kp = 8.0f, 
		.angle.ki = 0.f,
		.angle.kd = 0,
		.angle.integral_max = 1000,
		.angle.out_max = 8000,
	},
	[GIMBAL_IMU_PITCH] = {
		.speed.kp = -22.0f, 
		.speed.ki = -0.4f,
		.speed.kd = 0,
		.speed.integral_max = 20000,
		.speed.integral_bias = 18000,
		.speed.out_max = 28000,
		.angle.kp = 0.0f, 
		.angle.ki = 0.f,
		.angle.kd = 0,
		.angle.integral_max = 1000,
		.angle.out_max = 6000,
	},
};

gimbal_motor_pid_t 	gimbal_reset_pid[GIMBAL_MOTOR_CNT] = {
	[GIMBAL_YAW] = {
		.speed.kp = 40.0f, //35.0f,
		.speed.ki = 1.0f,
		.speed.kd = 0,
		.speed.integral_max = 10000,
		.speed.out_max = 26000,
		.angle.kp = 12.0f, //30.f,
		.angle.ki = 0.0f,
		.angle.kd = 0,
		.angle.integral_max = 1000,
		.angle.out_max = 8000,
	},
	[GIMBAL_M_PITCH] = {
		.speed.kp = -22.0f, //22.0f,
		.speed.ki = -0.4f,
		.speed.kd = 0,
		.speed.integral_max = 15000,
		.speed.integral_bias = 0,
		.speed.out_max = 28000,
		.angle.kp = -15.0f,//22.0f,
		.angle.ki = 0.f,
		.angle.kd = 0.f,
		.angle.integral_max = 2000,
		.angle.out_max = 8000,
	},
	[GIMBAL_IMU_YAW] = {
		.speed.kp = 40.0f, 
		.speed.ki = 1.0f,
		.speed.kd = 0,
		.speed.integral_max = 16000,
		.speed.out_max = 26000,
		.angle.kp = 8.0f, 
		.angle.ki = 0.0f,
		.angle.kd = 0,
		.angle.integral_max = 1000,
		.angle.out_max = 8000,
	},
};


//// 测试参数
//gimbal_motor_pid_t 	gimbal_motor_pid[GIMBAL_MOTOR_CNT] = {
//	[GIMBAL_YAW] = {
//		.speed.kp = 10.0f, //35.0f,
//		.speed.ki = 0.f,
//		.speed.kd = 0,
//		.speed.integral_max = 20000,
//		.speed.out_max = 26000,
//		.angle.kp = 5.0f, //30.f,
//		.angle.ki = 0.0f,
//		.angle.kd = 0,
//		.angle.integral_max = 1000,
//		.angle.out_max = 8000,
//	},
//	[GIMBAL_M_PITCH] = {
//		.speed.kp = -5.0f, //22.0f,
//		.speed.ki = -0.f,
//		.speed.kd = 0,
//		.speed.integral_max = 20000,
//		.speed.integral_bias = 18000,
//		.speed.out_max = 28000,
//		.angle.kp = -5.0f,//22.0f,
//		.angle.ki = 0.f,
//		.angle.kd = 0.f,
//		.angle.integral_max = 2000,
//		.angle.out_max = 8000,
//	},
//	[GIMBAL_IMU_YAW] = {
//		.speed.kp = 10.0f, 
//		.speed.ki = 0.0f,
//		.speed.kd = 0,
//		.speed.integral_max = 16000,
//		.speed.out_max = 26000,
//		.angle.kp = 6.0f, 
//		.angle.ki = 0.f,
//		.angle.kd = 0,
//		.angle.integral_max = 1000,
//		.angle.out_max = 8000,
//	},
//};

//gimbal_motor_pid_t 	gimbal_reset_pid[GIMBAL_MOTOR_CNT] = {
//	[GIMBAL_YAW] = {
//		.speed.kp = 10.0f, //35.0f,
//		.speed.ki = 0.f,
//		.speed.kd = 0,
//		.speed.integral_max = 20000,
//		.speed.out_max = 26000,
//		.angle.kp = 5.0f, //30.f,
//		.angle.ki = 0.0f,
//		.angle.kd = 0,
//		.angle.integral_max = 1000,
//		.angle.out_max = 8000,
//	},
//	[GIMBAL_M_PITCH] = {
//		.speed.kp = -5.0f, //22.0f,
//		.speed.ki = -0.f,
//		.speed.kd = 0,
//		.speed.integral_max = 20000,
//		.speed.integral_bias = 18000,
//		.speed.out_max = 28000,
//		.angle.kp = -5.0f,//22.0f,
//		.angle.ki = 0.f,
//		.angle.kd = 0.f,
//		.angle.integral_max = 2000,
//		.angle.out_max = 8000,
//	},
//	[GIMBAL_IMU_YAW] = {
//		.speed.kp = 40.0f, 
//		.speed.ki = 1.0f,
//		.speed.kd = 0,
//		.speed.integral_max = 16000,
//		.speed.out_max = 26000,
//		.angle.kp = 6.0f, 
//		.angle.ki = 0.f,
//		.angle.kd = 0,
//		.angle.integral_max = 1000,
//		.angle.out_max = 8000,
//	},
//};


// 卡尔曼滤波器
extKalman_t gimbal_speed_pid_Kal[GIMBAL_MOTOR_CNT];
extKalman_t gimbal_angle_pid_Kal[GIMBAL_MOTOR_CNT];

extKalman_t vision_dis_pitch_offset;
extKalman_t vision_absolute_Yaw_Kal, vision_absolute_Pitch_Kal;
extKalman_t vision_angleY_KF, vision_angleP_KF;
extKalman_t vision_speedY_KF, vision_speedP_KF;
extKalman_t vision_accelY_KF, vision_accelP_KF;

extKalman_t vision_distance_KF, vision_speedD_KF;

moving_Average_Filter vision_absolute_Y_MF, vision_absolute_P_MF;
moving_Average_Filter vision_angleY_MF, vision_angleP_MF;
moving_Average_Filter vision_speedY_MF, vision_speedP_MF;
moving_Average_Filter vision_accelY_MF, vision_accelP_MF;
moving_Average_Filter vision_distance_MF, vision_speedD_MF;

moving_Average_Filter zhen_time_ms;

LOW_Pass_Filter vision_speedY_LPF, vision_speedP_LPF;


//反陀螺变量
moving_Average_Filter Anti_top_yaw_MF;//滞后陀螺的滑动平均滤波器
moving_Average_Filter gimbal_now_yaw_raw, gimbal_now_pitch_raw;//记录近99ms内的陀螺仪角度,自瞄里面
moving_Average_Filter yaw_raw_ms, pitch_raw_ms; //记录近99ms内的陀螺仪角度,ms更新

Median_Filter Anti_top_yaw_MedianF;
/*
这一版反陀螺代码需要对T1, T2进行标定
T1标定方法：开客户端，开自瞄，用秒表看装甲板移动后激光的延时时间。
T2标定方法：设定速度，根据不同距离测几组射击延时，从拨下开关到击中目标的时间，然后拟合一条曲线
T0 由相邻两次装甲板绝对角度跳变的时间决定
一般而言
T3' = n*T0-T1-T2
T3'-(N-1)T0 = T0-T1-T2 = T3

while(T3<0)//保持T3滞后不超过一个周期
{
	T3+=T0;
}
然后由T3决定到底应该跟随前过去第几帧的绝对角度数据
*/
float Anti_top_Time0_enermy_circle = 0;//敌方小陀螺时，装甲板出现在中心的周期时间，单位毫秒，转一圈等于四个周期
float Anti_top_Time1_chase_delay = 200;//自瞄跟随延时，单位毫秒

//Anti_top_Time2_shoot_delay从发出开火指令到弹丸命中装甲板的时间。由机械结构，软件设计与敌方距离决定的命中延迟。
//2021/03/30  3m,5m,8m 数据测出来 T2 = 59.737f*distance+183.07; 距离单位为米
float Anti_top_Time2_shoot_delay = 0;		

float Anti_top_Time3_chase_force_delay = 0;//反陀螺时，为了击中下一块装甲板，强制让自瞄落后的时间

moving_Average_Filter Anti_top_AimPos_MAF;


extKalman_t AntiTop_dis_pitch_offset;
// 底盘模块控制器
gimbal_ctrl_t		gimbal_ctrl = {
	.motor = &gimbal_motor_pid,
};

// 底盘模块传感器
gimbal_dev_t		gimbal_dev = {
	.gimbal_motor[GIMBAL_YAW] = &gimbal_motor[GIMBAL_YAW],
	.gimbal_motor[GIMBAL_M_PITCH] = &gimbal_motor[GIMBAL_M_PITCH],
	.gimbal_motor[GIMBAL_S_PITCH] = &gimbal_motor[GIMBAL_S_PITCH],
	.gimbal_motor[GIMBAL_IMU_YAW] = &gimbal_motor[GIMBAL_IMU_YAW],
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
	.gimbal_imu_sensor = &gimbal_imu,
};

// 底盘模块信息
gimbal_info_t 	gimbal_info = {
	.remote_mode = RC,
	.co_mode = CO_MECH,
	.local_mode = GIMBAL_MODE_NORMAL,
	.co_angle_logic = LOGIC_FRONT,
	.if_fire_ready = 0,
};

gimbal_t gimbal = {
	.controller = &gimbal_ctrl,
	.dev = &gimbal_dev,
	.info = &gimbal_info,
	.reset = GIMBAL_Reset,
	.init = GIMBAL_Init,
	.test = GIMBAL_Test,
	.ctrl = GIMBAL_Ctrl,
	.gimbal_imu_if_back_mid = GIMBAL_IMU_IF_BACK_MID,
	.self_protect = GIMBAL_SelfProtect,
	
};

/* Private functions ---------------------------------------------------------*/
/* 驱动层 --------------------------------------------------------------------*/

/**
 *	@brief	云台电机PID参数初始化，保护用
 */
static void GIMBAL_PidParamsInit(gimbal_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		pid_val_init(&pid[i].angle);
		pid_val_init(&pid[i].speed);
		pid[i].out = 0;
		
		pid[i].angle.target = pid[i].angle.measure;
	}	
}

static void GIMBAL_Pidclear(gimbal_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		pid_val_init(&pid[i].angle);
		pid_val_init(&pid[i].speed);
		pid[i].out = 0;
	}	
}


static void GIMBAL_SinglePidParamsInit(gimbal_motor_pid_t *pid, uint8_t motor_cnt)
{
	
	
	pid_val_init(&pid[motor_cnt].angle);
	pid_val_init(&pid[motor_cnt].speed);
	pid[motor_cnt].out = 0;
	
	pid[motor_cnt].angle.target = pid[motor_cnt].angle.measure;

}

/**
 *	@brief	云台底盘电机卸力，保护用
 */
static void GIMBAL_Stop(gimbal_motor_pid_t *pid)
{
	
	pid[GIMBAL_S_PITCH].out = 0;
	pid[GIMBAL_M_PITCH].out = 0;
	pid[GIMBAL_YAW].out = 0;
	pid[GIMBAL_IMU_YAW].out = 0;
	
	
	CAN1_0X1ff_BUF[3] = 0;
	CAN2_0X1ff_BUF[2] = 0;
	CAN2_0X1ff_BUF[3] = 0;
	
}

/**
 *	@brief	云台电机PID输出
 */
static void GIMBAL_PidOut(gimbal_motor_pid_t *pid)
{
	
	if(gimb_motor[GIMBAL_YAW]->work_state == DEV_ONLINE) 
	{
		if(gimbal.info->co_mode == CO_MECH)
		{
			CAN1_0X1ff_BUF[3] = (int16_t)(pid[GIMBAL_YAW].out);
		}
		else if(gimbal.info->co_mode == CO_GYRO)
		{
			CAN1_0X1ff_BUF[3] = (int16_t)(pid[GIMBAL_IMU_YAW].out);
		}
	} 
	else 
	{
		CAN1_0X1ff_BUF[3] = 0;
	}

	if(gimb_motor[GIMBAL_M_PITCH]->work_state == DEV_ONLINE) 
	{
		CAN2_0X1ff_BUF[2] = (int16_t)(pid[GIMBAL_M_PITCH].out);
		CAN2_0X1ff_BUF[3] = (int16_t)(pid[GIMBAL_M_PITCH].out);
	} 
	else 
	{
		CAN2_0X1ff_BUF[2] = 0;
		CAN2_0X1ff_BUF[3] = 0;
	}

}

static void GIMBAL_Angle_PidCalc(gimbal_motor_pid_t *pid, gimbal_motor_cnt_t MOTORx)
{
	pid[MOTORx].angle.err = pid[MOTORx].angle.target - pid[MOTORx].angle.measure;
	
	if(MOTORx == GIMBAL_YAW || MOTORx == GIMBAL_IMU_YAW)
	{
		if(pid[MOTORx].angle.err>GIMBAL_YAW_CIRCULAR_STEP/2)
		{
			pid[MOTORx].angle.err -= GIMBAL_YAW_CIRCULAR_STEP;
		}
		else if(pid[MOTORx].angle.err<-GIMBAL_YAW_CIRCULAR_STEP/2)
		{
			pid[MOTORx].angle.err += GIMBAL_YAW_CIRCULAR_STEP;
		}
	}
	pid[MOTORx].angle.true_err = pid[MOTORx].angle.err;
	pid[MOTORx].angle.err = KalmanFilter(&gimbal_angle_pid_Kal[MOTORx], pid[MOTORx].angle.err);
	
	single_pid_ctrl(&pid[MOTORx].angle);
	pid[MOTORx].speed.target = pid[MOTORx].angle.out;
	
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
	pid[MOTORx].speed.err = KalmanFilter(&gimbal_speed_pid_Kal[MOTORx], pid[MOTORx].speed.err);
	
	single_pid_ctrl(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}

/**
 *	@brief	云台电机速度环
 */
static void GIMBAL_Speed_PidCalc(gimbal_motor_pid_t *pid, gimbal_motor_cnt_t MOTORx)
{
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
	pid[MOTORx].speed.err = KalmanFilter(&gimbal_speed_pid_Kal[MOTORx], pid[MOTORx].speed.err);
	
	single_pid_ctrl(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}



/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/

/**
 *	@brief	底盘获取系统信息
 */
void GIMBAL_GetSysInfo(void)//只要确保随时切换无问题即可就行
{
	static uint16_t sw_outtime = 0;
	
	/*----控制方式修改----*/
	if(sys.remote_mode == RC) {
		gimbal_info.remote_mode = RC;
	}
	else if(sys.remote_mode == KEY) {
		gimbal_info.remote_mode = KEY;
	}
	gimbal.info->if_fire_ready = 1;
	/*----本地模式修改----*/
	switch(sys.mode)	//切换模式
	{
		case SYS_MODE_LONGSHOOT: 
			if(gimbal_info.local_mode!=GIMBAL_MODE_SZUPUP)
			{
				gimbal_info.local_mode = GIMBAL_MODE_LONGSHOOT;	
			}
			break;

		case SYS_MODE_AUTO://自瞄模式
			gimbal_info.local_mode = GIMBAL_MODE_AUTO;
			break;
		case SYS_MODE_NORMAL:
			gimbal.info->if_fire_ready = 0;
			gimbal_info.local_mode = GIMBAL_MODE_NORMAL;
			break;
		case SYS_MODE_PARK: break;
		
		default: break;
	}
	
	if(sys.state == SYS_STATE_RCLOST)
	{
		gimbal_info.co_mode = sys.co_mode;
	}
	
	if(sys.co_mode != gimbal_info.co_mode)
	{
		if(gimbal_info.co_mode == CO_MECH)
		{
			gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target = gimbal_motor_pid[GIMBAL_IMU_YAW].angle.measure;
			GIMBAL_SinglePidParamsInit(gimbal_motor_pid, GIMBAL_YAW);
			gimbal_motor_pid[GIMBAL_YAW].angle.target = 0;
			gimbal_info.co_mode = CO_GYRO;
			sw_outtime = 0;
		}
		else if((abs(gimbal_motor[GIMBAL_YAW].info->angle_sum))<50 || sw_outtime>1000)
		{
			
			GIMBAL_SinglePidParamsInit(gimbal_motor_pid, GIMBAL_IMU_YAW);
			gimbal_motor_pid[GIMBAL_YAW].angle.target = 0;
			gimbal_info.co_mode = CO_MECH;
			
			sw_outtime = 0;
		}
		sw_outtime++;
	}
	else
	{
		gimbal_info.co_mode = sys.co_mode;
		sw_outtime = 0;
	}
	
}

void GIMBAL_GetJudgeInfo(void)
{
	//无？ 或者是裁判系统的数据获取
}

void GIMBAL_GetRcInfo(void)
{
	/* 系统正常 */
	if(sys.state == SYS_STATE_NORMAL)
	{
		if(sys.remote_mode == RC) 
		{
			key_lock_G = false;
			key_lock_Q = false;
			key_lock_E = false;
			key_lock_V = false;
		}
		else if(sys.remote_mode == KEY) 
		{
		}
	}
	/* 系统异常 失联 操作 其他 */
	else 
	{
		key_lock_G = false;
		key_lock_Q = false;
		key_lock_E = false;
		key_lock_V = false;
	}
}

void GIMBAL_GetTopGyroInfo(void)
{
	
}

void GIMBAL_GetSelfAttitude(void)
{
	
}

gimbal_motor_pid_t temp_yaw, temp_pitch, temp_mpu_yaw;
gimbal_motor_t motor_yaw, motor_pitch, motor_mpu_yaw;
float gimbal_imu_yaw_angle_sum = 0;
void GIMBAL_UpdateController(void)
{
	float err;
	static float prev_angle = 0;
	
	imu_sensor_info_t *imu = gimbal_dev.imu_sensor->info;
	
	temp_yaw = gimbal_motor_pid[GIMBAL_YAW];
	temp_pitch = gimbal_motor_pid[GIMBAL_M_PITCH];
	temp_mpu_yaw = gimbal_motor_pid[GIMBAL_IMU_YAW];
	
	motor_yaw = gimbal_motor[GIMBAL_YAW];
	motor_pitch = gimbal_motor[GIMBAL_M_PITCH];
	motor_mpu_yaw = gimbal_motor[GIMBAL_IMU_YAW];
	
	//gimbal_imu.rate_yaw = (imu->rate_yaw*cos(imu->pitch*PI/180.0f))+(imu->rate_pitch*sin(imu->pitch*PI/180.0f))
	//																					-	imu->rate_yaw*cos(imu->roll*PI/180.0f) + imu->rate_roll*sin(imu->roll*PI/180.0f);
	gimbal_imu.rate_yaw = (imu->rate_yaw*cos(imu->pitch*PI/180.0f)*cos(imu->roll*PI/180.0f))
												+	(imu->rate_pitch*sin(imu->pitch*PI/180.0f)) 
												+ imu->rate_roll*sin(imu->roll*PI/180.0f)*cos(imu->pitch*PI/180.0f);
	
	gimbal_imu.rate_roll = -imu->rate_pitch;
	
	//gimbal_imu.rate_pitch = imu->rate_roll;//
	gimbal_imu.rate_pitch = imu->rate_roll*cos(imu->pitch*PI/180.0f) + imu->rate_yaw*sin(imu->pitch*PI/180.0f);
	
	gimbal.dev->gimbal_imu_sensor->yaw = -imu->yaw;
	gimbal.dev->gimbal_imu_sensor->roll = -imu->roll;
	gimbal.dev->gimbal_imu_sensor->pitch = -imu->pitch;
	
	gimbal_motor_pid[GIMBAL_IMU_YAW].angle.measure = imu->yaw/360.0f*GIMBAL_YAW_CIRCULAR_STEP;
	gimbal_motor_pid[GIMBAL_IMU_YAW].speed.measure = -gimbal.dev->imu_sensor->info->rate_yaw;
	
	err = gimbal_motor_pid[GIMBAL_IMU_YAW].angle.measure - prev_angle;
	
	/* 过零点 计算陀螺仪角度*/
	if(abs(err) > GIMBAL_YAW_CIRCULAR_STEP/2)
	{
		/* 0↓ -> 12288 */
		if(err >= 0)
			gimbal_imu_yaw_angle_sum += -GIMBAL_YAW_CIRCULAR_STEP + err;
		/* 12288↑ -> 0 */
		else
			gimbal_imu_yaw_angle_sum += GIMBAL_YAW_CIRCULAR_STEP + err;
	}
	/* 未过零点 */
	else
	{
		gimbal_imu_yaw_angle_sum += err;
	}
	prev_angle = gimbal_motor_pid[GIMBAL_IMU_YAW].angle.measure;
	
	gimbal_motor_pid[GIMBAL_YAW].angle.measure = gimbal_motor[GIMBAL_YAW].info->angle_sum;
	gimbal_motor_pid[GIMBAL_YAW].speed.measure = -gimbal.dev->imu_sensor->info->rate_yaw;
	
	gimbal_motor_pid[GIMBAL_M_PITCH].angle.measure = gimbal_motor[GIMBAL_M_PITCH].info->angle;
	gimbal_motor_pid[GIMBAL_M_PITCH].speed.measure = -gimbal.dev->imu_sensor->info->rate_roll;
	
	gimbal_motor_pid[GIMBAL_IMU_PITCH].angle.measure = -imu->pitch*22.75556f;
	gimbal_motor_pid[GIMBAL_IMU_PITCH].speed.measure = -gimbal.dev->imu_sensor->info->rate_roll;
}

/* 应用层 --------------------------------------------------------------------*/
uint32_t binary = 30;
bool GIMBAL_IMU_IF_BACK_MID(void)
{
	float gimbal_angle_err;
	
	gimbal_angle_err = gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target - gimbal_motor_pid[GIMBAL_IMU_YAW].angle.measure;
	
	if(gimbal_angle_err>GIMBAL_YAW_CIRCULAR_STEP/2)
	{
		gimbal_angle_err -= GIMBAL_YAW_CIRCULAR_STEP;
	}
	else if(gimbal_angle_err<-GIMBAL_YAW_CIRCULAR_STEP/2)
	{
		gimbal_angle_err += GIMBAL_YAW_CIRCULAR_STEP;
	}
	
	return ((abs(gimbal_angle_err)) < (binary))?true:false;
}

float GIMBAL_GET_IMU_MID_ERR(void)
{
	float gimbal_angle_err;
	
	gimbal_angle_err = gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target - gimbal_motor_pid[GIMBAL_IMU_YAW].angle.measure;
	
	if(gimbal_angle_err>GIMBAL_YAW_CIRCULAR_STEP/2)
	{
		gimbal_angle_err -= GIMBAL_YAW_CIRCULAR_STEP;
	}
	else if(gimbal_angle_err<-GIMBAL_YAW_CIRCULAR_STEP/2)
	{
		gimbal_angle_err += GIMBAL_YAW_CIRCULAR_STEP;
	}
	
	return gimbal_angle_err;
}

//除了初次上电以外的初始化
uint8_t first_flag = 0;
void GIMBAL_Reset(void)
{
	float err_y, err_p;
	float err_to_pos, err_to_neg;
	
	gimbal.info->co_mode = CO_MECH;
	
	gimbal_motor_pid[GIMBAL_YAW].angle.out_max = 6000;//限制云台回正速度
	gimbal_motor_pid[GIMBAL_M_PITCH].angle.out_max = 2000;
	REBOOT_RELAY_SET_TOGGLE();//交替开启单触发继电器
	GIMBAL_UpdateController();//更新云台信息
	
	gimbal_motor_pid[GIMBAL_M_PITCH].angle.target = GIMBAL_PITCH_MID;
	
	
	gimbal_motor_pid[GIMBAL_YAW].angle.target = 0;
	gimbal_motor_pid[GIMBAL_YAW].angle.measure = gimbal_motor[GIMBAL_YAW].info->angle_sum;
	
	//原则上是可以选择只在上电时重设一遍，但是这里每当遥控丢失后都算重上电一次
	if(first_flag == 0)
	{
		gimbal_motor_pid[GIMBAL_YAW].angle.measure = 2000; //上电复位,以一定的角速度旋转
		if(REBOOT_READ() == 0)//感受到霍尔信号
		{
			//因为1：1.5的云台存在两个角度可以朝前，所以以就近原则对信号处理
			err_to_pos = abs(gimbal_motor[GIMBAL_YAW].info->angle - (float)(GIMBAL_MECH_ANGLE_POS_MID));
			err_to_neg = abs(gimbal_motor[GIMBAL_YAW].info->angle - (float)(GIMBAL_MECH_ANGLE_NEG_MID));
			
			//以就近原则选取时，需要对电机越界的情况加以判断，临界值推荐为电机最大角度的一半
			if(err_to_pos>4096)
			{
				err_to_pos = err_to_pos-4096;
			}
			if(err_to_neg > 4096)
			{
				err_to_neg = err_to_neg-4096;
			}
			
			if(err_to_neg>err_to_pos)
			{
				first_flag = 1;
				gimbal_motor[GIMBAL_YAW].info->angle_sum = -GIMBAL_MECH_ANGLE_POS_MID + (gimbal_motor[GIMBAL_YAW].info->angle);
			}
			else if(err_to_neg<err_to_pos)
			{
				first_flag = 1;
				gimbal_motor[GIMBAL_YAW].info->angle_sum = -GIMBAL_MECH_ANGLE_NEG_MID + (gimbal_motor[GIMBAL_YAW].info->angle);
			}
			//因为上面求err时偷懒用了绝对值，所以此处需要再计算一次角度的正负
			if((gimbal_motor[GIMBAL_YAW].info->angle_sum) > 4096)
			{
				gimbal_motor[GIMBAL_YAW].info->angle_sum = gimbal_motor[GIMBAL_YAW].info->angle_sum - 8192;
			}
			else if((gimbal_motor[GIMBAL_YAW].info->angle_sum) < -4096)
			{
				gimbal_motor[GIMBAL_YAW].info->angle_sum = 8192 + gimbal_motor[GIMBAL_YAW].info->angle_sum;
			}
			
		}
	}
	
	
	GIMBAL_PidCtrl();
	
	
	err_y = abs(gimbal_motor_pid[GIMBAL_YAW].angle.target - gimbal_motor_pid[GIMBAL_YAW].angle.measure);
	err_p = abs(gimbal_motor_pid[GIMBAL_M_PITCH].angle.target - gimbal_motor_pid[GIMBAL_M_PITCH].angle.measure);

	if(err_y <= 50 && err_p <= 50 && first_flag == 1)//已经回正
	{
		gimbal_motor_pid[GIMBAL_YAW].angle.target = 0;
		gimbal_motor_pid[GIMBAL_M_PITCH].angle.target = GIMBAL_PITCH_MID;
		GIMBAL_Pidclear(gimbal_motor_pid, GIMBAL_MOTOR_CNT);

		gimbal_motor_pid[GIMBAL_YAW].angle.out_max = 8000;
		gimbal_motor_pid[GIMBAL_M_PITCH].angle.out_max = 8000;
		REBOOT_RELAY_SET_ON();	//关闭继电器
		flag.gimbal.reset_ok = true;
		flag.gimbal.reset_start = false;
	}
	
}


/* 任务层 --------------------------------------------------------------------*/
void GIMBAL_Init(void)
{
	
	
	gimb_drv[GIMBAL_YAW] = gimbal_dev.gimbal_motor[GIMBAL_YAW]->driver;
	gimb_drv[GIMBAL_M_PITCH] = gimbal_dev.gimbal_motor[GIMBAL_M_PITCH]->driver;
	gimb_drv[GIMBAL_S_PITCH] = gimbal_dev.gimbal_motor[GIMBAL_S_PITCH]->driver;

	gimb_motor[GIMBAL_YAW] = gimbal_dev.gimbal_motor[GIMBAL_YAW];
	gimb_motor[GIMBAL_M_PITCH] = gimbal_dev.gimbal_motor[GIMBAL_M_PITCH];
	gimb_motor[GIMBAL_S_PITCH] = gimbal_dev.gimbal_motor[GIMBAL_S_PITCH];
	
	gimb_motor_info[GIMBAL_YAW] = gimbal_dev.gimbal_motor[GIMBAL_YAW]->info;
	gimb_motor_info[GIMBAL_M_PITCH] = gimbal_dev.gimbal_motor[GIMBAL_M_PITCH]->info;
	gimb_motor_info[GIMBAL_S_PITCH] = gimbal_dev.gimbal_motor[GIMBAL_S_PITCH]->info;
	
	KalmanCreate(&gimbal_speed_pid_Kal[GIMBAL_YAW], 1, 5);
	KalmanCreate(&gimbal_speed_pid_Kal[GIMBAL_M_PITCH], 1, 10);
	KalmanCreate(&gimbal_speed_pid_Kal[GIMBAL_IMU_YAW], 1, 0);
	
	KalmanCreate(&gimbal_angle_pid_Kal[GIMBAL_YAW], 1, 30);
	KalmanCreate(&gimbal_angle_pid_Kal[GIMBAL_M_PITCH], 1, 200);
	KalmanCreate(&gimbal_angle_pid_Kal[GIMBAL_IMU_YAW], 1, 20);
	
	//自瞄相关滤波器
	KalmanCreate(&vision_absolute_Yaw_Kal, 1, 20);//10);	//绝对角度的卡尔曼滤波器
	KalmanCreate(&vision_absolute_Pitch_Kal, 1, 20);
	KalmanCreate(&vision_dis_pitch_offset, 1, 30);
	
	KalmanCreate(&vision_angleY_KF, 1, 0);
	KalmanCreate(&vision_angleP_KF, 1, 0);
	KalmanCreate(&vision_speedY_KF, 1, 40);//5);
	KalmanCreate(&vision_speedP_KF, 1, 40);
	KalmanCreate(&vision_accelY_KF, 1, 10);
	KalmanCreate(&vision_accelP_KF, 1, 0);
	
	average_init(&vision_absolute_Y_MF, 5);//绝对角度的滑动平均滤波器
	average_init(&vision_absolute_P_MF, 10);
	
	average_init(&vision_angleY_MF, 1);
	average_init(&vision_angleP_MF, 1);
	average_init(&vision_speedY_MF, 5);//5);
	average_init(&vision_speedP_MF, 10);
	average_init(&vision_accelY_MF, 30);
	average_init(&vision_accelP_MF, 10);
	
	average_init(&Anti_top_yaw_MF, 3);
	
	KalmanCreate(&vision_distance_KF, 1, 30);
	KalmanCreate(&vision_speedD_KF, 1, 100);
	average_init(&vision_distance_MF, 30);
	average_init(&vision_speedD_MF, 80);
	
	
	
	median_init(&Anti_top_yaw_MedianF, 80);
	
	MAF_ANTI_TOP_init(&Absolute_yaw_angle_raw, AUTO_ANTI_TOP_TICK/10);//以近AUTO_ANTI_TOP_TICK/10帧作为反陀螺的判断
	MAF_ANTI_TOP_init(&Absolute_pitch_angle_raw, AUTO_ANTI_TOP_TICK/10);
	MAF_ANTI_TOP_init(&Absolute_distance_raw, 3);
	KalmanCreate(&AntiTop_dis_pitch_offset, 1, 10);

	average_init(&yaw_raw_ms, 99);
	average_init(&pitch_raw_ms, 99);
	average_init(&gimbal_now_yaw_raw, 99);
	average_init(&gimbal_now_pitch_raw, 99);
	average_init(&zhen_time_ms, 99);
	
	average_init(&Anti_top_AimPos_MAF, 10);
	
	LPF_Init(&vision_speedY_LPF, 5);
	LPF_Init(&vision_speedP_LPF, 1);
	
}

void GIMBAL_GetInfo(void)	//更新本地操作模式
{
	GIMBAL_GetSysInfo();								//获取系统整体模式状态
	GIMBAL_GetJudgeInfo();							//判断云台有效性
	GIMBAL_GetRcInfo();								//获取遥控器信息
	//GIMBAL_GetTopGyroInfo();						//判断是否处于旋转状态
	//GIMBAL_GetSelfAttitude();					//获取自身？？？	
	GIMBAL_UpdateController();					//更新控制器信息内容
	VISION_PREDICT();
}

void GIMBAL_SelfProtect(void)
{
	GIMBAL_Stop(gimbal_motor_pid);
	GIMBAL_PidParamsInit(gimbal_motor_pid, GIMBAL_MOTOR_CNT);
	first_flag = 0;
	GIMBAL_GetInfo();
}



void GIMBAL_PidCtrl(void)
{
	//计算pid
	if(gimbal.info->co_mode == CO_MECH)
	{
		GIMBAL_Angle_PidCalc(gimbal_motor_pid, GIMBAL_YAW);
	}
	else if(gimbal.info->co_mode == CO_GYRO)
	{
		GIMBAL_Angle_PidCalc(gimbal_motor_pid, GIMBAL_IMU_YAW);
	}
	
	GIMBAL_Angle_PidCalc(gimbal_motor_pid, GIMBAL_M_PITCH);
	
	GIMBAL_Angle_PidCalc(gimbal_motor_pid, GIMBAL_IMU_PITCH);
	
	// 云台电机速度环
	//GIMBAL_Speed_PidCalc(gimbal_motor_pid, GIMBAL_YAW);
	//GIMBAL_Speed_PidCalc(gimbal_motor_pid, GIMBAL_M_PITCH);
	
	// 云台电机输出响应
	GIMBAL_PidOut(gimbal_motor_pid);
}

void GIMBAL_KEY_GET_TARGET(void)
{
	
}

void GIMBAL_NormalCtrl(void)
{
	float pre_target_pitch;
	static uint32_t key_press_tick;
	launcher.info->fire_permit_flag = 1;
	
	if(gimbal.info->co_mode == CO_MECH)
	{
		
		if(IF_KEY_PRESSED_C)
		{
			TargetYaw = -0.0001f*(rc_move_info.key_d-rc_move_info.key_a);
			TargetPitch = 0.0001f*(rc_move_info.key_w-rc_move_info.key_s);
		}
		else
		{
			TargetPitch = -(float)(rc_move_info.mouse_vy)*gimbal_mouse_vp;
			TargetYaw = 0;
		}
		
		
		gimbal_motor_pid[GIMBAL_YAW].angle.target = gimbal_motor_pid[GIMBAL_YAW].angle.target + TargetYaw;
		
		TargetPitch = gimbal_motor_pid[GIMBAL_M_PITCH].angle.target - TargetPitch;
		gimbal_motor_pid[GIMBAL_M_PITCH].angle.target = constrain(TargetPitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
		
		
	}
	else if(gimbal.info->co_mode == CO_GYRO)//
	{
		TargetPitch = -(float)(rc_move_info.mouse_vy)*gimbal_mouse_vp;
		TargetYaw = -(float)(rc_move_info.mouse_vx)*gimbal_mouse_vy;
		
		
		TargetYaw = TargetYaw + gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target;
		if(TargetYaw>(GIMBAL_YAW_CIRCULAR_STEP/2))
		{
			TargetYaw -= GIMBAL_YAW_CIRCULAR_STEP;
		}
		else if(TargetYaw<-(GIMBAL_YAW_CIRCULAR_STEP/2))
		{
			TargetYaw += GIMBAL_YAW_CIRCULAR_STEP;
		}
		gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target = constrain(TargetYaw, -GIMBAL_YAW_CIRCULAR_STEP/2, GIMBAL_YAW_CIRCULAR_STEP/2);
		
		if(IF_KEY_PRESSED_G)//不更新pitch目标
		{

		}
		else//更新pitch目标
		{
			pre_target_pitch = gimbal_motor_pid[GIMBAL_M_PITCH].angle.target;
			pre_target_pitch = pre_target_pitch - TargetPitch;
			gimbal_motor_pid[GIMBAL_M_PITCH].angle.target = constrain(pre_target_pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
		}
		
		//按键部分
		if(gimbal.gimbal_imu_if_back_mid())
		{
			
//			if(IF_KEY_PRESSED_G && key_lock_G == false)//下降沿
//			{
//				//gimbal_motor_pid[GIMBAL_M_PITCH].angle.target = constrain(gimbal_motor_pid[GIMBAL_M_PITCH].angle.target + 500, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
//				gimbal_motor_pid[GIMBAL_M_PITCH].angle.target = 2689;
//				key_lock_G = true;
//			}
//			else if(IF_KEY_PRESSED_G == 0)
//			{
//				key_lock_G = false;
//			}
			
			
			if(IF_KEY_PRESSED_Q && key_lock_Q == false)//下降沿
			{
				TargetYaw = gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target + GIMBAL_YAW_CIRCULAR_STEP/4;
				
				key_lock_Q = true;
			}
			else if(IF_KEY_PRESSED_Q == 0)
			{
				key_lock_Q = false;
			}
			
			
			if(IF_KEY_PRESSED_E && key_lock_E == false)//下降沿
			{
				TargetYaw = gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target - GIMBAL_YAW_CIRCULAR_STEP/4;//核心控制
				key_lock_E = true;
			}
			else if(IF_KEY_PRESSED_E == 0)
			{
				key_lock_E = false;
			}
			
			if(IF_KEY_PRESSED_V && key_lock_V == false)//下降沿
			{
				if(IF_KEY_PRESSED_A)
				{
					TargetYaw = gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target + GIMBAL_YAW_CIRCULAR_STEP/2;
				}
				else 
				{
					TargetYaw = gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target - GIMBAL_YAW_CIRCULAR_STEP/2;
				}
				
				key_lock_V = true;
			}
			else if(IF_KEY_PRESSED_V == 0)
			{
				key_lock_V = false;
			}
			
			
		}
	}
	
	
	if(TargetYaw>(GIMBAL_YAW_CIRCULAR_STEP/2))
	{
		TargetYaw -= GIMBAL_YAW_CIRCULAR_STEP;
	}
	else if(TargetYaw<-(GIMBAL_YAW_CIRCULAR_STEP/2))
	{
		TargetYaw += GIMBAL_YAW_CIRCULAR_STEP;
	}
	gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target = constrain(TargetYaw, -GIMBAL_YAW_CIRCULAR_STEP/2, GIMBAL_YAW_CIRCULAR_STEP/2);

	
	TargetYaw = 0;
	TargetPitch = 0;
}

float get_auto_speed(moving_Average_Filter *AUTO_MF, float angle)
{
	float speed;

	if((angle-(AUTO_MF->aver_num))>GIMBAL_YAW_CIRCULAR_STEP/2 )
	{
			angle-=GIMBAL_YAW_CIRCULAR_STEP;
	}
	else if((angle-(AUTO_MF->aver_num))<-(GIMBAL_YAW_CIRCULAR_STEP/2))
	{
			angle+=GIMBAL_YAW_CIRCULAR_STEP;
	}

	//last_angle = Vision_y_angle.aver_num;
	//average_add(AUTO_MF, angle);

	speed = (-(AUTO_MF->aver_num) + angle);


    return speed;
}

float get_auto_speed_V2(float angle)
{
	float speed;
	static float last_raw_angle;
	
	if((angle-last_raw_angle)>GIMBAL_YAW_CIRCULAR_STEP/2 )
	{
			angle-=GIMBAL_YAW_CIRCULAR_STEP;
	}
	else if((angle-last_raw_angle)<-(GIMBAL_YAW_CIRCULAR_STEP/2))
	{
			angle+=GIMBAL_YAW_CIRCULAR_STEP;
	}

	//last_angle = Vision_y_angle.aver_num;
	//average_add(AUTO_MF, angle);

	speed = (-last_raw_angle + angle);


    return speed;
}


float get_auto_accel(moving_Average_Filter *AUTO_MF ,float speed)
{
	 float accel;

   // average_add(AUTO_MF, speed);

    accel = (-(AUTO_MF->aver_num) + speed);

    return accel;
}


float now_yaw, now_pitch;							//计算开始时的云台角度值
float now_imu_pitch, target_imu_pitch;

float last_yaw_angle_raw = 0;
float yaw_angle_raw, pitch_angle_raw;//绝对角度原始值和滤波值
float auto_yaw_angle, auto_pitch_angle, auto_distance;

float yaw_speed_raw, pitch_speed_raw;//绝对速度原始值和滤波值
float auto_yaw_speed, auto_pitch_speed, auto_distance_speed;

float yaw_accel_raw, pitch_accel_raw;			//绝对加速度原始值和滤波值
float auto_yaw_accel, auto_pitch_accel, auto_distance_accel;

float distance_raw, distance_speed_raw, distance_accel_raw;

//dis的speed kp部分在靠近的时候用80就够了，远离应该要大一点
float auto_distance_speed_kp = 80, auto_distance_accel_kp = 0;
float auto_pitch_speed_kp = 0, auto_pitch_accel_kp = 0;
float auto_yaw_speed_kp = -0, auto_yaw_accel_kp = 0;	//速度与加速度的预测比例

float predict_err_yaw = 0, predict_err_pitch = -240;//角度偏移量

float feed_pre_angle_yaw, predict_angle_yaw;	//预测偏移量
float feed_pre_angle_pitch, predict_angle_pitch;
float feed_pre_distance, predict_distance;

//理论上kp得和实际的角度比例一样，但是pitch 过大会超调，现在这样感觉好像还行，但是如果加上预测可能就不准了
float vision_y_kp = 33.5f, vision_p_kp = 22.755f;

uint32_t times_before_pre = 0;//未开启自瞄，或者目标丢失后设0
uint32_t times_before = 5;	//130 //times_before_pre大于times_before值时开启预测
float last_angle;
int32_t time_ms;
uint32_t target_lost_flag = 1;

float Auto_yaw_speed_offset, Auto_pitch_speed_offset;
float Auto_yaw_accel_offset, Auto_pitch_accel_offset;
float distance_offset = 0;
float dis_to_pitch_offset = 0;
int32_t vision_lost_flag = 0;
uint8_t temp_i = 12;//开图15，不开图18
int temp_i2 = 0;//不开图+前两帧取15， +前一帧取22
int temp_ms_zhen = 0;
float temp_yaw_zhen;
int temp_i3 = 0;

float Anti_top_pre_kp = 1.0f;
void VISION_PREDICT(void)
{
	
	static uint32_t blank_times = 0;
	uint8_t i = 0;
	int n_num = 0;
	
	now_yaw = gimbal_imu_yaw_angle_sum;
	now_pitch = gimbal_motor_pid[GIMBAL_M_PITCH].angle.measure;
	now_imu_pitch = (gimbal.dev->gimbal_imu_sensor->pitch);

	average_add(&gimbal_now_yaw_raw, now_yaw);
	average_add(&gimbal_now_pitch_raw, now_pitch);
	
	if(gimbal.info->local_mode != GIMBAL_MODE_AUTO)//不开启自瞄时，也开启预测
	{
		//times_before_pre = 0;
		//predict_angle_yaw = now_yaw;
		//predict_angle_pitch = now_pitch;
	}
	
	if(vision_sensor.info->flag == 1)//识别到的情况
	{
		
		vision_sensor.info->flag = 0;//清除接收标志，避免没有接收到时连续进入
		
		if(vision_sensor.info->zhen<40 && 0)//帧率小于40当作丢失
		{
			target_lost_flag = 1;
		}
		else
		{
			times_before_pre++;
		}
		
		//一个是修改time_ms
		time_ms = (int)Vision_get_interval(target_lost_flag);

		temp_ms_zhen = average_get(&zhen_time_ms, 2) + average_get(&zhen_time_ms, 3) + average_get(&zhen_time_ms, 1);
		
		yaw_angle_raw = (-vision_sensor.info->yaw_angle)*vision_y_kp + average_get(&yaw_raw_ms, temp_ms_zhen+temp_i2);//获取前一定帧数的yaw角
		pitch_angle_raw = (-vision_sensor.info->pitch_angle)*vision_p_kp + average_get(&pitch_raw_ms, temp_ms_zhen+5);
		target_imu_pitch = now_imu_pitch + (-vision_sensor.info->pitch_angle);
		//调试时主要看绝对角度是否有变化，理想情况下，目标不动解算出来的结果应该是不变的
		//绝对角度滤波
		
		
		if(target_lost_flag >= 1)//丢失后重装载值，避免角度突变
		{
				for(i = 0; i<vision_angleY_MF.lenth; i++)
				{
						average_add(&vision_angleY_MF, yaw_angle_raw);
						
				}
				for(i = 0; i<vision_angleP_MF.lenth; i++)
				{
					average_add(&vision_angleP_MF, pitch_angle_raw);
				}
				last_angle = yaw_angle_raw;
				
				for(i = 0; i<25; i++)//卡尔曼滤波
				{
					KalmanFilter(&vision_angleY_KF, vision_angleY_MF.aver_num);
				}
		}
		average_add(&vision_angleY_MF, yaw_angle_raw);
		average_add(&vision_angleP_MF, pitch_angle_raw);
		
		
		auto_pitch_angle = vision_angleP_MF.aver_num;
		
		yaw_speed_raw =  get_auto_speed(&vision_angleY_MF, last_angle)/time_ms;//先滤波再求速度
		//yaw_speed_raw =  get_auto_speed_V2(yaw_angle_raw)/time_ms;//先求速度再滤波，减少滤波延迟叠加
		pitch_speed_raw = get_auto_speed(&vision_angleP_MF, auto_pitch_angle)/time_ms;
		
		auto_yaw_angle = KalmanFilter(&vision_angleY_KF, vision_angleY_MF.aver_num);
		
		//算出速度值后需要加一个低通滤波，防止小陀螺阴人
		yaw_speed_raw = LPF_add(&vision_speedY_LPF, yaw_speed_raw);
		pitch_speed_raw = LPF_add(&vision_speedP_LPF, pitch_speed_raw);
		
		
		Predict_Anti_Top_binary_update(auto_yaw_angle);
		//判断是否跳变, 如果跳变了，就给反陀螺留下边界
		if(vision_speedY_LPF.High_flag)
		{
			Predict_Anti_Top_Cal_all(last_angle, yaw_angle_raw);
		}
		last_angle = auto_yaw_angle;
		
		if(target_lost_flag >= 1)//丢失后重装载0，避免速度突变
		{
				for(i = 0; i<vision_speedY_MF.lenth; i++)
				{
						average_add(&vision_speedY_MF, 0);
				}
				for(i = 0; i<vision_speedP_MF.lenth; i++)
				{
						average_add(&vision_speedP_MF, 0);
				}
		}
		
		average_add(&vision_speedY_MF, yaw_speed_raw);
		average_add(&vision_speedP_MF, pitch_speed_raw);
		
		auto_yaw_speed = KalmanFilter(&vision_speedY_KF, vision_speedY_MF.aver_num);
		auto_pitch_speed = KalmanFilter(&vision_speedP_KF, vision_speedP_MF.aver_num);
		
		//加速度项太小了，乘100倍放大一下
		yaw_accel_raw = 100*get_auto_accel(&vision_speedY_MF, auto_yaw_speed)/time_ms;
		pitch_accel_raw = 100*get_auto_accel(&vision_speedP_MF, auto_pitch_speed)/time_ms;
		
		if(target_lost_flag >= 1)//丢失后重装载值，避免速度突变
		{
				for(i = 0; i<vision_accelY_MF.lenth; i++)
				{
						average_add(&vision_accelY_MF, 0);
				}
				for(i = 0; i<vision_accelY_MF.lenth; i++)
				{
						average_add(&vision_accelP_MF, 0);
				}
		}
		average_add(&vision_accelY_MF, yaw_accel_raw);
		average_add(&vision_accelP_MF, pitch_accel_raw);
		
		auto_yaw_accel = KalmanFilter(&vision_accelY_KF, vision_accelY_MF.aver_num);
		auto_pitch_accel = KalmanFilter(&vision_accelP_KF, vision_accelP_MF.aver_num);
		
		target_lost_flag = 0;
		vision_lost_flag = 0;
		
		
		//计算自瞄获得的实际距离
		distance_raw = vision_sensor.info->distance;//距离
		if(target_lost_flag >= 1)//丢失后重装载值，避免速度突变
		{
				for(i = 0; i<vision_distance_MF.lenth; i++)
				{
						average_add(&vision_distance_MF, distance_raw);
				}
		}
		average_add(&vision_distance_MF, distance_raw);
		auto_distance = KalmanFilter(&vision_distance_KF, vision_distance_MF.aver_num);
		distance_speed_raw = get_auto_speed(&vision_distance_MF, distance_raw)/time_ms;//距离变化速度
		if(target_lost_flag >= 1)//丢失后重装载值，避免加速度突变
		{
				for(i = 0; i<vision_speedY_MF.lenth; i++)
				{
						average_add(&vision_speedD_MF, 0);
				}
		}
		average_add(&vision_speedD_MF, distance_speed_raw);
		
		auto_distance_speed = KalmanFilter(&vision_speedD_KF, vision_speedD_MF.aver_num);
		distance_accel_raw = get_auto_accel(&vision_speedD_MF, distance_speed_raw)/time_ms;//远离加速度
		
		predict_distance = auto_distance;
		

	}
	else if(vision_sensor.info->flag == 2 || (vision_sensor.info->flag == 0 && vision_sensor.info->identify_target == 1))//识别更新间隔
	{
		
			vision_lost_flag++;
		
			if(vision_lost_flag>200)//失联
			{
				vision_sensor.info->identify_target = 0;
			}
			average_add(&vision_distance_MF, distance_raw);
		
	}
	else if(vision_sensor.info->flag == 0 && vision_sensor.info->identify_target == 0)//目标丢失
	{
		average_clear(&vision_accelY_MF);//清空
		average_clear(&vision_speedY_MF);
		average_clear(&vision_accelP_MF);
		average_clear(&vision_speedP_MF);
		//KalmanFilter(&vision_angleY_KF, now_yaw);//丢失后一直处于跟随当前角度
		//KalmanFilter(&vision_angleP_KF, now_pitch);
		KalmanClear(&vision_speedY_KF);//丢失后清空速度卡尔曼滤波器内容
		LPF_Clear(&vision_speedY_LPF);//低通滤波清理
		LPF_Clear(&vision_speedP_LPF);
		//KalmanFilter(&vision_dis_pitch_offset, 60.0f);//不更新或许更好一点
		//auto_pitch_angle = KalmanFilter(&vision_angleP_KF, now_pitch);//丢失后一直处于跟随当前角度
		
		times_before_pre = 0;
		target_lost_flag += 1;
	}
	
	predict_angle_yaw = auto_yaw_angle;//每时更新，绝对角度,这样就相当与每次只增加一次距离
	predict_angle_pitch = auto_pitch_angle;//每时更新，绝对角度
	
	
	
	//根据弹丸速度（也可以说是弹丸飞行时间）修改自瞄预测距离的大小
	if(Friction_speed <4000)
	{
		
		if(auto_distance>4000)
		{
			auto_distance = 4000;
		}
		
		auto_yaw_speed_kp = (auto_distance * -0.1124f-244.67f)*1.5f;//9m/s 公式
		auto_yaw_speed_kp *= 0.4f;
		
		
	}
	else if(Friction_speed >4000)
	{
		auto_yaw_speed_kp = auto_distance * -0.1124f-244.67f;//14m/s 公式
		auto_yaw_speed_kp *= 0.5f;
		
		//auto_yaw_speed_kp = (auto_distance * -0.1124f-244.67f)*0.6f;//看哪个准一点
		//auto_yaw_speed_kp *= 0.4f;
	}
	
	Auto_yaw_speed_offset = auto_yaw_speed_kp *auto_yaw_speed;
	Auto_yaw_accel_offset = auto_yaw_accel*auto_yaw_accel_kp;
	Auto_pitch_speed_offset = auto_pitch_speed*auto_pitch_speed_kp;
	Auto_pitch_accel_offset = auto_pitch_accel*auto_pitch_accel_kp;
	
	//只有自瞄比较稳定的时候才加预测
	if(times_before_pre>times_before)
	{
		//feed_pre_distance = auto_distance_speed_kp*auto_distance_speed; 
		feed_pre_distance = 0;
		
		//预测斜坡
		feed_pre_angle_yaw = RampFloat(feed_pre_angle_yaw, Auto_yaw_speed_offset + Auto_yaw_accel_offset, 3);
		feed_pre_angle_pitch =  RampFloat(feed_pre_angle_pitch, Auto_pitch_speed_offset + Auto_pitch_accel_offset,3);//超前预测角
		//预测不斜坡
		//feed_pre_angle_yaw = Auto_yaw_speed_offset + Auto_yaw_accel_offset;
		//feed_pre_angle_pitch = Auto_pitch_speed_offset + Auto_pitch_accel_offset;//超前预测角
		
		feed_pre_angle_yaw = Auto_yaw_speed_offset + Auto_yaw_accel_offset;
		feed_pre_angle_pitch = Auto_pitch_speed_offset + Auto_pitch_accel_offset;
		
		if(feed_pre_angle_yaw > 300)//预测量限幅
		{
			feed_pre_angle_yaw = 300;
		}
		else if(feed_pre_angle_yaw < -300)
		{
			feed_pre_angle_yaw = -300;
		}
	}
	else 
	{
		feed_pre_angle_yaw = 0.0f*(Auto_yaw_speed_offset + Auto_yaw_accel_offset);
		feed_pre_angle_pitch = 0.0f*(Auto_pitch_speed_offset + Auto_pitch_accel_offset);
		feed_pre_distance = 0;
	}
	
	//根据对方远离我们的速度进行一个仰角预测
	
	if(predict_distance+feed_pre_distance<1500)
	{
		predict_distance = predict_distance;
	}
	else 
	{
		predict_distance+=feed_pre_distance;
	}		
	
	//抬头角
	//根据射速选择不同的距离俯仰角修正策略
	//低射速的时候弹丸的俯仰角变化较大，高射速的时候俯仰角变化较小
	//根据队里要求的命中率，10m/s射速上限（实际射击速度9.2±0.5m/s）在自瞄情况下不应该大于4m. 
	//在大于4m的时候，射速对俯仰角的影响非常大。目前摩擦轮以及枪管做不到低于±0.5m/s的精度，希望有待提高
	//并且由于仰角遮挡视线的原因，该档射速在大于5m开外的水平地面目标，装甲板已经贴近视觉视野的下方。
	
	//根据队里要求的命中率，16m/s射速上限（实际射击速度15.0±0.5m/s）
	//在自瞄情况下可以达到视觉识别距离上限（因为灯条太小而识别不到，反馈距离为10m，实际距离为9m），其命中率为80%
	
	//特别注意的是，在过于贴近的时候，俯仰角必须加大，否则会被反弹击中自己
	dis_to_pitch_offset = predict_cal_shoot_angle(predict_distance, predict_angle_pitch, Friction_speed);
	KalmanFilter(&vision_dis_pitch_offset, dis_to_pitch_offset);
	
	if(vision_dis_pitch_offset.X_now>180)//抬头量最多设置180
	{
			predict_angle_pitch = KalmanFilter(&vision_absolute_Pitch_Kal, predict_angle_pitch+feed_pre_angle_pitch) + 180 + predict_err_pitch;
	}
	else
	{
			predict_angle_pitch = KalmanFilter(&vision_absolute_Pitch_Kal, predict_angle_pitch+feed_pre_angle_pitch) + vision_dis_pitch_offset.X_now + predict_err_pitch;
	}
	
	
	
	
	//predict_angle_yaw = Predict_Anti_Top_Judge_Yaw(predict_angle_yaw);
	if(IF_KEY_PRESSED_C)
	{
		if(ANTI_TOP_TEST_FLAG == 1)
		{
			//普通预测
			predict_angle_yaw = predict_angle_yaw+feed_pre_angle_yaw;
			predict_angle_yaw =  KalmanFilter(&vision_absolute_Yaw_Kal, predict_angle_yaw) + predict_err_yaw;	
	
		}
		else
		{
			//反陀螺
			predict_angle_yaw = predict_angle_yaw+feed_pre_angle_yaw*Anti_top_pre_kp;
			predict_angle_yaw =  KalmanFilter(&vision_absolute_Yaw_Kal, predict_angle_yaw) + predict_err_yaw;	
	
			predict_angle_yaw = Predict_Anti_Top_Judge_Yaw(predict_angle_yaw);
		}
			
	}
	else 
	{
		if(ANTI_TOP_TEST_FLAG == 1)
		{
			//反陀螺
			predict_angle_yaw = predict_angle_yaw+feed_pre_angle_yaw*Anti_top_pre_kp;
			predict_angle_yaw =  KalmanFilter(&vision_absolute_Yaw_Kal, predict_angle_yaw) + predict_err_yaw;	
	
			predict_angle_yaw = Predict_Anti_Top_Judge_Yaw(predict_angle_yaw);
		}
		else
		{
			//普通预测
			predict_angle_yaw = predict_angle_yaw+feed_pre_angle_yaw;
			predict_angle_yaw =  KalmanFilter(&vision_absolute_Yaw_Kal, predict_angle_yaw) + predict_err_yaw;	
	
		}
		//predict_angle_yaw = Predict_Anti_Top_Judge_Yaw(predict_angle_yaw);
	}
	
	
	
	//predict_angle_yaw = constrain(predict_angle_yaw, yaw_angle_raw-300, yaw_angle_raw+300);
	//取出predict，小于整周长的部分
	if(predict_angle_yaw>(GIMBAL_YAW_CIRCULAR_STEP/2))
	{
		n_num =(int)(predict_angle_yaw/(GIMBAL_YAW_CIRCULAR_STEP));//n圈
		
		predict_angle_yaw = predict_angle_yaw-n_num*(GIMBAL_YAW_CIRCULAR_STEP);
		if(predict_angle_yaw>GIMBAL_YAW_CIRCULAR_STEP/2)
		{
			predict_angle_yaw -= GIMBAL_YAW_CIRCULAR_STEP;
		}
	}
	else if(predict_angle_yaw<-(GIMBAL_YAW_CIRCULAR_STEP/2))
	{
		n_num = (int)(predict_angle_yaw/(GIMBAL_YAW_CIRCULAR_STEP));
		predict_angle_yaw = predict_angle_yaw - n_num*(GIMBAL_YAW_CIRCULAR_STEP);
		if(predict_angle_yaw<-(GIMBAL_YAW_CIRCULAR_STEP/2))
		{
			predict_angle_yaw += GIMBAL_YAW_CIRCULAR_STEP;
		}
	}
	
}



float AntiTop_angle_yaw;	//预测偏移量
float AntiTop_angle_pitch;
float AntiTop_distance;
float Anti_dis_to_pitch_offset;
uint16_t pre_temp = 0;
float last_yaw_angle_raw_temp;
float top_moving_speed = 0;
float top_moving_speed_kp = 1;
void vision_anti_top_analyze(void)
{
	int n_num = 0;
	
	
	last_yaw_angle_raw_temp = Absolute_yaw_angle_raw.aver_num;
	MAF_ANTI_TOP_add(&Absolute_yaw_angle_raw, auto_yaw_angle);//利用自瞄中处理好的yaw轴数据，当作一个记录过去yaw轴信息的数组
	MAF_ANTI_TOP_add(&Absolute_pitch_angle_raw, auto_pitch_angle);
	MAF_ANTI_TOP_add(&Absolute_distance_raw, auto_distance);
	median_add(&Anti_top_yaw_MedianF, yaw_angle_raw);
	
	//瞄中值
	//AntiTop_angle_yaw = (Anti_top_yaw_MedianF.data_num[Anti_top_yaw_MedianF.lenth-1] + Anti_top_yaw_MedianF.data_num[0])/2;
	
	//瞄均值
	//AntiTop_angle_yaw = Absolute_yaw_angle_raw.aver_num;
	//AntiTop_angle_pitch = predict_angle_pitch;
	//AntiTop_distance = Absolute_distance_raw.aver_num;
	
	if(AntiTop_angle_yaw>(GIMBAL_YAW_CIRCULAR_STEP/2))
	{
		n_num =(int)(AntiTop_angle_yaw/(GIMBAL_YAW_CIRCULAR_STEP));//n圈
		
		AntiTop_angle_yaw = AntiTop_angle_yaw-n_num*(GIMBAL_YAW_CIRCULAR_STEP);
		if(AntiTop_angle_yaw>GIMBAL_YAW_CIRCULAR_STEP/2)
		{
			AntiTop_angle_yaw -= GIMBAL_YAW_CIRCULAR_STEP;
		}
	}
	else if(AntiTop_angle_yaw<-(GIMBAL_YAW_CIRCULAR_STEP/2))
	{
		n_num = (int)(AntiTop_angle_yaw/(GIMBAL_YAW_CIRCULAR_STEP));
		AntiTop_angle_yaw = AntiTop_angle_yaw - n_num*(GIMBAL_YAW_CIRCULAR_STEP);
		if(AntiTop_angle_yaw<-(GIMBAL_YAW_CIRCULAR_STEP/2))
		{
			AntiTop_angle_yaw += GIMBAL_YAW_CIRCULAR_STEP;
		}
	}
	
	//抬头角
	if(Friction_speed <5200)
	{
		if(AntiTop_distance>2000)
		{
			Anti_dis_to_pitch_offset = 0.0316f*(AntiTop_distance)+50.719f;
		}
		else if(AntiTop_distance>1000)
		{
			if(abs(Anti_dis_to_pitch_offset-170)<0.1f)
			{
				if(AntiTop_distance<1000+400)
				{
					Anti_dis_to_pitch_offset = 110;
				}
				else
				{
					Anti_dis_to_pitch_offset = 170;
				}
			}
			else 
			{
				Anti_dis_to_pitch_offset = 110;
			}
			
		}
		else if(AntiTop_distance<1000)
		{
			if(abs(Anti_dis_to_pitch_offset-110)<0.1f)//110的时候
			{
				if(AntiTop_distance<1000-400)//只有低于800才进入170
				{
					Anti_dis_to_pitch_offset = 170;
				}
				else
				{
					Anti_dis_to_pitch_offset = 110;
				}
			}
			else
			{
				Anti_dis_to_pitch_offset = 170;
			}
		}
	
		KalmanFilter(&AntiTop_dis_pitch_offset, Anti_dis_to_pitch_offset);//距离公式9.2m/s线性
	}
	else if(Friction_speed > 5200)
	{
		
		if(AntiTop_distance>800)
		{
			if(abs(Anti_dis_to_pitch_offset-200)<0.1f)//200的情况
			{
				if(AntiTop_distance>800)
				{
					Anti_dis_to_pitch_offset = (7*AntiTop_distance*AntiTop_distance + 43000*AntiTop_distance)/10000000.0f;//距离公式14.0m/s
					Anti_dis_to_pitch_offset += 58.904f;
				}
				else
				{
					Anti_dis_to_pitch_offset = 200;
				}
			}
			else 
			{
				Anti_dis_to_pitch_offset = (7*AntiTop_distance*AntiTop_distance + 43000*AntiTop_distance)/10000000.0f;//距离公式14.0m/s
				Anti_dis_to_pitch_offset += 58.904f;
			}
		}
		else if(AntiTop_distance<800)
		{
			if(AntiTop_distance<800-300)
			{
				Anti_dis_to_pitch_offset = 200;
			}
		}
		Anti_dis_to_pitch_offset-=6;
		KalmanFilter(&AntiTop_dis_pitch_offset, Anti_dis_to_pitch_offset);
	}
	
	
	AntiTop_angle_pitch+=AntiTop_dis_pitch_offset.X_now;
}

float yaw_speed_offset_limit = 100, yaw_accel_offset_limit = 20;
float anti_top_limit = 50;
float anti_top_err = 0;
int anti_top_aim_pos = 0;
void GIMBAL_AUTO(void)
{
	//anti_top_flag = 0;
	
	int distance_dm = 0;
	float aim_yaw, aim_pitch;

	
	if(IF_KEY_PRESSED_X)
	{
		launcher.info->fire_permit_flag = 1;
		//TargetYaw = AntiTop_angle_yaw;
		//TargetPitch = Absolute_pitch_angle_raw.aver_num;//均值高度
		TargetPitch = predict_angle_pitch;//现在高度
		
		//操作手自己瞄准陀螺
		TargetYaw = -(float)(rc_move_info.mouse_vx)*gimbal_mouse_vy;
		TargetYaw = TargetYaw + gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target;
		
		launcher.info->fire_permit_flag = 1;
		
		if(TargetYaw>(GIMBAL_YAW_CIRCULAR_STEP/2))
		{
			TargetYaw -= GIMBAL_YAW_CIRCULAR_STEP;
		}
		else if(TargetYaw<=-(GIMBAL_YAW_CIRCULAR_STEP/2))
		{
			TargetYaw += GIMBAL_YAW_CIRCULAR_STEP;
		}

		if(target_lost_flag<1)
		{
			
			aim_pitch = constrain(TargetPitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
			//gimbal_motor_pid[GIMBAL_M_PITCH].angle.target = RampFloat(aim_pitch, gimbal_motor_pid[GIMBAL_M_PITCH].angle.target, 3);
			gimbal_motor_pid[GIMBAL_M_PITCH].angle.target = aim_pitch;
		}
		aim_yaw = constrain(TargetYaw, -GIMBAL_YAW_CIRCULAR_STEP/2, GIMBAL_YAW_CIRCULAR_STEP/2);
		gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target =	aim_yaw;

	}
	else 
	{
		launcher.info->fire_permit_flag = 1;
		
		TargetYaw = predict_angle_yaw;
		TargetPitch = predict_angle_pitch;
		
		
		launcher.info->fire_permit_flag = 1;
		
		if(TargetYaw>(GIMBAL_YAW_CIRCULAR_STEP/2))
		{
			TargetYaw -= GIMBAL_YAW_CIRCULAR_STEP;
		}
		else if(TargetYaw<=-(GIMBAL_YAW_CIRCULAR_STEP/2))
		{
			TargetYaw += GIMBAL_YAW_CIRCULAR_STEP;
		}
		
		if(target_lost_flag<1)
		{
			aim_yaw = constrain(TargetYaw, -GIMBAL_YAW_CIRCULAR_STEP/2, GIMBAL_YAW_CIRCULAR_STEP/2);
			aim_pitch = constrain(TargetPitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
			
//			gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target =	RampFloat(aim_yaw, gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target, 3);
//			gimbal_motor_pid[GIMBAL_M_PITCH].angle.target = RampFloat(aim_pitch, gimbal_motor_pid[GIMBAL_M_PITCH].angle.target, 3);	
			gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target = aim_yaw;
			
			if(IF_KEY_PRESSED_G)//不更新pitch目标
			{

			}
			else//更新pitch目标
			{
				gimbal_motor_pid[GIMBAL_M_PITCH].angle.target = aim_pitch;
			}
		}
	}
	
	TargetPitch	= gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target;
	TargetYaw = gimbal_motor_pid[GIMBAL_M_PITCH].angle.target;
}


void GIMBAL_ReloadBulletCtrl(void)
{
}

void GIMBAL_SzuPupCtrl(void)
{
}

void GIMBAL_LONGSHOTCtrl(void)
{
}

void GIMBAL_RcCtrl(void)
{
	float pre_target_pitch;
	launcher.info->fire_permit_flag = 1;
	
	if(gimbal.info->co_mode == CO_MECH)
	{
		
		TargetPitch = (float)(gimbal.dev->rc_sensor->info->ch1)*0.017f;
		TargetYaw = 0;
		
		gimbal_motor_pid[GIMBAL_YAW].angle.target = TargetYaw + gimbal_motor_pid[GIMBAL_YAW].angle.target;
		
		TargetPitch = gimbal_motor_pid[GIMBAL_M_PITCH].angle.target - TargetPitch;
		gimbal_motor_pid[GIMBAL_M_PITCH].angle.target = constrain(TargetPitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
		
	}
	else if(gimbal.info->co_mode == CO_GYRO)//
	{
		if(sys.mode != SYS_MODE_AUTO)
		{
			TargetPitch = (float)(gimbal.dev->rc_sensor->info->ch1)*0.017f;
			TargetYaw = -(float)(gimbal.dev->rc_sensor->info->ch0)*0.034f;
			
			
			TargetYaw = TargetYaw + gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target;
			if(TargetYaw>(GIMBAL_YAW_CIRCULAR_STEP/2))
			{
				TargetYaw -= GIMBAL_YAW_CIRCULAR_STEP;
			}
			else if(TargetYaw<-(GIMBAL_YAW_CIRCULAR_STEP/2))
			{
				TargetYaw += GIMBAL_YAW_CIRCULAR_STEP;
			}
			//gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target = constrain(TargetYaw, -GIMBAL_YAW_CIRCULAR_STEP/2, GIMBAL_YAW_CIRCULAR_STEP/2);
			gimbal_motor_pid[GIMBAL_IMU_YAW].angle.target = TargetYaw;
			pre_target_pitch = gimbal_motor_pid[GIMBAL_M_PITCH].angle.target;
			pre_target_pitch = pre_target_pitch - TargetPitch;
			gimbal_motor_pid[GIMBAL_M_PITCH].angle.target = constrain(pre_target_pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
		}

	}
	
	TargetYaw = 0;
	TargetPitch = 0;
}

void GIMBAL_KeyCtrl(void)
{
	
	switch(gimbal_info.local_mode)
	{
		case GIMBAL_MODE_NORMAL:
			GIMBAL_NormalCtrl();
			break;
		case GIMBAL_MODE_AUTO:
			GIMBAL_AUTO();
			break;
		case GIMBAL_MODE_LONGSHOOT:
			GIMBAL_LONGSHOTCtrl();
			break;
		case GIMBAL_MODE_RELOAD_BULLET:
			GIMBAL_ReloadBulletCtrl();
			break;
		case GIMBAL_MODE_SZUPUP:
			GIMBAL_SzuPupCtrl();
			break;
		default:
			break;
	}
}



void GIMBAL_Angle_deliver(void)
{
	
	
	
	
	
	
}

void GIMBAL_Ctrl(void)
{
	/*----信息读入----*/
	GIMBAL_GetInfo();
	/*----期望修改----*/ 
	if(gimbal_info.remote_mode == RC) {
		GIMBAL_RcCtrl();
	}
	else if(gimbal_info.remote_mode == KEY) {
		GIMBAL_KeyCtrl();
	}
	GIMBAL_Angle_deliver();
	/*----最终输出----*/
	GIMBAL_PidCtrl();	
}

void GIMBAL_Test(void)
{
	
	/*----信息读入----*/
	GIMBAL_GetInfo();
	/*----期望修改----*/ 
	if(gimbal_info.remote_mode == RC) {
		GIMBAL_RcCtrl();
	}
	else if(gimbal_info.remote_mode == KEY) {
		GIMBAL_AUTO();
	}
	GIMBAL_Angle_deliver();
	/*----最终输出----*/
	GIMBAL_PidCtrl();	
	
}


void gimbal_now_angle_ms_update()
{	
	float now_ms_yaw, now_ms_pitch;
	now_ms_yaw = gimbal_imu_yaw_angle_sum;
	now_ms_pitch = gimbal_motor_pid[GIMBAL_M_PITCH].angle.measure;
	
	average_add(&yaw_raw_ms, now_yaw);
	average_add(&pitch_raw_ms, now_pitch);

}



