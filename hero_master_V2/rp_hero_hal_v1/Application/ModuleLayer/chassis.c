/**
 * @file        chassis.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        23-October-2020
 * @brief       Chassis Module.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "chassis.h"
#include "cmsis_os.h"
#include "judge.h"
#include "control.h"
#include "can_potocol.h"
#include "rp_math.h"
#include "kalman.h"
#include "kalman_filter.h"

/* Private macro -------------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/

static float GetRealMechAngleError(pid_ctrl_t *pid);	//就近角度差
static int16_t CHASSIS_GetMiddleAngle(void);					//归中角度
static int16_t CHASSIS_GetMiddleAngleOffset(void);		//归中角度的差值
bool CHASSIS_IfBackToMiddleAngle(void);								//回正与否
static bool CHASSIS_IfTopGyroOpen(void);							//陀螺与否
static bool CHASSIS_IfGyrateOpen(void);							//陀螺与否


void CHASSIS_Init(void);
void CHASSIS_Ctrl(void);
void CHASSIS_Test(void);
void CHASSIS_SelfProtect(void);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// 底盘电机本地驱动
drv_can_t				*chas_drv[CHAS_MOTOR_CNT];
chassis_motor_t			*chas_motor[CHAS_MOTOR_CNT];
chassis_motor_info_t	*chas_motor_info[CHAS_MOTOR_CNT];

//麦轮底盘运动三变量
short LOGIC_Vx = 0, LOGIC_Vy = 0, LOGIC_Vz = 0;//云台模式下的三方向速度
short Vx = 0, Vy = 0, Vz = 0;
uint8_t gyrate_open_flag = 0;

float Rudder_angle = 0;


//根据功率缓冲设置的Kp参数
float speed_power_Kp = 0.7f;	

// 底盘PID输出最大值
uint16_t CHAS_SPEED_MAX	= 8000;
uint16_t CHAS_PID_OUT_MAX = 9000;

// 卡尔曼滤波器,底盘角度
extKalman_t chas_kalman_err;

short Top_Speed = 3000;

// 遥控按键信息
bool rc_lock_Wheel = false;
bool rc_unlock_Wheel = false;
static uint8_t key_lock_F = false;
static uint8_t key_lock_C = false;

/* 斜坡 */
int16_t time_fron_y = 0;
int16_t time_back_y = 0;
int16_t time_left_x = 0;
int16_t time_righ_x = 0;
uint16_t Time_Inc_Normal;			// 正常斜坡增加量
uint16_t Time_Inc_Saltation = 1;	// 前后方向突变斜坡增加量

/* 键盘模式下的前后左后以及旋转斜坡变量 */
float Chas_Slope_Move_Fron, Chas_Slope_Move_Back, Chas_Slope_Move_Left, Chas_Slope_Move_Righ;
float Chas_Slope_Spin_Move;

/* 期望速度 */
float Chas_Target_Speed[4];

/* 摇杆灵敏度 */
//机械模式下底盘比例系数,控制摇杆响应速度,如果过小也会限制最高转速,max = 此系数 *660
float kRc_Mech_Chas_Standard, kRc_Mech_Chas_Spin;//平移，旋转,

//陀螺仪模式下底盘比例系数,控制摇杆响应速度,如果过小也会限制最高转速,max = 此系数 *660
float kRc_Gyro_Chas_Standard, kRc_Gyro_Chas_Spin;//平移，旋转

//机械模式下底盘比例系数,控制键盘斜坡变化率
float kKey_Mech_Chas_Standard, kKey_Mech_Chas_Spin;//平移，旋转

//陀螺仪模式下底盘比例系数,控制键盘斜坡变化率
float kKey_Gyro_Chas_Standard, kKey_Gyro_Chas_Spin;//平移，旋转

//小陀螺模式下底盘比例系数
float k_Gyro_Chas_Top;

//扭腰模式下底盘比例系数
float k_Gyro_Chas_Twist;

//小陀螺旋转转速步长
float Chas_Top_Gyro_Step = 8;
uint16_t max_top_gyro_speed = 20;
uint16_t Chas_Top_Gyro_Period = 0;	// 单项赛规则(T为3~7s的随机数)

//扭腰旋转步长
float Chas_Twist_Step;





/* Exported variables --------------------------------------------------------*/
// 底盘电机PID控制器
chassis_motor_pid_t 	chas_motor_pid[CHAS_MOTOR_CNT] = {
	[CHAS_LF] = {
		.speed.kp = 11.0f,
		.speed.ki = 0.25f,
		.speed.kd = 0,
		.speed.integral_max = 20000,
		.speed.out_max = 10000,
		.angle.kp = 0.25f,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 100,
		.angle.out_max = 8000,
	},
	[CHAS_RF] = {
		.speed.kp = 11.0f,
		.speed.ki = 0.25f,
		.speed.kd = 0,
		.speed.integral_max = 20000,
		.speed.out_max = 10000,
		.angle.kp = 0.25,
		.angle.ki = 0,
		.angle.kd = 0.008,
		.angle.integral_max = 100,
		.angle.out_max = 8000,
	},
	[CHAS_LB] = {
		.speed.kp = 11.0f,
		.speed.ki = 0.25f,
		.speed.kd = 0,
		.speed.integral_max = 20000,
		.speed.out_max = 10000,
		.angle.kp = 0.25,
		.angle.ki = 0,
		.angle.kd = 0.008,
		.angle.integral_max = 100,
		.angle.out_max = 8000,
	},
	[CHAS_RB] = {
		.speed.kp = 11.0f,
		.speed.ki = 0.25f,
		.speed.kd = 0,
		.speed.integral_max = 20000,
		.speed.out_max = 10000,
		.angle.kp = 0.25,
		.angle.ki = 0,
		.angle.kd = 0.008,
		.angle.integral_max = 100,
		.angle.out_max = 8000,
	},
};

extKalman_t chas_motor_pid_Kal[CHAS_MOTOR_CNT] = 
{
	[CHAS_LF] = {
		.Q = 1,
		.R = 1,
	},
	[CHAS_RF] = {
		.Q = 1,
		.R = 1,
	},
	[CHAS_LB] = {
		.Q = 1,
		.R = 1,
	},
	[CHAS_RB] = {
		.Q = 1,
		.R = 1,
	},
};



// 底盘Z方向角度控制器
chassis_z_pid_t		chas_z_pid = {
	.angle = {
		.target = CHAS_MECH_ANGLE_POS_MID,
		.measure = CHAS_MECH_ANGLE_POS_MID,
		.kp = 4.0f,
		.ki = 0.0f,
		.kd = 0,
		.integral_max = 3000,
		.out_max = 8000,
	},
	.speed = {
		.target = 0,
		.measure = 0,
		.kp = 2,
		.ki = 0,
		.kd = 0,
		.integral_max = 0,
		.out_max = 30000,
	},
	.out = 0,
};

// 底盘模块控制器
chassis_ctrl_t		chas_ctrl = {
	.motor = &chas_motor_pid,
	.z_atti = &chas_z_pid,
};

// 底盘模块传感器
chassis_dev_t		chas_dev = {
	.chas_motor[CHAS_LF] = &chassis_motor[CHAS_LF],
	.chas_motor[CHAS_RF] = &chassis_motor[CHAS_RF],
	.chas_motor[CHAS_LB] = &chassis_motor[CHAS_LB],
	.chas_motor[CHAS_RB] = &chassis_motor[CHAS_RB],
	.yaw_motor = &gimbal_motor[GIMBAL_YAW],
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
};

// 底盘模块信息
chassis_info_t 	chas_info = {
	.remote_mode = RC,
	.co_mode = CO_MECH,
	.local_mode = CHASSIS_MODE_NORMAL,
	.co_angle_logic = LOGIC_FRONT,
};

chassis_t chassis = {
	.controller = &chas_ctrl,
	.dev = &chas_dev,
	.info = &chas_info,
	.test_open = 0,
	.init = CHASSIS_Init,
	.test = CHASSIS_Test,
	.ctrl = CHASSIS_Ctrl,
	.self_protect = CHASSIS_SelfProtect,
	.if_back_to_mid_angle = CHASSIS_IfBackToMiddleAngle,
};

/* Private functions ---------------------------------------------------------*/
/* 驱动层 --------------------------------------------------------------------*/

/**
 *	@brief	底盘电机PID参数初始化，保护用
 */
static void CHASSIS_PidParamsInit(chassis_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		pid_val_init(&pid[i].angle);
		pid_val_init(&pid[i].speed);
		pid[i].out = 0;
	}	
}

/**
 *	@brief	底盘z角度PID参数初始化，保护用
 */
static void CHASSIS_Z_PidParamsInit(chassis_z_pid_t *pid)
{
	pid_val_init(&pid->angle);
	pid->angle.target = CHASSIS_GetMiddleAngle();
	pid->angle.measure = CHASSIS_GetMiddleAngle();
	pid->out = 0;
	pid->back_flag = 0;
}

/**
 *	@brief	底盘电机卸力，保护用
 */
static void CHASSIS_Stop(chassis_motor_pid_t *pid)
{

	pid[CHAS_LF].out = 0;
	pid[CHAS_RF].out = 0;
	pid[CHAS_LB].out = 0;
	pid[CHAS_RB].out = 0;
	
	
	CAN1_0X200_BUF[0] = 0;
	CAN1_0X200_BUF[1] = 0;
	CAN1_0X200_BUF[2] = 0;
	CAN1_0X200_BUF[3] = 0;
}

/**
 *	@brief	底盘电机PID输出
 */
static void CHASSIS_PidOut(chassis_motor_pid_t *pid)
{
	
	for(uint8_t i=0; i<4; i++) {
		if(chas_motor[i]->work_state == DEV_ONLINE) 
		{
			CAN1_0X200_BUF[chas_drv[i]->drv_id] = (int16_t)(pid[i].out);
		} else {
			CAN1_0X200_BUF[chas_drv[i]->drv_id] = 0;
		}
	}
}

/**
 *	@brief	底盘电机速度环
 */
static void CHASSIS_Speed_PidCalc(chassis_motor_pid_t *pid, chassis_motor_cnt_t MOTORx)
{
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
	pid[MOTORx].speed.err = KalmanFilter(&chas_motor_pid_Kal[MOTORx], pid[MOTORx].speed.err);
	
	single_pid_ctrl(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}

/**
 *	@brief	底盘电机z角度环
 */
extern gimbal_motor_pid_t 	gimbal_motor_pid[GIMBAL_MOTOR_CNT];
static float CHASSIS_Z_Angle_PidCalc(chassis_z_pid_t *pid)
{

	float abs_angle_err = 0;
	float threshold = 120;
	static float Frond_flag = 1;
	
	pid->angle.err = (gimbal_motor_pid[GIMBAL_YAW].angle.target) - (gimbal_motor_pid[GIMBAL_YAW].angle.measure);
	
	if(sys.co_mode == CO_GYRO)//90度回正，根据角度确定灯条还是弹仓朝前 135度优先序
	{
		if(pid->angle.err > GIMBAL_YAW_CIRCULAR_STEP/4+Frond_flag*GIMBAL_YAW_CIRCULAR_STEP/8)
		{
			pid->angle.err -= GIMBAL_YAW_CIRCULAR_STEP/2;
			Frond_flag = -1;
		}
		else if(pid->angle.err < -(GIMBAL_YAW_CIRCULAR_STEP/4+Frond_flag*GIMBAL_YAW_CIRCULAR_STEP/8))
		{	
			Frond_flag = -1;
			pid->angle.err += GIMBAL_YAW_CIRCULAR_STEP/2;
		}
		else
		{
			Frond_flag = 1;
		}
	}

	abs_angle_err = abs(pid->angle.err);
	
	if(abs_angle_err>threshold)
	{
		if(abs_angle_err<1000)
		{
			pid->angle.kp = 4+(abs_angle_err-threshold)*4.0/(1000-threshold);
		}
		else
		{
			pid->angle.kp = 8;
		}
	}
	else
	{
		pid->angle.kp = 4;
	}
	
	/* 未开小陀螺则死区控制 */
	if(CHASSIS_IfTopGyroOpen() == false) {
		if(abs(pid->angle.err) < 10) {
			pid->angle.err = 0;
		}
	}
	
	pid->angle.err = KalmanFilter(&chas_kalman_err, pid->angle.err);
	
	single_pid_ctrl(&pid->angle);
	
	
	return pid->angle.out;
}

/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/

/**
 *	@brief	Z角度环机械角度过零点
 */
static float GetRealMechAngleError(pid_ctrl_t *pid)
{
	float err;
	err = pid->target - pid->measure;
	
	if(err >= 4096.f) {
		err = -8192.f + err;
	} else if(err <= -4096.f) {
		err = +8192.f + err;
	}
	
	return err;
}

/**
 *	@brief	Z角度环机械角度中值
 */
static int16_t CHASSIS_GetMiddleAngle(void)
{
	if(chas_info.co_angle_logic == LOGIC_FRONT) {
		return CHAS_MECH_ANGLE_POS_MID;
	}
	else if(chas_info.co_angle_logic == LOGIC_BACK){
		return CHAS_MECH_ANGLE_NEG_MID;
	}
	return CHAS_MECH_ANGLE_POS_MID;
}

/**
 *	@brief	Z角度机械角度误差
 */
static int16_t CHASSIS_GetMiddleAngleOffset(void)
{
	return chassis.dev->yaw_motor->info->angle_sum;
}

bool CHASSIS_IfBackToMiddleAngle(void)
{
	return ((abs(CHASSIS_GetMiddleAngleOffset())) < (10+50))?true:false;
}

static bool CHASSIS_IfTopGyroOpen(void)
{
	if(chas_info.co_mode == CO_GYRO
	   && chas_info.top_gyro == true)
		return true;
	return false;
}


/**
 *	@brief	底盘获取系统信息
 */
void CHASSIS_GetSysInfo(void)//只要确保随时切换无问题即可
{
	/*----控制方式修改----*/
	if(sys.remote_mode == RC) {
		chas_info.remote_mode = RC;
	}
	else if(sys.remote_mode == KEY) {
		chas_info.remote_mode = KEY;
	}
	
	

	/*----本地模式修改----*/
	switch(sys.mode)	//切换模式
	{
		case SYS_MODE_LONGSHOOT: 
			if(chas_info.local_mode!=CHASSIS_MODE_SZUPUP)
			{
				chas_info.local_mode = CHASSIS_MODE_LONGSHOOT;	
			}
			break;

		case SYS_MODE_AUTO://自瞄模式
			chas_info.local_mode = CHASSIS_MODE_AUTO;
			break;
		case SYS_MODE_NORMAL:		
			chas_info.local_mode = CHASSIS_MODE_NORMAL;
			break;
		case SYS_MODE_PARK: break;
		
		default: break;
	}
	
	if(sys.state == SYS_STATE_RCLOST)
	{
		chas_info.co_mode = sys.co_mode;
	}
	
	if(sys.co_mode != chas_info.co_mode)
	{
		chassis.info->top_gyro = 0;
		
		if(chas_info.co_mode == CO_MECH) 
		{
			chas_info.co_mode = sys.co_mode;
		}
		else if(chas_info.co_mode == CO_GYRO && chassis.if_back_to_mid_angle())
		{
			chas_info.co_mode = sys.co_mode;
		}
		
	}
	
	
	
	
}

void CHASSIS_GetJudgeInfo(void)
{
	//无？ 或者是裁判系统的数据获取
}

void CHASSIS_GetRcInfo(void)
{
	/* 系统正常 */
	if(sys.state == SYS_STATE_NORMAL)
	{
		if(sys.remote_mode == RC) 
		{
			key_lock_F = false;
			key_lock_C = false;
			time_fron_y = 0;
			time_back_y = 0;
			time_left_x = 0;
			time_righ_x = 0;
		}
		else if(sys.remote_mode == KEY) 
		{
			rc_lock_Wheel = false;
			rc_unlock_Wheel = false;
		}
	}
	/* 系统异常 失联 操作 其他 */
	else 
	{
		key_lock_F = false;
		key_lock_C = false;
		rc_lock_Wheel = false;
		rc_unlock_Wheel = false;		
		time_fron_y = 0;
		time_back_y = 0;
		time_left_x = 0;
		time_righ_x = 0;
	}
	
	if(sys.co_mode == CO_MECH)
	{
		chassis.info->top_gyro = 0;
		gyrate_open_flag = 0;
	}
}

	int angle_sum;
void CHASSIS_UpdateController(void)
{

	
	chas_motor_pid[CHAS_LF].speed.measure = chas_motor_info[CHAS_LF]->speed;
	chas_motor_pid[CHAS_RF].speed.measure = chas_motor_info[CHAS_RF]->speed;
	chas_motor_pid[CHAS_LB].speed.measure = chas_motor_info[CHAS_LB]->speed;
	chas_motor_pid[CHAS_RB].speed.measure = chas_motor_info[CHAS_RB]->speed;

	angle_sum = chas_dev.yaw_motor->info->angle_sum;
	chas_z_pid.angle.measure = angle_sum;//以总和计数的角度比较好计算
}

/* 应用层 --------------------------------------------------------------------*/
/* 任务层 --------------------------------------------------------------------*/
void CHASSIS_Init(void)
{
	chas_drv[CHAS_LF] = chas_dev.chas_motor[CHAS_LF]->driver;
	chas_drv[CHAS_RF] = chas_dev.chas_motor[CHAS_RF]->driver;
	chas_drv[CHAS_LB] = chas_dev.chas_motor[CHAS_LB]->driver;
	chas_drv[CHAS_RB] = chas_dev.chas_motor[CHAS_RB]->driver;

	chas_motor[CHAS_LF] = chas_dev.chas_motor[CHAS_LF];
	chas_motor[CHAS_RF] = chas_dev.chas_motor[CHAS_RF];
	chas_motor[CHAS_LB] = chas_dev.chas_motor[CHAS_LB];
	chas_motor[CHAS_RB] = chas_dev.chas_motor[CHAS_RB];	
	
	chas_motor_info[CHAS_LF] = chas_dev.chas_motor[CHAS_LF]->info;
	chas_motor_info[CHAS_RF] = chas_dev.chas_motor[CHAS_RF]->info;
	chas_motor_info[CHAS_LB] = chas_dev.chas_motor[CHAS_LB]->info;
	chas_motor_info[CHAS_RB] = chas_dev.chas_motor[CHAS_RB]->info;
	
	KalmanCreate(&chas_kalman_err, 1, 0);
	
	KalmanCreate(&chas_motor_pid_Kal[CHAS_LF], 1, 1);
	KalmanCreate(&chas_motor_pid_Kal[CHAS_RF], 1, 1);
	KalmanCreate(&chas_motor_pid_Kal[CHAS_LB], 1, 1);
	KalmanCreate(&chas_motor_pid_Kal[CHAS_RB], 1, 1);
	
}

short speed_temp;
//TOP_SPEED 设置
void CHASSIS_JudgeInfo(void)
{
	if(Judge_Hero.state_flag == 1)
	{
		Top_Speed = 3000;
		Top_Speed = (short)(Judge_Hero.robot_state.max_chassis_power *58.879f + 66.379f);
		
		if(N2O_Trigger == 1)
		{
			Top_Speed *= 2.0f;
			Top_Speed = 8000;
		}
		else
		{
		}
		
		if(Top_Speed>8000)
		{
			Top_Speed = 8000;
		}
		else if(Top_Speed<3000)
		{
			Top_Speed = 3000;
		}
		
		
		//Top_Speed = speed_temp;//60W4000开始损耗功率，
		//Top_Speed = 4000;
		
		speed_power_Kp = 1.5f*Top_Speed/8000.0f;
		//speed_power_Kp = 1;
		if(speed_power_Kp>1)
		{
			speed_power_Kp = 1;
		}
	}
	else
	{
		Top_Speed = 3000;
	}
}

void CHASSIS_GetInfo(void)	//更新本地操作模式
{
	CHASSIS_GetSysInfo();								//获取系统整体模式状态
	CHASSIS_GetJudgeInfo();							//判断底盘有效性
	CHASSIS_GetRcInfo();								//获取遥控器信息
	CHASSIS_UpdateController();					//更新控制器信息内容
	
	CHASSIS_JudgeInfo();
}

void CHASSIS_SelfProtect(void)
{

	CHASSIS_Stop(chas_motor_pid);
	CHASSIS_PidParamsInit(chas_motor_pid, CHAS_MOTOR_CNT);
	CHASSIS_Z_PidParamsInit(&chas_z_pid);
	CHASSIS_GetInfo();
	
	chassis.info->top_gyro = 0;
	chassis.info->twist = 0;
}

short lowest_buffer = 20;
bool N2O_Trigger = 0;
uint32_t power_buffer_ticks = 0;
void power_limitation(short int* output,uint8_t length, uint8_t remain_buffer)//每次计算底盘输出的时候将输出电流送进来计算
{
	uint8_t i;
	float work_kp;
	float chassis_total_output = 0;
	float speed_total = 0;
	float speed_weights[4] = {0};
	
	for(i = 0; i<length; i++)
	{
		chassis_total_output += abs(output[i]);
	}
	
	if(Judge_Offline_flag)//如果裁判系统离线
	{
		work_kp = 0.5;
	}
	else
	{
		work_kp = (((float)((int)(remain_buffer))-lowest_buffer))*1.5f/60.0f; //预留较多能量
	}
		
	if(work_kp <0.0f)
	{
		work_kp = 0.0f;
	}
	else if(work_kp >1.0f)
	{
		work_kp = 1.0f;
	}
	
	work_kp = work_kp*work_kp;
	
//	for(i = 0; i<CHAS_MOTOR_CNT; i++)
//	{
//		speed_weights[i] = abs(chas_motor_pid[i].speed.measure/chas_motor_pid[i].speed.target);
//		speed_total += speed_weights[i];
//	}
	
	
	for(i = 0; i<length; i++)
	{
		//speed_weights[i] = (1-(speed_weights[i]/speed_total))/3.0f;
		output[i] = output[i]*abs(work_kp);
	}
}

float max_total_chassis_output = 30000;
void old_power_limitation(short int* output,uint8_t length, uint8_t remain_buffer)
{
	uint8_t i = 0;
	float max_power = (float)((int)(Judge_Hero.robot_state.max_chassis_power));
	float max_chassis_power_buffer = 60;
	float real_chassis_power_buffer = (float)((int)remain_buffer);
	static float limit_k = 0;
	float total_output = 0;
	
	limit_k = real_chassis_power_buffer/max_chassis_power_buffer;
	limit_k = limit_k*limit_k;//功率比例
	
  if(real_chassis_power_buffer < (max_chassis_power_buffer-1.0f))
	{
		max_total_chassis_output = CHAS_MAX_TOTAL_OUT*limit_k;
	}
	else
	{
		max_total_chassis_output = CHAS_MAX_TOTAL_OUT;
	}
	
	for(i = 0; i<4; i++)
	{
		total_output = total_output+output[i];
	}
	
	if(total_output>max_total_chassis_output)
	{
		for(i = 0; i<4; i++)
		{
			output[i] = (int16_t)(output[i]/total_output*max_total_chassis_output);
		}
	}
	
}

short output[4];
void CHASSIS_PidCtrl(void)
{
	//计算pid
	
	// 底盘电机速度环
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_LF);
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_LB);
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_RF);
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_RB);
	
	output[CHAS_LF] = chas_motor_pid[CHAS_LF].speed.out;
	output[CHAS_RF] = chas_motor_pid[CHAS_RF].speed.out;
	output[CHAS_LB] = chas_motor_pid[CHAS_LB].speed.out;
	output[CHAS_RB] = chas_motor_pid[CHAS_RB].speed.out;
	
	power_limitation(output, 4, Judge_Hero.out_and_heat.chassis_power_buffer);
	
	chas_motor_pid[CHAS_LF].out = output[CHAS_LF];
	chas_motor_pid[CHAS_RF].out = output[CHAS_RF];
	chas_motor_pid[CHAS_LB].out = output[CHAS_LB];
	chas_motor_pid[CHAS_RB].out = output[CHAS_RB];

	// 底盘电机输出响应
	CHASSIS_PidOut(chas_motor_pid);
}

short CHASSIS_Twist_speed(void)
{
	static uint8_t Twist_flag = 0;
	float target_angle = 3000;//1536;
	short Twist_Vz;
	
	if(Twist_flag)
	{
		//Twist_Vz = 3000;
		Twist_Vz = ((gimbal.controller->motor[GIMBAL_YAW]->angle.target) - \
				(gimbal.controller->motor[GIMBAL_YAW]->angle.measure) + target_angle);
	}
	else 
	{
		//Twist_Vz = -3000;
		Twist_Vz = ((gimbal.controller->motor[GIMBAL_YAW]->angle.target) - \
				(gimbal.controller->motor[GIMBAL_YAW]->angle.measure) - target_angle);
	}

	if(Twist_Vz> -200 && Twist_flag == 0)
	{
		Twist_flag = 1;
	}
	else if(Twist_Vz < 200 && Twist_flag == 1)
	{
		Twist_flag = 0; 
	}
	
	
	return Twist_Vz;
}

void CHASSIS_NormalCtrl(void)
{
	float theta = 0;
	float cos_temp = 0, sin_temp = 0;
	
	
	if(chassis.info->co_mode == CO_MECH)
	{
		if(IF_KEY_PRESSED_C)
		{
			Vx = 0;
			Vy = 0;
			Vz = 0;
		}
		else
		{
			Vx = rc_move_info.key_d-rc_move_info.key_a;
			Vy = rc_move_info.key_w-rc_move_info.key_s;
			Vz = rc_move_info.mouse_vx*180.0f;
		}
	}
	else if(chassis.info->co_mode == CO_GYRO)
	{
		LOGIC_Vx = (rc_move_info.key_d-rc_move_info.key_a);
		LOGIC_Vy = (rc_move_info.key_w-rc_move_info.key_s);
		
		//如果陀螺解算出了问题，可以考虑在这里加个负号
		theta = gimbal_motor[GIMBAL_YAW].info->angle_sum;
		
		cos_temp = cos(theta/GIMBAL_YAW_CIRCULAR_STEP*2.0f*PI);
		sin_temp = sin(theta/GIMBAL_YAW_CIRCULAR_STEP*2.0f*PI);
		
		Vx = (short int)(LOGIC_Vx*cos_temp - LOGIC_Vy * sin_temp);//云台坐标速度映射底盘速度
    Vy = (short int)(LOGIC_Vx*sin_temp + LOGIC_Vy * cos_temp);
		
		
		Vz = CHASSIS_Z_Angle_PidCalc(&(chas_z_pid));
		//Vz = ((gimbal.controller->motor[GIMBAL_YAW]->angle.target)-(gimbal.controller->motor[GIMBAL_YAW]->angle.measure))*4.0f;
		Vz = constrain(Vz, -20000, 20000);
		
		if(chassis.info->top_gyro)
		{
			Vx = 0.5f*constrain(Vx, -Top_Speed, Top_Speed);
			Vy = 0.5f*constrain(Vy, -Top_Speed, Top_Speed);
			Vz = Top_Speed;
		}
		else
		{
			if(abs(Vz)>4000)
			{
				Vx = 0.5f*constrain(Vx, -abs(Vz), abs(Vz));
				Vy = 0.5f*constrain(Vy, -abs(Vz), abs(Vz));;
			}
		}
		
		//开启小陀螺
		if(IF_KEY_PRESSED_F && key_lock_F == false)
		{
			key_lock_F = true;
			if(chassis.info->top_gyro == 0)
			{			
				chassis.info->top_gyro = 1;
				chassis.info->twist = 0;
			}
			else if(chassis.info->top_gyro == 1)
			{
				chassis.info->top_gyro = 0;
			}
		}
		else if(IF_KEY_PRESSED_F == 0)
		{
			key_lock_F = false;
		}
		
	}
	
	if(IF_KEY_PRESSED_SHIFT)
	{
		N2O_Trigger = 1;
	}
	else 
	{
		N2O_Trigger = 0;
	}
	
}


void CHASSIS_AUTO(void)
{
	float theta = 0;
	float cos_temp = 0, sin_temp = 0;
	
	
	if(chassis.info->co_mode == CO_MECH)
	{
		if(IF_KEY_PRESSED_C)
		{
			Vx = 0;
			Vy = 0;
			Vz = 0;
		}
		else
		{
			Vx = rc_move_info.key_d-rc_move_info.key_a;
			Vy = rc_move_info.key_w-rc_move_info.key_s;
			Vz = rc_move_info.mouse_vx*180.0f;
		}
	}
	else if(chassis.info->co_mode == CO_GYRO)
	{
		LOGIC_Vx = (rc_move_info.key_d-rc_move_info.key_a);
		LOGIC_Vy = (rc_move_info.key_w-rc_move_info.key_s);
		
		//如果陀螺解算出了问题，可以考虑在这里加个负号
		theta = gimbal_motor[GIMBAL_YAW].info->angle_sum;
		
		cos_temp = cos(theta/GIMBAL_YAW_CIRCULAR_STEP*2.0f*PI);
		sin_temp = sin(theta/GIMBAL_YAW_CIRCULAR_STEP*2.0f*PI);
		
		Vx = (short int)(LOGIC_Vx*cos_temp - LOGIC_Vy * sin_temp);//云台坐标速度映射底盘速度
    Vy = (short int)(LOGIC_Vx*sin_temp + LOGIC_Vy * cos_temp);
		
		
		Vz = CHASSIS_Z_Angle_PidCalc(&(chas_z_pid));
		//Vz = ((gimbal.controller->motor[GIMBAL_YAW]->angle.target)-(gimbal.controller->motor[GIMBAL_YAW]->angle.measure))*4.0f;
		Vz = constrain(Vz, -20000, 20000);
		
		if(chassis.info->top_gyro)
		{
			Vx = 0.5f*constrain(Vx, -Top_Speed, Top_Speed);
			Vy = 0.5f*constrain(Vy, -Top_Speed, Top_Speed);
			Vz = Top_Speed;
		}
		else
		{
			if(abs(Vz)>4000)
			{
				Vx = 0.5f*constrain(Vx, -abs(Vz), abs(Vz));
				Vy = 0.5f*constrain(Vy, -abs(Vz), abs(Vz));;
			}
		}
		
		//开启小陀螺
		if(IF_KEY_PRESSED_F && key_lock_F == false)
		{
			key_lock_F = true;
			if(chassis.info->top_gyro == 0)
			{			
				chassis.info->top_gyro = 1;
				chassis.info->twist = 0;
			}
			else if(chassis.info->top_gyro == 1)
			{
				chassis.info->top_gyro = 0;
			}
		}
		else if(IF_KEY_PRESSED_F == 0)
		{
			key_lock_F = false;
		}
		
	}
	
	if(IF_KEY_PRESSED_SHIFT)
	{
		N2O_Trigger = 1;
	}
	else 
	{
		N2O_Trigger = 0;
	}
	
}


void CHASSIS_ReloadBulletCtrl(void)
{
}

void CHASSIS_SzuPupCtrl(void)
{
}

void CHASSIS_LONGSHOOTCtrl(void)
{
}

void CHASSIS_RcCtrl(void)
{
	float theta = 0;
	float cos_temp = 0, sin_temp = 0;
	
	
	if(chassis.info->co_mode == CO_MECH)
	{
		Vx = chassis.dev->rc_sensor->info->ch2*8000/660.0f;
		Vy = chassis.dev->rc_sensor->info->ch3*8000/660.0f;
		Vz = chassis.dev->rc_sensor->info->ch0*8000/660.0f;
		
		if(chassis.dev->rc_sensor->info->thumbwheel > 330)
		{
			N2O_Trigger = 1;
		}
		else 
		{
			N2O_Trigger = 0;
		}
	}
	else if(chassis.info->co_mode == CO_GYRO)
	{
		LOGIC_Vx = chassis.dev->rc_sensor->info->ch2*8000/660.0f;
		LOGIC_Vy = chassis.dev->rc_sensor->info->ch3*8000/660.0f;
		
		//如果陀螺解算出了问题，可以考虑在这里加个负号
		theta = gimbal_motor[GIMBAL_YAW].info->angle_sum;
		
		Rudder_angle = 0;
		
		cos_temp = cos(theta/GIMBAL_YAW_CIRCULAR_STEP*2.0f*PI);
		sin_temp = sin(theta/GIMBAL_YAW_CIRCULAR_STEP*2.0f*PI);
		
		Vx = (short int)(LOGIC_Vx*cos_temp - LOGIC_Vy * sin_temp);//云台坐标速度映射底盘速度
    Vy = (short int)(LOGIC_Vx*sin_temp + LOGIC_Vy * cos_temp);
		
		
		Vz = CHASSIS_Z_Angle_PidCalc(&(chas_z_pid));
		//Vz = ((gimbal.controller->motor[GIMBAL_YAW]->angle.target)-(gimbal.controller->motor[GIMBAL_YAW]->angle.measure))*4.0f;
		Vz = constrain(Vz, -20000, 20000);
		
		
		if(chassis.info->top_gyro)
		{
			Vx = 0.5f*constrain(Vx, -Top_Speed, Top_Speed);
			Vy = 0.5f*constrain(Vy, -Top_Speed, Top_Speed);
			Vz = Top_Speed;
		}
		else
		{
			if(abs(Vz)>6000)
			{
				Vx = 0.5f*constrain(Vx, -abs(Vz), abs(Vz));
				Vy = 0.5f*constrain(Vy, -abs(Vz), abs(Vz));;
			}
		}
		
		if(chassis.dev->rc_sensor->info->thumbwheel > 330)
		{
			chassis.info->top_gyro = 1;
		}
		else if(chassis.dev->rc_sensor->info->thumbwheel < -330)
		{
			chassis.info->top_gyro = 0;
		}
		
		N2O_Trigger = 0;
	}
	
	
	
}



void CHASSIS_KeyCtrl(void)
{
	switch(chas_info.local_mode)
	{
		case CHASSIS_MODE_NORMAL:
			CHASSIS_NormalCtrl();
			break;
		case CHASSIS_MODE_AUTO:
			CHASSIS_AUTO();
			break;
		case CHASSIS_MODE_LONGSHOOT:
			CHASSIS_LONGSHOOTCtrl();
			break;
		case CHASSIS_MODE_RELOAD_BULLET:
			CHASSIS_ReloadBulletCtrl();
			break;
		case CHASSIS_MODE_SZUPUP:
			CHASSIS_SzuPupCtrl();
			break;
		default:
			break;
	}
}

short Motor_Speed_Shrink(short *pS1)
{
	uint8_t i = 0;
	short max = 0;
	short temp = 0; 
	short speed_max = CHAS_SPEED_MAX*speed_power_Kp;

	for(i = 0; i<4; i++)
	{
		temp = pS1[i]>0 ? pS1[i] : (-pS1[i]);//求绝对值
		
		if(temp>max)
		{
			max = temp;
		}
	}	
	
	if(max<speed_max)
	{
		max = speed_max;
	}
	
	return max;
}

void CHASSIS_Speed_deliver(void)
{
	float k_temp;
	short speed_buffer[4] = {0};
	
	speed_buffer[0] = +Vx + Vy + Vz;		//麦轮解算
	speed_buffer[1] = +Vx - Vy + Vz;		//
	speed_buffer[2] = -Vx + Vy + Vz;		//
	speed_buffer[3] = -Vx - Vy + Vz;		//
		
		//越界比例处理
	
	k_temp = (float)(Motor_Speed_Shrink(speed_buffer));
	k_temp = k_temp/(CHAS_SPEED_MAX*speed_power_Kp);
	speed_buffer[0] = (short int)(speed_buffer[0]/k_temp);		//
	speed_buffer[1] = (short int)(speed_buffer[1]/k_temp);		//
	speed_buffer[2] = (short int)(speed_buffer[2]/k_temp);		//
	speed_buffer[3] = (short int)(speed_buffer[3]/k_temp);		//
	
	//输出比例
	chas_motor_pid[CHAS_LF].speed.target = speed_buffer[0];
	chas_motor_pid[CHAS_RF].speed.target = speed_buffer[1];
	chas_motor_pid[CHAS_LB].speed.target = speed_buffer[2];
	chas_motor_pid[CHAS_RB].speed.target = speed_buffer[3];
}

void CHASSIS_Ctrl(void)
{
	/*----信息读入----*/
	CHASSIS_GetInfo();
	/*----期望修改----*/ 
	if(chas_info.remote_mode == RC) {
		CHASSIS_RcCtrl();
	}
	else if(chas_info.remote_mode == KEY) {
		CHASSIS_KeyCtrl();
	}
	CHASSIS_Speed_deliver();
	/*----最终输出----*/
	CHASSIS_PidCtrl();	
}

void CHASSIS_Test(void)
{
}


