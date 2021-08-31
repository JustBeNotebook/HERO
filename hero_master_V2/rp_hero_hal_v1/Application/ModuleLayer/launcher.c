/**
 * @file        launcher.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        23-October-2020
 * @brief       launcher Module.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "launcher.h"
#include "chassis.h"
#include "judge.h"
#include "control.h"
#include "cmsis_os.h"
#include "rp_math.h"
#include "kalman.h"
#include "kalman_filter.h"

/* Private macro -------------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/


void LAUNCHER_Init(void);
void LAUNCHER_Ctrl(void);
void LAUNCHER_Test(void);
void LAUNCHER_Reset(void);
void LAUNCHER_SelfProtect(void);

//获取系统信息
void LAUNCHER_GetSysInfo(void);
void LAUNCHER_GetJudgeInfo(void);
void LAUNCHER_GetRcInfo(void);
void LAUNCHER_GetTopGyroInfo(void);
void LAUNCHER_GetSelfAttitude(void);

//初始化PID内部数据
static void LAUNCHER_PidParamsInit(launcher_motor_pid_t *pid, uint8_t motor_cnt);

//停止PID输出
static void LAUNCHER_Stop(launcher_motor_pid_t *pid);

//pid输出函数
static void LAUNCHER_PidOut(launcher_motor_pid_t *pid);

//根据ID计算PID输出
static void LAUNCHER_Angle_PidCalc(launcher_motor_pid_t *pid, launcher_motor_cnt_t MOTORx);
static void LAUNCHER_Speed_PidCalc(launcher_motor_pid_t *pid, launcher_motor_cnt_t MOTORx);


//马上关闭弹仓
void LAUNCEHR_Close_MGZ_NOW(void);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// 云台电机本地驱动
drv_can_t				*luch_drv[LAUNCHER_MOTOR_CNT];
launcher_motor_t			*luch_motor[LAUNCHER_MOTOR_CNT];
launcher_motor_info_t	*luch_motor_info[LAUNCHER_MOTOR_CNT];


//云台运动两变量
uint8_t mgz_press_flag = 0;
uint8_t fire_press_flag = 0;
uint8_t friction_press_flag = 0;

int Friction_speed = 0;
uint32_t Friction_target = 0;
uint32_t friction_speed[6] = {1000, 1228, 1250, 1280, 1500, 1600};
int Fric_3508_speed[6] = {1000, 1700, 3250, 4300, 5580, 4620};//5000; 

//比赛弹丸 4620
//一红一黑 4670
//西地轮毂，深圳聚氨酯 4270

// 遥控按键信息
static uint8_t key_lock_B;
static uint8_t key_lock_mouse_L = false;


/* 斜坡 */


/* 键盘模式下的前后左后以及旋转斜坡变量 */


/* 期望速度 */


/* 摇杆灵敏度 */




/* Exported variables --------------------------------------------------------*/
launcher_motor_pid_t launcher_motor_pid[LAUNCHER_MOTOR_CNT] = {
	[LAUNCHER_MGZ] = {
		.speed.kp = 1.0f, //35.0f,
		.speed.ki = 0.f,
		.speed.kd = 0,
		.speed.integral_max = 60,
		.speed.out_max = 3000,
		.angle.kp = 1.0f, //30.f,
		.angle.ki = 0.0f,
		.angle.kd = 0,
		.angle.integral_max = 100,
		.angle.out_max = 5000,
	},
	[LAUNCHER_SAFE] = {
		.speed.kp = 6.0f, //22.0f,
		.speed.ki = 0.0f,
		.speed.kd = 0,
		.speed.integral_max = 6000,
		.speed.out_max = 13000,
		.angle.kp = 0,//22.0f,
		.angle.ki = 0,
		.angle.kd = 0.f,
		.angle.integral_max = 0,
		.angle.out_max = 200,
	},
	[LAUNCHER_DIAL] = {
		.speed.kp = 6.0f, 
		.speed.ki = 0.2f,
		.speed.kd = 0,
		.speed.integral_max = 15000,
		.speed.out_max = 9000,
		.angle.kp = 0.2f, 
		.angle.ki = 0.0f,
		.angle.kd = 0,
		.angle.integral_max = 500,
		.angle.out_max = 8000,
	},
	[LAUNCHER_FRICT_L] = {
		.speed.kp = 20.0f, 
		.speed.ki = 0.f,
		.speed.kd = 0,
		.speed.integral_max = 15000,
		.speed.out_max = 10000,
		.angle.kp = 0.2f, 
		.angle.ki = 0.0f,
		.angle.kd = 0,
		.angle.integral_max = 500,
		.angle.out_max = 8000,
	},
	[LAUNCHER_FRICT_R] = {
		.speed.kp = 20.0f, 
		.speed.ki = 0.f,
		.speed.kd = 0,
		.speed.integral_max = 15000,
		.speed.out_max = 10000,
		.angle.kp = 0.2f, 
		.angle.ki = 0.0f,
		.angle.kd = 0,
		.angle.integral_max = 500,
		.angle.out_max = 8000,
	},
};

launcher_motor_pid_t fric_l, fric_r;
// 卡尔曼滤波器
extKalman_t launcher_speed_pid_Kal[LAUNCHER_MOTOR_CNT] = 
{
	[LAUNCHER_MGZ] = {
		.Q = 1,
		.R = 0,
	},
	[LAUNCHER_SAFE] = {
		.Q = 1,
		.R = 0,
	},
	[LAUNCHER_DIAL] = {
		.Q = 1,
		.R = 5,
	},
	[LAUNCHER_FRICT_L] = {
		.Q = 1,
		.R = 0,
	},
	[LAUNCHER_FRICT_R] = {
		.Q = 1,
		.R = 0,
	}
};






// 底盘模块控制器
launcher_ctrl_t		launcher_ctrl = {
	.motor = &launcher_motor_pid,
};

// 底盘模块传感器
launcher_dev_t		launcher_dev = {
	.launcher_motor[LAUNCHER_MGZ] = &launcher_motor[LAUNCHER_MGZ],
	.launcher_motor[LAUNCHER_SAFE] = &launcher_motor[LAUNCHER_SAFE],
	.launcher_motor[LAUNCHER_DIAL] = &launcher_motor[LAUNCHER_DIAL],
	.launcher_motor[LAUNCHER_FRICT_L] = &launcher_motor[LAUNCHER_FRICT_L],
	.launcher_motor[LAUNCHER_FRICT_R] = &launcher_motor[LAUNCHER_FRICT_R],
	.rc_sensor = &rc_sensor,
};

// 底盘模块信息
launcher_info_t 	launcher_info = {
	.remote_mode = RC,
	.local_mode = LAUNCHER_MODE_NORMAL,
	.launcher_mgz_flag = MGZ_ON,
	.fire_permit_flag = 0,
};

launcher_t launcher = {
	.controller = &launcher_ctrl,
	.dev = &launcher_dev,
	.info = &launcher_info,
	.init = LAUNCHER_Init,
	.test = LAUNCHER_Test,
	.ctrl = LAUNCHER_Ctrl,
	.self_protect = LAUNCHER_SelfProtect,
};


/* Private functions ---------------------------------------------------------*/
/* 驱动层 --------------------------------------------------------------------*/

/**
 *	@brief	云台电机PID参数初始化，保护用
 */
static void LAUNCHER_PidParamsInit(launcher_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		pid_val_init(&pid[i].angle);
		pid_val_init(&pid[i].speed);
		pid[i].out = 0;
		
		pid[i].angle.target = pid[i].angle.measure;
	}	
}


static void LAUNCHER_SinglePidParamsInit(launcher_motor_pid_t *pid, uint8_t motor_cnt)
{
	
	
	pid_val_init(&pid[motor_cnt].angle);
	pid_val_init(&pid[motor_cnt].speed);
	pid[motor_cnt].out = 0;
	
	pid[motor_cnt].angle.target = pid[motor_cnt].angle.measure;

}

/**
 *	@brief	云台底盘电机卸力，保护用
 */
static void LAUNCHER_Stop(launcher_motor_pid_t *pid)
{
	
	pid[LAUNCHER_DIAL].out = 0;
	pid[LAUNCHER_SAFE].out = 0;
	pid[LAUNCHER_MGZ].out = 0;
	pid[LAUNCHER_FRICT_L].out = 0;
	pid[LAUNCHER_FRICT_R].out = 0;

	
	
	CAN2_0X200_BUF[1] = 0;//限位轮
	CAN2_0X1ff_BUF[0] = 0;//右摩擦轮
	CAN2_0X1ff_BUF[1] = 0;//左摩擦轮
	CAN1_0X1ff_BUF[2] = 0;//拨盘
}

/**
 *	@brief	云台电机PID输出
 */
static void LAUNCHER_PidOut(launcher_motor_pid_t *pid)
{
	
	if(luch_motor[LAUNCHER_MGZ]->work_state == DEV_ONLINE) 
	{
		CAN2_0X200_BUF[0] = pid[LAUNCHER_MGZ].out;
	} 
	else 
	{
		CAN2_0X200_BUF[0] = 0;
	}

	if(luch_motor[LAUNCHER_SAFE]->work_state == DEV_ONLINE) 
	{
		CAN2_0X200_BUF[1] = (int16_t)(pid[LAUNCHER_SAFE].out);
	} 
	else 
	{
		CAN2_0X200_BUF[1] = 0;
	}
	
	if(luch_motor[LAUNCHER_DIAL]->work_state == DEV_ONLINE)
	{
		CAN1_0X1ff_BUF[2] = (int16_t)(pid[LAUNCHER_DIAL].out);
	}
	else 
	{
		CAN1_0X1ff_BUF[2] = 0;
	}
	
	if(luch_motor[LAUNCHER_FRICT_L]->work_state == DEV_ONLINE)
	{
		CAN2_0X1ff_BUF[1] = (int16_t)(pid[LAUNCHER_FRICT_L].out);
	}
	else 
	{
		CAN2_0X1ff_BUF[1] = 0;
	}
	
	if(luch_motor[LAUNCHER_FRICT_R]->work_state == DEV_ONLINE)
	{
		CAN2_0X1ff_BUF[0] = (int16_t)(pid[LAUNCHER_FRICT_R].out);
	}
	else 
	{
		CAN2_0X1ff_BUF[0] = 0;
	}
	
	
}

static void LAUNCHER_Angle_PidCalc(launcher_motor_pid_t *pid, launcher_motor_cnt_t MOTORx)
{
	pid[MOTORx].angle.err = pid[MOTORx].angle.target - pid[MOTORx].angle.measure;
	if(MOTORx == LAUNCHER_DIAL)
	{
		if(abs(pid[MOTORx].angle.err)<50)
		{
			pid[MOTORx].angle.err = 0;
		}
	}
	single_pid_ctrl(&pid[MOTORx].angle);
	pid[MOTORx].speed.target = pid[MOTORx].angle.out;
	
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
	pid[MOTORx].speed.err = KalmanFilter(&launcher_speed_pid_Kal[MOTORx], pid[MOTORx].speed.err);
	
	single_pid_ctrl(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}

/**
 *	@brief	云台电机速度环
 */
static void LAUNCHER_Speed_PidCalc(launcher_motor_pid_t *pid, launcher_motor_cnt_t MOTORx)
{
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
	pid[MOTORx].speed.err = KalmanFilter(&launcher_speed_pid_Kal[MOTORx], pid[MOTORx].speed.err);
	
	single_pid_ctrl(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}



/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/

/**
 *	@brief	云台获取系统信息
 */
void LAUNCHER_GetSysInfo(void)//只要确保随时切换无问题即可就行
{
	
	/*----控制方式修改----*/
	if(sys.remote_mode != launcher_info.remote_mode)
	{
		uint8_t mgz_press_flag = 0;
		uint8_t fire_press_flag = 0;
		uint8_t friction_press_flag = 0;
	}
	
	if(sys.remote_mode == RC) {
		launcher_info.remote_mode = RC;
	}
	else if(sys.remote_mode == KEY) {
		launcher_info.remote_mode = KEY;
	}
	
	
	launcher_info.local_mode = LAUNCHER_MODE_NORMAL;
	
	launcher_info.co_mode = sys.co_mode;
}

void LAUNCHER_GetJudgeInfo(void)
{
	//无？ 或者是裁判系统的数据获取
}

void LAUNCHER_GetRcInfo(void)
{
	/* 系统正常 */
	if(sys.state == SYS_STATE_NORMAL)
	{
		if(sys.remote_mode == RC) 
		{
			key_lock_mouse_L = false;
			
		}
		else if(sys.remote_mode == KEY) 
		{
			
		}
	}
	/* 系统异常 失联 操作 其他 */
	else 
	{
		key_lock_mouse_L = false;
	}
}

void LAUNCHER_GetTopGyroInfo(void)
{
	
}

void LAUNCHER_GetSelfAttitude(void)
{
	
}

void LAUNCHER_UpdateController(void)
{
	
	launcher_motor_pid[LAUNCHER_MGZ].angle.measure = launcher_motor[LAUNCHER_MGZ].info->angle_sum;
	launcher_motor_pid[LAUNCHER_MGZ].speed.measure = launcher_motor[LAUNCHER_MGZ].info->speed;
	
	launcher_motor_pid[LAUNCHER_SAFE].angle.measure = launcher_motor[LAUNCHER_SAFE].info->angle;
	launcher_motor_pid[LAUNCHER_SAFE].speed.measure = launcher_motor[LAUNCHER_SAFE].info->speed;
	
	launcher_motor_pid[LAUNCHER_DIAL].angle.measure = launcher_motor[LAUNCHER_DIAL].info->angle_sum;
	launcher_motor_pid[LAUNCHER_DIAL].speed.measure = launcher_motor[LAUNCHER_DIAL].info->speed;
	
	launcher_motor_pid[LAUNCHER_FRICT_L].angle.measure = launcher_motor[LAUNCHER_FRICT_L].info->angle_sum;
	launcher_motor_pid[LAUNCHER_FRICT_L].speed.measure = launcher_motor[LAUNCHER_FRICT_L].info->speed;
	
	launcher_motor_pid[LAUNCHER_FRICT_R].angle.measure = launcher_motor[LAUNCHER_FRICT_R].info->angle_sum;
	launcher_motor_pid[LAUNCHER_FRICT_R].speed.measure = launcher_motor[LAUNCHER_FRICT_R].info->speed;
}

/* 应用层 --------------------------------------------------------------------*/
//除了初次上电以外的初始化
void LAUNCHER_Reset(void)
{
	
}

/***拨弹策略****/
//首先，收到发弹信号，限位轮先转
//限位轮转完minimumtime后停转，然后拨盘开始拨弹
//拨弹的时候象征性的反拨几次，反拨几次后对本次拨弹的步长进行判断
//如果拨格拨过0.5格，视为轻松拨过
//	轻松拨过就持续拨
//如果没拨过0.5格，视为卡住拨过
//	如果拨盘不容易卡的话，卡住拨过相当于弹链充盈，可以开大
//  如果拨盘容易卡的话，就叫浩哥解决卡弹问题
int dial_big_num = 0;
uint8_t dial_flag = 0;
uint8_t dial_ready_flag = 1;
uint8_t dial_empty_flag = 0;
int spin_time = SAFE_SPIN_TIME;
int safty_speed = SAFE_SPIN_SPEED;//12000
int dial_warning_output = DIAL_WARNING_OUTPUT;//绝对值大于这个力气才算卡住,卡住就算拨满
extern uint16_t shoot_heat;
void LAUNCHER_Shoot(void)
{
	
	static uint32_t timeout_cnt = 0;
	static uint32_t dial_timeout = 0;
	static uint8_t shoot_status = 1;//光电门 状态 无弹丸经过常低
	
	static float angle_err = 0 ;
	static uint8_t times = 0;
	
	//minimum_spin_time = (int)(0.2174f*(float)(Gimbal.Pitch_Motor.angle)-150.79f);//根据角度设置限位时间
	
	if(IF_KEY_PRESSED_Z)
	{
		dial_ready_flag = 1;
	}
	
	if(fire_press_flag == 1 && timeout_cnt<osKernelSysTick() && dial_ready_flag == 1)//拨动信号
	{
		fire_press_flag = 0;
		timeout_cnt = osKernelSysTick()+spin_time;//180ms回弹时间
		dial_ready_flag = 0;
		shoot_heat += 100;
	}
	else 
	{
		fire_press_flag = 0;
	}
	
	if(launcher.info->launcher_friction_flag ==	Friction_ON)
	{
//		//如果有加光电的话，最终旋转时间 = minimum_spin_time
//		if(shoot_status == 0 && PHOTOGATE_READ() == 0 && timeout_cnt>osKernelSysTick() )//有异物
//		{
//			shoot_status = 1;
//			timeout_cnt = osKernelSysTick()+minimum_spin_time;
//		}
		
		if(timeout_cnt>osKernelSysTick())	//顶部限位轮
		{
			launcher_motor_pid[LAUNCHER_SAFE].speed.target = safty_speed;
		}
		else
		{
			if(launcher_motor_pid[LAUNCHER_SAFE].speed.target == safty_speed)//切换转速的一瞬间说明发射成功，然后进行拨弹
			{
				
				launcher_motor_pid[LAUNCHER_DIAL].angle.target += DIAL_STEP;
				dial_timeout = osKernelSysTick()+200;			//堵转起点时间
				dial_flag = 1;
				if(FIRE_FASTER_ABILITY)
				{
					dial_ready_flag = 1;
				}
				
			}
			shoot_status = 0;
			launcher_motor_pid[LAUNCHER_SAFE].speed.target = 0;
		}
	}
	else
	{
		launcher_motor_pid[LAUNCHER_SAFE].speed.target = 0;
	}
	
	if(dial_flag==1 && dial_timeout<osKernelSysTick())//拨盘开拨
	{
		
		dial_flag = 2;
	}
	else if(dial_flag >= 2 && dial_timeout<osKernelSysTick())
	{
		if(abs((launcher_motor_pid[LAUNCHER_DIAL].speed.out))>dial_warning_output && dial_flag == 2)//先卸力
		{
			times++;
			dial_flag++;
			dial_timeout = osKernelSysTick()+150;
			
		}
		else if(dial_flag == 3)//回弹
		{
			dial_flag++;
			angle_err = launcher_motor_pid[LAUNCHER_DIAL].angle.err;
			launcher_motor_pid[LAUNCHER_DIAL].angle.target += -angle_err-(DIAL_STEP/2);
			dial_timeout = osKernelSysTick()+150;
		}
		else if(dial_flag == 4)
		{
			dial_flag++;
			launcher_motor_pid[LAUNCHER_DIAL].angle.target += angle_err+(DIAL_STEP/2);
			dial_timeout = osKernelSysTick()+150;
		}
		else if(dial_flag == 5)
		{
			if(abs(launcher_motor_pid[LAUNCHER_DIAL].speed.out)>dial_warning_output)//有堵转
			{
				dial_flag = 2;
				if(times>re_dial_times)//防堵转
				{
					times = 0;
					
					dial_ready_flag = 1;
					//防堵转版本
					if(abs(launcher_motor_pid[LAUNCHER_DIAL].angle.err)>(DIAL_STEP/2))//没有拨满一格
					{
						dial_flag = 0;
						launcher_motor_pid[LAUNCHER_DIAL].angle.target = launcher_motor_pid[LAUNCHER_DIAL].angle.measure;
						dial_ready_flag = 1;
						dial_big_num = 0;
						dial_empty_flag = 0;
					}
					else //拨满一格，重复拨
					{
						if(dial_big_num>=5)//拨盘空了
						{
							dial_flag = 0;
							launcher_motor_pid[LAUNCHER_DIAL].angle.target = launcher_motor_pid[LAUNCHER_DIAL].angle.measure;
							dial_ready_flag = 1;
							dial_big_num = 0;
							dial_empty_flag = 1;
						}
						else 
						{
							dial_big_num++;
							dial_flag = 2;
							launcher_motor_pid[LAUNCHER_DIAL].angle.target += DIAL_STEP;
							dial_timeout = osKernelSysTick()+200;			//堵转起点时间
							dial_empty_flag = 0;
							
							if(IF_KEY_PRESSED_Z)
							{
								dial_ready_flag = 1;
							}
						}
					}
					
					
					angle_err = 0;
					
					//不防堵转版本	
//					if(abs(launcher_motor_pid[LAUNCHER_DIAL].angle.err)<(DIAL_STEP/4))
//					{
//						dial_flag = 0;
//						angle_err = 0;
//					}
					
				}
			}
			else //防止堵转flag计算出错
			{
				dial_flag = 2;
				angle_err = 0;
				launcher_motor_pid[LAUNCHER_DIAL].angle.target += DIAL_STEP;
				dial_timeout = osKernelSysTick()+200;			//堵转起点时间
				dial_big_num++;
				dial_empty_flag = 0;
				//dial_ready_flag = 1;
			}
				
		}
		else//轻松拨过
		{
			if(dial_big_num>=5)//拨盘空了
			{
				dial_flag = 0;
				launcher_motor_pid[LAUNCHER_DIAL].angle.target = launcher_motor_pid[LAUNCHER_DIAL].angle.measure;
				dial_ready_flag = 1;
				dial_big_num = 0;
				dial_empty_flag = 1;
			}
			else 
			{
				dial_big_num++;
				dial_flag = 2;
				launcher_motor_pid[LAUNCHER_DIAL].angle.target += DIAL_STEP;
				dial_timeout = osKernelSysTick()+200;			//堵转起点时间
				dial_empty_flag =0;
				
				if(IF_KEY_PRESSED_Z)
				{
					dial_ready_flag = 1;
				}
				
			}
			
		}
		

	}
	
}



short fric_speed_err = 0;
void LAUNCHER_ON_OFF_Fric(void)
{
	if(Judge_Hero.robot_state.shooter_42mm_speed_limit>14)
	{
		Friction_speed = Fric_3508_speed[5];
	}
	else
	{
		Friction_speed = Fric_3508_speed[2];
	}
	
	
	if(friction_press_flag == 1)
	{
		friction_press_flag = 0;
		if(launcher.info->launcher_friction_flag == Friction_OFF)//打开摩擦轮
		{
			launcher.info->launcher_friction_flag = Friction_ON;
			dial_ready_flag = 1;
		}
		else if(launcher.info->launcher_friction_flag == Friction_ON)//停止摩擦轮
		{
			launcher.info->launcher_friction_flag = Friction_OFF;
		}
		
	}

	if(launcher.info->launcher_friction_flag == Friction_ON)
	{
		launcher_motor_pid[LAUNCHER_FRICT_L].speed.target = -Friction_speed;
		launcher_motor_pid[LAUNCHER_FRICT_R].speed.target = (Friction_speed+fric_speed_err);
		LASER_ON();
	}
	else 
	{
		launcher_motor_pid[LAUNCHER_FRICT_L].speed.target = 0;
		launcher_motor_pid[LAUNCHER_FRICT_R].speed.target = 0;
		dial_ready_flag = 0;
		LASER_OFF();
		
		if(sys.mode == SYS_MODE_AUTO)
		{
			LASER_ON();
		}
	}
	
}

uint32_t auto_off = 0;
int32_t mgz_step = 130000;
void LAUNCEHR_ON_OFF_MGZ(void)
{
	uint8_t flag;
	static uint8_t turning_flag = 0;//0代表可以切换角度
	static uint32_t time_out = 0;//堵塞倒计时，当前时间大于time_out就会进入堵塞状态
	
	if(auto_off>osKernelSysTick() && mgz_press_flag == 1)
	{
		flag = 0;
		auto_off = osKernelSysTick()+MAGEZINE_OPEM_TIME;
	}
	else if(mgz_press_flag == 1)
	{
		flag = 1;
	}
	
	if(flag == 1 && turning_flag == 0) // 1为切换角度命令
	{
		if(launcher.info->launcher_mgz_flag == MGZ_OFF)//关闭状态
		{
			launcher_motor_pid[LAUNCHER_MGZ].angle.target += mgz_step;	
		}
		else	//打开状态
		{
			launcher_motor_pid[LAUNCHER_MGZ].angle.target -= mgz_step;
		}
		auto_off = osKernelSysTick()+3000;
		time_out = osKernelSysTick() + 1000;
		turning_flag = 1;
	}
	
	if(auto_off <= osKernelSysTick() && launcher.info->launcher_mgz_flag == MGZ_ON && turning_flag == 0)//关闭弹仓的各种条件
	{
		launcher_motor_pid[LAUNCHER_MGZ].angle.target -= mgz_step;
		time_out = osKernelSysTick() + 1000;
		turning_flag = 1;
	}
	
	if(osKernelSysTick()>time_out && turning_flag==1 
		&& abs(launcher.dev->launcher_motor[LAUNCHER_MGZ]->info->speed)<1000) // 堵塞处理
	{
		if(launcher_motor_pid[LAUNCHER_MGZ].angle.err > 6000.0f)
		{
			launcher_motor_pid[LAUNCHER_MGZ].angle.target = launcher_motor_pid[LAUNCHER_MGZ].angle.measure;
			
			launcher.info->launcher_mgz_flag = MGZ_ON;
		}
		else if(launcher_motor_pid[LAUNCHER_MGZ].angle.err < -6000.0f)
		{
			launcher_motor_pid[LAUNCHER_MGZ].angle.target = launcher_motor_pid[LAUNCHER_MGZ].angle.measure;
			
			launcher.info->launcher_mgz_flag = MGZ_OFF;
		}
		else
		{
			if(launcher.info->launcher_mgz_flag == MGZ_ON)
			{
				launcher.info->launcher_mgz_flag = MGZ_OFF;
			}
			else
			{
				launcher.info->launcher_mgz_flag = MGZ_ON;
			}
		}
		turning_flag = 0;
		
	}
}

void LAUNCEHR_Close_MGZ_NOW(void)
{
	auto_off = osKernelSysTick();
	mgz_press_flag = 0;
}

/* 任务层 --------------------------------------------------------------------*/
void LAUNCHER_Init(void)
{
	luch_drv[LAUNCHER_MGZ] = launcher_dev.launcher_motor[LAUNCHER_MGZ]->driver;
	luch_drv[LAUNCHER_SAFE] = launcher_dev.launcher_motor[LAUNCHER_SAFE]->driver;
	luch_drv[LAUNCHER_DIAL] = launcher_dev.launcher_motor[LAUNCHER_DIAL]->driver;
	luch_drv[LAUNCHER_FRICT_L] = launcher_dev.launcher_motor[LAUNCHER_FRICT_L]->driver;
	luch_drv[LAUNCHER_FRICT_R] = launcher_dev.launcher_motor[LAUNCHER_FRICT_R]->driver;

	luch_motor[LAUNCHER_MGZ] = launcher_dev.launcher_motor[LAUNCHER_MGZ];
	luch_motor[LAUNCHER_SAFE] = launcher_dev.launcher_motor[LAUNCHER_SAFE];
	luch_motor[LAUNCHER_DIAL] = launcher_dev.launcher_motor[LAUNCHER_DIAL];
	luch_motor[LAUNCHER_FRICT_L] = launcher_dev.launcher_motor[LAUNCHER_FRICT_L];
	luch_motor[LAUNCHER_FRICT_R] = launcher_dev.launcher_motor[LAUNCHER_FRICT_R];
	
	luch_motor_info[LAUNCHER_MGZ] = launcher_dev.launcher_motor[LAUNCHER_MGZ]->info;
	luch_motor_info[LAUNCHER_SAFE] = launcher_dev.launcher_motor[LAUNCHER_SAFE]->info;
	luch_motor_info[LAUNCHER_DIAL] = launcher_dev.launcher_motor[LAUNCHER_DIAL]->info;
	luch_motor_info[LAUNCHER_FRICT_L] = launcher_dev.launcher_motor[LAUNCHER_FRICT_L]->info;
	luch_motor_info[LAUNCHER_FRICT_R] = launcher_dev.launcher_motor[LAUNCHER_FRICT_R]->info;
	
	KalmanCreate(&launcher_speed_pid_Kal[LAUNCHER_MGZ], 1, 0);
	KalmanCreate(&launcher_speed_pid_Kal[LAUNCHER_SAFE], 1, 0);
	KalmanCreate(&launcher_speed_pid_Kal[LAUNCHER_DIAL], 1, 0);

}

void LAUNCHER_GetInfo(void)	//更新本地操作模式
{
	LAUNCHER_GetSysInfo();								//获取系统整体模式状态
	LAUNCHER_GetJudgeInfo();							//判断云台有效性
	LAUNCHER_GetRcInfo();								//获取遥控器信息
	//LAUNCHER_GetTopGyroInfo();						//判断是否处于旋转状态
	//LAUNCHER_GetSelfAttitude();					//获取自身？？？	
	LAUNCHER_UpdateController();					//更新控制器信息内容
}

void LAUNCHER_SelfProtect(void)
{
	launcher.info->launcher_friction_flag = Friction_OFF;
	launcher.info->launcher_mgz_flag = MGZ_ON;
	LAUNCHER_Stop(launcher_motor_pid);
	LAUNCHER_PidParamsInit(launcher_motor_pid, LAUNCHER_MOTOR_CNT);
	LAUNCHER_GetInfo();
	LASER_OFF();
	dial_ready_flag = 1;
}

uint8_t slaver_photogate = 0;
uint8_t slaver_photogate_flag = 0;
void LAUNCHER_PidCtrl(void)
{
	
	if(launcher_motor[LAUNCHER_FRICT_L].work_state == DEV_OFFLINE || launcher_motor[LAUNCHER_FRICT_R].work_state == DEV_OFFLINE)
	{
		
	}
	else
	{
		LAUNCHER_Shoot();//比赛时要判断摩擦轮
	}
	//LAUNCHER_Shoot();//测拨盘时不用判断摩擦轮
	
	LAUNCEHR_ON_OFF_MGZ();
	LAUNCHER_ON_OFF_Fric();
	
	//计算pid
	LAUNCHER_Angle_PidCalc(launcher_motor_pid, LAUNCHER_MGZ);
	LAUNCHER_Speed_PidCalc(launcher_motor_pid, LAUNCHER_SAFE);
	LAUNCHER_Angle_PidCalc(launcher_motor_pid, LAUNCHER_DIAL);
	
	LAUNCHER_Speed_PidCalc(launcher_motor_pid, LAUNCHER_FRICT_L);
	LAUNCHER_Speed_PidCalc(launcher_motor_pid, LAUNCHER_FRICT_R);
	LAUNCHER_PidOut(launcher_motor_pid);
	
	fric_l = launcher_motor_pid[LAUNCHER_FRICT_L];
	fric_r = launcher_motor_pid[LAUNCHER_FRICT_R];
	fric_r.speed.measure *= -1;
}

uint16_t shoot_heat = 0;
void LAUNCHER_NormalCtrl(void)
{
	static uint32_t refresh_time = 0;
	
	if(refresh_time<=osKernelSysTick())//处于正常情况下的更新热量
	{
		shoot_heat = Judge_Hero.out_and_heat.shooter_heat2_42mm;
	}
	else	//刚打弹后等待延时时间
	{
		if(shoot_heat<Judge_Hero.out_and_heat.shooter_heat2_42mm+50)//热量降低50的时间内数据有更新，说明裁判系统检测到打弹，能直接用裁判系统热量了
		{
			refresh_time = osKernelSysTick()-2;//说明热量更新了，取消等待更新状态;
			shoot_heat = Judge_Hero.out_and_heat.shooter_heat2_42mm;
		}
	}
	
	if(chassis.info->top_gyro == 0 && chassis.info->twist==0)//底盘运动都不启动时才允许开弹仓
	{
		if(IF_KEY_PRESSED_B && key_lock_B == false)
		{
			mgz_press_flag = 1;
			key_lock_B = true;
			
		}
		else if(IF_KEY_PRESSED_B == 0)
		{
			mgz_press_flag = 0;
			key_lock_B = false;
		}
	}
	else
	{
		LAUNCEHR_Close_MGZ_NOW();
	}
	
	if(IF_MOUSE_PRESSED_LEFT && key_lock_mouse_L == false)
	{
		if(launcher.info->launcher_friction_flag == Friction_OFF)
		{
			friction_press_flag = 1;
			key_lock_mouse_L = true;
		}
		else if(launcher.info->launcher_friction_flag == Friction_ON)//开火判断
		{
			
			if(sys.mode != SYS_MODE_AUTO)
			{
				if(IF_KEY_PRESSED_Z || (shoot_heat+100<=(Judge_Hero.robot_state.shooter_42mm_cooling_limit) && refresh_time<=osKernelSysTick()))
				{
					fire_press_flag = 1;
					key_lock_mouse_L = true;
					refresh_time = osKernelSysTick()+200;
					
				}	
			}
			else if(sys.mode == SYS_MODE_AUTO)
			{
				if(IF_KEY_PRESSED_Z || (launcher.info->fire_permit_flag == 1 && (shoot_heat+100<=(Judge_Hero.robot_state.shooter_42mm_cooling_limit)) && refresh_time<=osKernelSysTick()))
				{
					fire_press_flag = 1;
					key_lock_mouse_L = true;
					refresh_time = osKernelSysTick()+200;
				}
			}
			
		}
		
	}
	else if(IF_MOUSE_PRESSED_LEFT == 0)
	{
		key_lock_mouse_L = false;
	}
	
	
	
}

void LAUNCHER_RcCtrl(void)
{
	static uint8_t RC_SW1_mid_flag = 0;
	
	if(launcher.dev->rc_sensor->info->s1 == 3)
	{
		RC_SW1_mid_flag = 1;
		fire_press_flag = 0;
		mgz_press_flag = 0;
		friction_press_flag = 0;
	}
	
	if(RC_SW1_mid_flag)
	{
		
		if(launcher.info->co_mode == CO_MECH)
		{
			if(launcher.dev->rc_sensor->info->s1 == 2)//发弹
			{
				fire_press_flag = 1;
				RC_SW1_mid_flag = 0;
			}
			else if(launcher.dev->rc_sensor->info->s1 == 1)//开弹仓
			{
				mgz_press_flag = 1;
				RC_SW1_mid_flag = 0;
			}
		}
		else if(launcher.info->co_mode == CO_GYRO)//
		{
			if(launcher.dev->rc_sensor->info->s1 == 2)//发弹
			{
				fire_press_flag = 1;
				RC_SW1_mid_flag = 0;
			}
			else if(launcher.dev->rc_sensor->info->s1 == 1)//开关摩擦轮
			{
				friction_press_flag = 1;
				RC_SW1_mid_flag = 0;
			}
		}
	}
	
	
	
}

void LAUNCHER_KeyCtrl(void)
{
	switch(launcher_info.local_mode)
	{
		case LAUNCHER_MODE_NORMAL:
			LAUNCHER_NormalCtrl();
			break;
		default:
			break;
	}
}



void LAUNCHER_Angle_deliver(void)
{
	
	
}

void LAUNCHER_Ctrl(void)
{
	/*----信息读入----*/
	LAUNCHER_GetInfo();
	/*----期望修改----*/ 
	if(launcher_info.remote_mode == RC) {
		LAUNCHER_RcCtrl();
	}
	else if(launcher_info.remote_mode == KEY) {
		LAUNCHER_KeyCtrl();
	}
	
	/*----最终输出----*/
	LAUNCHER_PidCtrl();	
}

void LAUNCHER_Test(void)
{
}








