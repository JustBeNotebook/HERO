/**
 * @file        system_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        27-October-2020
 * @brief       Decision Center.
 */

/* Includes ------------------------------------------------------------------*/
#include "system_task.h"
#include "launcher.h"
#include "cmsis_os.h"
#include "rc_sensor.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int8_t  prev_sw2 = RC_SW_MID;				//开关设置标志
bool gyro2mech_pid_mode = false;
static uint8_t mouse_lock_R = false;				//按键锁定标志
static uint8_t rc_lock_SW1 = false;
static uint8_t key_lock_Ctrl = false;
static uint8_t relife_flag = 1;							//初始上电经过一段延时后才将此位置0 才能切换陀螺仪模式


/* Exported variables --------------------------------------------------------*/
flag_t flag = {
	.gimbal = {
		.gimbal_mode_lock = false,
		.relife_flag = false,
		.reset_start = false,
		.reset_ok = false,
	},
};

system_t sys = {
	.remote_mode = RC,
	.co_mode = CO_MECH,
	.state = SYS_STATE_RCLOST,
	.mode = SYS_MODE_NORMAL,
};

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	遥控失联与否的处理，以及无失联时一些标志位的设置
 */
static void rc_update_info(void)
{
	if(sys.state != SYS_STATE_NORMAL) 
	{
		prev_sw2 = RC_SW_MID;	
		gyro2mech_pid_mode = false;
		
		mouse_lock_R = false;
		rc_lock_SW1 = false;
		key_lock_Ctrl = false;
	}
	else 
	{
		/* 遥控模式 */
		if(sys.remote_mode == RC) 
		{
			// 重置键盘模式下的参数
			mouse_lock_R = false;
			rc_lock_SW1 = false;
			key_lock_Ctrl = false;
		}
		else if(sys.remote_mode == KEY)
		{
			//暂不需要处理
			
		}
	}
}

/**
 *	@brief	根据遥控器切换控制方式
 */
static void system_ctrl_mode_switch(void)
{
	uint8_t sw2 = RC_SW2_VALUE;
	
	if(sw2 == RC_SW_UP)	//键盘模式
	{
		//陀螺转机械需要回正处理，未加
		
		if(prev_sw2 == RC_SW_MID) 
		{	
			gyro2mech_pid_mode = false;
			sys.remote_mode = KEY;
			sys.co_mode = CO_MECH;
			// 刚切换过去的时候设置为常规模式
			sys.mode = SYS_MODE_NORMAL;	
			
		}
		else
		{
			
		}
		prev_sw2 = RC_SW_UP;
	}
	else if(sw2 == RC_SW_MID || relife_flag == 1)	//机械模式
	{
		//陀螺转机械需要回正处理，未加
		
		sys.mode = SYS_MODE_NORMAL;	
		gyro2mech_pid_mode = false;
		sys.remote_mode = RC;
		sys.co_mode = CO_MECH;
		
		
		if(prev_sw2 != RC_SW_MID) //从云台切换过来时
		{	
			
//			Gimbal.Yaw_Motor.angle_turns = Gimbal.Yaw_Motor.angle_turns%2;
//			Gimbal.Yaw_Motor.tg_angle_turns = Gimbal.Yaw_Motor.angle_turns;
//			Gimbal.tg_PAngle = Gimbal.Pitch_Motor.angle;
//			Gimbal.tg_PTurns = Gimbal.Pitch_Motor.angle_turns;

//			Gimbal.Pitch_Motor.tg_angle = Gimbal.Pitch_Motor.angle;
//			Gimbal.Pitch_Motor.tg_angle_turns = Gimbal.Pitch_Motor.angle_turns;
			
		}

		prev_sw2 = RC_SW_MID;
	}
	else if(sw2 == RC_SW_DOWN)	//陀螺仪模式
	{
		gyro2mech_pid_mode = false;
		sys.mode = SYS_MODE_NORMAL;
		sys.remote_mode = RC;
		sys.co_mode = CO_GYRO;
		
		if(prev_sw2 != RC_SW_MID) //从机械切换过来时
		{	

		}
		
		prev_sw2 = RC_SW_DOWN;
		
	}
	
	
}

/**
 *	@brief	根据键盘切换系统行为
 */
uint8_t test_auto_aim = 0;
static void system_mode_act_switch(void)//键盘专用
{
	/*按键状态保留量*/
	
	static uint32_t KeyCtrlTime_Cur = 0;
	
	
	
	if(test_auto_aim == 0)//不测试自瞄
	{
		if(IF_MOUSE_PRESSED_RIGH)
		{
			if(mouse_lock_R == false && sys.mode == SYS_MODE_NORMAL) 
			{
				sys.mode = SYS_MODE_AUTO;
				sys.co_mode = CO_GYRO;
			}
			mouse_lock_R = true;	// 右键锁定
		}
		else
		{
			if(sys.mode == SYS_MODE_AUTO)
			{
				sys.mode = SYS_MODE_NORMAL;
				sys.co_mode = CO_GYRO;
				
			}
			mouse_lock_R = false;	// 右键锁定
		}
		
		if(rc_sensor.info->thumbwheel > 330)
		{
			sys.mode = SYS_MODE_AUTO;
			sys.co_mode = CO_GYRO;
		}
		else if(rc_sensor.info->thumbwheel < -330)
		{
			sys.mode = SYS_MODE_NORMAL;
			sys.co_mode = CO_GYRO;
		}
	}
	else
	{
//		if(IF_RC_SW1_MID) {
//			/* 常规 -> 自瞄 */
//			if(rc_lock_SW1 == false && sys.mode == SYS_MODE_NORMAL) 
//			{
//				sys.mode = SYS_MODE_AUTO;
//				sys.co_mode = CO_GYRO;
//			}
//			rc_lock_SW1 = true;// Sw1锁定
//		} 
//		/* Sw1 其他档 */
//		else {
//			/* 退出自瞄模式判断(自瞄 -> 常规)*/
//			if(sys.mode == SYS_MODE_AUTO) 
//			{
//				sys.mode = SYS_MODE_NORMAL;
//				sys.co_mode = CO_GYRO;
//			}
//			rc_lock_SW1 = false;// Sw1解锁
//		}
		
		
	}
	
	/*以下是与键盘有关的一系列模式切换操作*/
	if(IF_KEY_PRESSED_CTRL || relife_flag)
	{
		KeyCtrlTime_Cur = osKernelSysTick()+500;//获取当前时间
		if(sys.co_mode == CO_GYRO)
		{
			if(sys.mode == SYS_MODE_AUTO)
			{
				sys.mode = SYS_MODE_NORMAL;
			}
			 
			sys.co_mode = CO_MECH;
		}
		else if(sys.co_mode == CO_MECH)//不做处理，或者做按键判断
		{
			
		}
		
	}
	else if(KeyCtrlTime_Cur<osKernelSysTick())//没有按下ctrl时,且超时
	{
		if(sys.co_mode == CO_GYRO)//不做处理
		{
			
		}
		else if(sys.co_mode == CO_MECH)
		{
			if(sys.mode == SYS_MODE_LONGSHOOT)
			{
				sys.mode = SYS_MODE_NORMAL;
			}
			
			if(relife_flag == 0)
			{
				sys.co_mode = CO_GYRO;
			}
		}
		
	}
	
	
}

static void system_state_machine(void)
{
	// 控制方式切换
	system_ctrl_mode_switch();
	// 控制方式切换(键盘模式下才允许切换)
	if(sys.remote_mode == KEY)
		system_mode_act_switch();
}

/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	系统决策任务
 */
uint32_t relife_time;
uint32_t thotogate = 1;
void StartSystemTask(void const * argument)
{
	relife_time = osKernelSysTick()+3000;
	for(;;)
	{
		portENTER_CRITICAL();//进入临界区
		
		// 更新遥控信息，获取数据
		rc_update_info();
		
		if(relife_time<osKernelSysTick())
		{
			relife_flag = 0;
			BMI_SET_Kp(0.1f);
		}

		//信息处理
		/* 遥控离线 */
		if(rc_sensor.work_state == DEV_OFFLINE) 
		{
			sys.state = SYS_STATE_RCLOST;
			RC_ResetData(&rc_sensor);
			sys.remote_mode = RC;
			sys.co_mode = CO_MECH;
			flag.gimbal.reset_start = false;
			flag.gimbal.reset_ok = false;
		} 
		/* 遥控在线 */
		else if(rc_sensor.work_state == DEV_ONLINE)
		{
			/* 遥控正常 */
			if(rc_sensor.errno == NONE_ERR) 
			{
				/* 失联恢复 */
				if(sys.state == SYS_STATE_RCLOST) 
				{
					// 可在此处同步云台复位标志位					
					// 系统参数复位
					
					if(flag.gimbal.reset_ok == true)
					{
						sys.remote_mode = RC;
						sys.state = SYS_STATE_NORMAL;
						sys.mode = SYS_MODE_NORMAL;
					}
					else
					{
						
						flag.gimbal.reset_start = true;
						flag.gimbal.reset_ok = false;
						relife_flag = 1;
					}
					
				}
				else if(sys.state == SYS_STATE_NORMAL)
				{
					system_state_machine();
				}
				// 可在此处等待云台复位后才允许切换状态
				//
				
			}
			/* 遥控错误 */
			else if(rc_sensor.errno == DEV_DATA_ERR) {
				sys.state = SYS_STATE_RCERR;
				//reset CPU
				__set_FAULTMASK(1);//关闭总中断
				NVIC_SystemReset();
			} else {
				sys.state = SYS_STATE_WRONG;
				//reset CPU
				__set_FAULTMASK(1);//关闭总中断
				NVIC_SystemReset();
			}
		}
		thotogate = PEin(11);
		portEXIT_CRITICAL();//离开临界区
		
		osDelay(2);
	}
}
