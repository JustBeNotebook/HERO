/**
 * @file        pid.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        11-September-2020
 * @brief       Pid Algorithm.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "rp_math.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void pid_val_init(pid_ctrl_t *pid)
{
//	pid->target = 0;
//	pid->measure = 0;
	pid->err = 0;
	pid->last_err = 0;
	pid->integral = 0;
	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->out = 0;
	pid->true_err = 0;
}

void single_pid_ctrl(pid_ctrl_t *pid)
{
	// 保存误差值(需要在外面自行计算误差)
//	pid->err = err;
	// 积分
	pid->integral += pid->err;
	pid->integral = constrain(pid->integral, -(pid->integral_max)+pid->integral_bias, +(pid->integral_max)+pid->integral_bias);
	// p i d 输出项计算
	pid->pout = pid->kp * pid->err;
	pid->iout = pid->ki * pid->integral;
	pid->dout = pid->kd * (pid->err - pid->last_err);
	// 累加pid输出值
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
	// 记录上次误差值
	pid->last_err = pid->err;
}

//void cascade_pid_ctrl(pid_ctrl_t *pid)
//{
//	pid_ctrl_t *outPid = pid;
//	pid_ctrl_t *inPid = outPid->next;
//	
//	// 外环PID
//	outPid->err = outPid->target - outPid->measure;
//	single_pid_ctrl(outPid);
//	// 内环期望 = 外环输出
//	inPid->target = outPid->out;
//	// 内环PID
//	inPid->err = inPid->target - inPid->measure;
//	single_pid_ctrl(inPid);
//}
