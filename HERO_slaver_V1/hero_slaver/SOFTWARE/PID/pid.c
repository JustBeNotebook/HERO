#include "pid.h"

//PID公式计算
float Pos_PID_CAL_OUT(Pid_Module *PID, float err)
{
	//err死区限制
	if((err-PID->err_dz)<0.001f && (err+PID->err_dz)>-0.001f)
		err = 0;
	

	
	PID->i_err += err;
	PID->d_err = err-(PID->err);
	PID->err = err;
	
	if((PID->i_err)>(PID->i_err_limit))	//积分限幅
		PID->i_err = PID->i_err_limit;
	else if(PID->i_err<-(PID->i_err_limit))
		PID->i_err = -(PID->i_err_limit);
	
	PID->p_output = (PID->kp)*(PID->err);
	PID->i_output = (PID->ki)*(PID->i_err);
	PID->d_output = (PID->kd)*(PID->d_err);
	
	PID->output = PID->p_output + PID->i_output + PID->d_output;//PID输出需要限幅
	
	if((PID->output)>PID->output_limit)
			PID->output = PID->output_limit;
	else if(PID->output < -(PID->output_limit))
			PID->output = -PID->output_limit;
	
	
	return PID->output;
}

//PID初始化
void PID_Init(Pid_Module *PID)
{
	PID->err = 0;
	PID->err_dz = 0;
	PID->i_err = 0;
	PID->i_err_limit = 0;
	PID->d_err = 0;
	
	PID->kp = 0;
	PID->ki = 0;
	PID->kd = 0;
	PID->output = 0;
	PID->output_limit = 0;
	
	PID->startfalg = PID_DISABLE;
}

//设置PID参数
void PID_Set(Pid_Module *PID, float kp, float ki, float kd, float i_err_limit, float err_dz, float output_limit)
{
	
	PID->d_err = 0;
	PID->i_err = 0;
	PID->i_err_limit = i_err_limit;
	PID->err = 0;
	PID->err_dz = err_dz;
	PID->kp = kp;
	PID->ki = ki;
	PID->kd = kd;
	PID->output = 0;
	PID->output_limit = output_limit;


}

void PID_Set_Kpid(Pid_Module *PID, float kp, float ki, float kd)
{
	PID->kp = kp;
	PID->ki = ki;
	PID->kd = kd;
}

void PID_Set_limit(Pid_Module *PID,float err_limit, float i_err_limit, float err_dz, float output_limit)
{
	PID->err_limit = err_limit;
	PID->i_err_limit = i_err_limit;
	PID->err_dz = err_dz;
	PID->output_limit = output_limit;
}


//清空PID缓存
void PID_Clear(Pid_Module *PID)
{
	PID->d_err = 0;
	PID->i_err = 0;
	PID->err = 0;
	PID->output = 0;
	
}

