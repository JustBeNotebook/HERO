#include "gimbal.h"

Gimbal_Module Gimbal;
Gimbal_Module infantry_Gimbal;
//MPU6050_Module M_Gimbal;

void Gimbal_State_Update(void)
{
	
	Motor_State_Update(&(Gimbal.Pitch_Motor));
	Motor_State_Update(&(Gimbal.Yaw_Motor));
	
}

//云台电机配置初始化
void Gimbal_Config(void)
{

	Motor_Module_Init(&(Gimbal.Yaw_Motor), 28000);
	Motor_Module_Init(&(Gimbal.Pitch_Motor), 28000);
	
	
	PID_Init(&(Gimbal.Yaw_Motor.speed_PID));
	PID_Init(&(Gimbal.Yaw_Motor.angle_PID));
	PID_Init(&(Gimbal.Pitch_Motor.angle_PID));
	PID_Init(&(Gimbal.Pitch_Motor.angle_PID));
	PID_Init(&(Gimbal.Mpu_Yaw_SPID));
	PID_Init(&(Gimbal.Mpu_Yaw_APID));
	PID_Init(&(Gimbal.Mpu_Pitch_SPID));
	PID_Init(&(Gimbal.Mpu_Pitch_APID));
	
	Gimbal.tg_PAngle = 2000;
	
	PID_Set(&(Gimbal.Mpu_Yaw_SPID), 10, 6, 0, 60, 1, 28000);
	PID_Set(&(Gimbal.Mpu_Yaw_APID), 9, 0, 0, 60, 0, 7000);
	
	//机械模式PID设置
	PID_Set(&(Gimbal.Yaw_Motor.speed_PID), 10, 6, 0, 60, 0, 28000);//速度环PID要配合陀螺仪速度使用
	PID_Set(&(Gimbal.Yaw_Motor.angle_PID), 9, 1, 0, 60, 0, 9000);
	PID_Set(&(Gimbal.Pitch_Motor.speed_PID), 11, 1, 0, 350, 3, 28000);
	PID_Set(&(Gimbal.Pitch_Motor.angle_PID), 10, 1, 0, 80, 5, 8500);

	//PID参数要重新调整
	
}


//设置目标角度函数 对应角度的最大值8191 对应0x1fff;
void Gimbal_set_angle(u8 motor_code, int angle)
{
	if(motor_code == 1)
	{
		Motor_set_angle(&(Gimbal.Pitch_Motor), angle);
	}
	else if(motor_code == 2)
	{
		Motor_set_angle(&(Gimbal.Yaw_Motor), angle);
	}
}



//与外部变量有关系
void Gimbal_Set_Output(short int out_1, short int out_2)
{
	CAN1_0X2ff_BUF[0] = ((out_1&0xff00)>>8)&0xff;
	CAN1_0X2ff_BUF[1] = (out_1)&0xff;
	CAN1_0X2ff_BUF[2] = ((out_2&0xff00)>>8)&0xff;
	CAN1_0X2ff_BUF[3] = (out_2)&0xff;
}


Motor_Module Pitch_temp, yaw_temp;

short int output_temp;
void Gimbal_PID_Cal(u8 Motor_code)
{
	Motor_Module *M_temp;
	Motor_speed speed_temp;

	float err;
	
	if(Motor_code == 0)
	{
		M_temp = &Gimbal.Pitch_Motor;
		Pitch_temp = Gimbal.Pitch_Motor;
	}
	else if(Motor_code == 1)
	{
		M_temp = &Gimbal.Yaw_Motor;
		yaw_temp = Gimbal.Yaw_Motor;
	}

	
	if(M_temp->state != M_CONNECTING)//如果电机不处于连接态，就不输出
	{
		M_temp->output = 0;
		return;
	}
	
	if(M_temp->angle_PID.startfalg)//电机反馈位置环
	{
		//位置环
		err = 8192.0f*((float)(M_temp->tg_angle_turns-M_temp->angle_turns))+((float)((M_temp->tg_angle) - (M_temp->angle)));
		Pos_PID_CAL_OUT(&(M_temp->angle_PID), err);
		speed_temp = (short int)(M_temp->angle_PID.output);
		
		Motor_set_speed(M_temp, speed_temp);
	}
	

	//速度环
	err = (float)((M_temp->tg_speed) - (M_temp->speed_real));//利用电机反馈的PID计算
	Pos_PID_CAL_OUT(&(M_temp->speed_PID), err);
	output_temp = (short int)(M_temp->speed_PID.output);

	if((output_temp)>M_temp->output_limit)
			output_temp = M_temp->output_limit;
	else if(output_temp < -(M_temp->output_limit))
			output_temp = -M_temp->output_limit;

	M_temp->output = output_temp;
	
}


void Gimbal_MpuPID_Cal(u8 Motor_code)
{
	Motor_Module *M_temp;
	Motor_speed speed_temp;
	Pid_Module *SPID, *APID;
	float tg_angle = 0;
	float err = 0;
	
	if(Motor_code == 0)
	{
		M_temp = &Gimbal.Pitch_Motor;
		SPID = &(Gimbal.Mpu_Pitch_SPID);
		APID = &(Gimbal.Mpu_Pitch_APID);
	}
	else if(Motor_code == 1)
	{
		M_temp = &Gimbal.Yaw_Motor;
		SPID = &Gimbal.Mpu_Yaw_SPID;
		APID = &Gimbal.Mpu_Yaw_APID;
	}

	
	if(M_temp->state != M_CONNECTING)//如果电机不处于连接态，就不输出
	{
		M_temp->output = 0;
		return;
	}
	

	//位置环
	//因为极其蛋疼的原因，云台顺时针旋转，需要给电机一个负值，所以此处应该获得一个负号
	err = -8192.0f*((float)(Gimbal.tg_YTurns-Gimbal.YTurns))-((float)((Gimbal.tg_YAngle) - (Gimbal.YAngle)));//目标角度-当前角度

	Pos_PID_CAL_OUT(APID, err);
	speed_temp = (short int)(APID->output);

	//速度环
	err = (float)(speed_temp - Gimbal.gyroz);
	Pos_PID_CAL_OUT(SPID, err);
	output_temp = (short int)(SPID->output);

	if((output_temp)>M_temp->output_limit)
			output_temp = M_temp->output_limit;
	else if(output_temp < -(M_temp->output_limit))
			output_temp = -M_temp->output_limit;

	M_temp->output = output_temp;

}



u8 MPU_Get_Gyroscope(short *gyrox, short *gyroy, short *gyroz)
{
	return 0;
}

void gimbal_get_imu_speed(void)
{
//	MPU_Get_Gyroscope(&(MPU6050_Gimbal.gyrox),&MPU6050_Gimbal.gyroy,&MPU6050_Gimbal.gyroz);	//读取角速度
//	mpu_dmp_get_data(&MPU6050_Gimbal.chip_pitch,&MPU6050_Gimbal.chip_roll,&MPU6050_Gimbal.chip_yaw);		//读取欧拉角
//	
//	
//	Gimbal.gyrox = -MPU6050_Gimbal.gyrox+0;	//x对应俯仰
//	Gimbal.gyroy = -MPU6050_Gimbal.gyroy+0;			//y对应横滚
//	Gimbal.gyroz = -MPU6050_Gimbal.gyroz-32;	//Z对应偏航
//	Gimbal.yaw = -MPU6050_Gimbal.chip_yaw+180.0f;		//云台偏航对应芯片偏航
//	Gimbal.pitch = -MPU6050_Gimbal.chip_roll+180.0f;	//云台俯仰对应芯片横滚
//	Gimbal.roll = -MPU6050_Gimbal.chip_pitch+180.0f;	//云台横滚对应芯片俯仰
}

//MPU6050姿态解算以及单位转换
void Gimbal_Mpu_Analyze(void)
{
	int temp;
	u16 angle_temp;

	gimbal_get_imu_speed();
	
	//Gimbal.yaw = Gimbal.yaw<0.0000f ? 0.0000f : Gimbal.yaw;//限幅大于0小于360
	//Gimbal.yaw = Gimbal.yaw>359.9899f? 359.9899f:Gimbal.yaw;
	
	Gimbal.gyrox = ((Gimbal.gyrox) > 10000) ? 10000 : (Gimbal.gyrox);
	Gimbal.gyrox = ((Gimbal.gyrox) < -10000) ? -10000 : (Gimbal.gyrox);
	Gimbal.gyroy = ((Gimbal.gyroy) > 10000) ? 10000 : (Gimbal.gyroy);
	Gimbal.gyroy = ((Gimbal.gyroy) < -10000) ? -10000 : (Gimbal.gyroy);
	Gimbal.gyroz = ((Gimbal.gyroz) > 10000) ? 10000 : (Gimbal.gyroz);
	Gimbal.gyroz = ((Gimbal.gyroz) < -10000) ? -10000 : (Gimbal.gyroz);
	
	angle_temp = (u16)((Gimbal.yaw/360)*8192);
	
//	if(angle_temp>8192)//陀螺仪出现越界
//	{
//		angle_temp = Gimbal.YAngle;
//	}
	
	if(angle_temp >= Gimbal.YAngle)//低通滤波，范围可以再缩小
	{
		temp = (int)(8192-angle_temp + Gimbal.YAngle);
		if(-4000<temp && temp<4000)//angle变化绝对值小于4000
		{
			(Gimbal.YTurns)--;
		}
	}
	else if(angle_temp < Gimbal.YAngle)
	{
		temp = (int)(8192+angle_temp - Gimbal.YAngle);
		if(-4000<temp && temp<4000)//angle变化绝对值小于4000
		{
			Gimbal.YTurns++;
		}
	}
	
	Gimbal.YAngle = angle_temp;
	
}

float angle_temp;
//设置gimbal角度
void Mpu_Yaw_tgAngle_add(int angle)
{
	
	int turns_temp;
	
	angle_temp = ((float)((int)Gimbal.tg_YAngle)) + ((float)angle);
	
	if(angle_temp<0.00001f)
	{
		turns_temp = (int)(angle_temp/8192.0f);
		Gimbal.tg_YTurns += turns_temp-1;
		Gimbal.tg_YAngle = (u16)((int)angle_temp)+(1-turns_temp)*8192;
	}
	else if(angle_temp>8192.000001f)
	{
		turns_temp = (int)(angle_temp/8192);
		Gimbal.tg_YTurns += turns_temp;
		Gimbal.tg_YAngle = (u16)((int)angle_temp)%8192;
	}
	else
	{
		Gimbal.tg_YAngle = ((u16)angle_temp);
	}
}


