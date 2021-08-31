#ifndef __PREDICT__H
#define __PREDICT__H

#include "vision_sensor.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "rp_math.h"
#include "filter.h"
#include "kalman.h"

#ifndef PI 
	#define PI 3.14159265358979f
#endif

#ifndef Gravity_g
	#define Gravity_g 9.788f
#endif

#define SHOOT_SPEED 15.0f 

#define ARMER_SWITCH_THRESHOLD 50.0f

//反陀螺启动时间
#define AUTO_ANTI_TOP_TICK 1500

//反陀螺装甲板更新时间间隔的阈值，视觉防干扰能力越强，该值可设越小
//从时间上防止没有陀螺的时候仍会有切换装甲板情况导致边界不断更新，越缩越小
//单位毫秒
#define ANTI_TOP_TIME_THRESHOLD 30

//反陀螺装甲板更新时角度间隔的阈值，视觉防干扰能力越强，该值可设越小
//从空间上防止没有陀螺的时候仍会有切换装甲板情况导致边界不断更新，越缩越小
//单位，电机角度
#define ANTI_TOP_SPACE_THRESHOLD 80

typedef enum
{
	NONE,
	STANDING_BY,
	READY
}Anti_top_State;

typedef enum
{
	AT_DISABLE,
	AT_ENABLE
}Anti_top_Switch;

typedef struct Anti_top_Data
{
	float binary_low;
	float binary_high;
	float top_speed;
	float top_mid;
	float top_circle;
	uint32_t cnt;
	uint32_t cnt_max;
	Anti_top_State state;
	Anti_top_Switch AT_witch;
}Anti_top_Data;

//初始化
void PREDICT_Init(void);

//公式法求解发射角
float Fire_angle_cal(float distance, float angle);

//经验法求解发射角
float predict_cal_shoot_angle(float distance, float angle, float Friction_speed);

uint32_t Vision_get_interval(uint8_t lost_flag);

//判断跳变沿
uint8_t Predict_Judge_JumpEdge(float yaw_angle);

//自瞄基本属性解算
void PREDICT_Cal_Base(uint8_t lost_flag, float Tg_y, float Tg_p, float Distance);
void PREDICT_Base_Reset(float Tg_y, float Tg_p, float Distance);

//自瞄反陀螺数据计算
void Predict_Anti_Top_Cal_all(float binary_l, float binary_h);
void Predict_Anti_Top_Data_Heart_beat(void);
void Predict_Anti_Top_Data_Clear(void);

//根据边界对目标yaw进行循环操作
float Predict_Anti_Top_Judge_Yaw(float auto_yaw);

//判断输入角度是否处于陀螺区域内
int8_t Predict_Anti_Top_Binary_judge(float yaw_angle);

//更新陀螺移动边界
void Predict_Anti_Top_binary_update(float now_yaw);

void vision_anti_top(void);

extern moving_Average_Filter frams_MAF;

#endif
