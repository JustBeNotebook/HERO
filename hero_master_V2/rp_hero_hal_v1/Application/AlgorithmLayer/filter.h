#ifndef RM__FILTER__H
#define RM__FILTER__H

#include "rp_config.h"
#include "rp_math.h"


#define MAF_MaxSize 100
#define Median_F_MaxSize 100
#define MAF_anti_top_MaxSize 1000


typedef struct moving_Average_Filter
{
	float num[MAF_MaxSize];
	uint8_t lenth;
	uint8_t pot;//当前位置
	float total;
	float aver_num;
	
	
}moving_Average_Filter;	//最大设置MAF_MaxSize个


typedef struct MAF_Anti_top
{
	float num[MAF_anti_top_MaxSize];
	uint16_t lenth;
	uint16_t pot;//当前位置
	float total;
	float aver_num;
	float max;
	float min;
}MAF_Anti_top;	//最大设置MAF_MaxSize个


//索引数组负责记录数据进入数据窗口时pot是第几号
//提供删除数据的索引
typedef struct Median_Filter
{
	float data_num[Median_F_MaxSize];//数值数组
	int data_index_num[Median_F_MaxSize];
	uint8_t lenth;

	uint8_t index_pot;//始终指向下一个要删除的pot
	float median_data;
	
}Median_Filter;	//最大设置MAF_MaxSize个


typedef struct LOW_Pass_Filter
{
	float last;
	float now;
	float threshold;
	float output;
	uint8_t High_flag;//加了一个跳变时指示的flag
}LOW_Pass_Filter;


//滑动滤波器对应的操作函数
void average_add(moving_Average_Filter *Aver, float add_data);
float average_get(moving_Average_Filter *Aver, uint16_t pre);//获取前n次的数据
void average_init(moving_Average_Filter *Aver, uint8_t lenth);
void average_clear(moving_Average_Filter *Aver);
void average_fill(moving_Average_Filter *Aver, float temp);//往滑动滤波填充某个值

void MAF_ANTI_TOP_add(MAF_Anti_top *Aver, float add_data);
float MAF_ANTI_TOP_get(MAF_Anti_top *Aver, uint16_t pre);//获取前n次的数据
void MAF_ANTI_TOP_init(MAF_Anti_top *Aver, uint16_t lenth);
void MAF_ANTI_TOP_clear(MAF_Anti_top *Aver);


//中值滤波器对应的操作函数
void median_add(Median_Filter *Median, float add_data);
float median_get(Median_Filter *Median, uint16_t pre);//获取前n次的数据
void median_init(Median_Filter *Median, uint8_t lenth);
void median_clear(Median_Filter *Median);

//低通滤波器对应的操作函数
void LPF_Init(LOW_Pass_Filter *LPF, float threshold);
float LPF_add(LOW_Pass_Filter *LPF, float input_data);
void LPF_Clear(LOW_Pass_Filter *LPF);


////////////////////////////////外部变量滤波器/////////////////

/////////////滑动均值滤波器
extern moving_Average_Filter KEY_W, KEY_A, KEY_S, KEY_D;
extern moving_Average_Filter MOUSE_X, MOUSE_Y;

extern MAF_Anti_top Absolute_yaw_angle_raw, Absolute_pitch_angle_raw, Absolute_distance_raw;


#endif

