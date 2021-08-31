#include "filter.h"




moving_Average_Filter Vision_y_speed;
moving_Average_Filter Vision_p_speed;
moving_Average_Filter Vision_y_angle;
moving_Average_Filter Vision_p_angle;

moving_Average_Filter KEY_W, KEY_A, KEY_S, KEY_D;
moving_Average_Filter MOUSE_X, MOUSE_Y;


MAF_Anti_top Absolute_yaw_angle_raw, Absolute_pitch_angle_raw, Absolute_distance_raw;

/**
  * @brief    average_add
  * @note    滑动平均滤波器进入队列，先进先出
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
void average_add(moving_Average_Filter *Aver, float add_data)
{
	
	Aver->total -=  Aver->num[Aver->pot];
	Aver->total += add_data;
	
	Aver->num[Aver->pot] = add_data;
	
	Aver->aver_num = (Aver->total)/(Aver->lenth);
	Aver->pot++;
	
	if(Aver->pot == Aver->lenth)
	{
		Aver->pot = 0;
	}
	
}

/**
  * @brief    average_get
  * @note    获取第前pre次的数据，如果超出数组长度则取记录的最早的数据
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
float average_get(moving_Average_Filter *Aver, uint16_t pre)
{
	float member;
	uint8_t temp;
	
	if(Aver->pot != 0)
	{
		temp = Aver->pot-1;
	}
	else 
	{
		temp = Aver->lenth-1;
	}
	
	if(pre>Aver->lenth)
		pre = pre % Aver->lenth;
	
	if(pre>temp)
	{
		pre = Aver->lenth+temp-pre;
	}
	else
	{
		pre = temp-pre;
	}
	
	member = Aver->num[pre];
	
	return member;
}

/**
  * @brief    average_init
  * @note    滑动滤波器初始化，设置长度
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
void average_init(moving_Average_Filter *Aver, uint8_t lenth)
{
	uint16_t i;
	
	for(i = 0; i<MAF_MaxSize; i++)
		Aver->num[i] = 0;
	
	if(lenth >MAF_MaxSize)
	{
		lenth = MAF_MaxSize;
	}
	
	Aver->lenth = lenth;
	Aver->pot = 0;
	Aver->aver_num = 0;
	Aver->total = 0;
	
}

/**
  * @brief    average_clear
  * @note    滑动滤波器清空
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
void average_clear(moving_Average_Filter *Aver)
{
	uint16_t i;
	
	for(i = 0; i<MAF_MaxSize; i++)
		Aver->num[i] = 0;
	
	Aver->pot = 0;
	Aver->aver_num = 0;
	Aver->total = 0;
}


/**
  * @brief    average_fill
  * @note    滑动滤波器填充某个值
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
void average_fill(moving_Average_Filter *Aver, float temp)
{
	uint16_t i;
	
	for(i = 0; i<(Aver->lenth); i++)
		Aver->num[i] = temp;
	
	Aver->pot = 0;
	Aver->aver_num = temp;
	Aver->total = temp*(Aver->lenth);
}



void MAF_ANTI_TOP_add(MAF_Anti_top *Aver, float add_data)
{
	
	Aver->total -=  Aver->num[Aver->pot];
	Aver->total += add_data;
	
	Aver->num[Aver->pot] = add_data;
	
	Aver->aver_num = (Aver->total)/(Aver->lenth);
	Aver->pot++;
	
	if(Aver->pot == Aver->lenth)
	{
		Aver->pot = 0;
	}
	
}


float MAF_ANTI_TOP_get(MAF_Anti_top *Aver, uint16_t pre)//获取前n次的数据
{
	float member;
	
	if(pre>Aver->lenth)
		pre = pre % Aver->lenth;
	
	if(pre>Aver->pot)
	{
		pre = Aver->lenth+Aver->pot-pre;
	}
	else
	{
		pre = Aver->pot-pre;
	}
	
	member = Aver->num[pre];
	
	return member;
}
	
void MAF_ANTI_TOP_init(MAF_Anti_top *Aver, uint16_t lenth)
{
	uint16_t i;
	
	for(i = 0; i<MAF_anti_top_MaxSize; i++)
		Aver->num[i] = 0;
	
	
	if(lenth >MAF_anti_top_MaxSize)
	{
		lenth = MAF_anti_top_MaxSize;
	}
	
	Aver->lenth = lenth;
	Aver->pot = 0;
	Aver->aver_num = 0;
	Aver->total = 0;
	
}

void MAF_ANTI_TOP_clear(MAF_Anti_top *Aver)
{
	uint16_t i;
	
	for(i = 0; i<MAF_anti_top_MaxSize; i++)
		Aver->num[i] = 0;
	
	Aver->pot = 0;
	Aver->aver_num = 0;
	Aver->total = 0;
}

//add_data的索引与index_pot绑定
void median_add(Median_Filter *Median, float add_data)
{
	if(Median->lenth<=0)
	{
		return;
	}
	
	uint16_t pot = 0;
	uint16_t data_index_pot;
	float data_temp, index_temp;
	
	//遍历寻找要删掉的数据,pot最终停留位置就是要删除的数据
	for(pot = 0; pot<Median->lenth; pot++)
	{
		if(Median->data_index_num[pot] == Median->index_pot)
		{
			break;
		}
	}
	
	Median->data_num[pot] = add_data;
	
	//从默认有序的数组中进行冒泡排序
	if(Median->data_num[pot+1]<add_data)//adddata后移
	{
		while(Median->data_num[pot+1]<add_data && pot<Median->lenth)
		{
			Median->data_num[pot] = Median->data_num[pot+1];
			Median->data_index_num[pot] = Median->data_index_num[pot+1];
			pot++;
		}
	}
	else if(Median->data_num[pot-1]>add_data)//adddata前移
	{
		while(Median->data_num[pot-1]>add_data && pot>0)
		{
			Median->data_num[pot] = Median->data_num[pot-1];
			Median->data_index_num[pot] = Median->data_index_num[pot-1];
			pot--;
		}
	}
	
	Median->data_num[pot] = add_data;
	Median->data_index_num[pot] = Median->index_pot;
	
	Median->index_pot++;
	if(Median->index_pot == Median->lenth)
	{
		Median->index_pot = 0;
	}
	
	Median->median_data = Median->data_num[Median->lenth/2];
}

float median_get(Median_Filter *Median, uint16_t pre);//获取前n次的数据
void median_init(Median_Filter *Median, uint8_t lenth)
{
	uint16_t i;
	
	for(i = 0; i<MAF_anti_top_MaxSize; i++)
	{
		Median->data_num[i] = 0;
		Median->data_index_num[i] = i;
	}
	
	if(lenth >Median_F_MaxSize)
	{
		lenth = Median_F_MaxSize;
	}
	
	Median->lenth = lenth;
	Median->index_pot = 0;
	Median->median_data = Median->data_num[Median->lenth/2];
}
void median_clear(Median_Filter *Median)
{
	uint16_t i;
	
	for(i = 0; i<MAF_anti_top_MaxSize; i++)
	{
		Median->data_num[i] = 0;
		Median->data_index_num[i] = i;
	}
	Median->median_data = Median->data_num[Median->lenth/2];
	Median->index_pot = 0;
}



/**
  * @brief    LPF_Init
  * @note    LOW_PASS_FILTER Initialize
  * @param  	LPF 低通滤波器的指针
							threshold	低通滤波器的阈值
  * @retval 	none
  * @author  RobotPilots
  */
void LPF_Init(LOW_Pass_Filter *LPF, float threshold)
{
	LPF->last = 0;
	LPF->now = 0;
	LPF->threshold = threshold;
	LPF->output = 0;
	LPF->High_flag = 0;
}

/**
  * @brief    LPF_Init
  * @note    LOW_PASS_FILTER Initialize
  * @param  	LPF 低通滤波器的指针
							input_data 进入低通滤波的数据
  * @retval 	none
  * @author  RobotPilots
  */
	float err_temp;
float LPF_add(LOW_Pass_Filter *LPF, float input_data)
{

	
	
	//判断不正常的情况
	if((abs(LPF->last))>LPF->threshold || (abs(LPF->now))>LPF->threshold)
	{
		if(abs(input_data)<LPF->threshold)//如果输入小于阈值，通过
		{
			LPF->now = input_data;
			LPF->last = LPF->now;
			LPF->output = 0;
			LPF->High_flag = 0;
			return 0;
		}
		else
		{
			LPF->now = 0;
			LPF->last = LPF->now;
			LPF->output = 0;
			LPF->High_flag = 0;
			return 0;
		}
	}
	
	//判断正常的情况
	err_temp = abs(input_data-LPF->now);
	if(err_temp<LPF->threshold)//小于阈值
	{
		LPF->last = LPF->now;
		LPF->now = input_data;
		LPF->High_flag = 0;
	}
	else	//大于阈值
	{
		//不做处理
		LPF->High_flag = 1;
	}
	
	LPF->output = LPF->now;
	return LPF->output;
}


void LPF_Clear(LOW_Pass_Filter *LPF)
{
	LPF->last = 0;
	LPF->now = 0;
	LPF->output = 0;
	LPF->High_flag = 0;
}


