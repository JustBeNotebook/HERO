#ifndef __SOFT_IIC_H
#define __SOFT_IIC_H

#include "drv_io.h"

//IO口方向设置
#define SDA_IN() 	{GPIOB->MODER&=~((uint32_t)3<<(15*2));GPIOB->MODER|=(uint32_t)0<<(15*2);}	//PB7输入模式
#define SDA_OUT() 	{GPIOB->MODER&=~((uint32_t)3<<(15*2));GPIOB->MODER|=(uint32_t)1<<(15*2);}	//PB7输入模式
//IO口操作函数
#define IIC_SCL 	PBout(13)
#define IIC_SDA 	PBout(15)
#define READ_SDA 	PBin(15)			

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(uint8_t ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  

#endif

