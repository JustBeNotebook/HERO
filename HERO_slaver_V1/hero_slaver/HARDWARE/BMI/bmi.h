#ifndef __BMI_H
#define __BMI_H



#include "link.h"
//#include <arm_math.h>
#include "math.h"
#include "bmi2.h"
#include "bmi2_common.h"
#include "bmi270.h"
#include "spi2.h"


#define CHIPID 0x00   //  芯片ID   0xd1
#define PMU_STATUS 0x03  //显示传感器的电源模式
#define ACC_CONF 0x40   //设置输出数据速率  加速度传感器读取模式     默认0x28
#define ACC_RANGE 0x41   //设置加速度范围                             默认0x03    +- 2g
#define GYR_CON 0x42     //设置输出数据速率  陀螺仪读取的模式     默认0x28
#define GYR_RANGE 0x43     //设置角速度测量范围          默认0x00    +-2000 °/s
#define INT_EN 0x50        //中断设置
#define INT_OUT_CTRL 0x53   //输出使能控制
#define INT_LATCH 0x54   //设置中断锁存
#define CMD 0x7E    //命令寄存器触发操作
#define CONTROL 0x7e //  0x11:set pmu mode of accelerometer to normal   0x15:set pmu mode of gyroscope to normal

#define ACCD_X_LSB 0x0c
#define ACCD_X_MSB 0x0d
#define ACCD_Y_LSB 0x0e
#define ACCD_Y_MSB 0x0f
#define ACCD_Z_LSB 0x10
#define ACCD_Z_MSB 0x11
#define GYR_X_LSB 0x12
#define GYR_X_MSB 0x13
#define GYR_Y_LSB 0x14
#define GYR_Y_MSB 0x15
#define GYR_Z_LSB 0x16
#define GYR_Z_MSB 0x17

#define BMI_CS PBout(12)


void BMI_GET_DATA(short *ggx,short *ggy,short *ggz,short *aax,short *aay,short *aaz);
void BMI_Get_AUX(short *au1,short *au2,short *au3,short *au4);
u8 BMI_Read(u8 reg);

u8 BMI_Write(u8 reg,u8 val);

int8_t BMI_Init(void);


void BMI_Get_GRO(short *gx,short *gy,short *gz);


void BMI_Get_ACC(short *ax,short *ay,short *az);

u8 BMI_Get_data(float *pitch,float *roll,float *yaw,short *ggx,short *ggy,short *ggz,short *aax,short *aay,short *aaz);


#endif



