#ifndef __DRV_IO_H
#define __DRV_IO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "main.h"

/* Exported macro ------------------------------------------------------------*/
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //输出 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
// Led
#define LED_RED_ON()		(HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET))
#define LED_RED_OFF()		(HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET))
#define LED_RED_TOGGLE()	(HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin))

#define LED_GREEN_ON()		(HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET))
#define LED_GREEN_OFF()		(HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET))
#define LED_GREEN_TOGGLE()	(HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin))

#define LED_BLUE_ON()		(HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET))
#define LED_BLUE_OFF()		(HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET))
#define LED_BLUE_TOGGLE()	(HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin))

#define LED_ORANGE_ON()		(HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET))
#define LED_ORANGE_OFF()	(HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET))
#define LED_ORANGE_TOGGLE()	(HAL_GPIO_TogglePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin))

#define REBOOT_RELAY_SET_ON()		(HAL_GPIO_WritePin(REBOOT_RELAY_SET_GPIO_Port, REBOOT_RELAY_SET_Pin, GPIO_PIN_RESET))
#define REBOOT_RELAY_SET_OFF()	(HAL_GPIO_WritePin(REBOOT_RELAY_SET_GPIO_Port, REBOOT_RELAY_SET_Pin, GPIO_PIN_SET))
#define REBOOT_RELAY_SET_TOGGLE()	(HAL_GPIO_TogglePin(REBOOT_RELAY_SET_GPIO_Port, REBOOT_RELAY_SET_Pin))

// Laser
#define LASER_ON()			(HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET))
#define LASER_OFF()			(HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET))
#define LASER_TOGGLE()		(HAL_GPIO_TogglePin(LASER_GPIO_Port, LASER_Pin))

#define REBOOT_READ() 	(HAL_GPIO_ReadPin(REBOOT_READ_GPIO_Port,REBOOT_READ_Pin))
#define PHOTOGATE_READ() 	(HAL_GPIO_ReadPin(PHOTOGATE_GPIO_Port,PHOTOGATE_Pin))

#endif
