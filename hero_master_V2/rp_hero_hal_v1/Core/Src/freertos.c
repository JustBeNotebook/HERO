/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
osThreadId MonitorTaskHandle;
osThreadId ControlTaskHandle;
osThreadId SystemTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId GimbalTaskHandle;
osThreadId LauncherTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartMonitorTask(void const * argument);
extern void StartControlTask(void const * argument);
extern void StartSystemTask(void const * argument);
extern void StartChassisTask(void const * argument);
extern void StartGimbalTask(void const * argument);
extern void StartLauncherTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MonitorTask */
  osThreadDef(MonitorTask, StartMonitorTask, osPriorityRealtime, 0, 512);
  MonitorTaskHandle = osThreadCreate(osThread(MonitorTask), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, StartControlTask, osPriorityNormal, 0, 256);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of SystemTask */
  osThreadDef(SystemTask, StartSystemTask, osPriorityHigh, 0, 128);
  SystemTaskHandle = osThreadCreate(osThread(SystemTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, StartChassisTask, osPriorityHigh, 0, 256);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of GimbalTask */
  osThreadDef(GimbalTask, StartGimbalTask, osPriorityHigh, 0, 256);
  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

  /* definition and creation of LauncherTask */
  osThreadDef(LauncherTask, StartLauncherTask, osPriorityHigh, 0, 256);
  LauncherTaskHandle = osThreadCreate(osThread(LauncherTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	
	/* definition and creation of ChassisTask */
	
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartMonitorTask */
/**
  * @brief  Function implementing the MonitorTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartMonitorTask */
__weak void StartMonitorTask(void const * argument)
{
  /* USER CODE BEGIN StartMonitorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMonitorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
