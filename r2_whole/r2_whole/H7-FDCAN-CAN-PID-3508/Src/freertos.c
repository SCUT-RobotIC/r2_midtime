/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "PID_MODEL.h"
#include "rtwtypes.h"
#include <math.h>
#include "bsp_fdcan.h"
#include "motorctrl.h"
#include "stdio.h"
#include "math.h"
#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
#include "UPPER_LOCATION.h"
#include "stdio.h"
#include "string.h"
#include "sbus.h"
#include "runball.h"
//#include "runball.h"
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
/* Definitions for chassis_task */
osThreadId_t chassis_taskHandle;
const osThreadAttr_t chassis_task_attributes = {
  .name = "chassis_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RunBallTask */
osThreadId_t RunBallTaskHandle;
const osThreadAttr_t RunBallTask_attributes = {
  .name = "RunBallTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void start_chassis_task(void *argument);
void StartRunBallTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of chassis_task */
  chassis_taskHandle = osThreadNew(start_chassis_task, NULL, &chassis_task_attributes);

  /* creation of RunBallTask */
  RunBallTaskHandle = osThreadNew(StartRunBallTask, NULL, &RunBallTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_start_chassis_task */
/**
  * @brief  Function implementing the chassis_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_chassis_task */
void start_chassis_task(void *argument)
{
  /* USER CODE BEGIN start_chassis_task */
  /* Infinite loop */
  for(;;)
  {
		chassis_control();
    osDelay(1);
  }
  /* USER CODE END start_chassis_task */
}

/* USER CODE BEGIN Header_StartRunBallTask */
/**
* @brief Function implementing the RunBallTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRunBallTask */
void StartRunBallTask(void *argument)
{
  /* USER CODE BEGIN StartRunBallTask */
  /* Infinite loop */
  for(;;)
  {	
		runball_Init();
		
		runball_ready();
		runball_Getkey();
		runball_Control();
    osDelay(1);
  }
  /* USER CODE END StartRunBallTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

