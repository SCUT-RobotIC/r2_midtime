/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

#include "bsp_fdcan.h"
#include "main.h"
#include "motorctrl.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
#define m3519_init -30*8191/360

extern int HelmInit[10];
extern motor_measure_t *motor_data_can1[8];
extern motor_measure_t *motor_data_can2[8];
extern motor_measure_t *motor_data_can3[8];
extern uint8_t KEY_STA[10];
extern int KEY_CNT[10];
/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = chassis_id1_Pin|chassis_id2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = chassis_id3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(chassis_id3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 2 */
void HAL_GPIO_EXTI_Callback(uint16_t KEYNUM)
{
	if(KEYNUM==chassis_id1_Pin)
	{
		
		HelmInit[0]=motor_data_can2[0]->circle*8191+motor_data_can2[0]->ecd;
		KEY_STA[0]=1;
//		rtU.status_CH2_1=2;
//		set_target(2,1,HelmInit[0]+m3519_init*9);
		//set_target(2,1,0);

	}
		if(KEYNUM==chassis_id2_Pin)
	{
		
		HelmInit[1]=motor_data_can2[1]->circle*8191+motor_data_can2[1]->ecd;
		KEY_STA[1]=1;
//		rtU.status_CH2_2=2;
//		set_target(2,2,HelmInit[1]+m3519_init*9);
//		set_target(2,2,0);

	}
		if(KEYNUM==chassis_id3_Pin)
	{
		
		HelmInit[2]=motor_data_can2[2]->circle*8191+motor_data_can2[2]->ecd;
		KEY_STA[2]=1;
//		rtU.status_CH2_3=2;
//		set_target(2,3,HelmInit[2]+m3519_init*9);
//		set_target(2,3,0);
	}

}
/* USER CODE END 2 */
