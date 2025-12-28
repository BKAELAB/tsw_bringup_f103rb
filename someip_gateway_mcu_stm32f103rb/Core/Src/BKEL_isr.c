/*
 * BKEL_isr.c
 *
 *  Created on: Dec 20, 2025
 *      Author: seokjun.kang
 */

#include "BKEL_typedef.h"

// ISR for GPIO EXTI 15:10

//void EXTI15_10_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
//
//  /* USER CODE END EXTI15_10_IRQn 0 */
//
//  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
//  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
//
//  /* USER CODE END EXTI15_10_IRQn 1 */
//}

// brief  This function is executed in case of error occurrence.
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
}
