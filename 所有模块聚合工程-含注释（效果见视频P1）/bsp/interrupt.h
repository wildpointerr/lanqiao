#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

#include "main.h"
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
#endif
