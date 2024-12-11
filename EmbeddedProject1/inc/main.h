#pragma once


/******************************************************************************
* Includes
******************************************************************************/
#include "stm32f4xx_hal.h"
#include "stm32_hal_legacy.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"
#include "stdio.h"
#include "stdlib.h"

#include "stm32f4xx_it.h"
#include "arm_math.h"
#include "types.h"

/******************************************************************************
* Types
******************************************************************************/













/******************************************************************************
* Global functions
******************************************************************************/

void Error_Handler(void);


/******************************************************************************
* Inline functions
******************************************************************************/

#define _ERROR	GPIOA->BSRR = LL_GPIO_PIN_7<<16




