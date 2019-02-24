/**
  ******************************************************************************
  * @file    mc_stm_types.h 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Includes HAL/LL headers relevant to the current configuration.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */ 
#ifndef __MC_STM_TYPES_H
#define __MC_STM_TYPES_H
  
#ifndef USE_FULL_LL_DRIVER
#define USE_FULL_LL_DRIVER
#endif

#ifdef MISRA_C_2004_BUILD
#error "The code is not ready for that..."
#endif

  #include "stm32g0xx_ll_bus.h"
  #include "stm32g0xx_ll_rcc.h"
  #include "stm32g0xx_ll_system.h"
  #include "stm32g0xx_ll_adc.h"
  #include "stm32g0xx_ll_tim.h"
  #include "stm32g0xx_ll_gpio.h"
  #include "stm32g0xx_ll_usart.h"
  #include "stm32g0xx_ll_dac.h"
  #include "stm32g0xx_ll_dma.h"
  #include "stm32g0xx_ll_comp.h"

/* Enable Fast division optimization for cortex-M0[+] micros*/
  # define FASTDIV
/* USER CODE BEGIN DEFINITIONS */
/* Definitions placed here will not be erased by code generation */

/* USER CODE END DEFINITIONS */

#endif /* __MC_STM_TYPES_H */
/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
