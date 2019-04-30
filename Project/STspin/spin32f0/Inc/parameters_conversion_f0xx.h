/**
  ******************************************************************************
  * @file    parameters_conversion_f0xx.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions needed to convert MC SDK parameters
  *          so as to target the STM32F0 Family.
  *
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_F0XX_H
#define __PARAMETERS_CONVERSION_F0XX_H

#include "pmsm_motor_parameters.h"
#include "power_stage_parameters.h"
#include "drive_parameters.h"

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ      48000000uL
#define TIM_CLOCK_DIVIDER  1 
#define ADV_TIM_CLK_MHz    48
#define ADC_CLK_MHz    14uL /* Maximum ADC Clock Frequency expressed in MHz */
#define HALL_TIM_CLK       48000000uL
#define ADC1_2  ADC1

/*********************** SENSORLESS REV-UP PARAMETERS *************************/
#define FIRST_SLESS_ALGO_PHASE (ENABLE_SL_ALGO_FROM_PHASE-1u)  

/* Legacy for WB 4.0 Beta */
#if !defined(OPEN_LOOP_VF)
#define OPEN_LOOP_VF false
#endif
#if !defined(OPEN_LOOP_OFF)
#define OPEN_LOOP_OFF 4400
#endif
#if !defined(OPEN_LOOP_K)
#define OPEN_LOOP_K 44
#endif

/*************************  IRQ Handler Mapping  *********************/														  
 #define CURRENT_REGULATION_IRQHandler          DMA1_Channel1_IRQHandler
#define TIMx_UP_BRK_M1_IRQHandler               TIM1_BRK_UP_TRG_COM_IRQHandler
#define DMAx_R1_M1_IRQHandler                   DMA1_Channel4_5_IRQHandler

/**********  AUXILIARY TIMER (SINGLE SHUNT) *************/
 
#define R1_PWM_AUX_TIM                  TIM3

#define TRIG_CONV_LATENCY_NS	259ul /* Referred to Maximum value indicated 
                                    in the Datasheet Table 50 if ADC clock = HSI14 */ 				   
#define SAMPLING_TIME_CORRECTION_FACTOR_NS           500ul/ADC_CLK_MHz                 /* 0.5*1000/ADC_CLK_MHz */ 
#define SAMPLING_TIME_NS ((7 * 1000uL/ADC_CLK_MHz)+SAMPLING_TIME_CORRECTION_FACTOR_NS)

#define ADC_CONV_NB_CK 13u
#define ADC_CONV_TIME_NS    (uint16_t) (((ADC_CONV_NB_CK*1000ul)-500ul)/ADC_CLK_MHz)

#define TW_BEFORE (((uint16_t)(((2*SAMPLING_TIME_NS)+ TRIG_CONV_LATENCY_NS + ADC_CONV_TIME_NS)*ADV_TIM_CLK_MHz)/1000ul)+1u)

#define M1_VBUS_SW_FILTER_BW_FACTOR      10u

/* Sampling time allowed for F0xx are: 1, 7, 13, 28 ADC clock cycle */    
#define M1_VBUS_SAMPLING_TIME  LL_ADC_SAMPLINGTIME_28CYCLES_5

/* Sampling time allowed for F0xx are:  1, 7, 13, 28 ADC clock cycle */    
#define M1_TEMP_SAMPLING_TIME  LL_ADC_SAMPLINGTIME_1CYCLE_5

#endif /*__PARAMETERS_CONVERSION_F0XX_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
