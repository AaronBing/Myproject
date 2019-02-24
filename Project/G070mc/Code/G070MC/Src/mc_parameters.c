
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the 
  *          configuration of the subsystem.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "parameters_conversion.h"

#include "r1_g0xx_pwm_curr_fdbk.h"
 
 
 

  #define MAX_TWAIT 0                 /* Dummy value for single drive */
  #define FREQ_RATIO 1                /* Dummy value for single drive */
  #define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/**
  * @brief  Current sensor parameters Single Drive - one shunt
  */
const R1_G0XXParams_t R1_G0XX_Params =
{
/* Current reading A/D Conversions initialization -----------------------------*/
  .hIChannel = MC_ADC_CHANNEL_6,
  
/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime = DEAD_TIME_COUNTS, 
  .bRepetitionCounter = REP_COUNTER,
  .hTafter = TAFTER,
  .hTbefore = TBEFORE,
  .hTMin = TMIN,
  .hHTMin = HTMIN,
  .hCHTMin = CHTMIN,
  .hTSample = SAMPLING_TIME,
  .hMaxTrTs = MAX_TRTS,

/* PWM Driving signals initialization ----------------------------------------*/
  .TIMx = PWM_TIM1,
  
  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
    
};

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
