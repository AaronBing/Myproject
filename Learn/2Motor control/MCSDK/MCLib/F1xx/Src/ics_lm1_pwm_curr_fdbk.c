/**
  ******************************************************************************
  * @file    ics_lm1_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the ICS
  *          LM1 PWM Current Feedback component of the Motor Control SDK.
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
#include "pwm_curr_fdbk.h"
#include "ics_lm1_pwm_curr_fdbk.h"

#include "mc_type.h"

/** @addtogroup MCSDK
* @{
*/

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @defgroup ics_lm1_pwm_curr_fdbk ICS LM1 PWM & Current Feedback
 *
 * @brief STM32F1 Low & Medium Density, ICS PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F103 Low & Medium Density MCU
 * and using an Insulated Current Sensors topology.
 *
 * *STM32F1 Low & Medium Density MCUs* refers to STM32F103x4, STM32F103x6, STM32F103x8 and
 * STM32F103xB MCUs.
 *
 * @todo: TODO: complete documentation.
 *
 * @{
 */

/* ADC1 Data Register address */
#define NB_CONVERSIONS 16u

#define ADC_RIGHT_ALIGNMENT 3u

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))

/**
* @brief  It initializes TIM1, ADC and DMA1 for current reading
*         in ICS configuration using STM32F103x Low/Medium Density
* @param  ICS LM1 PWM Current Feedback Handle
* @retval none
*/
void ILM1_Init(PWMC_ICS_LM1_Handle_t *pHandle)
{

  pHandle->_Super.bTurnOnLowSidesAction = false;

  RCC->AHBENR |= LL_AHB1_GRP1_PERIPH_CRC;

  ILM1_TIM1Init(pHandle);

  /* TIM1 Counter Clock stopped when the core is halted */
  LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);

  /* Enable ADC1 and ADC2 */
  LL_ADC_Enable(ADC1);
  LL_ADC_Enable(ADC2);

  /* Enable external trigger (it will be SW) for ADC1 and ADC2 regular
  conversions */
  LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_StartConversionExtTrig(ADC2, LL_ADC_REG_TRIG_EXT_RISING);

  /* ADC1 Injected conversions configuration */
  LL_ADC_INJ_SetSequencerLength( ADC1, LL_ADC_INJ_SEQ_SCAN_DISABLE );
  LL_ADC_INJ_SetSequencerLength( ADC2, LL_ADC_INJ_SEQ_SCAN_DISABLE );

  /* Start calibration of ADC1 and ADC2 */
  LL_ADC_StartCalibration(ADC1);
  LL_ADC_StartCalibration(ADC2);

  /* Wait for the end of ADCs calibration */
  while ( LL_ADC_IsCalibrationOnGoing(ADC1) || LL_ADC_IsCalibrationOnGoing(ADC2) )
  {
  }
  
  /* Enable external trigger fo injected conv of ADC2 */
  LL_ADC_INJ_StartConversionExtTrig(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

  pHandle->OverCurrentFlag = false;
  pHandle->_Super.DTTest = 0u;
  pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;

}

/**
* @brief  It initializes TIM1 peripheral for PWM generation
* @param  ICS LM1 PWM Current Feedback Handle
* @retval none
*/
void ILM1_TIM1Init(PWMC_ICS_LM1_Handle_t *pHandle)
{

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH3);

  if (pHandle->pParams_str->LowSideOutputs == LS_PWM_TIMER)
  {
     LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N|LL_TIM_CHANNEL_CH2N|LL_TIM_CHANNEL_CH3N);
  }
  
  /* BKIN, if enabled */
  if ( (pHandle->pParams_str->EmergencyStop) != DISABLE )
  {
    LL_TIM_ClearFlag_BRK(TIM1);
    LL_TIM_EnableIT_BRK(TIM1);
  }

  /* Trigger Output signal is Update Event */
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);

  /* TIM1 counter enable */
  LL_TIM_EnableCounter(TIM1);
}

/**
* @brief  Offset computation for both current phases Ia and Ib. It is called
*         only during current calibration.
* @param pHandle ICS LM1 PWM Current Feedback Handle
* @retval none
*/
void ILM1_CurrentReadingCalibration(PWMC_Handle_t *pHandle)
{
  uint8_t index;
  uint32_t phaseaoffset = 0u, phaseboffset = 0u;

  PWMC_ICS_LM1_Handle_t * pH = (PWMC_ICS_LM1_Handle_t *) pHandle;

  /* ADC1 Injected end of conversions interrupt disabling */
  LL_ADC_DisableIT_JEOS(ADC1);

  /* ADC1 Injected conversions trigger is given by software and enabled */
  LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_SOFTWARE);
  LL_ADC_INJ_StartConversionExtTrig(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);

  /* ADC Channel used for current reading are read
  in order to get zero currents ADC values*/
  for(index = NB_CONVERSIONS; index != 0u; index--)
  {
    /* Clear the ADC1 JEOC pending flag */
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_INJ_StartConversionSWStart(ADC1);
    while(!LL_ADC_IsActiveFlag_JEOS(ADC1)) { }

    phaseaoffset += LL_ADC_INJ_ReadConversionData32 (ADC1, LL_ADC_INJ_RANK_1);
    phaseboffset += LL_ADC_INJ_ReadConversionData32 (ADC2, LL_ADC_INJ_RANK_1);
  }

  pH->wPhaseAOffset = (uint16_t)(phaseaoffset / (NB_CONVERSIONS/2) );
  pH->wPhaseBOffset = (uint16_t)(phaseboffset / (NB_CONVERSIONS/2) );

  LL_ADC_ClearFlag_JEOS(ADC1);

  /* ADC1 Injected conversions trigger is TIM1 Trigger Output */
  LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO);

  /* ADC1 Injected conversions end interrupt enabling */
  LL_ADC_EnableIT_JEOS(ADC1);
}

/**
* @brief Computes and return latest converted motor phase currents motor
* @param pHandle ICS LM1 PWM Current Feedback Handle
* @retval Ia and Ib current in Curr_Components format
*/
void ILM1_GetPhaseCurrents(PWMC_Handle_t *pHandle, Curr_Components* pStator_Currents)
{
  int32_t aux;
  uint16_t reg;
  PWMC_ICS_LM1_Handle_t * pH = (PWMC_ICS_LM1_Handle_t *) pHandle;

  /* Clear TIMx Update Flag necessary to detect FOC duration SW error */
  LL_TIM_ClearFlag_UPDATE(TIM1);

  /* Ia = (wPhaseAOffset)-(PHASE_A_ADC_CHANNEL value)  */
  reg = (uint16_t)((ADC1->JDR1) << 1);
  aux = (int32_t)(reg) - (int32_t)(pH->wPhaseAOffset);

  /* Saturation of Ia */
  if (aux < -INT16_MAX)
  {
    pStator_Currents->qI_Component1 = -INT16_MAX;
  }
  else  if (aux > INT16_MAX)
  {
    pStator_Currents->qI_Component1 = INT16_MAX;
  }
  else
  {
    pStator_Currents->qI_Component1 = (int16_t)aux;
  }

  /* Ib = (wPhaseBOffset)-(PHASE_B_ADC_CHANNEL value) */
  reg = (uint16_t)((ADC2->JDR1) << 1);
  aux = (int32_t)(reg) - (int32_t)(pH->wPhaseBOffset);

  /* Saturation of Ib */
  if (aux < -INT16_MAX)
  {
    pStator_Currents->qI_Component2 = -INT16_MAX;
  }
  else  if (aux > INT16_MAX)
  {
    pStator_Currents->qI_Component2 = INT16_MAX;
  }
  else
  {
    pStator_Currents->qI_Component2 = (int16_t)aux;
  }

  pH->_Super.hIa = pStator_Currents->qI_Component1;
  pH->_Super.hIb = pStator_Currents->qI_Component2;
  pH->_Super.hIc = -pStator_Currents->qI_Component1 - pStator_Currents->qI_Component2;

}

/**
  * @brief  Turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param  pHandle ICS LM1 PWM Current Feedback Handle
  * @retval none
  */
void ILM1_TurnOnLowSides(PWMC_Handle_t *pHandle)
{
  PWMC_ICS_LM1_Handle_t * pH = (PWMC_ICS_LM1_Handle_t *) pHandle;

  pH->_Super.hCntPhA =
      pH->_Super.hCntPhB =
          pH->_Super.hCntPhC = 0u;

  ILM1_WriteTIMRegisters(pHandle);

  LL_TIM_ClearFlag_UPDATE(TIM1);
  while (LL_TIM_IsActiveFlag_UPDATE(TIM1) == RESET)
  {}

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIM1);
  if ((pH->pParams_str->LowSideOutputs) == ES_GPIO)
  {
     LL_GPIO_SetOutputPin(pH->pParams_str->pwm_en_u_port, pH->pParams_str->pwm_en_u_pin);
     LL_GPIO_SetOutputPin(pH->pParams_str->pwm_en_v_port, pH->pParams_str->pwm_en_v_pin);
     LL_GPIO_SetOutputPin(pH->pParams_str->pwm_en_w_port, pH->pParams_str->pwm_en_w_pin);
  }
  return;
}


/**
* @brief  Enables PWM generation on the proper Timer peripheral acting on MOE
*         bit
* @param  pHandle ICS LM1 PWM Current Feedback Handle
* @retval none
*/
void ILM1_SwitchOnPWM(PWMC_Handle_t *pHandle)
{
  PWMC_ICS_LM1_Handle_t * pH = (PWMC_ICS_LM1_Handle_t *) pHandle;

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIM1);
  if ( (pH->pParams_str->LowSideOutputs) == ES_GPIO )
  {
	 LL_GPIO_SetOutputPin(pH->pParams_str->pwm_en_u_port, pH->pParams_str->pwm_en_u_pin);
	 LL_GPIO_SetOutputPin(pH->pParams_str->pwm_en_v_port, pH->pParams_str->pwm_en_v_pin);
	 LL_GPIO_SetOutputPin(pH->pParams_str->pwm_en_w_port, pH->pParams_str->pwm_en_w_pin);
  }
  LL_ADC_INJ_StartConversionExtTrig(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
  return;
}


/**
* @brief  Disables PWM generation on the proper Timer peripheral acting on MOE bit
* @param  pHandle ICS LM1 PWM Current Feedback Handle
* @retval none
*/
void ILM1_SwitchOffPWM(PWMC_Handle_t *pHandle)
{
  PWMC_ICS_LM1_Handle_t * pH = (PWMC_ICS_LM1_Handle_t *) pHandle;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(TIM1);
  if ((pH->pParams_str->LowSideOutputs) == ES_GPIO)
  {
    LL_GPIO_ResetOutputPin(pH->pParams_str->pwm_en_u_port, pH->pParams_str->pwm_en_u_pin);
    LL_GPIO_ResetOutputPin(pH->pParams_str->pwm_en_v_port, pH->pParams_str->pwm_en_v_pin);
    LL_GPIO_ResetOutputPin(pH->pParams_str->pwm_en_w_port, pH->pParams_str->pwm_en_w_pin);
  }
  ADC1->CR2 &= 0xFFFF7FFFU;

  return;
}

/**
* @brief  Stores into the component's instance handle the voltage present on Ia and
*         Ib current feedback analog channels when no current is flowing into the motor
* @param  pHandle ICS LM1 PWM Current Feedback Handle
* @retval none
*/
uint16_t ILM1_WriteTIMRegisters(PWMC_Handle_t *pHandle)
{
  uint16_t aux;

  TIM1->CCR1 = pHandle->hCntPhA;
  TIM1->CCR2 = pHandle->hCntPhB;
  TIM1->CCR3 = pHandle->hCntPhC;

  if (LL_TIM_IsActiveFlag_UPDATE(TIM1))
  {
    aux = MC_FOC_DURATION;
  }
  else
  {
    aux = MC_NO_ERROR;
  }

  return aux;
}

/**
 * @brief  It contains the Break event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
void *ILM1_BRK_IRQHandler(PWMC_ICS_LM1_Handle_t *pHandle)
{

  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
     LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_v_pin);
     LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_u_pin);
     LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  pHandle->OverCurrentFlag = true;

  return &(pHandle->_Super.bMotor);
}

/**
* @brief Used to check if an overcurrent occurred since last call.
* @param pHandle pointer on the target component instance handle
* @retval Returns MC_BREAK_IN whether an overcurrent has been
*                  detected since last method call, MC_NO_FAULTS otherwise.
*/
uint16_t ILM1_IsOverCurrentOccurred(PWMC_Handle_t *pHdl)
{
  PWMC_ICS_LM1_Handle_t * pHandle = (PWMC_ICS_LM1_Handle_t *) pHdl;
  uint16_t retVal = MC_NO_FAULTS;
  if (pHandle->OverCurrentFlag == true )
  {
    retVal = MC_BREAK_IN;
    pHandle->OverCurrentFlag = false;
  }
  return retVal;
}

/**
* @}
*/

/**
* @}
*/

/** @} */


/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
