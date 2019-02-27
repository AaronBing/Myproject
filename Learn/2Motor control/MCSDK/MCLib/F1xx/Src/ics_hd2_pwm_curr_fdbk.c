/**
  ******************************************************************************
  * @file    ics_hd2_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the ICS
  *          HD2 PWM Current Feedback component of the Motor Control SDK.
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
#include "ics_hd2_pwm_curr_fdbk.h"
#include "mc_type.h"

/** @addtogroup MCSDK
* @{
*/

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup ics_hd2_pwm_curr_fdbk ICS HD2 PWM & Current Feedback
 *
 * @brief STM32F1 High Density, ICS PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F103 High Density MCU
 * and using an Insulated Current Sensors topology.
 *
 * *STM32F103 High Density* refers to STM32F103xC, STM32F103xD and STM32F103xE MCUs.
 *
 * @todo: TODO: complete documentation.
 *
 * @{
 */

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))

#define TIMxCCER_MASK              ((uint16_t)  ~0x1555u)
#define TIMxCCER_MASK_CH123         (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2| LL_TIM_CHANNEL_CH2N |\
                                    LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N)

#define CONV_STARTED               ((uint32_t) (0x8))
#define CONV_FINISHED              ((uint32_t) (0xC))
#define FLAGS_CLEARED              ((uint32_t) (0x0))
#define ADC_SR_MASK                ((uint32_t) (0xC))


#define NB_CONVERSIONS 16u

#define PHASE_A_MSK       (uint32_t)((uint32_t)(pHandle->pParams_str->bIaChannel) << 15)
#define PHASE_B_MSK       (uint32_t)((uint32_t)(pHandle->pParams_str->bIbChannel) << 15)

/**
* @brief  It initializes TIMx, ADC and DMA1 for current reading
*         in ICS configuration using STM32F103x High Density
* @param pHandle ICS HD2 PWM Current Feedback Handle
* @retval none
*/
void IHD2_Init(PWMC_ICS_HD2_Handle_t *pHandle)
{
  pHandle->_Super.bTurnOnLowSidesAction = false;

  RCC->AHBENR |= LL_AHB1_GRP1_PERIPH_CRC;

  IHD2_TIMxInit(pHandle->pParams_str->TIMx, pHandle);

  if(pHandle->pParams_str->TIMx == TIM1)
  {
    /* TIM1 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);
  }
  else
  {
    /* TIM8 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM8_STOP);
  }

  /* Enable ADC1 and ADC2 */
  LL_ADC_Enable(ADC1);
  LL_ADC_Enable(ADC2);

  LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_DISABLE);
  LL_ADC_SetSequencersScanMode(ADC2, LL_ADC_SEQ_SCAN_DISABLE);

  LL_ADC_INJ_SetSequencerDiscont(ADC2, LL_ADC_INJ_SEQ_DISCONT_1RANK);

  LL_ADC_INJ_StartConversionExtTrig(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
  LL_ADC_INJ_StartConversionExtTrig(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);
    
  /* Start calibration of ADC1 */
  LL_ADC_StartCalibration(ADC1);
  LL_ADC_StartCalibration(ADC2);

  /* Wait for the end of ADCs calibration */
  while ( LL_ADC_IsCalibrationOnGoing(ADC1) || LL_ADC_IsCalibrationOnGoing(ADC2) )
  {
  }
  
  /* ADC1 Injected conversions end interrupt enabling */
  LL_ADC_ClearFlag_JEOS(ADC1);
  LL_ADC_EnableIT_JEOS(ADC1);
  pHandle->_Super.DTTest = 0u;
  pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;

}

/**
* @brief  It initializes TIMx peripheral for PWM generation
* @param TIMx Timer to be initialized
* @param pHandle ICS HD2 PWM Current Feedback Handle
* @retval none
*/
void IHD2_TIMxInit(TIM_TypeDef* TIMx, PWMC_ICS_HD2_Handle_t *pHandle)
{
  /* disable main TIM counter to ensure
   * a synchronous start by TIM2 trigger */
  LL_TIM_DisableCounter(TIMx);

  /* Enables the TIMx Preload on CC1 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  /* Enables the TIMx Preload on CC2 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  /* Enables the TIMx Preload on CC3 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  /* Enables the TIMx Preload on CC4 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);

  /* BKIN, if enabled */
  if ( (pHandle->pParams_str->EmergencyStop) != DISABLE )
  {
    LL_TIM_ClearFlag_BRK(TIMx);
    LL_TIM_EnableIT_BRK(TIMx);
  }

  /* Prepare timer for synchronization */
  LL_TIM_GenerateEvent_UPDATE(TIMx);

  if ( pHandle->pParams_str->bFreqRatio == 2u )
  {
    if ( pHandle->pParams_str->bIsHigherFreqTim == HIGHER_FREQ )
    {
      if ( pHandle->pParams_str->bRepetitionCounter == 3u )
      {
        /* Set TIMx repetition counter to 1 */
        LL_TIM_SetRepetitionCounter(TIMx,1u);
        LL_TIM_GenerateEvent_UPDATE(TIMx);
        /* Repetition counter will be set to 3 at next Update */
        LL_TIM_SetRepetitionCounter(TIMx, 3);
      }
    }

    LL_TIM_SetCounter(TIMx, (uint32_t)(pHandle->Half_PWMPeriod)-1u);
  }
  else /* bFreqRatio equal to 1 or 3 */
  {
    if ( pHandle->pParams_str->bInstanceNbr == 1u )
    {
      LL_TIM_SetCounter(TIMx, (uint32_t)(pHandle->Half_PWMPeriod)-1u);
    }
  }
}


/**
* @brief  Stores into the component's handle the voltage present on Ia and
*         Ib current feedback analog channels when no current is flowing into the
*         motor
* @param  pHandle ICS HD2 PWM Current Feedback Handle
*/
void IHD2_CurrentReadingCalibration( PWMC_Handle_t * pHandle )
{
  PWMC_ICS_HD2_Handle_t * pH = (PWMC_ICS_HD2_Handle_t *) pHandle;
  TIM_TypeDef*  TIMx = pH->pParams_str->TIMx;

  pH->wPhaseAOffset = 0u;
  pH->wPhaseBOffset = 0u;

  pH->bIndex = 0u;

  /* Force inactive level on TIMx CHy and TIMx CHyN */
  LL_TIM_CC_DisableChannel(TIMx, TIMxCCER_MASK_CH123);
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  /* Change function to be executed in ADCx_ISR */
  pH->_Super.pFctGetPhaseCurrents = &IHD2_HFCurrentsCalibration;

  IHD2_SwitchOnPWM(pHandle);

  /* Wait for NB_CONVERSIONS to be executed */
  while (pH->bIndex < (NB_CONVERSIONS))
  {
    if (TIMx->DIER & TIM_DIER_UIE)
    {}
    else
    {
      pH->bIndex = NB_CONVERSIONS;
    }
  }

  IHD2_SwitchOffPWM(pHandle);

  pH->wPhaseAOffset >>= 3;
  pH->wPhaseBOffset >>= 3;

  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
   force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */
  TIMx->CCMR1 &= 0xF7F7u;
  TIMx->CCMR2 &= 0xF7F7u;
  TIMx->CCR1 = pH->Half_PWMPeriod;
  TIMx->CCR2 = pH->Half_PWMPeriod;
  TIMx->CCR3 = pH->Half_PWMPeriod;

  /* Enable TIMx preload */
  TIMx->CCMR1 |= 0x0808u;
  TIMx->CCMR2 |= 0x0808u;

  /* Set back TIMx CCER register */
  TIMx->CCER |= TIMxCCER_MASK_CH123;

  /* Change back function to be executed in ADCx_ISR */
  pH->_Super.pFctGetPhaseCurrents = &IHD2_GetPhaseCurrents;

}

/**
* @brief  Computes and return latest converted motor phase currents motor
* @param pHandle ICS HD2 PWM Current Feedback Handle
* @retval Ia and Ib current in Curr_Components format
*/
void IHD2_GetPhaseCurrents(PWMC_Handle_t *pHandle, Curr_Components* pStator_Currents)
{
  int32_t aux;
  uint16_t reg;
  PWMC_ICS_HD2_Handle_t * pH = (PWMC_ICS_HD2_Handle_t *) pHandle;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pH->hFlags &= (~SOFOC);

  /* Ia = (hPhaseAOffset)-(PHASE_A_ADC_CHANNEL vale)  */
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

  /* Ib = (hPhaseBOffset)-(PHASE_B_ADC_CHANNEL value) */
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
* @brief  Sums up injected conversion data into wPhaseXOffset. It is called
*         only during current calibration
* @param pHandle ICS HD2 PWM Current Feedback Handle
* @retval Always returns {0,0} in Curr_Components format
*/
void IHD2_HFCurrentsCalibration(PWMC_Handle_t *pHandle, Curr_Components* pStator_Currents)
{

  PWMC_ICS_HD2_Handle_t * pH = (PWMC_ICS_HD2_Handle_t *) pHandle;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pH->hFlags &= (~SOFOC);

  if (pH->bIndex < NB_CONVERSIONS)
  {
	  pH->wPhaseAOffset += ADC1->JDR1;
	  pH->wPhaseBOffset += ADC2->JDR1;
	  pH->bIndex++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->qI_Component1 = 0;
  pStator_Currents->qI_Component2 = 0;
}

/**
  * @brief  Turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param pHandle ICS HD2 PWM Current Feedback Handle
  * @retval none
  */
void IHD2_TurnOnLowSides( PWMC_Handle_t *pHandle )
{
  PWMC_ICS_HD2_Handle_t * pH = (PWMC_ICS_HD2_Handle_t *) pHandle;
  TIM_TypeDef*  TIMx = pH->pParams_str->TIMx;

  pHandle->bTurnOnLowSidesAction = true;

  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1(TIMx,0);
  LL_TIM_OC_SetCompareCH2(TIMx,0);
  LL_TIM_OC_SetCompareCH3(TIMx,0);

  /*Disable ADC trigger */
  TIMx->CCMR2 = 0x7068u;
  TIMx->CCR4 = (uint32_t)(pH->Half_PWMPeriod) + 1u;

  pH->hFlags &= (~SOFOC);

  LL_TIM_ClearFlag_UPDATE(TIMx);
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx)==RESET)
  {}

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);
  if ((pH->pParams_str->LowSideOutputs) == ES_GPIO)
  {
     LL_GPIO_SetOutputPin(pH->pParams_str->pwm_en_u_port, pH->pParams_str->pwm_en_u_pin);
     LL_GPIO_SetOutputPin(pH->pParams_str->pwm_en_v_port, pH->pParams_str->pwm_en_v_pin);
     LL_GPIO_SetOutputPin(pH->pParams_str->pwm_en_w_port, pH->pParams_str->pwm_en_w_pin);
  }
  return;
}


/**
* @brief  Enables PWM generation on the proper Timer peripheral acting on MOE bit
* @param pHandle ICS HD2 PWM Current Feedback Handle
* @retval none
*/
void IHD2_SwitchOnPWM(PWMC_Handle_t *pHandle)
{
  PWMC_ICS_HD2_Handle_t * pH = (PWMC_ICS_HD2_Handle_t *) pHandle;
  TIM_TypeDef*  TIMx = pH->pParams_str->TIMx;

  pHandle->bTurnOnLowSidesAction = false;

  /* It clears ADCs JSTRT and JEOC bits */
  ADC1->SR &= ~ADC_SR_MASK;
  ADC2->SR &= ~ADC_SR_MASK;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Enable TIMx preload and ADC trigger on next update */
  TIMx->CCMR2 = 0x7868u;
  TIMx->CCR4 = (uint32_t)(pH->Half_PWMPeriod) - 5u;

  LL_TIM_EnableIT_UPDATE(TIMx);
  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);
  if ( (pH->pParams_str->LowSideOutputs) == ES_GPIO )
  {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u)
    {
	 LL_GPIO_SetOutputPin(pH->pParams_str->pwm_en_u_port, pH->pParams_str->pwm_en_u_pin);
	 LL_GPIO_SetOutputPin(pH->pParams_str->pwm_en_v_port, pH->pParams_str->pwm_en_v_pin);
	 LL_GPIO_SetOutputPin(pH->pParams_str->pwm_en_w_port, pH->pParams_str->pwm_en_w_pin);
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin(pH->pParams_str->pwm_en_u_port, pH->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pH->pParams_str->pwm_en_v_port, pH->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pH->pParams_str->pwm_en_w_port, pH->pParams_str->pwm_en_w_pin);
    }
  }
  pH->hFlags &= (~SOFOC);
  return;
}


/**
* @brief  Disables PWM generation on the proper Timer peripheral acting on
*         MOE bit
* @param pHandle ICS HD2 PWM Current Feedback Handle
* @retval none
*/
void IHD2_SwitchOffPWM(PWMC_Handle_t *pHandle)
{
  PWMC_ICS_HD2_Handle_t * pH = (PWMC_ICS_HD2_Handle_t *) pHandle;
  TIM_TypeDef*  TIMx = pH->pParams_str->TIMx;
  pHandle->bTurnOnLowSidesAction = false;

  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE(TIMx);

  TIMx->CCER &= (uint16_t)(~TIMxCCER_MASK_CH123);

  while (LL_TIM_IsActiveFlag_UPDATE(TIMx)==RESET)
  {
    if (LL_TIM_IsEnabledIT_UPDATE(TIMx))
    {
      break;
    }
  }

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(TIMx);
  if ((pH->pParams_str->LowSideOutputs) == ES_GPIO)
  {
    LL_GPIO_ResetOutputPin(pH->pParams_str->pwm_en_u_port, pH->pParams_str->pwm_en_u_pin);
    LL_GPIO_ResetOutputPin(pH->pParams_str->pwm_en_v_port, pH->pParams_str->pwm_en_v_pin);
    LL_GPIO_ResetOutputPin(pH->pParams_str->pwm_en_w_port, pH->pParams_str->pwm_en_w_pin);
  }

  TIMx->CCER |= TIMxCCER_MASK_CH123;
  
  TIMx->CCMR2 = 0x7068u;
  TIMx->CCR4 = (uint32_t)(pH->Half_PWMPeriod) + 1u;

  return;
}

/**
* @brief  Stores into component's instance handle the voltage present on Ia and
*         Ib current feedback analog channels when no current is flowing into the
*         motor
* @param pHandle ICS HD2 PWM Current Feedback Handle
* @retval none
*/
uint16_t IHD2_WriteTIMRegisters(PWMC_Handle_t *pHandle)
{
  uint16_t aux;
  PWMC_ICS_HD2_Handle_t * pH = (PWMC_ICS_HD2_Handle_t *) pHandle;
  TIM_TypeDef*  TIMx = pH->pParams_str->TIMx;

  TIMx->CCR1 = pHandle->hCntPhA;
  TIMx->CCR2 = pHandle->hCntPhB;
  TIMx->CCR3 = pHandle->hCntPhC;

  /* Disable TIMx preload */
  TIMx->CCMR2 = 0x7068u;
  TIMx->CCR4 = (uint32_t)(pH->Half_PWMPeriod) + 1u;
  /* Enable TIMx preload */
  TIMx->CCMR2 = 0x7868u;
  TIMx->CCR4 = (uint32_t)(pH->Half_PWMPeriod) - 5u;

  /* Limit for update event */
  /* Check the status of SOFOC flag. If it is set, an update event has occurred
  and thus the FOC rate is too high */
  if ((pH->hFlags & SOFOC) != 0u)
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
* @brief Contains the TIMx Update event interrupt
* @param pHandle ICS HD2 PWM Current Feedback Handle
* @retval none
*/
void *IHD2_IRQHandler(PWMC_ICS_HD2_Handle_t *pHandle)
{
  uint32_t adcinjflags;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  adcinjflags = (ADC1-> SR) & ADC_SR_MASK;

  if (adcinjflags == CONV_STARTED)
  {
    do
    {
      adcinjflags = (ADC1-> SR) & ADC_SR_MASK;
    }
    while (adcinjflags != CONV_FINISHED);
  }
  else if (adcinjflags == FLAGS_CLEARED)
  {
    while ((TIMx->CNT) < (pHandle->pParams_str->Tw))
    {}
    adcinjflags = (ADC1-> SR) & ADC_SR_MASK;

    if (adcinjflags == CONV_STARTED)
    {
      do
      {
        adcinjflags = (ADC1-> SR) & ADC_SR_MASK;
      }
      while (adcinjflags != CONV_FINISHED);
    }
  }
  else {}

  /* Set the SOFOC flag to indicate the execution of Update IRQ*/
  pHandle->hFlags |= SOFOC;

  /* Switch Context */
  /* It re-initilize AD converter in run time when using dual MC */
  if(TIMx == TIM8)
  {
    /* ADCx Injected conversions trigger is swithced to TIM8 CH4
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_Ext_IT15_TIM8_CC4);*/

    ADC1->CR2 = 0x1EE801u;
    ADC2->CR2 = 0x1EE801u;
  }
  else
  {
    /* ADCx Injected conversions trigger is swithced to TIM8 CH4
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_CC4);*/
    ADC1->CR2 = 0x1E9801u;
    ADC2->CR2 = 0x1E9801u;
  }
  /* Change channels keeping equal to 1 element the sequencer lenght */
  ADC1->JSQR = (uint32_t)(pHandle->pParams_str->bIaChannel) << 15;
  ADC2->JSQR = (uint32_t)(pHandle->pParams_str->bIbChannel) << 15;

  return &(pHandle->_Super.bMotor);
}

/**
 * @brief  It contains the Break event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
void *IHD2_BRK_IRQHandler(PWMC_ICS_HD2_Handle_t *pHandle)
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
* @brief  Checks if an over current condition occurred since last call.
* @param pHandle ICS HD2 PWM Current Feedback Handle
* @retval Returns MC_BREAK_IN whether an overcurrent has been
*                  detected since last method call, MC_NO_FAULTS otherwise.
*/
uint16_t IHD2_IsOverCurrentOccurred(PWMC_Handle_t *pHdl)
{
  PWMC_ICS_HD2_Handle_t * pHandle = (PWMC_ICS_HD2_Handle_t *) pHdl;

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

/**
* @}
*/

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
