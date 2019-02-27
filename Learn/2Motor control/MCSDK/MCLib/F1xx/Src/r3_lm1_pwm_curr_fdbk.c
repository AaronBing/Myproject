/**
  ******************************************************************************
  * @file    r3_lm1_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the r3_lm1_pwm_curr_fdbk component of the Motor Control SDK.
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
#include "r3_lm1_pwm_curr_fdbk.h"

#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @defgroup r3_lm1_pwm_curr_fdbk R3 LM1 PWM & Current Feedback
 *
 *
 * @brief STM32F1 Low & Medium Density, 3-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F103 Low & Medium Density MCU
 * and using a three shunt resistors current sensing topology.
 *
 * *STM32F1 Low & Medium Density MCUs* refers to STM32F103x4, STM32F103x6, STM32F103x8 and
 * STM32F103xB MCUs.
 *
 * @todo: TODO: complete documentation. and check applicable MCUs.
 * @{
 */

/* Private defines -----------------------------------------------------------*/

/* ADC1 Data Register address */
#define ADC1_CR2_Address    0x40012408u

/* @todo TODO: document */
#define NB_CONVERSIONS 16u

#define PHASE_A_MSK       (uint32_t)((uint32_t)(pHandle->pParams_str->bIaChannel) << 15)
#define PHASE_B_MSK       (uint32_t)((uint32_t)(pHandle->pParams_str->bIbChannel) << 15)
#define PHASE_C_MSK       (uint32_t)((uint32_t)(pHandle->pParams_str->bIcChannel) << 15)

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))

#define CR2_JEXTTRIG_Reset      ((uint32_t)0xFFFF7FFFu)

/* Private function prototypes -----------------------------------------------*/
static void R3LM1_TIMxInit( PWMC_R3_LM1_Handle_t * pHandle );
static uint16_t R3LM1_WriteTIMRegisters( PWMC_Handle_t * pHdl );

/* Global variables ---------------------------------------------------------*/
/* TODO: WARNING!!! A pointer on this var is used as DMA transfer base address..... IS THIS CORRECT? */
/* TODO: Shouldn't this be a static var? */
uint32_t ADC_CR2_Enable_Trig = 0x001E9801u;

/* Global functions ---------------------------------------------------------*/

/**
 * @brief  Initializes TIMx, ADC and DMA1 for three shunt current
 *         reading configuration using STM32 F130x Medium and Low Density.
 *
 * @param  pHandle Pointer on the handle of the component to initialize.
 *
 * @todo TODO: document or reference the documentation that states which fields of the handle
 *             are to be set prior to calling this function.
 */
void R3LM1_Init( PWMC_R3_LM1_Handle_t * pHandle )
{

  pHandle->_Super.bTurnOnLowSidesAction = false;

  RCC->AHBENR |= LL_AHB1_GRP1_PERIPH_CRC;

  R3LM1_TIMxInit( pHandle );

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
  
  /*Enable external trigger fo injected conv of ADC2 (mandatory for Dual mode)*/
  LL_ADC_INJ_StartConversionExtTrig(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

  /* DMA Event related to TIM1 Update event*/
  /* DMA1 channel5 configuration */
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t) (&ADC_CR2_Enable_Trig));
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t) ADC1_CR2_Address);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, 1u);

  /* Disable DMA1 Channel5 */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
  LL_TIM_EnableDMAReq_UPDATE(TIM1);

  pHandle->OverCurrentFlag = false;
  pHandle->_Super.DTTest = 0u;
  pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;

}

/**
 * @brief  Initializes TIMx peripheral for PWM generation
 *
 * @param  pHandle Handle of the component being initialized
 */
static void R3LM1_TIMxInit( PWMC_R3_LM1_Handle_t * pHandle )
{

  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH3);

  if ((pHandle->pParams_str->LowSideOutputs)== LS_PWM_TIMER)
  {
     LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1N|LL_TIM_CHANNEL_CH2N|LL_TIM_CHANNEL_CH3N);
  }
  
  /* Channel 4 */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  /* BKIN, if enabled */
  if ( (pHandle->pParams_str->EmergencyStop) != DISABLE )
  {
    LL_TIM_ClearFlag_BRK(TIMx);
    LL_TIM_EnableIT_BRK(TIMx);
  }
  
  /* Trigger Output signal is Update Event */
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_UPDATE);

  /* TIMer counter enable */
  LL_TIM_EnableCounter(TIMx);

  /* Resynch the timer to have the Update event during Underflow */
  LL_TIM_GenerateEvent_UPDATE(TIMx);
}

/**
 * @brief  Calibrates the ADC used for reading current
 *
 *  This function stores the voltage measured on Ia and Ib current
 * feedback analog channels when no current is flowing into the motor
 * in the handle of the component.
 *
 * @param  pHandle Handle of the component to calibrate
 */
void R3LM1_CurrentReadingCalibration( PWMC_Handle_t * pHdl )
{
  PWMC_R3_LM1_Handle_t * pHandle = (PWMC_R3_LM1_Handle_t *) pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  uint8_t bIndex;
  uint32_t wPhaseAOffset = 0u, wPhaseBOffset = 0u, wPhaseCOffset = 0u;

  /* ADC1 Injected conversions configuration */
  LL_ADC_INJ_SetSequencerLength( ADC1, LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS );
  LL_ADC_INJ_SetSequencerLength( ADC2, LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS );

  /* ADC1 Injected end of conversions interrupt disabling */
  LL_ADC_DisableIT_JEOS(ADC1);

  /* ADC1 Injected conversions trigger is given by software and enabled */
  LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_SOFTWARE);
  LL_ADC_INJ_StartConversionExtTrig(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
  
  /* ADC Channel used for current reading are read 
   in order to get zero currents ADC values*/
  for ( bIndex = NB_CONVERSIONS; bIndex != 0u; bIndex-- )
  {
    /* Clear the ADC1 JEOC pending flag */
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_INJ_StartConversionSWStart(ADC1);
    while ( !LL_ADC_IsActiveFlag_JEOS(ADC1) ) ;

    wPhaseAOffset += (uint16_t)ADC1->JDR1;
    wPhaseBOffset += (uint16_t)ADC1->JDR2;
    wPhaseCOffset += (uint16_t)ADC1->JDR3;
  }

  pHandle->wPhaseAOffset = (uint16_t) (wPhaseAOffset >> 3);
  pHandle->wPhaseBOffset = (uint16_t) (wPhaseBOffset >> 3);
  pHandle->wPhaseCOffset = (uint16_t) (wPhaseCOffset >> 3);

  LL_ADC_ClearFlag_JEOS(ADC1);

  /* Disable ADC Triggering */
  ADC1->CR2 &= 0xFFFF7FFFU;

  /* ADC1 Injected conversions trigger is TIM1 CC4 */
  LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_EXT_TIM1_CH4);

  LL_ADC_INJ_SetSequencerLength( ADC1, LL_ADC_INJ_SEQ_SCAN_DISABLE );
  LL_ADC_INJ_SetSequencerLength( ADC2, LL_ADC_INJ_SEQ_SCAN_DISABLE );

  /* ADC1 Injected conversions end interrupt enabling */
  LL_ADC_EnableIT_JEOS(ADC1);

  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to 
     force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */
  TIMx->CCMR1 &= 0xF7F7u;
  TIMx->CCMR2 &= 0xF7F7u;
  LL_TIM_OC_SetCompareCH1(TIMx,pHandle->Half_PWMPeriod);
  LL_TIM_OC_SetCompareCH2(TIMx,pHandle->Half_PWMPeriod);
  LL_TIM_OC_SetCompareCH3(TIMx,pHandle->Half_PWMPeriod);

  /* Enable TIMx preload */
  TIMx->CCMR1 |= 0x0808u;
  TIMx->CCMR2 |= 0x0808u;
}

/**
 * @brief Computes and returns the most recently converted motor phase currents
 *
 * @param pHandle Handle on the PWMC component in charge of the target motor
 * @param pStatorCurrents pointer on the variable where the result is stored
 */
void R3LM1_GetPhaseCurrents( PWMC_Handle_t * pHdl, Curr_Components* pStator_Currents )
{
  uint8_t bSector;
  int32_t wAux;
  PWMC_R3_LM1_Handle_t * pHandle = (PWMC_R3_LM1_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /* Disabling the Injectec conversion for ADCx after EOC*/
  /* Stdlib replaced: ADC_ExternalTrigInjectedConvCmd(ADC1,DISABLE);*/
  /* ADC1->CR2 &= CR2_JEXTTRIG_Reset replaced using bit banding */
  ADC1->CR2 = 0x001E1801u;

  /* Clear TIMx Update Flag necessary to detect FOC duration SW error */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  bSector = (uint8_t) pHdl->hSector;

  switch ( bSector )
  {
  case SECTOR_4:
  case SECTOR_5:
    /* Current on Phase C is not accessible     */
    /* Ia = PhaseAOffset - ADC converted value) */
    wAux = (int32_t)( ADC1->JDR1 );
    wAux *= 2;
    wAux = (int32_t)( pHandle->wPhaseAOffset ) - wAux;

    /* Saturation of Ia */
    if ( wAux < -INT16_MAX )
    {
      pStator_Currents->qI_Component1 = -INT16_MAX;
    }
    else if ( wAux > INT16_MAX )
    {
      pStator_Currents->qI_Component1 = INT16_MAX;
    }
    else
    {
      pStator_Currents->qI_Component1 = (int16_t) wAux;
    }
    
    /* Ib = PhaseBOffset - ADC converted value) */
    wAux = (int32_t)( ADC2->JDR1 );
    wAux *= 2;
    wAux = (int32_t)( pHandle->wPhaseBOffset ) - wAux;

    /* Saturation of Ib */
    if ( wAux < -INT16_MAX )
    {
      pStator_Currents->qI_Component2 = -INT16_MAX;
    }
    else if ( wAux > INT16_MAX )
    {
      pStator_Currents->qI_Component2 = INT16_MAX;
    }
    else
    {
      pStator_Currents->qI_Component2 = (int16_t) wAux;
    }
    break;
    
  case SECTOR_6:
  case SECTOR_1:
    /* Current on Phase A is not accessible     */
    /* Ib = PhaseBOffset - ADC converted value) */
    wAux = (int32_t)( ADC1->JDR1 );
    wAux *= 2;
    wAux = (int32_t)( pHandle->wPhaseBOffset ) - wAux;

    /* Saturation of Ib */
    if ( wAux < -INT16_MAX )
    {
      pStator_Currents->qI_Component2 = -INT16_MAX;
    }
    else if ( wAux > INT16_MAX )
    {
      pStator_Currents->qI_Component2 = INT16_MAX;
    }
    else
    {
      pStator_Currents->qI_Component2 = (int16_t) wAux;
    }

    /* Ia = -Ic -Ib */
    wAux = (int32_t)( ADC2->JDR1 );
    wAux *= 2;
    wAux -= (int32_t) pHandle->wPhaseCOffset;
    wAux -= (int32_t) pStator_Currents->qI_Component2;

    /* Saturation of Ia */
    if ( wAux > INT16_MAX )
    {
      pStator_Currents->qI_Component1 = INT16_MAX;
    }
    else if ( wAux < -INT16_MAX )
    {
      pStator_Currents->qI_Component1 = -INT16_MAX;
    }
    else
    {
      pStator_Currents->qI_Component1 = (int16_t) wAux;
    }
    break;
    
  case SECTOR_2:
  case SECTOR_3:
    /* Current on Phase B is not accessible     */
    /* Ia = PhaseAOffset - ADC converted value) */
    wAux = (int32_t)( ADC1->JDR1 );
    wAux *= 2;
    wAux = (int32_t)( pHandle->wPhaseAOffset ) - wAux;

    /* Saturation of Ia */
    if ( wAux < -INT16_MAX )
    {
      pStator_Currents->qI_Component1 = -INT16_MAX;
    }
    else if ( wAux > INT16_MAX )
    {
      pStator_Currents->qI_Component1 = INT16_MAX;
    }
    else
    {
      pStator_Currents->qI_Component1 = (int16_t) wAux;
    }

    /* Ib = -Ic -Ia */
    wAux = (int32_t)( ADC2->JDR1 );
    wAux *= 2;
    wAux -= (int32_t) pHandle->wPhaseCOffset;
    wAux -= (int32_t) pStator_Currents->qI_Component1;

    /* Saturation of Ib */
    if ( wAux > INT16_MAX )
    {
      pStator_Currents->qI_Component2 = INT16_MAX;
    }
    else if ( wAux < -INT16_MAX )
    {
      pStator_Currents->qI_Component2 = -INT16_MAX;
    }
    else
    {
      pStator_Currents->qI_Component2 = (int16_t) wAux;
    }                     
    break;

  default:
    break;
  }

  pHandle->_Super.hIa = pStator_Currents->qI_Component1;
  pHandle->_Super.hIb = pStator_Currents->qI_Component2;
  pHandle->_Super.hIc = -pStator_Currents->qI_Component1 - pStator_Currents->qI_Component2;

}

/**
 * @brief  Turns on Low Sides Switches of the power stage.
 *
 * This function is intended to be used for charging the boot capacitors of the driving
 * section. It has to be called each motor start-up when using high voltage drivers
 *
 * @param pHandle Handle on the PWMC component in charge of the target motor
 */
void R3LM1_TurnOnLowSides( PWMC_Handle_t * pHdl )
{
  PWMC_R3_LM1_Handle_t * pHandle = (PWMC_R3_LM1_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = true;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1(TIMx,0);
  LL_TIM_OC_SetCompareCH2(TIMx,0);
  LL_TIM_OC_SetCompareCH3(TIMx,0);

  /* Wait until next update */
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx)==RESET)
  {}

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);
  if ( (pHandle->pParams_str->LowSideOutputs) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
}

/**
* @brief  It enables PWM generation on the proper Timer peripheral acting on MOE
*         bit
* @param  this: related object of class CPWMC
* @retval none
*/
/**
 * @brief Starts PWM generation for the target motor.
 *
 * @todo TODO: Provide more details
 *
 * @param pHandle Handle on the PWMC component in charge of the target motor
 */
void R3LM1_SwitchOnPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R3_LM1_Handle_t * pHandle = (PWMC_R3_LM1_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);
  if ( (pHandle->pParams_str->LowSideOutputs) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_SetOutputPin (pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }

  /* Enable DMA1 Channel5 - ADC Triggering on Update */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
}


/**
 * @brief  Stops PWM generation for the target motor.
 *
 * @todo TODO: Provide more details
 *
 * @param pHandle Handle on the PWMC component in charge of the target motor
 * @retval none
 */
void R3LM1_SwitchOffPWM( PWMC_Handle_t * pHdl )
{ 
  PWMC_R3_LM1_Handle_t * pHandle = (PWMC_R3_LM1_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
 
  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(TIMx);
  if ( (pHandle->pParams_str->LowSideOutputs) == ES_GPIO )
  {
    LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }

  /* Disabling the Injected conversion for ADCx after EOC*/
  ADC1->CR2 = 0x001E1801u;

  /* Disable DMA1 Channel5 - ADC Triggering on Update */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
}

/**
 * @brief Writes PWM duty cycle values in Timer
 *
 *  The last computed duty cycle values for phases A, B & C are copied into
 * related Compare Capture registers of the PWM timer configured for the component
 * in @p pHandle.
 *
 *  The function then checks whether the copy was done on time. If this is the case
 * #MC_NO_ERROR is returned, otherwise #MC_FOC_DURATION is returned.
 *
 * @param  pHandle Handle of the component managing the timer to write
 * @retval none
 */
static uint16_t R3LM1_WriteTIMRegisters( PWMC_Handle_t * pHdl )
{
  uint16_t ret_val;
  PWMC_R3_LM1_Handle_t * pHandle = (PWMC_R3_LM1_Handle_t *) pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  
  TIMx->CCR1 = pHandle->_Super.hCntPhA;
  TIMx->CCR2 = pHandle->_Super.hCntPhB;
  TIMx->CCR3 = pHandle->_Super.hCntPhC;
  
  if (LL_TIM_IsActiveFlag_UPDATE(TIMx))
  {
    ret_val = MC_FOC_DURATION;
  }
  else
  {
    ret_val = MC_NO_ERROR;
  }
  return ret_val;
}

/**
* @brief  Sets the ADC sampling point for sector 1 and programs PWM duty cycle for the
*         next PWM period
*
*  This function computes the time in the next PWM period when the current sampling
* shall be executed. This time is then programmed in the Compare Capture register
* of the channel4 of the PWM timer used by this component (Channel 4 is used to
* trigger current sampling on the ADC). The function also configures the channels on
* which the ADCs will sample the current.
*
* R3HD2_SetADCSampPointSect1 is used when the vector is in sector 1. TODO: More precision.
*
* Finally, PWM duty cycles are programmed in the Timer's Compare Capture registers
* by calling the the WriteTIMRegisters function.
*
* @param  pHandle Handle of the component to deal with
* @retval #MC_NO_ERROR if PWM duty cycle programming was done on time;
* @retval #MC_FOC_DURATION otherwise.
*/
uint16_t R3LM1_SetADCSampPointSect1( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_LM1_Handle_t * pHandle = (PWMC_R3_LM1_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /* Set Polarity of CC4 active high (default) */
  TIMx->CCER &= 0xDFFFu;    
  
  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 1 (i.e phase A duty cycle) */
  if ( (uint16_t) (pHandle->Half_PWMPeriod - pHdl->hCntPhA) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    ADC1->JSQR = PHASE_A_MSK;
    ADC2->JSQR = PHASE_B_MSK;

  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = (uint16_t) (pHdl->hCntPhA - pHdl->hCntPhB);

    /* Definition of crossing point */
    if ( hDeltaDuty > (uint16_t) (pHandle->Half_PWMPeriod - pHdl->hCntPhA) * 2u )
    {
      hCntSmp = pHdl->hCntPhA - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHdl->hCntPhA + pHandle->pParams_str->hTafter;
      
      if (hCntSmp >= pHandle->Half_PWMPeriod)
      {        
        /* Set Polarity of CC4 active low */
        TIMx->CCER |= 0x2000u;
        
        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
    }
    ADC1->JSQR = PHASE_B_MSK;
    ADC2->JSQR = PHASE_C_MSK;
  }

  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;

  return R3LM1_WriteTIMRegisters( pHdl );
}

/**
* @brief  Sets the ADC sampling point for sector 2 and programs PWM duty cycle for the
*         next PWM period
*
*  This function computes the time in the next PWM period when the current sampling
* shall be executed. This time is then programmed in the Compare Capture register
* of the channel4 of the PWM timer used by this component (Channel 4 is used to
* trigger current sampling on the ADC). The function also configures the channels on
* which the ADCs will sample the current.
*
* R3HD2_SetADCSampPointSect2 is used when the vector is in sector 2. TODO: More precision.
*
* Finally, PWM duty cycles are programmed in the Timer's Compare Capture registers
* by calling the the WriteTIMRegisters function.
*
* @param  pHandle Handle of the component to deal with
* @retval #MC_NO_ERROR if PWM duty cycle programming was done on time;
* @retval #MC_FOC_DURATION otherwise.
*/
uint16_t R3LM1_SetADCSampPointSect2( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_LM1_Handle_t * pHandle = (PWMC_R3_LM1_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  
  /* Set Polarity of CC4 active high (default) */
  TIMx->CCER &= 0xDFFFu;
  
  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 2 (i.e phase B duty cycle) */
  if ( (uint16_t) (pHandle->Half_PWMPeriod - pHdl->hCntPhB) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHdl->hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    ADC1->JSQR = PHASE_A_MSK;
    ADC2->JSQR = PHASE_B_MSK;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = (uint16_t) (pHdl->hCntPhB - pHdl->hCntPhA);

    /* Definition of crossing point */
    if ( hDeltaDuty > (uint16_t) (pHandle->Half_PWMPeriod - pHdl->hCntPhB) * 2u )
    {
      hCntSmp = pHdl->hCntPhB - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHdl->hCntPhB + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        /* Set Polarity of CC4 active low */
        TIMx->CCER |= 0x2000u;
        
        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
    }
    ADC1->JSQR = PHASE_A_MSK;
    ADC2->JSQR = PHASE_C_MSK;
  }

  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;
  
  return R3LM1_WriteTIMRegisters( pHdl );
}

/**
* @brief  Sets the ADC sampling point for sector 3 and programs PWM duty cycle for the
*         next PWM period
*
*  This function computes the time in the next PWM period when the current sampling
* shall be executed. This time is then programmed in the Compare Capture register
* of the channel4 of the PWM timer used by this component (Channel 4 is used to
* trigger current sampling on the ADC). The function also configures the channels on
* which the ADCs will sample the current.
*
* R3HD2_SetADCSampPointSect3 is used when the vector is in sector 3. TODO: More precision.
*
* Finally, PWM duty cycles are programmed in the Timer's Compare Capture registers
* by calling the the WriteTIMRegisters function.
*
* @param  pHandle Handle of the component to deal with
* @retval #MC_NO_ERROR if PWM duty cycle programming was done on time;
* @retval #MC_FOC_DURATION otherwise.
*/
uint16_t R3LM1_SetADCSampPointSect3( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_LM1_Handle_t * pHandle = (PWMC_R3_LM1_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  
  /* Set Polarity of CC4 active high (default) */
  TIMx->CCER &= 0xDFFFu;    
  
  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 3 (i.e phase B duty cycle) */
  if ( (uint16_t) (pHandle->Half_PWMPeriod - pHdl->hCntPhB) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    ADC1->JSQR = PHASE_A_MSK;
    ADC2->JSQR = PHASE_B_MSK;
  }
  else
  {
    hDeltaDuty = (uint16_t) (pHdl->hCntPhB - pHdl->hCntPhC);
    
    /* Definition of crossing point */
    if ( hDeltaDuty > (uint16_t) (pHandle->Half_PWMPeriod - pHdl->hCntPhB) * 2u )
    {
      hCntSmp = pHdl->hCntPhB - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHdl->hCntPhB + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        /* Set Polarity of CC4 active low */
        TIMx->CCER |= 0x2000u;
        
        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
    }
    ADC1->JSQR = PHASE_A_MSK;
    ADC2->JSQR = PHASE_C_MSK;
  }

  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;
  
  return R3LM1_WriteTIMRegisters( pHdl );
}

/**
* @brief  Sets the ADC sampling point for sector 4 and programs PWM duty cycle for the
*         next PWM period
*
*  This function computes the time in the next PWM period when the current sampling
* shall be executed. This time is then programmed in the Compare Capture register
* of the channel4 of the PWM timer used by this component (Channel 4 is used to
* trigger current sampling on the ADC). The function also configures the channels on
* which the ADCs will sample the current.
*
* R3HD2_SetADCSampPointSect4 is used when the vector is in sector 4. TODO: More precision.
*
* Finally, PWM duty cycles are programmed in the Timer's Compare Capture registers
* by calling the the WriteTIMRegisters function.
*
* @param  pHandle Handle of the component to deal with
* @retval #MC_NO_ERROR if PWM duty cycle programming was done on time;
* @retval #MC_FOC_DURATION otherwise.
*/
uint16_t R3LM1_SetADCSampPointSect4( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_LM1_Handle_t * pHandle = (PWMC_R3_LM1_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /* Set Polarity of CC4 active high (default) */
  TIM1->CCER &= 0xDFFFu;    
  
  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 4 (i.e phase C duty cycle) */
  if ( (uint16_t) (pHandle->Half_PWMPeriod - pHdl->hCntPhC) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    ADC1->JSQR = PHASE_A_MSK;
    ADC2->JSQR = PHASE_B_MSK;
  }
  else
  {
    hDeltaDuty = (uint16_t) (pHdl->hCntPhC - pHdl->hCntPhB);

    /* Definition of crossing point */
    if ( hDeltaDuty > (uint16_t) (pHandle->Half_PWMPeriod - pHdl->hCntPhC) * 2u )
    {
      hCntSmp = pHdl->hCntPhC - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHdl->hCntPhC + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        /* Set Polarity of CC4 active low */
        TIMx->CCER |= 0x2000u;
        
        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
    }
    ADC1->JSQR = PHASE_A_MSK;
    ADC2->JSQR = PHASE_B_MSK;
  }
  
  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;
  
  return R3LM1_WriteTIMRegisters( pHdl );
}

/**
* @brief  Sets the ADC sampling point for sector 5 and programs PWM duty cycle for the
*         next PWM period
*
*  This function computes the time in the next PWM period when the current sampling
* shall be executed. This time is then programmed in the Compare Capture register
* of the channel4 of the PWM timer used by this component (Channel 4 is used to
* trigger current sampling on the ADC). The function also configures the channels on
* which the ADCs will sample the current.
*
* R3HD2_SetADCSampPointSect5 is used when the vector is in sector 5. TODO: More precision.
*
* Finally, PWM duty cycles are programmed in the Timer's Compare Capture registers
* by calling the the WriteTIMRegisters function.
*
* @param  pHandle Handle of the component to deal with
* @retval #MC_NO_ERROR if PWM duty cycle programming was done on time;
* @retval #MC_FOC_DURATION otherwise.
*/
uint16_t R3LM1_SetADCSampPointSect5( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_LM1_Handle_t * pHandle = (PWMC_R3_LM1_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  
  /* Set Polarity of CC4 active high (default) */
  TIM1->CCER &= 0xDFFFu;    
  
  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 5 (i.e phase C duty cycle) */
  if ( (uint16_t) (pHandle->Half_PWMPeriod - pHdl->hCntPhC) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHdl->hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    ADC1->JSQR = PHASE_A_MSK;
    ADC2->JSQR = PHASE_B_MSK;
  }
  else
  {
    hDeltaDuty = (uint16_t) (pHdl->hCntPhC - pHdl->hCntPhA);

    /* Definition of crossing point */
    if ( hDeltaDuty > (uint16_t) (pHandle->Half_PWMPeriod - pHdl->hCntPhC) * 2u )
    {
      hCntSmp = pHdl->hCntPhC - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHdl->hCntPhC + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        /* Set Polarity of CC4 active low */
        TIMx->CCER |= 0x2000u;
        
        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
    }
    ADC1->JSQR = PHASE_A_MSK;
    ADC2->JSQR = PHASE_B_MSK;
  }
  
  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;
  
  return R3LM1_WriteTIMRegisters( pHdl );
}

/**
* @brief  Sets the ADC sampling point for sector 6 and programs PWM duty cycle for the
*         next PWM period
*
*  This function computes the time in the next PWM period when the current sampling
* shall be executed. This time is then programmed in the Compare Capture register
* of the channel4 of the PWM timer used by this component (Channel 4 is used to
* trigger current sampling on the ADC). The function also configures the channels on
* which the ADCs will sample the current.
*
* R3HD2_SetADCSampPointSect6 is used when the vector is in sector 6. TODO: More precision.
*
* Finally, PWM duty cycles are programmed in the Timer's Compare Capture registers
* by calling the the WriteTIMRegisters function.
*
* @param  pHandle Handle of the component to deal with
* @retval #MC_NO_ERROR if PWM duty cycle programming was done on time;
* @retval #MC_FOC_DURATION otherwise.
*/
uint16_t R3LM1_SetADCSampPointSect6( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_LM1_Handle_t * pHandle = (PWMC_R3_LM1_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  
  /* Set Polarity of CC4 active high (default) */
  TIM1->CCER &= 0xDFFFu;    
  
  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 6 (i.e phase A duty cycle) */
  if ( (uint16_t) (pHandle->Half_PWMPeriod - pHdl->hCntPhA) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    ADC1->JSQR = PHASE_A_MSK;
    ADC2->JSQR = PHASE_B_MSK;
  }
  else
  {
    hDeltaDuty = (uint16_t) (pHdl->hCntPhA - pHdl->hCntPhC);

    /* Definition of crossing point */
    if ( hDeltaDuty > (uint16_t) (pHandle->Half_PWMPeriod - pHdl->hCntPhA) * 2u )
    {
      hCntSmp = pHdl->hCntPhA - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHdl->hCntPhA + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        /* Set Polarity of CC4 active low */
        TIMx->CCER |= 0x2000u;
        
        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
    }
    ADC1->JSQR = PHASE_B_MSK;
    ADC2->JSQR = PHASE_C_MSK;
  }

  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;
  
  return R3LM1_WriteTIMRegisters( pHdl );
}

/**
 * @brief  It contains the Break event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
void *R3LM1_BRK_IRQHandler(PWMC_R3_LM1_Handle_t *pHandle)
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
 *
 * @param pHandle Handle on the PWMC component in charge of the target motor
 *
 * @retval # MC_BREAK_IN if an over current condition has been detected since the
 *         function was last called; # MC_NO_FAULTS otherwise.
 */
uint16_t R3LM1_IsOverCurrentOccurred( PWMC_Handle_t * pHdl )
{
  PWMC_R3_LM1_Handle_t * pHandle = (PWMC_R3_LM1_Handle_t *) pHdl;
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
