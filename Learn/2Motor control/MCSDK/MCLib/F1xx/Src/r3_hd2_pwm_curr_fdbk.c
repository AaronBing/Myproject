/**
  ******************************************************************************
  * @file    r3_hd2_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the r3_hd2_pwm_curr_fdbk component of the Motor Control SDK.
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
#include "r3_hd2_pwm_curr_fdbk.h"

#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup r3_hd2_pwm_curr_fdbk R3 HD2 PWM & Current Feedback
 *
 * @brief STM32F1 High Density, 3-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F103 High Density MCU
 * and using a three shunt resistors current sensing topology.
 *
 * *STM32F103 High Density* refers to STM32F103xC, STM32F103xD and STM32F103xE MCUs.
 *
 * @todo: TODO: complete documentation.
 * @{
 */

/* Private defines -----------------------------------------------------------*/

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))

#define TIMxCCER_MASK              ((uint16_t)  ~0x1555u)
#define TIMxCCER_MASK_CH123        ((uint16_t)  0x555u)

/** @{ */
/* ADC Conversion statuses and masks */
/** @brief ADV Conversion is ongoing */
#define CONV_STARTED               ((uint32_t) (0x8))
/** @brief ADC Conversion has finished */
#define CONV_FINISHED              ((uint32_t) (0xC))
/** @brief No ADC Conversion is started, none has completed */
#define FLAGS_CLEARED              ((uint32_t) (0x0))
/** @brief ADC SR register mask to grab Conversion status */
#define ADC_SR_MASK                ((uint32_t) (0xC))
/** @} */

#define NB_CONVERSIONS 16u

#define PHASE_A_MSK       (uint32_t)((uint32_t)(pHandle->pParams_str->bIaChannel) << 15)
#define PHASE_B_MSK       (uint32_t)((uint32_t)(pHandle->pParams_str->bIbChannel) << 15)
#define PHASE_C_MSK       (uint32_t)((uint32_t)(pHandle->pParams_str->bIcChannel) << 15)

#define CCMR2_CH4_DISABLE 0x8FFFu
#define CCMR2_CH4_PWM1    0x6000u
#define CCMR2_CH4_PWM2    0x7000u

/* Private function prototypes -----------------------------------------------*/
static void R3HD2_TIMxInit( TIM_TypeDef* TIMx, PWMC_R3_HD2_Handle_t * pHandle );
static uint16_t R3HD2_WriteTIMRegisters( PWMC_Handle_t * pHdl );
static void R3HD2_HFCurrentsCalibrationAB( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents );
static void R3HD2_HFCurrentsCalibrationC( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents );

/* Global functions ---------------------------------------------------------*/

/**
 * @brief  Initializes TIMx, ADC and DMA1 for current reading
 *         in 3 Shunt configuration using STM32F103x High Density.
 * @param  pHandle Pointer on the handle of the component to initialize.
  * @retval none
 */
void R3HD2_Init( PWMC_R3_HD2_Handle_t * pHandle )
{
  pHandle->_Super.bTurnOnLowSidesAction = false;

  RCC->AHBENR |= LL_AHB1_GRP1_PERIPH_CRC;

  if ( pHandle->pParams_str->TIMx == TIM1 )
  {
    /* Stores the value for the context switching */
    /* ADCx Injected conversions trigger is switched to TIM1 CH4 */
    /* Stdlib replaced: ADC_ExternalTrigInjectedConvConfig(ADCx, ADC_ExternalTrigInjecConv_T1_CC4);*/
    pHandle->wADCTriggerSet = 0x1E9801u;
    pHandle->wADCTriggerUnSet = 0x1E1801u;

    /* Store the bit-banding address to activate/deactivate TIMx CH4 channel */
    pHandle->wTIMxCH4_BB_Addr = 0x42258430u;
  }
  else
  {
    /* Stores the value for the context switching */
    /* ADCx Injected conversions trigger is switched to TIM8 CH4 */
    /* Stdlib replaced: ADC_ExternalTrigInjectedConvConfig(ADCx, ADC_ExternalTrigInjecConv_Ext_IT15_TIM8_CC4);*/
    pHandle->wADCTriggerSet = 0x1EE801u;
    pHandle->wADCTriggerUnSet = 0x1E6801u;

    /* Store the bit-banding address to activate/deactivate TIMx CH4 channel */
    pHandle->wTIMxCH4_BB_Addr = 0x42268430u;
  }

  R3HD2_TIMxInit( pHandle->pParams_str->TIMx, pHandle );
  
  if ( pHandle->pParams_str->TIMx == TIM1 )
  {
    /* TIM1 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);
  }
  else
  {
    /* TIM8 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM8_STOP);
  }

  /* ADC1 and ADC2 registers configuration ---------------------------------*/
  /* Enable ADCx1 and ADCx2 */
  LL_ADC_Enable( ADC1 );
  LL_ADC_Enable( pHandle->pParams_str->ADCx2 );

  /* Disable regular conversion sequencer length set by CubeMX */
  LL_ADC_REG_SetSequencerLength( ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE );
  
  /* Enable external trigger (it will be SW) for ADCx1 and ADCx2 regular
   conversions */
  LL_ADC_REG_StartConversionExtTrig( ADC1, LL_ADC_REG_TRIG_EXT_RISING );
  LL_ADC_REG_StartConversionExtTrig( pHandle->pParams_str->ADCx2, LL_ADC_REG_TRIG_EXT_RISING );

  /* Start calibration of ADCx1 and ADCx2 */
  LL_ADC_StartCalibration( ADC1 );
  LL_ADC_StartCalibration( pHandle->pParams_str->ADCx2 );

  /* Wait for the end of ADCs calibration */
  while ( LL_ADC_IsCalibrationOnGoing( ADC1 ) & LL_ADC_IsCalibrationOnGoing( pHandle->pParams_str->ADCx2 ) )
  {
  }
  
  /* ADC1 Injected conversions end interrupt enabling */
  LL_ADC_ClearFlag_JEOS(ADC1);
  LL_ADC_EnableIT_JEOS(ADC1);

  pHandle->_Super.DTTest = 0u;
  pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;

}
/**
 * @brief  Initializes TIMx peripheral for PWM generation
 *
 * @param  TIMx Timer to be initialized
 * @param  pHandle Handle of the component being initialized
 * @retval none
 */
static void R3HD2_TIMxInit( TIM_TypeDef* TIMx, PWMC_R3_HD2_Handle_t * pHandle )
{

  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH3);

  if ((pHandle->pParams_str->LowSideOutputs)== LS_PWM_TIMER)
  {
     LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1N|LL_TIM_CHANNEL_CH2N|LL_TIM_CHANNEL_CH3N);
  }


  /* Channel 4 */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);

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
 * @brief  Calibrates the ADC used for reading current
 *
 *  This function stores the voltage measured on Ia and Ib current
 * feedback analog channels when no current is flowing into the motor
 * in the handle of the component.
 *
 * @param  pHandle Handle of the component to calibrate
 */
void R3HD2_CurrentReadingCalibration( PWMC_Handle_t * pHdl )
{
  PWMC_R3_HD2_Handle_t * pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  pHandle->wPhaseAOffset = 0u;
  pHandle->wPhaseBOffset = 0u;
  pHandle->wPhaseCOffset = 0u;

  pHandle->bIndex = 0u;

  /* It forces inactive level on TIMx CHy and CHyN */
  TIMx->CCER &= TIMxCCER_MASK;

  /* Offset calibration for A & B phases */
  /* Change function to be executed in ADCx_ISR */
  pHdl->pFctGetPhaseCurrents = &R3HD2_HFCurrentsCalibrationAB;

  pHandle->wADC1Channel = PHASE_A_MSK;
  pHandle->wADC2Channel = PHASE_B_MSK;

  R3HD2_SwitchOnPWM( pHdl );

  /* Wait for NB_CONVERSIONS to be executed */
  while ( pHandle->bIndex < (NB_CONVERSIONS) )
  {
    if ( !(LL_TIM_IsEnabledIT_UPDATE(TIMx)) )
    {
      pHandle->bIndex = NB_CONVERSIONS;
    }
  }

  /* Offset calibration for C phase */
  /* Reset bIndex */
  pHandle->bIndex = 0u;

  /* Change function to be executed in ADCx_ISR */
  pHdl->pFctGetPhaseCurrents = &R3HD2_HFCurrentsCalibrationC;

  pHandle->wADC1Channel = PHASE_C_MSK;
  pHandle->wADC2Channel = PHASE_C_MSK;

  R3HD2_SwitchOnPWM( pHdl );

  /* Wait for NB_CONVERSIONS to be executed */
  while ( pHandle->bIndex < (NB_CONVERSIONS / 2u) )
  {
    if ( !(LL_TIM_IsEnabledIT_UPDATE(TIMx)) )
    {
      pHandle->bIndex = NB_CONVERSIONS;
    }
  }

  pHandle->wPhaseAOffset >>= 3;
  pHandle->wPhaseBOffset >>= 3;
  pHandle->wPhaseCOffset >>= 3;

  /* Change back function to be executed in ADCx_ISR */
  pHdl->pFctGetPhaseCurrents = &R3HD2_GetPhaseCurrents;

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

  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  TIMx->CCER |= 0x555u;
}

/**
 * @brief Computes and returns the most recently converted motor phase currents
 *
 * @param pHandle Handle on the PWMC component in charge of the target motor
 * @param pStatorCurrents pointer on the variable where the result is stored
 */
void R3HD2_GetPhaseCurrents( PWMC_Handle_t * pHdl, Curr_Components* pStator_Currents )
{
  uint8_t bSector;
  int32_t wAux;
  PWMC_R3_HD2_Handle_t * pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;

  /* Deactivate TIMx CH4 to disable next triggers using bit-banding access */
  *(uint32_t*) (pHandle->wTIMxCH4_BB_Addr) = 0u;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pHandle->bSoFOC = 0u;

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
    wAux = (int32_t)( pHandle->pParams_str->ADCx2->JDR1 );
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
    wAux = (int32_t)( pHandle->pParams_str->ADCx2->JDR1 );
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
    wAux = (int32_t)( pHandle->pParams_str->ADCx2->JDR1 );
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
 * @brief Sums ADC injected conversion data into @p pHandle. Called during current reading
 *        network calibration only.
 *
 * This function is an implementation of the PWMC_GetPhaseCurrents interface meant to be called
 * during current feedback network calibration. It sums injected conversion data into
 * PWMC_R3_HD2_Handle_t::wPhaseAOffset and PWMC_R3_HD2_Handle_t::wPhaseBOffset to compute the
 * offset introduced in the current feedback network. Calling this function is required to properly
 * configure ADC inputs before to enable the actual offset computation.
 *
 * @param pHandle handle on the component to calibrate
 * @param Pointer on a Curr_Components variable. Set to {0,0} by this function.
 */
static void R3HD2_HFCurrentsCalibrationAB( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents )
{
  PWMC_R3_HD2_Handle_t * pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;
  
  /* Deactivate TIMx CH4 to disable next triggers using bit-banding access */
  *(uint32_t*) (pHandle->wTIMxCH4_BB_Addr) = 0u;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pHandle->bSoFOC = 0u;

  if ( pHandle->bIndex < NB_CONVERSIONS )
  {
    pHandle->wPhaseAOffset += ADC1->JDR1;
    pHandle->wPhaseBOffset += pHandle->pParams_str->ADCx2->JDR1;
    pHandle->bIndex++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->qI_Component1 = 0;
  pStator_Currents->qI_Component2 = 0;
}

/**
 * @brief Sums ADC injected conversion data into @p pHandle. Called during current reading
 *        network calibration only.
 *
 * This function is an implementation of the PWMC_GetPhaseCurrents interface meant to be called
 * during current feedback network calibration. It sums injected conversion data into
 * PWMC_R3_HD2_Handle_t::wPhaseCOffset to compute the offset introduced in the current feedback
 * network. Calling this function is required to properly configure ADC inputs before to enable
 * the actual offset computation.
 *
 * @param pHandle handle on the component to calibrate
 * @param Pointer on a Curr_Components variable. Set to {0,0} by this function.
 */
static void R3HD2_HFCurrentsCalibrationC( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents )
{
  PWMC_R3_HD2_Handle_t * pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;

  /* Deactivate TIMx CH4 to disable next triggers using bit-banding access */
  *(uint32_t*) (pHandle->wTIMxCH4_BB_Addr) = 0u;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pHandle->bSoFOC = 0u;

  if ( pHandle->bIndex < NB_CONVERSIONS / 2u )
  {
    pHandle->wPhaseCOffset += ADC1->JDR1;
    pHandle->wPhaseCOffset += pHandle->pParams_str->ADCx2->JDR1;
    pHandle->bIndex++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->qI_Component1 = 0;
  pStator_Currents->qI_Component2 = 0;
}

/**
 * @brief  Turns on Low Sides Switches of the power stage.
 *
 * This function is intended to be used for charging the boot capacitors of the driving
 * section. It has to be called each motor start-up when using high voltage drivers
 *
 * @param pHandle Handle on the PWMC component in charge of the target motor
 * @retval none
 */
void R3HD2_TurnOnLowSides( PWMC_Handle_t * pHdl )
{
  PWMC_R3_HD2_Handle_t * pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;
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
  return;
}

/**
 * @brief Starts PWM generation for the target motor.
 *
 *
 * @param pHandle Handle on the PWMC component in charge of the target motor
 */
void R3HD2_SwitchOnPWM( PWMC_Handle_t * pHdl )
{  
  PWMC_R3_HD2_Handle_t * pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* It clears ADCs JSTRT and JEOC bits */
  ADC1->SR &= ~ADC_SR_MASK;
  pHandle->pParams_str->ADCx2->SR &= ~ADC_SR_MASK;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  LL_TIM_EnableIT_UPDATE(TIMx);

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
 * @brief  Stops PWM generation for the target motor.
 *
 * @todo TODO: Provide more details
 *
 * @param pHandle Handle on the PWMC component in charge of the target motor
 * @retval none
 */
void R3HD2_SwitchOffPWM( PWMC_Handle_t * pHdl )
{ 
  PWMC_R3_HD2_Handle_t * pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

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
  if ( (pHandle->pParams_str->LowSideOutputs) == ES_GPIO )
  {
    LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_ResetOutputPin (pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }

  TIMx->CCER |= TIMxCCER_MASK_CH123;
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
static uint16_t R3HD2_WriteTIMRegisters( PWMC_Handle_t * pHdl )
{
  uint16_t ret_val;
  PWMC_R3_HD2_Handle_t * pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  TIMx->CCR1 = pHandle->_Super.hCntPhA;
  TIMx->CCR2 = pHandle->_Super.hCntPhB;
  TIMx->CCR3 = pHandle->_Super.hCntPhC;

  /* Limit for update event */
  /* Check the status of SOFOC flag. If it is set, an update event has occurred 
     and thus the FOC rate is too high */
  if (pHandle->bSoFOC != 0u)
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
uint16_t R3HD2_SetADCSampPointSect1( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_HD2_Handle_t *pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 1 (i.e phase A duty cycle) */
  if ( (uint16_t) (pHandle->Half_PWMPeriod - pHdl->hCntPhA) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHdl->hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
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

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        /* Set CC4 as PWM mode 1 */
        TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        TIMx->CCMR2 |= CCMR2_CH4_PWM1;

        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
    }
    pHandle->wADC1Channel = PHASE_B_MSK;
    pHandle->wADC2Channel = PHASE_C_MSK;
  }


  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;

  return R3HD2_WriteTIMRegisters( pHdl );
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
uint16_t R3HD2_SetADCSampPointSect2( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_HD2_Handle_t *pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

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

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
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
        /* Set CC4 as PWM mode 1 */
        TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        TIMx->CCMR2 |= CCMR2_CH4_PWM1;

        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
    }
    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_C_MSK;
  }


  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;

  return R3HD2_WriteTIMRegisters( pHdl );
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
uint16_t R3HD2_SetADCSampPointSect3( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_HD2_Handle_t *pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

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

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
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
        /* Set CC4 as PWM mode 1 */
        TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        TIMx->CCMR2 |= CCMR2_CH4_PWM1;

        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
    }
    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_C_MSK;
  }


  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;

  return R3HD2_WriteTIMRegisters( pHdl );
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
uint16_t R3HD2_SetADCSampPointSect4( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_HD2_Handle_t *pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

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

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
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
        /* Set CC4 as PWM mode 1 */
        TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        TIMx->CCMR2 |= CCMR2_CH4_PWM1;

        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
    }
    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }


  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;

  return R3HD2_WriteTIMRegisters( pHdl );
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
uint16_t R3HD2_SetADCSampPointSect5( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_HD2_Handle_t *pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

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

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
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
        /* Set CC4 as PWM mode 1 */
        TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        TIMx->CCMR2 |= CCMR2_CH4_PWM1;

        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
    }
    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }


  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;

  return R3HD2_WriteTIMRegisters( pHdl );
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
uint16_t R3HD2_SetADCSampPointSect6( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_HD2_Handle_t *pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 6 (i.e phase A duty cycle) */
  if ( (uint16_t) (pHandle->Half_PWMPeriod - pHdl->hCntPhA) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHdl->hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
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
        /* Set CC4 as PWM mode 1 */
        TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        TIMx->CCMR2 |= CCMR2_CH4_PWM1;

        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
    }
    pHandle->wADC1Channel = PHASE_B_MSK;
    pHandle->wADC2Channel = PHASE_C_MSK;
  }


  /* Set TIMx_CH4 value */
  TIMx->CCR4 = hCntSmp;

  return R3HD2_WriteTIMRegisters( pHdl );
}

/**
 * @brief PWMC_R3_HD2 TIMer Update Interrupt handler
 *
 * Handles the Timer Update interrupt related to the PWMC_R3_HD2 component pointed to
 * by @p pHandle.
 *
 * @param pHandle Handle of the component processed interrupts relate to.
 * @param flag Identifier of the the IRQ to process. Not used here.
 *
 * @retval Actually a pointer on an uint8_t data that is set to 0 for Motor 1 and to 1 for Motor 2.
 */
void * R3HD2_TIMx_UP_IRQHandler( PWMC_R3_HD2_Handle_t * pHandle)
{
  uint32_t wADCInjFlags;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /* Set the SOFOC flag to indicate the execution of Update IRQ*/
  pHandle->bSoFOC = 1u;

  wADCInjFlags = (ADC1->SR) & ADC_SR_MASK;

  if ( wADCInjFlags == CONV_STARTED )
  {
    do
    {
      wADCInjFlags = (ADC1->SR) & ADC_SR_MASK;
    } while ( wADCInjFlags != CONV_FINISHED );
  }
  else if ( wADCInjFlags == FLAGS_CLEARED )
  {
    while ( (TIMx->CNT) < (pHandle->pParams_str->Tw) )
    {}
    wADCInjFlags = (ADC1->SR) & ADC_SR_MASK;

    if ( wADCInjFlags == CONV_STARTED )
    {
      do
      {
        wADCInjFlags = (ADC1->SR) & ADC_SR_MASK;
      } while ( wADCInjFlags != CONV_FINISHED );
    }
  }
  else
  {
    /* Nothing to do here */
  }

  /* Switch Context */
  /* Disabling trigger to avoid unwanted conversion */
  ADC1->CR2 = pHandle->wADCTriggerUnSet;
  pHandle->pParams_str->ADCx2->CR2 = pHandle->wADCTriggerUnSet;

  /* Enabling next Trigger */
  TIMx->CCER |= 0x1000u;

  /* It re-initialize AD converter in run time when using dual MC */
  ADC1->CR2 = pHandle->wADCTriggerSet;
  pHandle->pParams_str->ADCx2->CR2 = pHandle->wADCTriggerSet;

  /* Change channels keeping equal to 1 element the sequencer length */
  ADC1->JSQR = pHandle->wADC1Channel;
  pHandle->pParams_str->ADCx2->JSQR = pHandle->wADC2Channel;

  return &( pHandle->_Super.bMotor );
}


/**
 * @brief  It contains the Break event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
void *R3HD2_BRK_IRQHandler(PWMC_R3_HD2_Handle_t *pHandle)
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
uint16_t R3HD2_IsOverCurrentOccurred( PWMC_Handle_t * pHdl )
{
  PWMC_R3_HD2_Handle_t * pHandle = (PWMC_R3_HD2_Handle_t *) pHdl;
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
