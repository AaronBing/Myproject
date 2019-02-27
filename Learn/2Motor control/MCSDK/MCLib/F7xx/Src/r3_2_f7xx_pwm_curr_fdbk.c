/**
  ******************************************************************************
  * @file    r3_f7xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement current sensor
  *          class to be stantiated when the three shunts current sensing
  *          topology is used. It is specifically designed for STM32F302x8
  *          microcontrollers and implements the successive sampling of two motor
  *          current using only one ADC.
  *           + MCU peripheral and handle initialization fucntion
  *           + three shunt current sesnsing
  *           + space vector modulation function
  *           + ADC sampling function
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
#include "r3_2_f7xx_pwm_curr_fdbk.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @defgroup r3_f7XX_pwm_curr_fdbk R3 F7xx PWM & Current Feedback
 *
 * @brief STM32F7, 3-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F7 MCU
 * and using a three shunt resistors current sensing topology.
 *
 * @todo: TODO: complete documentation.
  * @{
  */

/* Private defines -----------------------------------------------------------*/
/* #define SAMPLING_POINT_DEBUG */
/* ADC SMPx mask */
#define SMPR_SMP_Set              ((uint32_t) (0x00000007u))

#define TIMxCCER_MASK              (LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|\
                                    LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|\
                                    LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N|\
                                    LL_TIM_CHANNEL_CH4)
#define TIMxCCER_MASK_CH123        (LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|\
                                    LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|\
                                    LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N)
#define ADC_SR_MASK                ((uint32_t) (0xC))
#define NB_CONVERSIONS 16u
#define PHASE_A_MSK       (uint32_t)((uint32_t)((pHandle->pParams_str->bIaChannel) << 15))
#define PHASE_B_MSK       (uint32_t)((uint32_t)((pHandle->pParams_str->bIbChannel) << 15))
#define PHASE_C_MSK       (uint32_t)((uint32_t)((pHandle->pParams_str->bIcChannel) << 15))
#define CCMR2_CH4_DISABLE 0x8FFFu
#define CCMR2_CH4_PWM1    0x6000u
#define CCMR2_CH4_PWM2    0x7000u

/* Private function prototypes -----------------------------------------------*/
static void R3_2_F7XX_TIMxInit( TIM_TypeDef * TIMx, PWMC_Handle_t * pHdl );
static uint16_t R3_2_F7XX_WriteTIMRegisters( PWMC_Handle_t * pHdl, uint16_t hCCR4Reg );
static void R3_2_F7XX_HFCurrentsCalibrationAB( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents );
static void R3_2_F7XX_HFCurrentsCalibrationC( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents );
static void R3_2_F7XX_RLGetPhaseCurrents( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents );
static void R3_2_F7XX_RLTurnOnLowSides( PWMC_Handle_t * pHdl );
static void R3_2_F7XX_RLSwitchOnPWM( PWMC_Handle_t * pHdl );

/* Private functions ---------------------------------------------------------*/

/* Local redefinition of both LL_TIM_OC_EnablePreload & LL_TIM_OC_DisablePreload */
__STATIC_INLINE void __LL_TIM_OC_EnablePreload(TIM_TypeDef *TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register volatile uint32_t *pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1) + OFFSET_TAB_CCMRx[iChannel]));
  SET_BIT(*pReg, (TIM_CCMR1_OC1PE << SHIFT_TAB_OCxx[iChannel]));
}

__STATIC_INLINE void __LL_TIM_OC_DisablePreload(TIM_TypeDef *TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register volatile uint32_t *pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1) + OFFSET_TAB_CCMRx[iChannel]));
  CLEAR_BIT(*pReg, (TIM_CCMR1_OC1PE << SHIFT_TAB_OCxx[iChannel]));
}

/**
  * @brief  It initializes TIM, ADC, GPIO, DMA and NVIC for current reading and
  *         PWM generation in three shunt configuration using STM32F7XX
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void R3_2_F7XX_Init( PWMC_R3_2_F7_Handle_t * pHandle )
{
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  if ( ( uint32_t )pHandle == ( uint32_t )&pHandle->_Super )
  {

    LL_ADC_DisableIT_EOCS( ADCx_1 );
    LL_ADC_ClearFlag_EOCS( ADCx_1 );
    LL_ADC_DisableIT_JEOS( ADCx_1 );
    LL_ADC_ClearFlag_JEOS( ADCx_1 );
    LL_ADC_DisableIT_EOCS( ADCx_2 );
    LL_ADC_ClearFlag_EOCS( ADCx_2 );
    LL_ADC_DisableIT_JEOS( ADCx_2 );
    LL_ADC_ClearFlag_JEOS( ADCx_2 );

    R3_2_F7XX_TIMxInit( pHandle->pParams_str->TIMx, &pHandle->_Super );

    if ( pHandle->pParams_str->TIMx == TIM1 )
    {
      /* TIM1 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM1_STOP );
      pHandle->ADC_ExternalTriggerInjected = LL_ADC_INJ_TRIG_EXT_TIM1_CH4;
    }
    else
    {
      /* TIM8 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM8_STOP );
      pHandle->ADC_ExternalTriggerInjected = LL_ADC_INJ_TRIG_EXT_TIM8_CH4;
    }

    /* ADCx_1 and ADCx_2 registers configuration ---------------------------------*/
    /* Enable ADCx_1 and ADCx_2 */
    LL_ADC_Enable( ADCx_1 );
    LL_ADC_Enable( ADCx_2 );

    /* ADCx_1 Injected conversions end interrupt enabling */
    LL_ADC_ClearFlag_JEOS( ADCx_1 );
    LL_ADC_EnableIT_JEOS( ADCx_1 );

    /* reset regular conversion sequencer length set by cubeMX */
    LL_ADC_REG_SetSequencerLength( ADCx_1, LL_ADC_REG_SEQ_SCAN_DISABLE );

    /* reset injected conversion sequencer length set by cubeMX */
    LL_ADC_INJ_SetSequencerLength( ADCx_1, LL_ADC_INJ_SEQ_SCAN_DISABLE );
    LL_ADC_INJ_SetSequencerLength( ADCx_2, LL_ADC_INJ_SEQ_SCAN_DISABLE );
    LL_ADC_INJ_SetTriggerSource( ADCx_1, LL_ADC_INJ_TRIG_SOFTWARE );
    LL_ADC_INJ_SetTriggerSource( ADCx_2, LL_ADC_INJ_TRIG_SOFTWARE );

    pHandle->OverCurrentFlag = false;
    pHandle->_Super.DTTest = 0u;
    pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;
  }
}

/**
  * @brief  It initializes TIMx peripheral for PWM generation
  * @param TIMx: Timer to be initialized
  * @param pHandle: handler of the current instance of the PWM component
  * @retval none
  */
static void R3_2_F7XX_TIMxInit( TIM_TypeDef * TIMx, PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;

  /* disable main TIM counter to ensure
   * a synchronous start by TIM2 trigger */
  LL_TIM_DisableCounter( TIMx );

  /* Enables the TIMx Preload on CC1 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH1 );
  /* Enables the TIMx Preload on CC2 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH2 );
  /* Enables the TIMx Preload on CC3 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH3 );
  /* Enables the TIMx Preload on CC4 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH4 );

  if ( ( pHandle->pParams_str->EmergencyStop ) != DISABLE )
  {
    LL_TIM_ClearFlag_BRK( TIMx );
    LL_TIM_EnableIT_BRK( TIMx );
  }

    LL_TIM_SetCounter( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );
  
}

/**
  * @brief  It stores into the component's handle the voltage present on Ia and
  *         Ib current feedback analog channels when no current is flowin into the
  *         motor
  * @param  pHandle handler of the current instance of the PWM component
  */
void R3_2_F7XX_CurrentReadingCalibration( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->wPhaseAOffset = 0u;
  pHandle->wPhaseBOffset = 0u;
  pHandle->wPhaseCOffset = 0u;

  pHandle->bIndex = 0u;

  LL_TIM_CC_DisableChannel(TIMx, TIMxCCER_MASK);
  
  /* Offset calibration for A & B phases */
  /* Change function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R3_2_F7XX_HFCurrentsCalibrationAB;

  pHandle->wADC1Channel = PHASE_A_MSK;
  pHandle->wADC2Channel = PHASE_B_MSK;

  R3_2_F7XX_SwitchOnPWM( &pHandle->_Super );

  /* Wait for NB_CONVERSIONS to be executed */
  while ( pHandle->bIndex < ( NB_CONVERSIONS ) )
  {
  }

  /* Offset calibration for C phase */
  /* Reset bIndex */
  pHandle->bIndex = 0u;

  /* Change function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R3_2_F7XX_HFCurrentsCalibrationC;

  pHandle->wADC1Channel = PHASE_C_MSK;
  pHandle->wADC2Channel = PHASE_C_MSK;

  R3_2_F7XX_SwitchOnPWM( &pHandle->_Super );

  /* Wait for NB_CONVERSIONS to be executed */
  while ( pHandle->bIndex < ( NB_CONVERSIONS / 2u ) )
  {
  }

  R3_2_F7XX_SwitchOffPWM( &pHandle->_Super );

  pHandle->wPhaseAOffset >>= 3;
  pHandle->wPhaseBOffset >>= 3;
  pHandle->wPhaseCOffset >>= 3;

  /* Change back function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R3_2_F7XX_GetPhaseCurrents;

  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
     force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_SetCompareCH1( TIMx, pHandle->Half_PWMPeriod );
  LL_TIM_OC_SetCompareCH2( TIMx, pHandle->Half_PWMPeriod );
  LL_TIM_OC_SetCompareCH3( TIMx, pHandle->Half_PWMPeriod );
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIMx,TIMxCCER_MASK_CH123);

}

/**
  * @brief  It computes and return latest converted motor phase currents motor
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval Ia and Ib current in Curr_Components format
  */
void R3_2_F7XX_GetPhaseCurrents( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents )
{
  uint8_t bSector;
  int32_t wAux;
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE( pHandle->pParams_str->TIMx );
  bSector = pHandle->_Super.hSector;

  switch ( bSector )
  {
    case SECTOR_4:
    case SECTOR_5:
      /* Current on Phase C is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      wAux = ( int32_t )( ADCx_1->JDR1 );
      wAux *= 2;
      wAux = ( int32_t )( pHandle->wPhaseAOffset ) - wAux;

      /* Saturation of Ia */
      if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component1 = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component1 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1 = ( int16_t )wAux;
      }

      /* Ib = PhaseBOffset - ADC converted value) */
      wAux = ( int32_t )( ADCx_2->JDR1 );
      wAux *= 2;
      wAux = ( int32_t )( pHandle->wPhaseBOffset ) - wAux;

      /* Saturation of Ib */
      if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component2 = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component2 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2 = ( int16_t )wAux;
      }
      break;

    case SECTOR_6:
    case SECTOR_1:
      /* Current on Phase A is not accessible     */
      /* Ib = PhaseBOffset - ADC converted value) */
      wAux = ( int32_t )( ADCx_1->JDR1 );
      wAux *= 2;
      wAux = ( int32_t )( pHandle->wPhaseBOffset ) - wAux;

      /* Saturation of Ib */
      if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component2 = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component2 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2 = ( int16_t )wAux;
      }

      /* Ia = -Ic -Ib */
      wAux = ( int32_t )( ADCx_2->JDR1 );
      wAux *= 2;
      wAux -= ( int32_t )pHandle->wPhaseCOffset;
      wAux -= ( int32_t )pStator_Currents->qI_Component2;

      /* Saturation of Ia */
      if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component1 = INT16_MAX;
      }
      else  if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component1 = -INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1 = ( int16_t )wAux;
      }
      break;

    case SECTOR_2:
    case SECTOR_3:
      /* Current on Phase B is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      wAux = ( int32_t )( ADCx_1->JDR1 );
      wAux *= 2;
      wAux = ( int32_t )( pHandle->wPhaseAOffset ) - wAux;

      /* Saturation of Ia */
      if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component1 = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component1 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1 = ( int16_t )wAux;
      }

      /* Ib = -Ic -Ia */
      wAux = ( int32_t )( ADCx_2->JDR1 );
      wAux *= 2;
      wAux -= ( int32_t )pHandle->wPhaseCOffset;
      wAux -= ( int32_t )pStator_Currents->qI_Component1;

      /* Saturation of Ib */
      if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component2 = INT16_MAX;
      }
      else  if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component2 = -INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2 = ( int16_t )wAux;
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
  * @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into wPhaseAOffset and
  *         wPhaseBOffset to compute the offset introduced in the current feedback
  *         network. It is requied to proper configure ADC inputs before to enable
  *         the offset computation.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval It always returns {0,0} in Curr_Components format
  */
static void R3_2_F7XX_HFCurrentsCalibrationAB( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE( pHandle->pParams_str->TIMx );

  if ( pHandle->bIndex < NB_CONVERSIONS )
  {
    pHandle-> wPhaseAOffset += ADCx_1->JDR1;
    pHandle-> wPhaseBOffset += ADCx_2->JDR1;
    pHandle->bIndex++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->qI_Component1 = 0;
  pStator_Currents->qI_Component2 = 0;
}

/**
  * @brief  Implementation of PWMC_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into wPhaseCOffset
  *         to compute the offset introduced in the current feedback
  *         network. It is required to proper configure ADC input before to enable
  *         the offset computation.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval It always returns {0,0} in Curr_Components format
  */
static void R3_2_F7XX_HFCurrentsCalibrationC( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE( pHandle->pParams_str->TIMx );
  if ( pHandle->bIndex < NB_CONVERSIONS / 2u )
  {
    pHandle-> wPhaseCOffset += ADCx_1->JDR1;
    pHandle-> wPhaseCOffset += ADCx_2->JDR1;
    pHandle->bIndex++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->qI_Component1 = 0;
  pStator_Currents->qI_Component2 = 0;
}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void R3_2_F7XX_TurnOnLowSides( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = true;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1( TIMx, 0 );
  LL_TIM_OC_SetCompareCH2( TIMx, 0 );
  LL_TIM_OC_SetCompareCH3( TIMx, 0 );

  /* Wait until next update */
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == RESET )
  {}
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );
  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }
  return;
}


/**
  * @brief  It enables PWM generation on the proper Timer peripheral acting on MOE
  *         bit
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void R3_2_F7XX_SwitchOnPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* It clears ADCs JSTRT and JEOC bits */
  LL_ADC_ClearFlag_JEOS(ADCx_1);
  LL_ADC_ClearFlag_JEOS(ADCx_2);

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );


  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    if ( LL_TIM_CC_IsEnabledChannel(TIMx,TIMxCCER_MASK_CH123) != 0u )
    {
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
  }

  LL_ADC_INJ_SetTriggerSource( ADCx_1, pHandle->ADC_ExternalTriggerInjected);
  LL_ADC_INJ_SetTriggerSource( ADCx_2, pHandle->ADC_ExternalTriggerInjected);
  LL_ADC_INJ_StartConversionExtTrig(ADCx_1,LL_ADC_INJ_TRIG_EXT_RISING);
  LL_ADC_INJ_StartConversionExtTrig(ADCx_2,LL_ADC_INJ_TRIG_EXT_RISING);

  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);
  
  return;
}


/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void R3_2_F7XX_SwitchOffPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE( TIMx );

  LL_TIM_CC_DisableChannel(TIMx,TIMxCCER_MASK_CH123);

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == RESET )
  {
    if ( LL_TIM_IsEnabledIT_UPDATE( TIMx ) )
    {
      break;
    }
  }
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs( TIMx );
  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }
  LL_TIM_CC_EnableChannel (TIMx,TIMxCCER_MASK_CH123);

  return;
}

/**
  * @brief  writes into peripheral registers the new duty cycles and
  *        sampling point
  * @param  pHandle: handler of the current instance of the PWM component
  * @param hCCR4Reg: new capture/compare register value.
  * @retval none
  */
static uint16_t R3_2_F7XX_WriteTIMRegisters( PWMC_Handle_t * pHdl, uint16_t hCCR4Reg )
{
  uint16_t hAux;
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  LL_TIM_OC_SetCompareCH1( TIMx, pHandle->_Super.hCntPhA );
  LL_TIM_OC_SetCompareCH2( TIMx, pHandle->_Super.hCntPhB );
  LL_TIM_OC_SetCompareCH3( TIMx, pHandle->_Super.hCntPhC );

  /* deactivate trigger to avoid a ADC conversion every PWM period
   * in case of FOC execution rate higher than 1 */
  __LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH4 );
  LL_TIM_OC_SetCompareCH4 ( TIMx, 0xFFFFu );
  __LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH4 );
  LL_TIM_OC_SetCompareCH4 ( TIMx, hCCR4Reg );

  ADCx_1->JSQR = pHandle->wADC1Channel;
  ADCx_2->JSQR = pHandle->wADC2Channel;
  
  /* Limit for update event */
  /* Check the status of SOFOC flag. If it is set, an update event has occurred
  and thus the FOC rate is too high */
  if ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) )
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  return hAux;
}

/**
  * @brief  Configure the ADC for the current sampling related to sector 1.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  this related object of class CPWMC
  * @retval none
  */
uint16_t R3_2_F7XX_SetADCSampPointSect1( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }
  else
  {
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhB );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        /* Set CC4 as PWM mode 1 */
        TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        TIMx->CCMR2 |= CCMR2_CH4_PWM1;

        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }
    pHandle->wADC1Channel = PHASE_B_MSK;
    pHandle->wADC2Channel = PHASE_C_MSK;
  }

  return R3_2_F7XX_WriteTIMRegisters( &pHandle->_Super, hCntSmp );
}

/**
  * @brief  Configure the ADC for the current sampling related to sector 2.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_2_F7XX_SetADCSampPointSect2( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }
  else
  {
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhA );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhB + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        /* Set CC4 as PWM mode 1 */
        TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        TIMx->CCMR2 |= CCMR2_CH4_PWM1;

        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }
    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_C_MSK;
  }

  return R3_2_F7XX_WriteTIMRegisters( &pHandle->_Super, hCntSmp );
}

/**
  * @brief  Configure the ADC for the current sampling related to sector 3.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_2_F7XX_SetADCSampPointSect3( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }
  else
  {
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhC );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhB + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        /* Set CC4 as PWM mode 1 */
        TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        TIMx->CCMR2 |= CCMR2_CH4_PWM1;

        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }
    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_C_MSK;
  }

  return R3_2_F7XX_WriteTIMRegisters( &pHandle->_Super, hCntSmp );
}

/**
  * @brief  Configure the ADC for the current sampling related to sector 4.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_2_F7XX_SetADCSampPointSect4( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }
  else
  {
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhB );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        /* Set CC4 as PWM mode 1 */
        TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        TIMx->CCMR2 |= CCMR2_CH4_PWM1;

        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }
    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }

  return R3_2_F7XX_WriteTIMRegisters( &pHandle->_Super, hCntSmp );
}

/**
  * @brief  Configure the ADC for the current sampling related to sector 4.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_2_F7XX_SetADCSampPointSect5( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }
  else
  {
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhA );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        /* Set CC4 as PWM mode 1 */
        TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        TIMx->CCMR2 |= CCMR2_CH4_PWM1;

        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }
    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }

  return R3_2_F7XX_WriteTIMRegisters( &pHandle->_Super, hCntSmp );
}

/**
  * @brief  Configure the ADC for the current sampling related to sector 6.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_2_F7XX_SetADCSampPointSect6( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;

  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = pHandle->Half_PWMPeriod - 1u;

    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;
  }
  else
  {
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhC );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        /* Set CC4 as PWM mode 1 */
        TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        TIMx->CCMR2 |= CCMR2_CH4_PWM1;

        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }
    pHandle->wADC1Channel = PHASE_B_MSK;
    pHandle->wADC2Channel = PHASE_C_MSK;
  }

  return R3_2_F7XX_WriteTIMRegisters( &pHandle->_Super, hCntSmp );
}

/**
  * @brief  It contains the TIMx Update event interrupt
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void * R3_2_F7XX_TIMx_UP_IRQHandler( PWMC_R3_2_F7_Handle_t * pHandle )
{

  return &( pHandle->_Super.bMotor );
}

/**
 * @brief  It contains the Break event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
void * R3_2_F7XX_BRK_IRQHandler( PWMC_R3_2_F7_Handle_t * pHandle )
{
  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }
  pHandle->OverCurrentFlag = true;

  return &( pHandle->_Super.bMotor );
}

/**
  * @brief  It is used to check if an overcurrent occurred since last call.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
  *                  detected since last method call, MC_NO_FAULTS otherwise.
  */
uint16_t R3_2_F7XX_IsOverCurrentOccurred( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;

  uint16_t retVal = MC_NO_FAULTS;
  if ( pHandle->OverCurrentFlag == true )
  {
    retVal = MC_BREAK_IN;
    pHandle->OverCurrentFlag = false;
  }
  return retVal;
}

/**
  * @brief  It is used to set the PWM mode for R/L detection.
  * @param  pHandle: handler of the current instance of the PWM component
  * @param  hDuty to be applied in uint16_t
  * @retval none
  */
void R3_2_F7XX_RLDetectionModeEnable( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  if ( pHandle->_Super.RLDetectionMode == false )
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    LL_TIM_OC_SetCompareCH1( TIMx, 0u );

    /*  Channel2 configuration */
    if ( ( pHandle->pParams_str->LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE );
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2 );
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE );
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2 );
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else
    {
    }

    /*  Channel3 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM2 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3N );

    pHandle->wPhaseAOffset = pHandle->wPhaseBOffset; /* Use only the offset of phB */
  }

  pHandle->_Super.pFctGetPhaseCurrents = &R3_2_F7XX_RLGetPhaseCurrents;
  pHandle->_Super.pFctTurnOnLowSides = &R3_2_F7XX_RLTurnOnLowSides;
  pHandle->_Super.pFctSwitchOnPwm = &R3_2_F7XX_RLSwitchOnPWM;
  pHandle->_Super.pFctSwitchOffPwm = &R3_2_F7XX_SwitchOffPWM;

  pHandle->_Super.RLDetectionMode = true;
}

/**
  * @brief  It is used to disable the PWM mode in 6-step.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void R3_2_F7XX_RLDetectionModeDisable( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  if ( pHandle->_Super.RLDetectionMode == true )
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1 );

    if ( ( pHandle->pParams_str->LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    }
    else if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH1( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );

    /*  Channel2 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2 );

    if ( ( pHandle->pParams_str->LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH2( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );

    /*  Channel3 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH3 );

    if ( ( pHandle->pParams_str->LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH3N );
    }
    else if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH3( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );

    pHandle->_Super.pFctGetPhaseCurrents = &R3_2_F7XX_GetPhaseCurrents;
    pHandle->_Super.pFctTurnOnLowSides = &R3_2_F7XX_TurnOnLowSides;
    pHandle->_Super.pFctSwitchOnPwm = &R3_2_F7XX_SwitchOnPWM;
    pHandle->_Super.pFctSwitchOffPwm = &R3_2_F7XX_SwitchOffPWM;

    pHandle->_Super.RLDetectionMode = false;
  }
}

/**
  * @brief  It is used to set the PWM dutycycle in 6-step mode.
  * @param  pHandle: handler of the current instance of the PWM component
  * @param  hDuty to be applied in uint16_t
  * @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR'
  *         otherwise. These error codes are defined in mc_type.h
  */
uint16_t R3_2_F7XX_RLDetectionModeSetDuty( PWMC_Handle_t * pHdl, uint16_t hDuty )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  uint16_t hAux;


  uint32_t val = ( ( uint32_t )( pHandle->Half_PWMPeriod ) * ( uint32_t )( hDuty ) ) >> 16;
  pHandle->_Super.hCntPhA = ( uint16_t )( val );

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM1;

  TIMx->CCR4 = ( uint32_t )( pHandle->Half_PWMPeriod ) - pHandle->_Super.Ton;
  TIMx->CCR3 = pHandle->_Super.Toff;

  TIMx->CCR1 = pHandle->_Super.hCntPhA;

  /* Change channels keeping equal to 1 element the sequencer lenght */
  ADCx_1->JSQR = PHASE_B_MSK;
  ADCx_2->JSQR = PHASE_B_MSK;

  /* Limit for update event */
  /* Check the status flag. If an update event has occurred before to set new
  values of regs the FOC rate is too high */
  if ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) )
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  if ( pHandle->_Super.SWerror == 1u )
  {
    hAux = MC_FOC_DURATION;
    pHandle->_Super.SWerror = 0u;
  }
  return hAux;
}

/**
  * @brief  It computes and return latest converted motor phase currents motor
  *         during RL detection phase
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval Ia and Ib current in Curr_Components format
  */
static void R3_2_F7XX_RLGetPhaseCurrents( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  int32_t wAux;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE( TIMx );

  wAux = ( int32_t )( pHandle->wPhaseBOffset ) - (int32_t)(ADCx_1->JDR1 * 2);

  /* Check saturation */
  if ( wAux > -INT16_MAX )
  {
    if ( wAux < INT16_MAX )
    {
    }
    else
    {
      wAux = INT16_MAX;
    }
  }
  else
  {
    wAux = -INT16_MAX;
  }

  pStator_Currents->qI_Component1 = (int16_t)wAux;
  pStator_Currents->qI_Component2 = (int16_t)wAux;
}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers.
  *         This function is specific for RL detection phase.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
static void R3_2_F7XX_RLTurnOnLowSides( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /*Turn on the phase A low side switch */
  LL_TIM_OC_SetCompareCH1 ( TIMx, 0u );

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Wait until next update */
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }
  return;
}


/**
  * @brief  It enables PWM generation on the proper Timer peripheral
  *         This function is specific for RL detection phase.
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
static void R3_2_F7XX_RLSwitchOnPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;


  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE( TIMx );
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  LL_TIM_OC_SetCompareCH1( TIMx, 1u );
  LL_TIM_OC_SetCompareCH4( TIMx, ( pHandle->Half_PWMPeriod ) - 5u );

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE ;
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    if ( ( TIMx->CCER & TIMxCCER_MASK_CH123 ) != 0u )
    {
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
  }

  LL_ADC_INJ_SetTriggerSource( ADCx_1, pHandle->ADC_ExternalTriggerInjected);
  LL_ADC_INJ_SetTriggerSource( ADCx_2, pHandle->ADC_ExternalTriggerInjected);
  LL_ADC_INJ_StartConversionExtTrig(ADCx_1,LL_ADC_INJ_TRIG_EXT_RISING);
  LL_ADC_INJ_StartConversionExtTrig(ADCx_2,LL_ADC_INJ_TRIG_EXT_RISING);

  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  ADCx_1->JSQR = PHASE_B_MSK;
  ADCx_2->JSQR = PHASE_B_MSK;

  return;
}

/**
 * @brief  It turns on low sides switches and start ADC triggering.
 *         This function is specific for MP phase.
 * @param  pHandle Pointer on the target component instance
 * @retval none
 */
void RLTurnOnLowSidesAndStart( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  LL_TIM_OC_SetCompareCH1 ( TIMx, 0x0u );
  LL_TIM_OC_SetCompareCH2 ( TIMx, 0x0u );
  LL_TIM_OC_SetCompareCH3 ( TIMx, 0x0u );

  TIMx->CCR4 = ( uint32_t )( pHandle->Half_PWMPeriod ) - 5u;

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE ;
  LL_TIM_EnableAllOutputs ( TIMx );

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }

  LL_ADC_INJ_SetTriggerSource( ADCx_1, pHandle->ADC_ExternalTriggerInjected);
  LL_ADC_INJ_SetTriggerSource( ADCx_2, pHandle->ADC_ExternalTriggerInjected);
  LL_ADC_INJ_StartConversionExtTrig(ADCx_1,LL_ADC_INJ_TRIG_EXT_RISING);
  LL_ADC_INJ_StartConversionExtTrig(ADCx_2,LL_ADC_INJ_TRIG_EXT_RISING);

  ADCx_1->JSQR = PHASE_A_MSK;
  ADCx_2->JSQR = PHASE_B_MSK;

  return;
}


/**
 * @brief  It sets ADC sampling points.
 *         This function is specific for MP phase.
 * @param  pHandle Pointer on the target component instance
 * @retval none
 */
void RLSetADCSampPoint( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_F7_Handle_t * pHandle = ( PWMC_R3_2_F7_Handle_t * )pHdl;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  /* dummy sector setting to get correct Ia value */
  pHdl->hSector = SECTOR_5;

  ADCx_1->JSQR = PHASE_A_MSK;
  ADCx_2->JSQR = PHASE_B_MSK;

  return;
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

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
