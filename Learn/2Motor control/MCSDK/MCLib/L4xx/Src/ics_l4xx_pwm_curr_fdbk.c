/**
  ******************************************************************************
  * @file    ics_l4xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the ICS
  *          PWM current feedback component for L4xx of the Motor Control SDK.
  ********************************************************************************
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
#include "ics_l4xx_pwm_curr_fdbk.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup ics_l4xx_pwm_curr_fdbk ICS L4xx PWM & Current Feedback
 *
 * @brief STM32L4, ICS PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32L4 MCU
 * and using an Insulated Current Sensors topology.
 *
 * @todo: TODO: complete documentation.
 *
 * @{
 */

/* ADC SMPx mask */
#define SMPR_SMP_Set              ((uint32_t) (0x00000007u))

/* ADC registers reset values */
#define ADC_HTR_RESET_VALUE        ((uint32_t) (0x00000FFFu))
#define TIMxCCER_MASK_CH123        (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 | \
                                    LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N)

#define TIMx_CC4E_BIT              ((uint16_t)  0x1000u)

#define CONV_STARTED               ((uint32_t) (0x8))
#define CONV_FINISHED              ((uint32_t) (0xC))
#define FLAGS_CLEARED              ((uint32_t) (0x0))

#define ADC_RIGHT_ALIGNMENT 3u

#define NB_CONVERSIONS 16u

static void IL4XX_TIMxInit( TIM_TypeDef * TIMx, PWMC_ICS_L4_Handle_t * pHandle );

/**
* @brief  It initializes TIMx, ADC, GPIO and NVIC for current reading
*         in ICS configuration using STM32L4XX
* @param  ICS L4xx PWM Current Feedback Handle
* @retval none
*/
void IL4XX_Init( PWMC_ICS_L4_Handle_t * pHandle )
{
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;
  uint32_t TriggerSource;

  if ( ( uint32_t )pHandle == ( uint32_t )&pHandle->_Super )
  {

    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
    LL_ADC_DisableIT_EOC( ADCx_1 );
    LL_ADC_ClearFlag_EOC( ADCx_1 );
    LL_ADC_DisableIT_JEOC( ADCx_1 );
    LL_ADC_ClearFlag_JEOC( ADCx_1 );
    LL_ADC_DisableIT_EOC( ADCx_2 );
    LL_ADC_ClearFlag_EOC( ADCx_2 );
    LL_ADC_DisableIT_JEOC( ADCx_2 );
    LL_ADC_ClearFlag_JEOC( ADCx_2 );

    IL4XX_TIMxInit( TIMx, pHandle );

    if ( TIMx == TIM1 )
    {
      /* TIM1 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM1_STOP );
      TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO;
    }
#if defined(TIM8)
    else
    {
      /* TIM8 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM8_STOP );
      TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM8_TRGO;
    }
#endif
    /* ADCx_1 and ADCx_2 registers configuration ---------------------------------*/
    /* ADCx_1 and ADCx_2 registers reset */

    LL_ADC_EnableInternalRegulator( ADCx_1 );
    LL_ADC_EnableInternalRegulator( ADCx_2 );

    /* Wait for Regulator Startup time, once for both */
    uint16_t waittime = 0u;
    for ( waittime = 0u; waittime < 65000u; waittime++ )
    {
    }

    LL_ADC_StartCalibration( ADCx_1, LL_ADC_SINGLE_ENDED );
    while ( LL_ADC_IsCalibrationOnGoing( ADCx_1 ) )
    {
    }

    LL_ADC_StartCalibration( ADCx_2, LL_ADC_SINGLE_ENDED );
    while ( LL_ADC_IsCalibrationOnGoing( ADCx_2 ) )
    {
    }

    /* ADCx_1 and ADCx_2 registers configuration ---------------------------------*/
    /* Enable ADCx_1 and ADCx_2 */
    LL_ADC_Enable( ADCx_1 );
    LL_ADC_Enable( ADCx_2 );

    /* reset regular conversion sequencer length set by cubeMX */
    LL_ADC_REG_SetSequencerLength( ADCx_1, LL_ADC_REG_SEQ_SCAN_DISABLE );

    /* ADCx_1 Injected conversions end interrupt enabling */
    LL_ADC_ClearFlag_JEOC( ADCx_1 );
    LL_ADC_EnableIT_JEOC( ADCx_1 );

    LL_ADC_INJ_SetQueueMode( ADCx_1, LL_ADC_INJ_QUEUE_2CONTEXTS_LAST_ACTIVE );
    LL_ADC_INJ_SetQueueMode( ADCx_2, LL_ADC_INJ_QUEUE_2CONTEXTS_LAST_ACTIVE );

    /* Change channels keeping equal to 1 element the sequencer length */
    LL_ADC_INJ_ConfigQueueContext( ADCx_1,
                                   TriggerSource,
                                   LL_ADC_INJ_TRIG_EXT_RISING,
                                   LL_ADC_INJ_SEQ_SCAN_DISABLE,
                                   __LL_ADC_DECIMAL_NB_TO_CHANNEL( pHandle->pParams_str->bIaChannel ),
                                   __LL_ADC_DECIMAL_NB_TO_CHANNEL( pHandle->pParams_str->bIaChannel ),
                                   __LL_ADC_DECIMAL_NB_TO_CHANNEL( pHandle->pParams_str->bIaChannel ),
                                   __LL_ADC_DECIMAL_NB_TO_CHANNEL( pHandle->pParams_str->bIaChannel ) );

    LL_ADC_INJ_ConfigQueueContext( ADCx_2,
                                   TriggerSource,
                                   LL_ADC_INJ_TRIG_EXT_RISING,
                                   LL_ADC_INJ_SEQ_SCAN_DISABLE,
                                   __LL_ADC_DECIMAL_NB_TO_CHANNEL( pHandle->pParams_str->bIbChannel ),
                                   __LL_ADC_DECIMAL_NB_TO_CHANNEL( pHandle->pParams_str->bIbChannel ),
                                   __LL_ADC_DECIMAL_NB_TO_CHANNEL( pHandle->pParams_str->bIbChannel ),
                                   __LL_ADC_DECIMAL_NB_TO_CHANNEL( pHandle->pParams_str->bIbChannel ) );

    LL_ADC_INJ_StartConversion( ADCx_1 );
    LL_ADC_INJ_StartConversion( ADCx_2 );

    /* Clear the flags */
    pHandle->OverVoltageFlag = false;
    pHandle->OverCurrentFlag = false;

    pHandle->_Super.DTTest = 0u;
    pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;

  }
}

/**
* @brief  It initializes TIMx peripheral for PWM generation
* @param 'TIMx': Timer to be initialized
* @param ICS L4xx PWM Current Feedback Handle
* @retval none
*/
static void IL4XX_TIMxInit( TIM_TypeDef * TIMx, PWMC_ICS_L4_Handle_t * pHandle )
{

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

  /* Always enable BKIN for safety feature */
  LL_TIM_ClearFlag_BRK( TIMx );
  if ( ( pHandle->pParams_str->bBKIN2Mode ) != NONE )
  {
    LL_TIM_ClearFlag_BRK2( TIMx );
  }
  LL_TIM_EnableIT_BRK( TIMx );

  /* Prepare timer for synchronization */
  LL_TIM_GenerateEvent_UPDATE( TIMx );
  if ( pHandle->pParams_str->bFreqRatio == 2u )
  {
    if ( pHandle->pParams_str->bIsHigherFreqTim == HIGHER_FREQ )
    {
      if ( pHandle->pParams_str->bRepetitionCounter == 3u )
      {
        /* Set TIMx repetition counter to 1 */
        LL_TIM_SetRepetitionCounter( TIMx, 1 );
        LL_TIM_GenerateEvent_UPDATE( TIMx );
        /* Repetition counter will be set to 3 at next Update */
        LL_TIM_SetRepetitionCounter( TIMx, 3 );
      }
    }
    LL_TIM_SetCounter( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );
  }
  else /* bFreqRatio equal to 1 or 3 */
  {
    if ( pHandle->_Super.bMotor == M1 )
    {
      if ( pHandle->pParams_str->bRepetitionCounter == 1u )
      {
        LL_TIM_SetCounter( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );
      }
      else if ( pHandle->pParams_str->bRepetitionCounter == 3u )
      {
        /* Set TIMx repetition counter to 1 */
        LL_TIM_SetRepetitionCounter( TIMx, 1 );
        LL_TIM_GenerateEvent_UPDATE( TIMx );
        /* Repetition counter will be set to 3 at next Update */
        LL_TIM_SetRepetitionCounter( TIMx, 3 );
      }
    }
  }
}

/**
* @brief  It stores handler variable the voltage present on Ia and
*         Ib current feedback analog channels when no current is flowing into the
*         motor
* @param  ICS L4xx PWM Current Feedback Handle
* @retval none
*/
void IL4XX_CurrentReadingCalibration( PWMC_Handle_t * pHandle )
{
  PWMC_ICS_L4_Handle_t * pH = ( PWMC_ICS_L4_Handle_t * ) pHandle;
  TIM_TypeDef * TIMx = pH->pParams_str->TIMx;

  pH->wPhaseAOffset = 0u;
  pH->wPhaseBOffset = 0u;
  pH->bIndex = 0u;

  /* Force inactive level on TIMx CHy and TIMx CHyN */
  LL_TIM_CC_DisableChannel( TIMx, TIMxCCER_MASK_CH123 );

  /* Change function to be executed in ADCx_ISR */
  pH->_Super.pFctGetPhaseCurrents = &IL4XX_HFCurrentsCalibration;
  IL4XX_SwitchOnPWM( pHandle );

  /* Wait for NB_CONVERSIONS to be executed */
  while ( pH->bIndex < ( NB_CONVERSIONS ) )
  {
  }

  IL4XX_SwitchOffPWM( pHandle );

  pH->wPhaseAOffset >>= 4;
  pH->wPhaseBOffset >>= 4;

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
  LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

  /* Change back function to be executed in ADCx_ISR */
  pH->_Super.pFctGetPhaseCurrents = &IL4XX_GetPhaseCurrents;

  pH->BrakeActionLock = false;
}

/**
* @brief  It computes and return latest converted motor phase currents motor
* @param  ICS L4xx PWM Current Feedback Handle
* @retval Ia and Ib current in Curr_Components format
*/
void IL4XX_GetPhaseCurrents( PWMC_Handle_t * pHandle, Curr_Components * pStator_Currents )
{
  int32_t aux;
  uint16_t reg;
  PWMC_ICS_L4_Handle_t * pH = ( PWMC_ICS_L4_Handle_t * ) pHandle;
  ADC_TypeDef * ADCx_1 = pH->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pH->pParams_str->ADCx_2;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pH->hFlags &= ( ~SOFOC );

  /* Ia = (hPhaseAOffset)-(PHASE_A_ADC_CHANNEL value)  */
  reg = ( uint16_t )( ADCx_1->JDR1 );
  aux = ( int32_t )( reg ) - ( int32_t )( pH->wPhaseAOffset );

  /* Saturation of Ia */
  if ( aux < -INT16_MAX )
  {
    pStator_Currents->qI_Component1 = -INT16_MAX;
  }
  else  if ( aux > INT16_MAX )
  {
    pStator_Currents->qI_Component1 = INT16_MAX;
  }
  else
  {
    pStator_Currents->qI_Component1 = ( int16_t )aux;
  }

  /* Ib = (hPhaseBOffset)-(PHASE_B_ADC_CHANNEL value) */
  reg = ( uint16_t )( ADCx_2->JDR1 );
  aux = ( int32_t )( reg ) - ( int32_t )( pH->wPhaseBOffset );

  /* Saturation of Ib */
  if ( aux < -INT16_MAX )
  {
    pStator_Currents->qI_Component2 = -INT16_MAX;
  }
  else  if ( aux > INT16_MAX )
  {
    pStator_Currents->qI_Component2 = INT16_MAX;
  }
  else
  {
    pStator_Currents->qI_Component2 = ( int16_t )aux;
  }

  pH->_Super.hIa = pStator_Currents->qI_Component1;
  pH->_Super.hIb = pStator_Currents->qI_Component2;
  pH->_Super.hIc = -pStator_Currents->qI_Component1 - pStator_Currents->qI_Component2;

}


/**
* @brief  It sum up injected conversion data into wPhaseXOffset. It is called
*         only during current calibration
* @param  ICS L4xx PWM Current Feedback Handle
* @retval It always returns {0,0} in Curr_Components format
*/
void IL4XX_HFCurrentsCalibration( PWMC_Handle_t * pHandle, Curr_Components * pStator_Currents )
{
  PWMC_ICS_L4_Handle_t * pH = ( PWMC_ICS_L4_Handle_t * ) pHandle;
  ADC_TypeDef * ADCx_1 = pH->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pH->pParams_str->ADCx_2;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pH->hFlags &= ( ~SOFOC );

  if ( pH->bIndex < NB_CONVERSIONS )
  {
    pH->wPhaseAOffset += ADCx_1->JDR1;
    pH->wPhaseBOffset += ADCx_2->JDR1;
    pH->bIndex++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->qI_Component1 = 0;
  pStator_Currents->qI_Component2 = 0;
}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param  ICS L4xx PWM Current Feedback Handle
  * @retval none
  */
void IL4XX_TurnOnLowSides( PWMC_Handle_t * pHandle )
{
  PWMC_ICS_L4_Handle_t * pH = ( PWMC_ICS_L4_Handle_t * ) pHandle;
  TIM_TypeDef * TIMx = pH->pParams_str->TIMx;

  pH->_Super.bTurnOnLowSidesAction = true;

  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1 ( TIMx, 0u );
  LL_TIM_OC_SetCompareCH2 ( TIMx, 0u );
  LL_TIM_OC_SetCompareCH3 ( TIMx, 0u );

  /*Disable ADC trigger */
  TIMx->CCMR2 = 0x7068u;
  TIMx->CCR4 = ( uint32_t )( pH->Half_PWMPeriod ) + 1u;

  pH->hFlags &= ( ~SOFOC );

  LL_TIM_ClearFlag_UPDATE( TIMx );
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == RESET ) ;

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );
  if ( ( pH->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pH->pParams_str->pwm_en_u_port, pH->pParams_str->pwm_en_u_pin );
    LL_GPIO_SetOutputPin( pH->pParams_str->pwm_en_v_port, pH->pParams_str->pwm_en_v_pin );
    LL_GPIO_SetOutputPin( pH->pParams_str->pwm_en_w_port, pH->pParams_str->pwm_en_w_pin );
  }

  return;
}


/**
* @brief  It enables PWM generation on the proper Timer peripheral acting on MOE
*         bit
* @param  ICS L4xx PWM Current Feedback Handle
* @retval none
*/
void IL4XX_SwitchOnPWM( PWMC_Handle_t * pHandle )
{
  PWMC_ICS_L4_Handle_t * pH = ( PWMC_ICS_L4_Handle_t * ) pHandle;
  TIM_TypeDef * TIMx = pH->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pH->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pH->pParams_str->ADCx_2;

  pH->_Super.bTurnOnLowSidesAction = false;

  /* It clears ADCs JSTRT and JEOC bits */
  LL_ADC_ClearFlag_JEOC( ADCx_1 );
  LL_ADC_ClearFlag_JEOC( ADCx_2 );

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Enable TIMx preload and ADC trigger on next update */
  TIMx->CCMR2 = 0x7868u;
  TIMx->CCR4 = ( uint32_t )( pH->Half_PWMPeriod ) - 5u;

  /* Main PWM Output ensable */
  LL_TIM_EnableAllOutputs( TIMx );
  if ( ( pH->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pH->pParams_str->pwm_en_u_port, pH->pParams_str->pwm_en_u_pin );
    LL_GPIO_SetOutputPin( pH->pParams_str->pwm_en_v_port, pH->pParams_str->pwm_en_v_pin );
    LL_GPIO_SetOutputPin( pH->pParams_str->pwm_en_w_port, pH->pParams_str->pwm_en_w_pin );
  }
  pH->hFlags &= ( ~SOFOC );

  return;
}


/**
* @brief  It disables PWM generation on the proper Timer peripheral acting on
*         MOE bit
* @param  ICS L4xx PWM Current Feedback Handle
* @retval none
*/
void IL4XX_SwitchOffPWM( PWMC_Handle_t * pHandle )
{
  PWMC_ICS_L4_Handle_t * pH = ( PWMC_ICS_L4_Handle_t * ) pHandle;
  TIM_TypeDef * TIMx = pH->pParams_str->TIMx;

  pH->_Super.bTurnOnLowSidesAction = false;

  LL_TIM_CC_DisableChannel( TIMx, TIMxCCER_MASK_CH123 );

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == RESET )
  {
    if ( LL_TIM_IsEnabledIT_UPDATE( TIMx ) )
    {
      break;
    }
  }

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs( TIMx );
  if ( ( pH->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_ResetOutputPin( pH->pParams_str->pwm_en_u_port, pH->pParams_str->pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pH->pParams_str->pwm_en_v_port, pH->pParams_str->pwm_en_v_pin );
    LL_GPIO_ResetOutputPin( pH->pParams_str->pwm_en_w_port, pH->pParams_str->pwm_en_w_pin );
  }
  LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

  TIMx->CCMR2 = 0x7068u;
  TIMx->CCR4 = ( uint32_t )( pH->Half_PWMPeriod ) + 1u;

  return;
}

/**
* @brief  It stores into the component instance's handle the voltage present on Ia and
*         Ib current feedback analog channels when no current is flowin into the
*         motor
* @param pHandle ICS L4xx PWM Current Feedback Handle
*/
uint16_t IL4XX_WriteTIMRegisters( PWMC_Handle_t * pHandle )
{
  uint16_t aux;
  PWMC_ICS_L4_Handle_t * pH = ( PWMC_ICS_L4_Handle_t * ) pHandle;
  TIM_TypeDef * TIMx = pH->pParams_str->TIMx;

  TIMx->CCR1 = pH->_Super.hCntPhA;
  TIMx->CCR2 = pH->_Super.hCntPhB;
  TIMx->CCR3 = pH->_Super.hCntPhC;

  /* Disable TIMx preload */
  TIMx->CCMR2 = 0x7068u;
  TIMx->CCR4 = ( uint32_t )( pH->Half_PWMPeriod ) + 1u;
  /* Enable TIMx preload */
  TIMx->CCMR2 = 0x7868u;
  TIMx->CCR4 = ( uint32_t )( pH->Half_PWMPeriod ) - 5u;

  /* Limit for update event */
  /* Check the status of SOFOC flag. If it is set, an update event has occurred
  and thus the FOC rate is too high */

  if ( ( pH->hFlags & SOFOC ) != 0u )
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
  * @brief  It contains the TIMx Update event interrupt
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void * IL4XX_TIMx_UP_IRQHandler( PWMC_ICS_L4_Handle_t * pHandle )
{
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  /* Set the SOFOC flag to indicate the execution of Update IRQ*/
  pHandle->hFlags |= SOFOC;

  /* Switch Context */
  /* It re-initilize AD converter in run time when using dual MC */
  /* Removed because the trigger is set writing directly the JSQR */

  /* Change channels keeping equal to 1 element the sequencer lenght */
  ADCx_1->JSQR = ( uint32_t )( pHandle->pParams_str->bIaChannel ) << 8 | LL_ADC_INJ_TRIG_EXT_RISING;
  ADCx_2->JSQR = ( uint32_t )( pHandle->pParams_str->bIbChannel ) << 8 | LL_ADC_INJ_TRIG_EXT_RISING;

  return &( pHandle->_Super.bMotor );
}

/**
  * @brief  It contains the TIMx Break1 event interrupt
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void * IL4XX_BRK_IRQHandler( PWMC_ICS_L4_Handle_t * pHandle )
{
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  pHandle->OverVoltageFlag = true;
  pHandle->BrakeActionLock = true;

  return &( pHandle->_Super.bMotor );
}

/**
  * @brief  It contains the TIMx Break2 event interrupt
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void * IL4XX_BRK2_IRQHandler( PWMC_ICS_L4_Handle_t * pHandle )
{
  if ( pHandle->BrakeActionLock == false )
  {
    if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
    {
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
  }
  pHandle->OverCurrentFlag = true;

  return &( pHandle->_Super.bMotor );
}

/**
* @brief  It is used to check if an overcurrent occurred since last call.
* @param  ICS L4xx PWM Current Feedback Handle
* @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
*                  detected since last method call, MC_NO_FAULTS otherwise.
*/
uint16_t IL4XX_IsOverCurrentOccurred( PWMC_Handle_t * pHandle )
{
  uint16_t retVal = MC_NO_FAULTS;
  PWMC_ICS_L4_Handle_t * pH = ( PWMC_ICS_L4_Handle_t * )pHandle;

  if ( pH->OverVoltageFlag == true )
  {
    retVal = MC_OVER_VOLT;
    pH->OverVoltageFlag = false;
  }

  if ( pH->OverCurrentFlag == true )
  {
    retVal |= MC_BREAK_IN;
    pH->OverCurrentFlag = false;
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
