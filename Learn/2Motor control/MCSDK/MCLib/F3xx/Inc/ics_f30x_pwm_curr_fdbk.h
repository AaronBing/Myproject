/**
  ******************************************************************************
  * @file    ics_f30x_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          ICS PWM current feedback component for F30x of the Motor Control SDK.
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
  * @ingroup ics_f30x_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ICS_F30X_PWMCURRFDBK_H
#define __ICS_F30X_PWMCURRFDBK_H

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup ics_f30x_pwm_curr_fdbk
  * @{
  */

#define NONE    ((uint8_t)(0x00))
#define EXT_MODE  ((uint8_t)(0x01))
#define INT_MODE  ((uint8_t)(0x02))
#define SOFOC 0x0008u /**< This flag is reset at the beginning of FOC
                           and it is set in the TIM UP IRQ. If at the end of
                           FOC this flag is set, it means that FOC rate is too 
                           high and thus an error is generated */


/**
  * @brief  ICS_F30X component parameters definition
  */
typedef struct
{
  /* Dual MC parameters --------------------------------------------------------*/
  uint8_t  bFreqRatio;               /*!< It is used in case of dual MC to
                                             synchronize TIM1 and TIM8. It has
                                             effect only on the second instanced
                                             object and must be equal to the
                                             ratio between the two PWM frequencies
                                             (higher/lower). Supported values are
                                             1, 2 or 3 */
  uint8_t  bIsHigherFreqTim;         /*!< When bFreqRatio is greather than 1
                                            this param is used to indicate if this
                                            instance is the one with the highest
                                            frequency. Allowed value are: HIGHER_FREQ
                                            or LOWER_FREQ */


  /* Current reading A/D Conversions initialization -----------------------------*/
  ADC_TypeDef * ADCx_1;                 /*!< It contains the pointer to the first ADC
                                              used for current reading. */
  ADC_TypeDef * ADCx_2;                 /*!< It contains the pointer to the second ADC
                                              used for current reading. */
  uint8_t bIaChannel;                  /*!< ADC channel used for conversion of
                                              current Ia. It must be equal to
                                              ADC_CHANNEL_x x= 0, ..., 15*/
  uint8_t bIbChannel;                  /*!< ADC channel used for conversion of
                                              current Ib. It must be equal to
                                              ADC_CHANNEL_x x= 0, ..., 15*/
  /* PWM generation parameters --------------------------------------------------*/
  uint8_t  bRepetitionCounter;          /*!< It expresses the number of PWM
                                               periods to be elapsed before compare
                                               registers are updated again. In
                                               particular:
                                               RepetitionCounter= (2* #PWM periods)-1 */
  TIM_TypeDef * TIMx;                   /*!< It contains the pointer to the timer
                                               used for PWM generation. It must
                                               equal to TIM1 if bInstanceNbr is
                                               equal to 1, to TIM8 otherwise */
  /* PWM Driving signals initialization ----------------------------------------*/

  LowSideOutputsFunction_t LowSideOutputs; /*!< Low side or enabling signals
                                                  generation method are defined
                                                  here.*/
  GPIO_TypeDef * pwm_en_u_port;                 /*!< Channel 1N (low side) GPIO output
                                                 port (if used, after re-mapping).
                                                 It must be GPIOx x= A, B, ...*/
  uint16_t pwm_en_u_pin;                       /*!< Channel 1N (low side) GPIO output pin
                                                 (if used, after re-mapping). It must be
                                                 GPIO_Pin_x x= 0, 1, ...*/
  GPIO_TypeDef * pwm_en_v_port;                 /*!< Channel 2N (low side) GPIO output
                                                 port (if used, after re-mapping).
                                                 It must be GPIOx x= A, B, ...*/
  uint16_t pwm_en_v_pin;                       /*!< Channel 2N (low side) GPIO output pin
                                                 (if used, after re-mapping). It must be
                                                 GPIO_Pin_x x= 0, 1, ...*/
  GPIO_TypeDef * pwm_en_w_port;                 /*!< Channel 3N (low side)  GPIO output
                                                 port (if used, after re-mapping).
                                                 It must be GPIOx x= A, B, ...*/
  uint16_t pwm_en_w_pin;                       /*!< Channel 3N (low side)  GPIO output pin
                                                  (if used, after re-mapping). It must be
                                                  GPIO_Pin_x x= 0, 1, ...*/
  /* Emergency input (BKIN2) signal initialization -----------------------------*/
  uint8_t bBKIN2Mode;                 /*!< It defines the modality of emergency
                                             input 2. It must be any of the
                                             the following:
                                             NONE - feature disabled.
                                             INT_MODE - Internal comparator used
                                             as source of emergency event.
                                             EXT_MODE - External comparator used
                                             as source of emergency event.*/

} ICS_F30XParams_t;


/**
  * @brief  PWMnCurrFdbk ICS F30X handle
  */
typedef struct
{
  PWMC_Handle_t _Super;     /**< PWMC handler */
  uint32_t wPhaseAOffset;   /**< Offset of Phase A current sensing network  */
  uint32_t wPhaseBOffset;   /**< Offset of Phase B current sensing network  */
  uint16_t Half_PWMPeriod;     /**< Half PWM Period in timer clock counts */
  volatile uint16_t hFlags;   /**< Variable containing private flags */
  volatile uint8_t  bIndex;
  uint32_t wADCTriggerSet;  /**< Store the value for ADC CR2 to proper configure
                                 current sampling during the context switching*/
  uint32_t wADCTriggerUnSet;/**< Store the value for ADC CR2 to disable the
                                 current sampling during the context switching*/

  bool OverCurrentFlag;     /*!< This flag is set when an overcurrent occurs.*/
  bool OverVoltageFlag;     /*!< This flag is set when an overvoltage occurs.*/
  bool BrakeActionLock;     /*!< This flag is set to avoid that brake action is
                                 interrupted.*/
  ICS_F30XParams_t const * pParams_str; /*! TODO */
} PWMC_ICS_F3_Handle_t;



/* Exported functions ------------------------------------------------------- */

void IF3XX_Init( PWMC_ICS_F3_Handle_t * pHandle );

void IF3XX_CurrentReadingCalibration( PWMC_Handle_t * pHandle );

void IF3XX_GetPhaseCurrents( PWMC_Handle_t * pHandle, Curr_Components * pStator_Currents );

void IF3XX_TurnOnLowSides( PWMC_Handle_t * pHandle );

void IF3XX_SwitchOnPWM( PWMC_Handle_t * pHandle );

void IF3XX_SwitchOffPWM( PWMC_Handle_t * pHandle );

uint16_t IF3XX_WriteTIMRegisters( PWMC_Handle_t * pHandle );

uint16_t IF3XX_ExecRegularConv( PWMC_Handle_t * pHandle, uint8_t Channel );

void IF3XX_ADC_SetSamplingTime( PWMC_Handle_t * pHandle, ADConv_t ADConv_struct );

void IF3XX_HFCurrentsCalibration( PWMC_Handle_t * pHandle, Curr_Components * pStator_Currents );

uint16_t IF3XX_IsOverCurrentOccurred( PWMC_Handle_t * pHandle );

void * IF3XX_TIMx_UP_IRQHandler( PWMC_ICS_F3_Handle_t * pHandle );

void * IF3XX_BRK_IRQHandler( PWMC_ICS_F3_Handle_t * pHandle );

void * IF3XX_BRK2_IRQHandler( PWMC_ICS_F3_Handle_t * pHandle );

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__ICS_F30X_PWMCURRFDBK_H*/

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
