/**
  ******************************************************************************
  * @file    r3_lm1_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          r3_lm1_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup r3_lm1_pwm_curr_fdbk
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __R3_LM1_PWM_CURR_FDBK_H
#define __R3_LM1_PWM_CURR_FDBK_H

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"
#include "r3_dd_pwm_curr_fdbk.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup r3_lm1_pwm_curr_fdbk
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/**
  * @brief The PWMC_R3_LM1_Handle_t structure defines the handle of PWM & Current Feedback
  *        component designed for STM32F103 Low & Medium Density with 3 shunt current sensing topology.
  *
  * The design of the PWMC_R3_LM1 component is based on that of the PWMC generic component.
  */
typedef struct
{
  PWMC_Handle_t _Super;        /**< @brief The handle on the base PWMC component. */

  uint32_t wPhaseAOffset;   /**< @brief Offset of Phase A current sensing network  */
  uint32_t wPhaseBOffset;   /**< @brief Offset of Phase B current sensing network  */
  uint32_t wPhaseCOffset;   /**< @brief Offset of Phase C current sensing network  */
  uint16_t Half_PWMPeriod;  /**< @brief Half PWM Period in timer clock counts */
  uint16_t hRegConv;        /**< @brief Variable used to store regular conversions
                              *  result*/

  bool OverCurrentFlag;     /*!< This flag is set when an over current occurs.*/

  R3_DDParams_t const * pParams_str; /**< @brief Pointer on the parameters structure for the PWMC R3 LM1 component.
                                 *
                                 * Mostly contains HW related parameters. These HW only params will be
                                 * removed at cubification time. Other parameters will be reintegrated
                                 * into the Handle structure */
} PWMC_R3_LM1_Handle_t;

/* Exported functions ------------------------------------------------------- */

/* Initializes a PWMC_R3_LM1 component */
void R3LM1_Init( PWMC_R3_LM1_Handle_t * pHandle );

/* Calibrates current reading for a PWMC_R3_LM1 component */
void R3LM1_CurrentReadingCalibration( PWMC_Handle_t * pHandle );

/* Returns the last phase currents values measured*/
void R3LM1_GetPhaseCurrents( PWMC_Handle_t * pHandle,Curr_Components* pStator_Currents);

/* Turns the low side switches on */
void R3LM1_TurnOnLowSides( PWMC_Handle_t * pHandle );

/* Switches on PWM generation */
void R3LM1_SwitchOnPWM( PWMC_Handle_t * pHandle );

/* Switches off PWM generation */
void R3LM1_SwitchOffPWM( PWMC_Handle_t * pHandle );

/* Sets the ADC sampling point for sector 1 */
uint16_t R3LM1_SetADCSampPointSect1( PWMC_Handle_t * pHandle );

/* Sets the ADC sampling point for sector 2 */
uint16_t R3LM1_SetADCSampPointSect2( PWMC_Handle_t * pHandle );

/* Sets the ADC sampling point for sector 3 */
uint16_t R3LM1_SetADCSampPointSect3( PWMC_Handle_t * pHandle );

/* Sets the ADC sampling point for sector 4 */
uint16_t R3LM1_SetADCSampPointSect4( PWMC_Handle_t * pHandle );

/* Sets the ADC sampling point for sector 5 */
uint16_t R3LM1_SetADCSampPointSect5( PWMC_Handle_t * pHandle );

/* Sets the ADC sampling point for sector 6 */
uint16_t R3LM1_SetADCSampPointSect6( PWMC_Handle_t * pHandle );

/* @brief  It contains the Break event interrupt */
void *R3LM1_BRK_IRQHandler(PWMC_R3_LM1_Handle_t *pHandle);

/* Returns whether an over current condition has occurred */
uint16_t R3LM1_IsOverCurrentOccurred( PWMC_Handle_t * pHandle );

/**
  * @}
  */
  
/**
  * @}
  */

/** @} */

#endif /*__R3_LM1_PWM_CURR_FDBK_H */
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
