/**
  ******************************************************************************
  * @file    r3_hd2_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          r3_hd2_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup r3_hd2_pwm_curr_fdbk
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __R3_HD2_PWM_CURR_FDBK_H
#define __R3_HD2_PWM_CURR_FDBK_H

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

/** @addtogroup r3_hd2_pwm_curr_fdbk
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/**
  * @brief The PWMC_R3_HD2_Handle_t structure defines the handle of PWM & Current Feedback
  *        component designed for STM32F103 High Density with 3 shunt current sensing topology.
  *
  * The design of the PWMC_R3_HD2 component is based on that of the PWMC generic component.
  */
typedef struct
{
  PWMC_Handle_t _Super;     /**< @brief The handle on the base PWMC component. */

  volatile uint32_t wPhaseAOffset;   /**< @brief Offset of Phase A current sensing network  */
  volatile uint32_t wPhaseBOffset;   /**< @brief Offset of Phase B current sensing network  */
  volatile uint32_t wPhaseCOffset;   /**< @brief Offset of Phase C current sensing network  */
  uint32_t wADC1Channel;    /**< @brief ADC1 programmed channel for motor current
                              *  sampling */
  uint32_t wADC2Channel;    /**< @brief ADC2 programmed channel for motor current
                              *  sampling */
  uint16_t Half_PWMPeriod;  /**< @brief Half PWM Period in timer clock counts */
  uint16_t hRegConv;        /**< @brief Variable used to store regular conversions
                              *  result*/
  volatile uint8_t bSoFOC;  /**< @brief This flag is reset at the beginning of FOC
                              *  and it is set in the TIM UP IRQ. If at the end of
                              *  FOC this flag is set, it means that FOC rate is too
                              *  high and thus an error is generated */
  volatile uint8_t  bIndex; /**< @brief Conversion index */
  volatile uint32_t wADCTriggerSet;  /**< @brief Store the value for ADC CR2 to proper configure
                              *  current sampling during the context switching*/
  volatile uint32_t wADCTriggerUnSet;/**< @brief Store the value for ADC CR2 to disable the
                              *  current sampling during the context switching*/
  uint32_t wTIMxCH4_BB_Addr;/**< @brief Store the bit-banding address to activate/deactivate
                              *  TIMx CH4 channel */
  bool OverCurrentFlag;       /*!< This flag is set when an over current occurs.*/

  R3_DDParams_t const * pParams_str; /**< @brief Pointer on the parameters structure for the PWMC R3 HD2 component.
                                *
                                * Mostly contains HW related parameters. These HW only params will be
                                * removed at cubification time. Other parameters will be reintegrated
                                * into the Handle structure */
} PWMC_R3_HD2_Handle_t;


/* Exported functions ------------------------------------------------------- */

/* Initializes a PWMC_R3_HD2 component */
void R3HD2_Init( PWMC_R3_HD2_Handle_t * pHandle );

/* Calibrates current reading for a PWMC_R1_HD2 component */
void R3HD2_CurrentReadingCalibration( PWMC_Handle_t * pHandle );

/* Returns the last phase currents values measured*/
void R3HD2_GetPhaseCurrents( PWMC_Handle_t * pHandle,Curr_Components* pStator_Currents);

/* Turns the low side switches on */
void R3HD2_TurnOnLowSides( PWMC_Handle_t * pHandle );

/* Switches on PWM generation */
void R3HD2_SwitchOnPWM( PWMC_Handle_t * pHandle );

/* Switches off PWM generation */
void R3HD2_SwitchOffPWM( PWMC_Handle_t * pHandle );

/* Sets the ADC sampling point for sector 1 */
uint16_t R3HD2_SetADCSampPointSect1( PWMC_Handle_t * pHandle );

/* Sets the ADC sampling point for sector 2 */
uint16_t R3HD2_SetADCSampPointSect2( PWMC_Handle_t * pHandle );

/* Sets the ADC sampling point for sector 3 */
uint16_t R3HD2_SetADCSampPointSect3( PWMC_Handle_t * pHandle );

/* Sets the ADC sampling point for sector 4 */
uint16_t R3HD2_SetADCSampPointSect4( PWMC_Handle_t * pHandle );

/* Sets the ADC sampling point for sector 5 */
uint16_t R3HD2_SetADCSampPointSect5( PWMC_Handle_t * pHandle );

/* Sets the ADC sampling point for sector 6 */
uint16_t R3HD2_SetADCSampPointSect6( PWMC_Handle_t * pHandle );

/* Handles the Timer Interrupts of a PWMC_R1_HD2 component */
void *R3HD2_TIMx_UP_IRQHandler( PWMC_R3_HD2_Handle_t * pHandle);

/* @brief  It contains the Break event interrupt */
void *R3HD2_BRK_IRQHandler(PWMC_R3_HD2_Handle_t *pHdl);

/* Returns whether an over current condition has occurred */
uint16_t R3HD2_IsOverCurrentOccurred( PWMC_Handle_t * pHandle );


/**
  * @}
  */

/**
  * @}
  */

/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__R3_HD2_PWM_CURR_FDBK_H */
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
