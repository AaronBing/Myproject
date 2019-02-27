/**
******************************************************************************
* @file    ics_hd2_pwm_curr_fdbk.h
* @author  Motor Control SDK Team, ST Microelectronics
* @brief   This file contains all definitions and functions prototypes for the
*          ICS HD2 PWM current feedback component for F1xx of the Motor Control SDK.
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
  * @ingroup ics_hd2_pwm_curr_fdbk
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ICS_HD2_PWM_CURR_FDBK_H
#define __ICS_HD2_PWM_CURR_FDBK_H

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"
#include "ics_dd_pwmncurrfdbk.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup ics_hd2_pwm_curr_fdbk
  * @{
  */

#define SOFOC 0x0008u /**< This flag is reset at the beginning of FOC
                           and it is set in the TIM UP IRQ. If at the end of
                           FOC this flag is set, it means that FOC rate is too
                           high and thus an error is generated */

                         
/**
* @brief  PWMnCurrFdbk_ICS_HD2 handle
*/

typedef struct
{
  PWMC_Handle_t _Super;     /**< PWMC handler */
  uint32_t wPhaseAOffset;   /**< Offset of Phase A current sensing network  */
  uint32_t wPhaseBOffset;   /**< Offset of Phase B current sensing network  */
  uint16_t Half_PWMPeriod;     /**< Half PWM Period in timer clock counts */
  uint16_t hRegConv;          /**< Variable used to store regular conversions
                              result*/
  volatile uint16_t hFlags;   /**< Variable containing private flags */
  volatile uint8_t  bIndex; 

  bool OverCurrentFlag;       /*!< This flag is set when an over current occurs.*/

  ICS_DDParams_t const * pParams_str;

} PWMC_ICS_HD2_Handle_t;


/* Exported functions ------------------------------------------------------- */

void IHD2_Init(PWMC_ICS_HD2_Handle_t *pHandle);

void IHD2_TIMxInit(TIM_TypeDef* TIMx, PWMC_ICS_HD2_Handle_t *pHandle);

void IHD2_CurrentReadingCalibration( PWMC_Handle_t *pHandle );

void IHD2_GetPhaseCurrents(PWMC_Handle_t *pHandle, Curr_Components* pStator_Currents);

void IHD2_TurnOnLowSides(PWMC_Handle_t *pHandle);

void IHD2_SwitchOnPWM(PWMC_Handle_t *pHandle);

void IHD2_SwitchOffPWM(PWMC_Handle_t *pHandle);

uint16_t IHD2_WriteTIMRegisters(PWMC_Handle_t *pHandle);

void *IHD2_IRQHandler(PWMC_ICS_HD2_Handle_t *pHandle);

void IHD2_HFCurrentsCalibration(PWMC_Handle_t *pHandle, Curr_Components* pStator_Currents);
/* @brief  It contains the Break event interrupt */
void *IHD2_BRK_IRQHandler(PWMC_ICS_HD2_Handle_t *pHandle);


uint16_t IHD2_IsOverCurrentOccurred(PWMC_Handle_t *pHandle);

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

#endif /*__ICS_HD2_PWM_CURR_FDBK_H*/

 /************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
