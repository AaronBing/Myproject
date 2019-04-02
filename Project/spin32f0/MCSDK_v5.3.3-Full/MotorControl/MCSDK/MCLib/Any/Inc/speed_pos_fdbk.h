/**
 ******************************************************************************
 * @file    speed_pos_fdbk.h
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file provides all definitions and functions prototypes
 *          of the Speed & Position Feedback component of the Motor Control SDK.
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
 * @ingroup SpeednPosFdbk
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPEEDNPOSFDBK_H
#define __SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  SpeednPosFdbk  handle definition
  */
typedef struct
{

  int16_t hElAngle;

  int16_t hMecAngle;

  int16_t hAvrMecSpeed01Hz;

  int16_t hElSpeedDpp;

  int16_t hMecAccel01HzP;

  uint8_t bSpeedErrorNumber;

  uint8_t bElToMecRatio;  /*!< Coefficient used to transform electrical to
                               mechanical quantities and viceversa. It usually
                               coincides with motor pole pairs number*/
  uint16_t hMaxReliableMecSpeed01Hz; /*!< Maximum value of measured speed that is
                                        considered to be valid. It's expressed
                                        in tenth of mechanical Hertz.*/
  uint16_t hMinReliableMecSpeed01Hz; /*!< Minimum value of measured speed that is
                                        considered to be valid. It's expressed
                                        in tenth of mechanical Hertz.*/
  uint8_t bMaximumSpeedErrorsNumber; /*!< Maximum value of not valid measurements
                                        before an error is reported.*/
  uint16_t hMaxReliableMecAccel01HzP; /*!< Maximum value of measured acceleration
                                        that is considered to be valid. It's
                                        expressed in 01HzP (tenth of Hertz per
                                        speed calculation period)*/
  uint16_t hMeasurementFrequency;  /*!< Frequency on which the user will request
                                    a measurement of the rotor electrical angle.
                                    It's also used to convert measured speed from
                                    tenth of Hz to dpp and viceversa.*/

} SpeednPosFdbk_Handle_t;

/**
  * @brief input structure type definition for SPD_CalcAngle
  */
typedef struct
{
  Volt_Components  Valfa_beta;
  Curr_Components  Ialfa_beta;
  uint16_t         Vbus;
} Observer_Inputs_t;


int16_t SPD_GetElAngle( SpeednPosFdbk_Handle_t * pHandle );

int16_t SPD_GetMecAngle( SpeednPosFdbk_Handle_t * pHandle );

int16_t SPD_GetAvrgMecSpeed01Hz( SpeednPosFdbk_Handle_t * pHandle );

int16_t SPD_GetElSpeedDpp( SpeednPosFdbk_Handle_t * pHandle );

bool SPD_Check( SpeednPosFdbk_Handle_t * pHandle );

bool SPD_IsMecSpeedReliable( SpeednPosFdbk_Handle_t * pHandle, int16_t * pMecSpeed01Hz );

int16_t SPD_GetS16Speed( SpeednPosFdbk_Handle_t * pHandle );

uint8_t SPD_GetElToMecRatio( SpeednPosFdbk_Handle_t * pHandle );

void SPD_SetElToMecRatio( SpeednPosFdbk_Handle_t * pHandle, uint8_t bPP );

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __SPEEDNPOSFDBK_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
