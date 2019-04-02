/**
  ******************************************************************************
  * @file    speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the  features
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
  */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup SpeednPosFdbk Speed & Position Feedback
 *
  * @brief Speed & Position Feedback components of the Motor Control SDK
  *
  * These components provide the speed and the angular position of the rotor of a motor (both
  * electrical and mechanical).
  *
  * Several implementations of the Speed and Position Feedback feature are provided by the Motor
  * to account for the specificities of the motor used on the application:
  *
  * - @ref hall_speed_pos_fdbk "Hall Speed & Position Feedback" for motors with Hall effect sensors
  * - @ref Encoder  "Encoder Speed & Position Feedback" for motors with a quadrature encoder
  * - two general purpose sensorless implementations are provided:
  *   @ref SpeednPosFdbk_STO "State Observer with PLL" and
  *   @ref STO_CORDIC_SpeednPosFdbk "State Observer with CORDIC"
  * - "High Frequency Injection" for anisotropic I-PMSM motors (Not included in this release).
  *
  * @{
  */

/**
  * @brief  It returns the last computed rotor electrical angle, expressed in
  *         s16degrees. 1 s16degree = 360ï¿½/65536
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor electrical angle (s16degrees)
  */
int16_t SPD_GetElAngle( SpeednPosFdbk_Handle_t * pHandle )
{
  return ( pHandle->hElAngle );
}

/**
  * @brief  It returns the last computed rotor mechanical angle, expressed in
  *         s16degrees. Mechanical angle frame is based on parameter bElToMecRatio
  *         and, if occasionally provided - through function SPD_SetMecAngle -
  *         of a measured mechanical angle, on information computed thereof.
  * @note   both Hall sensor and Sensor-less do not implement either
  *         mechanical angle computation or acceleration computation.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor mechanical angle (s16degrees)
  */
int16_t SPD_GetMecAngle( SpeednPosFdbk_Handle_t * pHandle )
{
  return ( pHandle->hMecAngle );
}

/**
  * @brief  It returns the last computed average mechanical speed, expressed in
  *         01Hz (tenth of Hertz).
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor average mechanical speed (01Hz)
  */
int16_t SPD_GetAvrgMecSpeed01Hz( SpeednPosFdbk_Handle_t * pHandle )
{
  return ( pHandle->hAvrMecSpeed01Hz );
}

/**
  * @brief  It returns the last computed electrical speed, expressed in Dpp.
  *         1 Dpp = 1 s16Degree/control Period. The control period is the period
  *         on which the rotor electrical angle is computed (through function
  *         SPD_CalcElectricalAngle).
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor electrical speed (Dpp)
  */
int16_t SPD_GetElSpeedDpp( SpeednPosFdbk_Handle_t * pHandle )
{
  return ( pHandle->hElSpeedDpp );
}

/**
  * @brief  It returns the result of the last reliability check performed.
  *         Reliability is measured with reference to parameters
  *         hMaxReliableElSpeed01Hz, hMinReliableElSpeed01Hz,
  *         bMaximumSpeedErrorsNumber and/or specific parameters of the derived
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval bool sensor reliability state
  */
bool SPD_Check( SpeednPosFdbk_Handle_t * pHandle )
{
  bool SpeedSensorReliability = true;
  if ( pHandle->bSpeedErrorNumber ==
       pHandle->bMaximumSpeedErrorsNumber )
  {
    SpeedSensorReliability = false;
  }
  return ( SpeedSensorReliability );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and returns - through
  *         parameter pMecSpeed01Hz - the rotor average mechanical speed,
  *         expressed in 01Hz. It computes and returns the reliability state of
  *         the sensor; reliability is measured with reference to parameters
  *         hMaxReliableElSpeed01Hz, hMinReliableElSpeed01Hz,
  *         bMaximumSpeedErrorsNumber and/or specific parameters of the derived
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @param  pMecSpeed01Hz pointer to int16_t, used to return the rotor average
  *         mechanical speed (01Hz)
  * @retval none
  */
bool SPD_IsMecSpeedReliable( SpeednPosFdbk_Handle_t * pHandle, int16_t * pMecSpeed01Hz )
{
  bool SpeedSensorReliability = true;
  uint8_t bSpeedErrorNumber;
  uint8_t bMaximumSpeedErrorsNumber = pHandle->bMaximumSpeedErrorsNumber;

  bool SpeedError = false;
  uint16_t hAbsMecSpeed01Hz, hAbsMecAccel01HzP;
  int16_t hAux;

  bSpeedErrorNumber = pHandle->bSpeedErrorNumber;

  /* Compute absoulte value of mechanical speed */
  if ( *pMecSpeed01Hz < 0 )
  {
    hAux = -( *pMecSpeed01Hz );
    hAbsMecSpeed01Hz = ( uint16_t )( hAux );
  }
  else
  {
    hAbsMecSpeed01Hz = ( uint16_t )( *pMecSpeed01Hz );
  }

  if ( hAbsMecSpeed01Hz > pHandle->hMaxReliableMecSpeed01Hz )
  {
    SpeedError = true;
  }

  if ( hAbsMecSpeed01Hz < pHandle->hMinReliableMecSpeed01Hz )
  {
    SpeedError = true;
  }

  /* Compute absoulte value of mechanical acceleration */
  if ( pHandle->hMecAccel01HzP < 0 )
  {
    hAux = -( pHandle->hMecAccel01HzP );
    hAbsMecAccel01HzP = ( uint16_t )( hAux );
  }
  else
  {
    hAbsMecAccel01HzP = ( uint16_t )( pHandle->hMecAccel01HzP );
  }

  if ( hAbsMecAccel01HzP > pHandle->hMaxReliableMecAccel01HzP )
  {
    SpeedError = true;
  }

  if ( SpeedError == true )
  {
    if ( bSpeedErrorNumber < bMaximumSpeedErrorsNumber )
    {
      bSpeedErrorNumber++;
    }
  }
  else
  {
    if ( bSpeedErrorNumber < bMaximumSpeedErrorsNumber )
    {
      bSpeedErrorNumber = 0u;
    }
  }

  if ( bSpeedErrorNumber == bMaximumSpeedErrorsNumber )
  {
    SpeedSensorReliability = false;
  }

  pHandle->bSpeedErrorNumber = bSpeedErrorNumber;

  return ( SpeedSensorReliability );
}

/**
  * @brief  This method returns the average mechanical rotor speed expressed in
  *         "S16Speed". It means that:\n
  *         - it is zero for zero speed,\n
  *         - it become INT16_MAX when the average mechanical speed is equal to
  *           hMaxReliableMecSpeed01Hz,\n
  *         - it becomes -INT16_MAX when the average mechanical speed is equal to
  *         -hMaxReliableMecSpeed01Hz.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t The average mechanical rotor speed expressed in "S16Speed".
  */
int16_t SPD_GetS16Speed( SpeednPosFdbk_Handle_t * pHandle )
{
  int32_t wAux = ( int32_t ) pHandle->hAvrMecSpeed01Hz;
  wAux *= INT16_MAX;
  wAux /= ( int16_t ) pHandle->hMaxReliableMecSpeed01Hz;
  return ( int16_t )wAux;
}

/**
  * @brief  This method returns the coefficient used to transform electrical to
  *         mechanical quantities and viceversa. It usually coincides with motor
  *         pole pairs number.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval uint8_t The motor pole pairs number.
  */
uint8_t SPD_GetElToMecRatio( SpeednPosFdbk_Handle_t * pHandle )
{
  return ( pHandle->bElToMecRatio );
}

/**
  * @brief  This method sets the coefficient used to transform electrical to
  *         mechanical quantities and viceversa. It usually coincides with motor
  *         pole pairs number.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @param  bPP The motor pole pairs number to be set.
  */
void SPD_SetElToMecRatio( SpeednPosFdbk_Handle_t * pHandle, uint8_t bPP )
{
  pHandle->bElToMecRatio = bPP;
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
