/**
  ******************************************************************************
  * @file    sto_cordic_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the State Observer + CORDIC Speed & Position Feedback component of the
  *          Motor Control SDK.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sto_cordic_speed_pos_fdbk.h"
#include "mc_math.h"

#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @defgroup STO_CORDIC_SpeednPosFdbk State Observer CORDIC Speed & Position Feedback
  * @brief State Observer with CORDIC Speed & Position Feedback implementation
  *
  * This component uses a State Observer coupled with a CORDIC (COordinate Rotation DIgital
  * Computer) to provide an estimation of the speed and the position of the rotor of the motor.
  *
  * @todo Document the State Observer + CORDIC Speed & Position Feedback "module".
  *
  * @{
  */

/* Private defines -----------------------------------------------------------*/

#define C6_COMP_CONST1  (int32_t) 1043038
#define C6_COMP_CONST2  (int32_t) 10430

#define ATAN1DIV1     (int16_t)8192
#define ATAN1DIV2     (int16_t)4836
#define ATAN1DIV4     (int16_t)2555
#define ATAN1DIV8     (int16_t)1297
#define ATAN1DIV16    (int16_t)651
#define ATAN1DIV32    (int16_t)326
#define ATAN1DIV64    (int16_t)163
#define ATAN1DIV128   (int16_t)81
#define ATAN1DIV256   (int16_t)41
#define ATAN1DIV512   (int16_t)20
#define ATAN1DIV1024  (int16_t)10
#define ATAN1DIV2048  (int16_t)5
#define ATAN1DIV4096  (int16_t)3
#define ATAN1DIV8192  (int16_t)1

/* Private functions prototypes ----------------------------------------------*/
static void STO_CR_Store_Rotor_Speed( STO_CR_Handle_t * pHandle, int16_t
                                      hRotor_Speed, int16_t hOrRotor_Speed );
static int16_t STO_CR_ExecuteCORDIC( STO_CR_Handle_t * pHandle,  int32_t
                                     wBemf_alfa_est, int32_t wBemf_beta_est );
static void STO_CR_InitSpeedBuffer( STO_CR_Handle_t * pHandle );

/**
  * @brief  It initializes the state observer object
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval none
  */
void STO_CR_Init( STO_CR_Handle_t * pHandle )
{
  int16_t htempk;
  int32_t wAux;

  pHandle->ConsistencyCounter = pHandle->StartUpConsistThreshold;
  pHandle->EnableDualCheck = true;

  wAux = ( int32_t )1;
  pHandle->F3POW2 = 0u;

  htempk = ( int16_t )( C6_COMP_CONST1 / ( pHandle->hF2 ) );

  while ( htempk != 0 )
  {
    htempk /= ( int16_t )2;
    wAux *= ( int32_t )2;
    pHandle->F3POW2++;
  }

  pHandle->hF3 = ( int16_t )wAux;
  wAux = ( int32_t )( pHandle->hF2 ) * pHandle->hF3;
  pHandle->hC6 = ( int16_t )( wAux / C6_COMP_CONST2 );

  STO_CR_Clear( pHandle );

  /* Acceleration measurement set to zero */
  pHandle->_Super.hMecAccel01HzP = 0;

  return;
}


/**
  * @brief  It only returns, necessary to implement fictitious IRQ_Handler
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @param  uint8_t Fictitious interrupt flag
  * @retval none
  */
void STO_CR_Return( STO_CR_Handle_t * pHandle, uint8_t flag )
{
  return;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This method executes Luenberger state observer equations and calls
  *         CORDIC with the purpose of computing a new speed estimation and
  *         updating the estimated electrical angle.
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  *         pInputs pointer to the observer inputs structure
  * @retval int16_t rotor electrical angle (s16Degrees)
  */
int16_t STO_CR_CalcElAngle( STO_CR_Handle_t * pHandle, Observer_Inputs_t * pInputs )
{

  int32_t wAux, wDirection;
  int32_t wAux_Alpha, wAux_Beta;
  int32_t wIalfa_est_Next, wIbeta_est_Next;
  int32_t wBemf_alfa_est_Next, wBemf_beta_est_Next;
  int16_t hAux, hAux_Alfa, hAux_Beta, hIalfa_err, hIbeta_err, hRotor_Speed,
          hOrRotor_Speed, hRotor_Acceleration, hRotor_Angle, hValfa, hVbeta;

  int16_t hPrev_Rotor_Angle = pHandle->_Super.hElAngle;
  int16_t hPrev_Rotor_Speed = pHandle->_Super.hElSpeedDpp;
  int16_t hMax_Instant_Accel = pHandle->MaxInstantElAcceleration;

  if ( pHandle->wBemf_alfa_est > ( int32_t )( pHandle->hF2 )*INT16_MAX )
  {
    pHandle->wBemf_alfa_est = INT16_MAX * ( int32_t )( pHandle->hF2 );
  }
  else if ( pHandle->wBemf_alfa_est <= -INT16_MAX * ( int32_t )( pHandle->hF2 ) )
  {
    pHandle->wBemf_alfa_est = -INT16_MAX * ( int32_t )( pHandle->hF2 );
  }
  else
  {
  }
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux_Alfa = ( int16_t )( pHandle->wBemf_alfa_est / pHandle->hF2 );
#else
  hAux_Alfa = ( int16_t )( pHandle->wBemf_alfa_est >> pHandle->F2LOG );
#endif

  if ( pHandle->wBemf_beta_est > INT16_MAX * ( int32_t )( pHandle->hF2 ) )
  {
    pHandle->wBemf_beta_est = INT16_MAX * ( int32_t )( pHandle->hF2 );
  }
  else if ( pHandle->wBemf_beta_est <= -INT16_MAX * ( int32_t )( pHandle->hF2 ) )
  {
    pHandle->wBemf_beta_est = -INT16_MAX * ( int32_t )( pHandle->hF2 );
  }
  else
  {
  }
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux_Beta = ( int16_t )( pHandle->wBemf_beta_est / pHandle->hF2 );
#else
  hAux_Beta = ( int16_t )( pHandle->wBemf_beta_est >> pHandle->F2LOG );
#endif

  if ( pHandle->Ialfa_est > INT16_MAX * ( int32_t )( pHandle->hF1 ) )
  {
    pHandle->Ialfa_est = INT16_MAX * ( int32_t )( pHandle->hF1 );
  }
  else if ( pHandle->Ialfa_est <= -INT16_MAX * ( int32_t )( pHandle->hF1 ) )
  {
    pHandle->Ialfa_est = -INT16_MAX * ( int32_t )( pHandle->hF1 );
  }
  else
  {
  }

  if ( pHandle->Ibeta_est > INT16_MAX * ( int32_t )( pHandle->hF1 ) )
  {
    pHandle->Ibeta_est = INT16_MAX * ( int32_t )( pHandle->hF1 );
  }
  else if ( pHandle->Ibeta_est <= -INT16_MAX * ( int32_t )( pHandle->hF1 ) )
  {
    pHandle->Ibeta_est = -INT16_MAX * ( int32_t )( pHandle->hF1 );
  }
  else
  {
  }

#ifdef FULL_MISRA_C_COMPLIANCY
  hIalfa_err = ( int16_t )( pHandle->Ialfa_est / pHandle->hF1 );
#else
  hIalfa_err = ( int16_t )( pHandle->Ialfa_est >> pHandle->F1LOG );
#endif

  hIalfa_err = hIalfa_err - pInputs->Ialfa_beta.qI_Component1;

#ifdef FULL_MISRA_C_COMPLIANCY
  hIbeta_err = ( int16_t )( pHandle->Ibeta_est / pHandle->hF1 );
#else
  hIbeta_err = ( int16_t )( pHandle->Ibeta_est >> pHandle->F1LOG );
#endif

  hIbeta_err = hIbeta_err - pInputs->Ialfa_beta.qI_Component2;

  wAux = ( int32_t )( pInputs->Vbus ) * pInputs->Valfa_beta.qV_Component1;
#ifdef FULL_MISRA_C_COMPLIANCY
  hValfa = ( int16_t ) ( wAux / 65536 );
#else
  hValfa = ( int16_t ) ( wAux >> 16 );
#endif

  wAux = ( int32_t )( pInputs->Vbus ) * pInputs->Valfa_beta.qV_Component2;
#ifdef FULL_MISRA_C_COMPLIANCY
  hVbeta = ( int16_t ) ( wAux / 65536 );
#else
  hVbeta = ( int16_t ) ( wAux >> 16 );
#endif

  /*alfa axes observer*/
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux = ( int16_t ) ( pHandle->Ialfa_est / pHandle->hF1 );
#else
  hAux = ( int16_t ) ( pHandle->Ialfa_est >> pHandle->F1LOG );
#endif

  wAux = ( int32_t ) ( pHandle->hC1 ) * hAux;
  wIalfa_est_Next = pHandle->Ialfa_est - wAux;

  wAux = ( int32_t ) ( pHandle->hC2 ) * hIalfa_err;
  wIalfa_est_Next += wAux;

  wAux = ( int32_t ) ( pHandle->hC5 ) * hValfa;
  wIalfa_est_Next += wAux;

  wAux = ( int32_t )  ( pHandle->hC3 ) * hAux_Alfa;
  wIalfa_est_Next -= wAux;

  wAux = ( int32_t )( pHandle->hC4 ) * hIalfa_err;
  wBemf_alfa_est_Next = pHandle->wBemf_alfa_est + wAux;

#ifdef FULL_MISRA_C_COMPLIANCY
  wAux = ( int32_t ) hAux_Beta / pHandle->hF3;
#else
  wAux = ( int32_t ) hAux_Beta >> pHandle->F3POW2;
#endif

  wAux = wAux * pHandle->hC6;
  wAux = hPrev_Rotor_Speed * wAux;
  wBemf_alfa_est_Next += wAux;

  /*beta axes observer*/
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux = ( int16_t ) ( pHandle->Ibeta_est / pHandle->hF1 );
#else
  hAux = ( int16_t ) ( pHandle->Ibeta_est >> pHandle->F1LOG );
#endif

  wAux = ( int32_t )  ( pHandle->hC1 ) * hAux;
  wIbeta_est_Next = pHandle->Ibeta_est - wAux;

  wAux = ( int32_t ) ( pHandle->hC2 ) * hIbeta_err;
  wIbeta_est_Next += wAux;

  wAux = ( int32_t ) ( pHandle->hC5 ) * hVbeta;
  wIbeta_est_Next += wAux;

  wAux = ( int32_t )  ( pHandle->hC3 ) * hAux_Beta;
  wIbeta_est_Next -= wAux;

  wAux = ( int32_t )( pHandle->hC4 ) * hIbeta_err;
  wBemf_beta_est_Next = pHandle->wBemf_beta_est + wAux;

#ifdef FULL_MISRA_C_COMPLIANCY
  wAux = ( int32_t )hAux_Alfa / pHandle->hF3;
#else
  wAux = ( int32_t ) hAux_Alfa >> pHandle->F3POW2;
#endif

  wAux = wAux * pHandle->hC6;
  wAux = hPrev_Rotor_Speed * wAux;
  wBemf_beta_est_Next -= wAux;

  if ( pHandle->Orig_ElSpeedDpp >= 0 )
  {
    wDirection = 1;
  }
  else
  {
    wDirection = -1;
  }

  /*Stores observed b-emfs */
  pHandle->hBemf_alfa_est = hAux_Alfa;
  pHandle->hBemf_beta_est = hAux_Beta;

  /*Calls the CORDIC blockset*/
  wAux_Alpha = pHandle->wBemf_alfa_est * wDirection;
  wAux_Beta = pHandle->wBemf_beta_est * wDirection;

  hRotor_Angle = STO_CR_ExecuteCORDIC( pHandle, wAux_Alpha, -wAux_Beta );
  hOrRotor_Speed = ( int16_t )( hRotor_Angle - hPrev_Rotor_Angle );
  hRotor_Acceleration = hOrRotor_Speed - hPrev_Rotor_Speed;

  hRotor_Speed = hOrRotor_Speed;

  if ( wDirection == 1 )
  {
    if ( hRotor_Speed < 0 )
    {
      hRotor_Speed = 0;
    }
    else
    {
      if ( hRotor_Acceleration > hMax_Instant_Accel )
      {
        hRotor_Speed = hPrev_Rotor_Speed + hMax_Instant_Accel;

        pHandle->_Super.hElAngle = hPrev_Rotor_Angle + hRotor_Speed;
      }
      else
      {
        pHandle->_Super.hElAngle = hRotor_Angle;
      }
    }
  }
  else
  {
    if ( hRotor_Speed > 0 )
    {
      hRotor_Speed = 0;
    }
    else
    {
      if ( hRotor_Acceleration < ( -hMax_Instant_Accel ) )
      {
        hRotor_Speed = hPrev_Rotor_Speed - hMax_Instant_Accel;

        pHandle->_Super.hElAngle = hPrev_Rotor_Angle + hRotor_Speed;
      }
      else
      {
        pHandle->_Super.hElAngle = hRotor_Angle;
      }
    }
  }


  if ( hRotor_Acceleration > hMax_Instant_Accel )
  {
    hOrRotor_Speed = hPrev_Rotor_Speed + hMax_Instant_Accel;
  }
  else if ( hRotor_Acceleration < ( -hMax_Instant_Accel ) )
  {
    hOrRotor_Speed = hPrev_Rotor_Speed - hMax_Instant_Accel;
  }
  else
  {}


  STO_CR_Store_Rotor_Speed( pHandle, hRotor_Speed, hOrRotor_Speed );

  /*storing previous values of currents and bemfs*/
  pHandle->Ialfa_est = wIalfa_est_Next;
  pHandle->wBemf_alfa_est = wBemf_alfa_est_Next;

  pHandle->Ibeta_est = wIbeta_est_Next;
  pHandle->wBemf_beta_est = wBemf_beta_est_Next;

  return ( pHandle->_Super.hElAngle );
}

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and returns - through
  *         parameter hMecSpeed01Hz - the rotor average mechanical speed,
  *         expressed in 01Hz. Average is computed considering a FIFO depth
  *         equal to bSpeedBufferSize01Hz. Moreover it also computes and returns
  *         the reliability state of the sensor.
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @param  pMecSpeed01Hz pointer to int16_t, used to return the rotor average
  *         mechanical speed (01Hz)
  * @retval bool speed sensor reliability, measured with reference to parameters
  *         bReliability_hysteresys, hVariancePercentage and bSpeedBufferSize
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
  */

bool STO_CR_CalcAvrgMecSpeed01Hz( STO_CR_Handle_t * pHandle, int16_t * pMecSpeed01Hz )
{
  int32_t wAvrSpeed_dpp = ( int32_t )0;
  int32_t wError, wAux, wAvrSquareSpeed, wAvrQuadraticError = 0;
  uint8_t i, bSpeedBufferSize01Hz = pHandle->SpeedBufferSize01Hz;
  int32_t wObsBemf, wEstBemf;
  int32_t wObsBemfSq = 0, wEstBemfSq = 0;
  int32_t wEstBemfSqLo;

  bool bIs_Speed_Reliable = false, bAux = false;
  bool bIs_Bemf_Consistent = false;

  for ( i = 0u; i < bSpeedBufferSize01Hz; i++ )
  {
    wAvrSpeed_dpp += ( int32_t )( pHandle->Speed_Buffer[i] );
  }

  wAvrSpeed_dpp = wAvrSpeed_dpp / ( int16_t )bSpeedBufferSize01Hz;

  for ( i = 0u; i < bSpeedBufferSize01Hz; i++ )
  {
    wError = ( int32_t )( pHandle->Speed_Buffer[i] ) - wAvrSpeed_dpp;
    wError = ( wError * wError );
    wAvrQuadraticError += wError;
  }

  /*It computes the measurement variance   */
  wAvrQuadraticError = wAvrQuadraticError / ( int16_t )bSpeedBufferSize01Hz;

  /* The maximum variance acceptable is here calculated as a function of average
     speed                                                                    */
  wAvrSquareSpeed = wAvrSpeed_dpp * wAvrSpeed_dpp;
  wAvrSquareSpeed = ( wAvrSquareSpeed * ( int32_t )( pHandle->VariancePercentage )) / ( int16_t )128;

  if ( wAvrQuadraticError < wAvrSquareSpeed )
  {
    bIs_Speed_Reliable = true;
  }

  /*Computation of Mechanical speed 01Hz*/
  wAux = wAvrSpeed_dpp * ( int32_t )( pHandle->_Super.hMeasurementFrequency );
  wAux = wAux * ( int32_t ) 10;
  wAux = wAux / ( int32_t )65536;
  wAux = wAux / ( int16_t )( pHandle->_Super.bElToMecRatio );

  *pMecSpeed01Hz = ( int16_t )wAux;
  pHandle->_Super.hAvrMecSpeed01Hz = ( int16_t )wAux;

  pHandle->IsSpeedReliable = bIs_Speed_Reliable;

  /*Bemf Consistency Check algorithm*/
  if ( pHandle->EnableDualCheck == true ) /*do algorithm if it's enabled*/
  {
    wAux = ( wAux < 0 ? ( -wAux ) : ( wAux ) ); /* wAux abs value   */
    if ( wAux < ( int32_t )( pHandle->MaxAppPositiveMecSpeed01Hz ) )
    {
      /*Computation of Observed back-emf*/
      wObsBemf = ( int32_t )( pHandle->hBemf_alfa_est );
      wObsBemfSq = wObsBemf * wObsBemf;
      wObsBemf = ( int32_t )( pHandle->hBemf_beta_est );
      wObsBemfSq += wObsBemf * wObsBemf;

      /*Computation of Estimated back-emf*/
      wEstBemf = ( wAux * 32767 ) / ( int16_t )( pHandle->_Super.hMaxReliableMecSpeed01Hz );
      wEstBemfSq = ( wEstBemf * ( int32_t )( pHandle->BemfConsistencyGain ) ) / 64;
      wEstBemfSq *= wEstBemf;

      /*Computation of threshold*/
      wEstBemfSqLo = wEstBemfSq -
                     ( wEstBemfSq / 64 ) * ( int32_t )( pHandle->BemfConsistencyCheck );

      /*Check*/
      if ( wObsBemfSq > wEstBemfSqLo )
      {
        bIs_Bemf_Consistent = true;
      }
    }

    pHandle->IsBemfConsistent = bIs_Bemf_Consistent;
    pHandle->Obs_Bemf_Level = wObsBemfSq;
    pHandle->Est_Bemf_Level = wEstBemfSq;
  }
  else
  {
    bIs_Bemf_Consistent = true;
  }

  /*Decision making*/
  if ( pHandle->IsAlgorithmConverged == false )
  {
    bAux = SPD_IsMecSpeedReliable ( &pHandle->_Super, pMecSpeed01Hz );
  }
  else
  {
    if ( ( pHandle->IsSpeedReliable == false ) || ( bIs_Bemf_Consistent == false ) )
    {
      pHandle->ReliabilityCounter++;
      if ( pHandle->ReliabilityCounter >= pHandle->Reliability_hysteresys )
      {
        pHandle->ReliabilityCounter = 0u;
        pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
        bAux = false;
      }
      else
      {
        bAux = SPD_IsMecSpeedReliable ( &pHandle->_Super, pMecSpeed01Hz );;
      }
    }
    else
    {
      pHandle->ReliabilityCounter = 0u;
      bAux = SPD_IsMecSpeedReliable ( &pHandle->_Super, pMecSpeed01Hz );
    }
  }
  return ( bAux );
}

/**
  * @brief  It clears state observer object by re-initializing private variables
  * @param  pHandle pointer on the component instance to work on.
  */
void STO_CR_Clear( STO_CR_Handle_t * pHandle )
{

  pHandle->Ialfa_est = ( int32_t )0;
  pHandle->Ibeta_est = ( int32_t )0;
  pHandle->wBemf_alfa_est = ( int32_t )0;
  pHandle->wBemf_beta_est = ( int32_t )0;
  pHandle->_Super.hElAngle = ( int16_t )0;
  pHandle->_Super.hElSpeedDpp = ( int16_t )0;
  pHandle->Orig_ElSpeedDpp = ( int16_t )0;
  pHandle->ConsistencyCounter = 0u;
  pHandle->ReliabilityCounter = 0u;
  pHandle->IsAlgorithmConverged = false;
  pHandle->IsBemfConsistent = false;
  pHandle->Obs_Bemf_Level = ( int32_t )0;
  pHandle->Est_Bemf_Level = ( int32_t )0;
  pHandle->DppBufferSum = ( int32_t )0;
  pHandle->DppOrigBufferSum = ( int32_t )0;
  pHandle->ForceConvergency = false;
  pHandle->ForceConvergency2 = false;

  STO_CR_InitSpeedBuffer( pHandle );
}

/**
  * @brief  It could be used to set istantaneous information on rotor mechanical
  *         angle.
  * @Note   Mechanical angle management is not implemented in this version of
            State observer sensor class.
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @param  hMecAngle istantaneous measure of rotor mechanical angle
  * @retval none
  */
void STO_CR_SetMecAngle( STO_CR_Handle_t * pHandle, int16_t hMecAngle )
{
}

/**
  * @brief  It stores in estimated speed FIFO latest calculated value of motor
  *         speed
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval none
  */
inline static void STO_CR_Store_Rotor_Speed( STO_CR_Handle_t * pHandle, int16_t hRotor_Speed, int16_t hOrRotor_Speed )
{
  uint8_t bBuffer_index = pHandle->Speed_Buffer_Index;

  bBuffer_index++;
  if ( bBuffer_index == pHandle->SpeedBufferSize01Hz )
  {
    bBuffer_index = 0u;
  }

  pHandle->SpeedBufferOldestEl = pHandle->Speed_Buffer[bBuffer_index];
  pHandle->OrigSpeedBufferOldestEl = pHandle->Orig_Speed_Buffer[bBuffer_index];

  pHandle->Speed_Buffer[bBuffer_index] = hRotor_Speed;
  pHandle->Orig_Speed_Buffer[bBuffer_index] = hOrRotor_Speed;
  pHandle->Speed_Buffer_Index = bBuffer_index;
}

/**
  * @brief  It executes CORDIC algorithm for rotor position extraction from B-emf
  *         alpha and beta
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  *         wBemf_alfa_est estimated Bemf alpha on the stator reference frame
  *         wBemf_beta_est estimated Bemf beta on the stator reference frame
  * @retval int16_t rotor electrical angle (s16degrees)
  */
inline static int16_t STO_CR_ExecuteCORDIC( STO_CR_Handle_t * pHandle, int32_t wBemf_alfa_est, int32_t
                                     wBemf_beta_est )
{
  int16_t hAngle;
  int32_t wXi, wYi, wXold;

  /*Determining quadrant*/
  if ( wBemf_alfa_est < 0 )
  {
    if ( wBemf_beta_est < 0 )
    {
      /*Quadrant III, add 90 degrees so as to move to quadrant IV*/
      hAngle = 16384;
      wXi = - ( wBemf_beta_est / 2 );
      wYi = wBemf_alfa_est / 2;
    }
    else
    {
      /*Quadrant II, subtract 90 degrees so as to move to quadrant I*/
      hAngle = -16384;
      wXi = wBemf_beta_est / 2;
      wYi = - ( wBemf_alfa_est / 2 );
    }
  }
  else
  {
    /* Quadrant I or IV*/
    hAngle = 0;
    wXi = wBemf_alfa_est / 2;
    wYi = wBemf_beta_est / 2;
  }
  wXold = wXi;

  /*begin the successive approximation process*/
  /*iteration0*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV1;
    wXi = wXi - wYi;
    wYi = wXold + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV1;
    wXi = wXi + wYi;
    wYi = -wXold + wYi;
  }
  wXold = wXi;

  /*iteration1*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV2;
    wXi = wXi - wYi / 2;
    wYi = wXold / 2 + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV2;
    wXi = wXi + wYi / 2;
    wYi = -wXold / 2 + wYi;
  }
  wXold = wXi;

  /*iteration2*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV4;
    wXi = wXi - wYi / 4;
    wYi = wXold / 4 + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV4;
    wXi = wXi + wYi / 4;
    wYi = -wXold / 4 + wYi;
  }
  wXold = wXi;

  /*iteration3*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV8;
    wXi = wXi - wYi / 8;
    wYi = wXold / 8 + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV8;
    wXi = wXi + wYi / 8;
    wYi = -wXold / 8 + wYi;
  }
  wXold = wXi;

  /*iteration4*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV16;
    wXi = wXi - wYi / 16;
    wYi = wXold / 16 + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV16;
    wXi = wXi + wYi / 16;
    wYi = -wXold / 16 + wYi;
  }
  wXold = wXi;

  /*iteration5*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV32;
    wXi = wXi - wYi / 32;
    wYi = wXold / 32 + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV32;
    wXi = wXi + wYi / 32;
    wYi = -wXold / 32 + wYi;
  }
  wXold = wXi;

  /*iteration6*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV64;
    wXi = wXi - wYi / 64;
    wYi = wXold / 64 + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV64;
    wXi = wXi + wYi / 64;
    wYi = -wXold / 64 + wYi;
  }
  wXold = wXi;

  /*iteration7*/
  if ( wYi < 0 )
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV128;
    wXi = wXi - wYi / 128;
    wYi = wXold / 128 + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV128;
    wXi = wXi + wYi / 128;
    wYi = -wXold / 128 + wYi;
  }

  return ( -hAngle );
}

/**
  * @brief  It clears the estimated speed buffer
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval none
  */
static void STO_CR_InitSpeedBuffer( STO_CR_Handle_t * pHandle )
{
  uint8_t b_i;
  uint8_t bSpeedBufferSize01Hz = pHandle->SpeedBufferSize01Hz;

  /*init speed buffer*/
  for ( b_i = 0u; b_i < bSpeedBufferSize01Hz; b_i++ )
  {
    pHandle->Speed_Buffer[b_i] = ( int16_t )0;
    pHandle->Orig_Speed_Buffer[b_i] = ( int16_t )0;
  }

  pHandle->Speed_Buffer_Index = 0u;
  pHandle->SpeedBufferOldestEl = ( int16_t )0;
  pHandle->OrigSpeedBufferOldestEl = ( int16_t )0;

  return;
}


/**
  * @brief  It internally performes a set of checks necessary to state whether
  *         the state observer algorithm converged. To be periodically called
  *         during motor open-loop ramp-up (e.g. at the same frequency of
  *         SPD_CalcElAngle), it returns true if the estimated angle and speed
  *         can be considered reliable, false otherwise
  * @param  pHandle pointer on the component instance
  * @param  hForcedMecSpeed01Hz Mechanical speed in 0.1Hz unit as forced by VSS
  * @retval bool sensor reliability state
  */
bool STO_CR_IsObserverConverged( STO_CR_Handle_t * pHandle, int16_t hForcedMecSpeed01Hz )
{
  int16_t hEstimatedSpeed01Hz, hUpperThreshold, hLowerThreshold;
  int32_t wAux;
  bool bAux = false;
  int32_t wtemp;

  if ( pHandle->ForceConvergency2 == true )
  {
    hForcedMecSpeed01Hz = pHandle->_Super.hAvrMecSpeed01Hz;
  }

  if ( pHandle->ForceConvergency == true )
  {
    bAux = true;
    pHandle->IsAlgorithmConverged = true;
    pHandle->_Super.bSpeedErrorNumber = 0u;
  }
  else
  {
    hEstimatedSpeed01Hz = pHandle->_Super.hAvrMecSpeed01Hz;
    wtemp = ( int32_t )hEstimatedSpeed01Hz * ( int32_t )hForcedMecSpeed01Hz;

    if ( wtemp > 0 )
    {
      if ( hEstimatedSpeed01Hz < 0 )
      {
        hEstimatedSpeed01Hz = -hEstimatedSpeed01Hz;
      }

      if ( hForcedMecSpeed01Hz < 0 )
      {
        hForcedMecSpeed01Hz = -hForcedMecSpeed01Hz;
      }
      wAux = ( int32_t ) ( hForcedMecSpeed01Hz ) * ( int16_t )pHandle->SpeedValidationBand_H;
      hUpperThreshold = ( int16_t )( wAux / ( int32_t )16 );

      wAux = ( int32_t ) ( hForcedMecSpeed01Hz ) * ( int16_t )pHandle->SpeedValidationBand_L;
      hLowerThreshold = ( int16_t )( wAux / ( int32_t )16 );

      /* If the variance of the estimated speed is low enough...*/
      if ( pHandle->IsSpeedReliable == true )
      {
        if ( ( uint16_t )hEstimatedSpeed01Hz > pHandle->MinStartUpValidSpeed )
        {
          /*...and the estimated value is quite close to the expected value... */
          if ( hEstimatedSpeed01Hz >= hLowerThreshold )
          {
            if ( hEstimatedSpeed01Hz <= hUpperThreshold )
            {
              pHandle->ConsistencyCounter++;

              /*... for hConsistencyThreshold consecutive times... */
              if ( pHandle->ConsistencyCounter >= pHandle->StartUpConsistThreshold )
              {
                /* the algorithm converged.*/
                bAux = true;
                pHandle->IsAlgorithmConverged = true;
                pHandle->_Super.bSpeedErrorNumber = 0u;
              }
            }
            else
            {
              pHandle->ConsistencyCounter = 0u;
            }
          }
          else
          {
            pHandle->ConsistencyCounter = 0u;
          }
        }
        else
        {
          pHandle->ConsistencyCounter = 0u;
        }
      }
      else
      {
        pHandle->ConsistencyCounter = 0u;
      }
    }
  }
  return ( bAux );
}

/**
  * @brief  It exports estimated Bemf alpha-beta in Volt_Components format
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval Volt_Components Bemf alpha-beta
  */
Volt_Components STO_CR_GetEstimatedBemf( STO_CR_Handle_t * pHandle )
{
  Volt_Components Vaux;
  Vaux.qV_Component1 = pHandle->hBemf_alfa_est;
  Vaux.qV_Component2 = pHandle->hBemf_beta_est;
  return ( Vaux );
}

/**
  * @brief  It exports the stator current alpha-beta as estimated by state
  *         observer
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval Curr_Components State observer estimated stator current Ialpha-beta
  */
Curr_Components STO_CR_GetEstimatedCurrent( STO_CR_Handle_t * pHandle )
{
  Curr_Components Iaux;

#ifdef FULL_MISRA_C_COMPLIANCY
  Iaux.qI_Component1 = ( int16_t )( pHandle->Ialfa_est / ( pHandle->hF1 ) );
#else
  Iaux.qI_Component1 = ( int16_t )( pHandle->Ialfa_est >> pHandle->F1LOG );
#endif

#ifdef FULL_MISRA_C_COMPLIANCY
  Iaux.qI_Component2 = ( int16_t )( pHandle->Ibeta_est / ( pHandle->hF1 ) );
#else
  Iaux.qI_Component2 = ( int16_t )( pHandle->Ibeta_est >> pHandle->F1LOG );
#endif

  return ( Iaux );
}

/**
  * @brief  It exports current observer gains through parameters hhC2 and hhC4
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @param  phC2 pointer to int16_t used to return parameters hhC2
  * @param  phC4 pointer to int16_t used to return parameters hhC4
  * @retval none
  */
void STO_CR_GetObserverGains( STO_CR_Handle_t * pHandle, int16_t * phC2, int16_t * phC4 )
{
  *phC2 = pHandle->hC2;
  *phC4 = pHandle->hC4;
}

/**
  * @brief  It allows setting new values for observer gains
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @param  wK1 new value for observer gain hhC1
  * @param  wK2 new value for observer gain hhC2
  * @retval none
  */
void STO_CR_SetObserverGains( STO_CR_Handle_t * pHandle, int16_t hhC1, int16_t hhC2 )
{
  pHandle->hC2 = hhC1;
  pHandle->hC4 = hhC2;
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
  *         on which speed control is executed. It computes and update object
  *         variable hElSpeedDpp that is estimated average electrical speed
  *         expressed in dpp used for instance in observer equations.
  *         Average is computed considering a FIFO depth equal to
  *         bSpeedBufferSizedpp.
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval none
  */
void STO_CR_CalcAvrgElSpeedDpp( STO_CR_Handle_t * pHandle )
{
  int16_t hIndexNew = ( int16_t )pHandle->Speed_Buffer_Index;
  int16_t hIndexOld;
  int16_t hIndexOldTemp;
  int32_t wSum = pHandle->DppBufferSum;
  int32_t wSumOrig = pHandle->DppOrigBufferSum;
  int32_t wAvrSpeed_dpp;
  int16_t hSpeedBufferSizedpp = ( int16_t )( pHandle->SpeedBufferSizedpp );
  int16_t hSpeedBufferSize01Hz = ( int16_t )( pHandle->SpeedBufferSize01Hz );
  int16_t hBufferSizeDiff;

  hBufferSizeDiff = hSpeedBufferSize01Hz - hSpeedBufferSizedpp;

  if ( hBufferSizeDiff == 0 )
  {
    wSum = wSum + pHandle->Speed_Buffer[hIndexNew] -
           pHandle->SpeedBufferOldestEl;

    wSumOrig = wSumOrig + pHandle->Orig_Speed_Buffer[hIndexNew] -
               pHandle->OrigSpeedBufferOldestEl;
  }
  else
  {
    hIndexOldTemp = hIndexNew + hBufferSizeDiff;

    if ( hIndexOldTemp >= hSpeedBufferSize01Hz )
    {
      hIndexOld = hIndexOldTemp - hSpeedBufferSize01Hz;
    }
    else
    {
      hIndexOld = hIndexOldTemp;
    }

    wSum = wSum + pHandle->Speed_Buffer[hIndexNew] -
           pHandle->Speed_Buffer[hIndexOld];

    wSumOrig = wSumOrig + pHandle->Orig_Speed_Buffer[hIndexNew] -
               pHandle->Orig_Speed_Buffer[hIndexOld];
  }

#ifdef FULL_MISRA_C_COMPLIANCY
  wAvrSpeed_dpp = wSum / hSpeedBufferSizedpp;
#else
  wAvrSpeed_dpp = wSum >> pHandle->SpeedBufferSizedppLOG;
#endif

  pHandle->_Super.hElSpeedDpp = ( int16_t )wAvrSpeed_dpp;

#ifdef FULL_MISRA_C_COMPLIANCY
  wAvrSpeed_dpp = wSumOrig / hSpeedBufferSizedpp;
#else
  wAvrSpeed_dpp = wSumOrig >> pHandle->SpeedBufferSizedppLOG;
#endif

  pHandle->Orig_ElSpeedDpp = ( int16_t )wAvrSpeed_dpp;

  pHandle->DppBufferSum = wSum;

  pHandle->DppOrigBufferSum = wSumOrig;
}

/**
  * @brief  It exports estimated Bemf squared level
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval int32_t
  */
int32_t STO_CR_GetEstimatedBemfLevel( STO_CR_Handle_t * pHandle )
{
  return ( pHandle->Est_Bemf_Level );
}

/**
  * @brief  It exports observed Bemf squared level
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval int32_t
  */
int32_t STO_CR_GetObservedBemfLevel( STO_CR_Handle_t * pHandle )
{
  return ( pHandle->Obs_Bemf_Level );
}

/**
  * @brief  It enables/disables the bemf consistency check
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @param  bSel boolean; true enables check; false disables check
  */
void STO_CR_BemfConsistencyCheckSwitch( STO_CR_Handle_t * pHandle, bool bSel )
{
  pHandle->EnableDualCheck = bSel;
}

/**
  * @brief  It returns the result of the Bemf consistency check
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval bool Bemf consistency state
  */
bool STO_CR_IsBemfConsistent( STO_CR_Handle_t * pHandle )
{
  return ( pHandle->IsBemfConsistent );
}

/**
  * @brief  This method returns the reliability of the speed sensor
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval bool speed sensor reliability, measured with reference to parameters
  *         bReliability_hysteresys, hVariancePercentage and bSpeedBufferSize
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
  */
bool STO_CR_IsSpeedReliable( STO_Handle_t * pHandle )
{
  STO_CR_Handle_t * pHdl = ( STO_CR_Handle_t * )pHandle->_Super;
  return ( pHdl->IsSpeedReliable );
}

/* @brief  It forces the state-observer to declare converged
 * @param  pHandle: handler of the current instance of the STO CORDIC component
 */
void STO_CR_ForceConvergency1( STO_Handle_t * pHandle )
{
  STO_CR_Handle_t * pHdl = ( STO_CR_Handle_t * )pHandle->_Super;
  pHdl->ForceConvergency = true;
}

/* @brief  It forces the state-observer to declare converged
 * @param  pHandle: handler of the current instance of the STO CORDIC component
 */
void STO_CR_ForceConvergency2( STO_Handle_t * pHandle )
{
  STO_CR_Handle_t * pHdl = ( STO_CR_Handle_t * )pHandle->_Super;
  pHdl->ForceConvergency2 = true;
}

/**
  * @}
  */

/**
  * @}
  */

/** @} */
/************************ (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
