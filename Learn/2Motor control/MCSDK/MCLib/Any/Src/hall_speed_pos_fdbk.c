/**
  ******************************************************************************
  * @file    hall_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features of
  *          the Hall Speed & Position Feedback component of the Motor Control SDK.
            此文件提供的固件功能可实现Motor Control SDK的霍尔速度和位置反馈组件的功能。
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"
#include "hall_speed_pos_fdbk.h"
#include "mc_irq_handler.h"

#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/**
 * @defgroup hall_speed_pos_fdbk Hall Speed & Position Feedback
 *
 * @brief Hall Sensor based Speed & Position Feedback implementation
      基于霍尔传感器的速度和位置反馈实现
  *
  *
  *
  *
  * 该组件用于控制配备霍尔效应传感器的电机的应用。
  * 该组件使用两个霍尔效应传感器的输出来测量速度*和电机转子的位置。
  *
 * @todo Document the Hall Speed & Position Feedback "module".
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/

/* Lower threshold to reques a decrease of clock prescaler
      降低阈值以请求减少时钟预分频器
*/
#define LOW_RES_THRESHOLD   ((uint16_t)0x5500u)

#define HALL_COUNTER_RESET  ((uint16_t) 0u)

#define S16_120_PHASE_SHIFT (int16_t)(65536/3)
#define S16_60_PHASE_SHIFT  (int16_t)(65536/6)

#define STATE_0 (uint8_t)0
#define STATE_1 (uint8_t)1
#define STATE_2 (uint8_t)2
#define STATE_3 (uint8_t)3
#define STATE_4 (uint8_t)4
#define STATE_5 (uint8_t)5
#define STATE_6 (uint8_t)6
#define STATE_7 (uint8_t)7

#define NEGATIVE          (int8_t)-1
#define POSITIVE          (int8_t)1
#define NEGATIVE_SWAP     (int8_t)-2
#define POSITIVE_SWAP     (int8_t)2

/* With digit-per-PWM unit (here 2*PI rad = 0xFFFF):
  使用每PWM数字单位（此处2 * PI rad = 0xFFFF）：
  */
#define HALL_MAX_PSEUDO_SPEED        ((int16_t)0x7FFF)

#define CCER_CC1E_Set               ((uint16_t)0x0001)
#define CCER_CC1E_Reset             ((uint16_t)0xFFFE)

static void HALL_Init_Electrical_Angle( HALL_Handle_t * pHandle );
static int16_t HALL_CalcAvrgElSpeedDpp( HALL_Handle_t * pHandle );

/**
  * @brief  It initializes the hardware peripherals (TIMx, GPIO and NVIC)
            required for the speed position sensor management using HALL
            sensors.
            它使用HALL传感器初始化速度位置传感器管理所需的硬件外设（TIMx，GPIO和NVIC）。
  * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
  * @retval none
  */
void HALL_Init( HALL_Handle_t * pHandle )
{
  TIM_TypeDef * TIMx = pHandle->TIMx;

  uint16_t hMinReliableElSpeed01Hz = pHandle->_Super.hMinReliableMecSpeed01Hz *
                                     pHandle->_Super.bElToMecRatio;
  uint16_t hMaxReliableElSpeed01Hz = pHandle->_Super.hMaxReliableMecSpeed01Hz *
                                     pHandle->_Super.bElToMecRatio;
  uint8_t bSpeedBufferSize;
  uint8_t bIndex;

  /* Adjustment factor: minimum measurable speed is x time less than the minimum
  reliable speed
  调整系数：最小可测量速度是小于最小可靠速度的x时间*/
  hMinReliableElSpeed01Hz /= 4u;

  /* Adjustment factor: maximum measurable speed is x time greather than the
  maximum reliable speed
  调整系数：最大可测量速度是大于最大可靠速度的x时间 */
  hMaxReliableElSpeed01Hz *= 2u;

  pHandle->OvfFreq = ( uint16_t )( pHandle->TIMClockFreq / 65536u );

  /* SW Init */
  if ( hMinReliableElSpeed01Hz == 0u )
  {
    /* Set fixed to 150 ms
    设置固定为150毫秒  */
    pHandle->HallTimeout = 150u;
  }
  else
  {
    /* Set accordingly the min reliable speed
    相应地设定最小可靠速度*/
    /* 10000 comes from mS and 01Hz
    10000来自mS和01Hz
    * 6 comes from the fact that sensors are toggling each 60 deg
    6来自传感器每60度切换一次的事实 */
    pHandle->HallTimeout = 10000u / ( 6u * hMinReliableElSpeed01Hz );
  }

  /* Compute the prescaler to the closet value of the TimeOut (in mS )
  计算预分频器到TimeOut的壁橱值（以mS为单位） */
  pHandle->HALLMaxRatio = ( pHandle->HallTimeout * pHandle->OvfFreq ) / 1000 ;

  /* Align MaxPeriod to a multiple of Overflow.
  将MaxPeriod对齐为溢出的倍数。*/
  pHandle->MaxPeriod = ( pHandle->HALLMaxRatio ) * 65536uL;

  pHandle->SatSpeed = hMaxReliableElSpeed01Hz;

  pHandle->PseudoFreqConv = ( ( pHandle->TIMClockFreq / 6u )
                              / ( pHandle->_Super.hMeasurementFrequency ) ) * 65536u;

  pHandle->MinPeriod = ( ( 10u * pHandle->TIMClockFreq ) / 6u )
                       / hMaxReliableElSpeed01Hz;

  pHandle->PWMNbrPSamplingFreq = ( pHandle->_Super.hMeasurementFrequency /
                                   pHandle->SpeedSamplingFreqHz ) - 1u;

  /* Reset speed reliability
      重置速度可靠性*/
  pHandle->SensorIsReliable = true;

  /* Force the TIMx prescaler with immediate access (gen update event)
    强制TIMx预分频器立即访问（gen更新事件）
  */
  LL_TIM_SetPrescaler ( TIMx, pHandle->HALLMaxRatio );
  LL_TIM_GenerateEvent_UPDATE ( TIMx );


  /* Clear the TIMx's pending flags
  清除TIMx的挂起标志 */
  LL_TIM_WriteReg( TIMx, SR, 0 );

  /* Selected input capture and Update (overflow) events generate interrupt
    选定的输入捕获和更新（溢出）事件会产生中断
  */

  /* Source of Update event is only counter overflow/underflow
    Update事件源仅为计数器溢出/下溢
   */
  LL_TIM_SetUpdateSource ( TIMx, LL_TIM_UPDATESOURCE_COUNTER );

  LL_TIM_EnableIT_CC1 ( TIMx );
  LL_TIM_EnableIT_UPDATE ( TIMx );
  LL_TIM_SetCounter ( TIMx, HALL_COUNTER_RESET );

  LL_TIM_CC_EnableChannel  ( TIMx, LL_TIM_CHANNEL_CH1 );
  LL_TIM_EnableCounter ( TIMx );


  /* Erase speed buffer
  擦除速度缓冲区
  */
  bSpeedBufferSize = pHandle->SpeedBufferSize;

  for ( bIndex = 0u; bIndex < bSpeedBufferSize; bIndex++ )
  {
    pHandle->SensorSpeed[bIndex]  = 0;
  }
}

/**
* @brief  Clear software FIFO where are "pushed" latest speed information
*         This function must be called before starting the motor to initialize
*         the speed measurement process.
          清除软件FIFO“推送”最新速度信息*必须在启动电机初始化*速度测量过程之前调用此功能。
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component*
          pHandle：hall_speed_pos_fdbk组件的当前实例的处理程序*
* @retval none
*/
void HALL_Clear( HALL_Handle_t * pHandle )
{
  TIM_TypeDef * TIMx = pHandle->TIMx;

  /* Mask interrupts to insure a clean intialization 屏蔽中断以确保初始化干净*/
  LL_TIM_DisableIT_CC1 ( TIMx );

  pHandle->RatioDec = false;
  pHandle->RatioInc = false;

  /* Reset speed reliability 重置速度可靠性*/
  pHandle->SensorIsReliable = true;

  /* Acceleration measurement not implemented.没有实施加速度测量*/
  pHandle->_Super.hMecAccel01HzP = 0;

  pHandle->FirstCapt = 0u;
  pHandle->BufferFilled = 0u;
  pHandle->OVFCounter = 0u;

  pHandle->CompSpeed = 0;
  pHandle->ElSpeedSum = 0;

  pHandle->Direction = POSITIVE;

  /* Initialize speed buffer index 初始化速度缓冲区索引*/
  pHandle->SpeedFIFOIdx = 0u;

  /* Clear new speed acquisitions flag 清除新的速度收购标志*/
  pHandle->NewSpeedAcquisition = 0;

  /* Re-initialize partly the timer 部分重新初始化计时器*/
  LL_TIM_SetPrescaler ( TIMx, pHandle->HALLMaxRatio );

  LL_TIM_SetCounter ( TIMx, HALL_COUNTER_RESET );

  LL_TIM_EnableCounter ( TIMx );

  LL_TIM_EnableIT_CC1 ( TIMx );

  HALL_Init_Electrical_Angle( pHandle );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
* @brief  Update the rotor electrical angle integrating the last measured
*         instantaneous electrical speed express in dpp.
            更新转子电角度，将最后测量的瞬时电气速度表达为dpp。
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
          pHandle：hall_speed_pos_fdbk组件的当前实例的处理程序
* @retval int16_t Measured electrical angle in s16degree format.   int16_t以s16degree格式测量的电角度。
*/
int16_t HALL_CalcElAngle( HALL_Handle_t * pHandle )
{

  if ( pHandle->_Super.hElSpeedDpp != HALL_MAX_PSEUDO_SPEED )
  {
    pHandle->MeasuredElAngle += pHandle->_Super.hElSpeedDpp;
    pHandle->TargetElAngle += pHandle->_Super.hElSpeedDpp;
    pHandle->_Super.hElAngle += pHandle->_Super.hElSpeedDpp + pHandle->CompSpeed;
    pHandle->PrevRotorFreq = pHandle->_Super.hElSpeedDpp;
  }
  else
  {
    pHandle->_Super.hElAngle += pHandle->PrevRotorFreq;
  }

  return pHandle->_Super.hElAngle;
}


/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed.
  *         This method compute and store rotor istantaneous el speed (express
  *         in dpp considering the measurement frequency) in order to provide it
  *         to HALL_CalcElAngle function and SPD_GetElAngle.
  *         Then compute rotor average el speed (express in dpp considering the
  *         measurement frequency) based on the buffer filled by IRQ, then - as
  *         a consequence - compute, store and return - through parameter
  *         hMecSpeed01Hz - the rotor average mech speed, expressed in 01Hz.
  *         Then check, store and return the reliability state of
  *         the sensor; in this function the reliability is measured with
  *         reference to specific parameters of the derived
  *         sensor (HALL) through internal variables managed by IRQ.
  必须至少调用此方法，并且执行速度控制的周期相同。
  *该方法计算并存储转子的瞬时速度（考虑到测量频率在dpp中表示），以便将其提供给HALL_CalcElAngle函数和SPD_GetElAngle。
  *然后根据IRQ填充的缓冲区计算转子平均el速度（以dpp表示），然后 - 作为结果 - 计算，存储和返回 - 通过参数* hMecSpeed01Hz  - 转子平均机械速度，表示在01Hz。
  *然后检查，存储并返回传感器的可靠性状态;在此函数中，通过IRQ管理的内部变量，参考派生的传感器（HALL）的特定参数来测量可靠性。


  * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
            pHandle：hall_speed_pos_fdbk组件的当前实例的处理程序
  * @param  hMecSpeed01Hz pointer to int16_t, used to return the rotor average
  *         mechanical speed (01Hz)
            hMecSpeed01Hz指向int16_t的指针，用于返回转子平均值*机械速度（01Hz）
  * @retval true = sensor information is reliable
            true =传感器信息可靠
  *         false = sensor information is not reliable
            false =传感器信息不可靠
  */
bool HALL_CalcAvrgMecSpeed01Hz( HALL_Handle_t * pHandle, int16_t * hMecSpeed01Hz )
{
  TIM_TypeDef * TIMx = pHandle->TIMx;
  int16_t SpeedMeasAux;
  bool bReliability;

  /* Computing the rotor istantaneous el speed
    计算转子瞬时速度*/
  SpeedMeasAux = pHandle->CurrentSpeed;

  if ( pHandle->SensorIsReliable )
  {
    /* No errors have been detected during rotor speed information
    extrapolation 在转子速度信息外推期间没有检测到错误 */
    if ( LL_TIM_GetPrescaler ( TIMx ) >= pHandle->HALLMaxRatio )
    {
      /* At start-up or very low freq 在启动时或非常低的频率*/
      /* Based on current prescaler value only
        仅基于当前预分频值 */
      pHandle->_Super.hElSpeedDpp = 0;
      *hMecSpeed01Hz = 0;
    }
    else
    {
      pHandle->_Super.hElSpeedDpp = SpeedMeasAux;
      if ( SpeedMeasAux == 0 )
      {
        /* Speed is too low 速度太低了*/
        *hMecSpeed01Hz = 0;
      }
      else
      {
        /* Check if speed is not to fast 检查速度是否快 */
        if ( SpeedMeasAux != HALL_MAX_PSEUDO_SPEED )
        {
#ifdef HALL_MTPA
          {
            pHandle->CompSpeed = 0;
          }
#else
          {
            pHandle->TargetElAngle = pHandle->MeasuredElAngle;
            pHandle->DeltaAngle = pHandle->MeasuredElAngle - pHandle->_Super.hElAngle;
            pHandle->CompSpeed = ( int16_t )
            ( ( int32_t )( pHandle->DeltaAngle ) /
              ( int32_t )( pHandle->PWMNbrPSamplingFreq ) );
          }
#endif

          *hMecSpeed01Hz = HALL_CalcAvrgElSpeedDpp( pHandle );

          /* Converto el_dpp to Mec01Hz 将el_dpp转换为Mec01Hz*/
          *hMecSpeed01Hz = ( int16_t )( ( *hMecSpeed01Hz *
                                          ( int32_t )pHandle->_Super.hMeasurementFrequency * 10 ) /
                                        ( 65536 * ( int32_t )pHandle->_Super.bElToMecRatio ) );

        }
        else
        {
          *hMecSpeed01Hz = ( int16_t )pHandle->SatSpeed;
        }
      }
    }
    bReliability = SPD_IsMecSpeedReliable( &pHandle->_Super, hMecSpeed01Hz );
  }
  else
  {
    bReliability = false;
    pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
    /* If speed is not reliable the El and Mec speed is set to 0
     如果速度不可靠，则El和Mec速度设置为0*/
    pHandle->_Super.hElSpeedDpp = 0;
    *hMecSpeed01Hz = 0;
  }

  pHandle->_Super.hAvrMecSpeed01Hz = *hMecSpeed01Hz;

  return ( bReliability );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
* @brief  Example of private method of the class HALL to implement an MC IRQ function
*         to be called when TIMx capture event occurs
          类HALL的私有方法示例，用于实现在发生TIMx捕获事件时要调用的MC IRQ函数*
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
            句柄：hall_speed_pos_fdbk组件的当前实例的处理程序
* @retval none
*/
void * HALL_TIMx_CC_IRQHandler( void * pHandleVoid )
{
  HALL_Handle_t * pHandle = ( HALL_Handle_t * ) pHandleVoid;
  TIM_TypeDef * TIMx = pHandle->TIMx;
  uint8_t bPrevHallState;
  uint32_t wCaptBuf;
  uint16_t hPrscBuf;
  uint16_t hHighSpeedCapture;

  if ( pHandle->SensorIsReliable )
  {
    /* A capture event generated this interrupt 捕获事件生成此中断*/
    bPrevHallState = pHandle->HallState;

    if ( pHandle->SensorPlacement == DEGREES_120 )
    {
      pHandle->HallState  = LL_GPIO_IsInputPinSet( pHandle->H3Port, pHandle->H3Pin ) << 2
                            | LL_GPIO_IsInputPinSet( pHandle->H2Port, pHandle->H2Pin ) << 1
                            | LL_GPIO_IsInputPinSet( pHandle->H1Port, pHandle->H1Pin );
    }
    else
    {
      pHandle->HallState  = ( LL_GPIO_IsInputPinSet( pHandle->H2Port, pHandle->H2Pin ) ^ 1 ) << 2
                            | LL_GPIO_IsInputPinSet( pHandle->H3Port, pHandle->H3Pin ) << 1
                            | LL_GPIO_IsInputPinSet( pHandle->H1Port, pHandle->H1Pin );
    }

    switch ( pHandle->HallState )
    {
      case STATE_5:
        if ( bPrevHallState == STATE_4 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = pHandle->PhaseShift;
        }
        else if ( bPrevHallState == STATE_1 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_60_PHASE_SHIFT );
        }
        else
        {
        }
        break;

      case STATE_1:
        if ( bPrevHallState == STATE_5 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = pHandle->PhaseShift + S16_60_PHASE_SHIFT;
        }
        else if ( bPrevHallState == STATE_3 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT );
        }
        else
        {
        }
        break;

      case STATE_3:
        if ( bPrevHallState == STATE_1 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT );
        }
        else if ( bPrevHallState == STATE_2 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT +
                                                  S16_60_PHASE_SHIFT );
        }
        else
        {
        }

        break;

      case STATE_2:
        if ( bPrevHallState == STATE_3 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT
                                                  + S16_60_PHASE_SHIFT );
        }
        else if ( bPrevHallState == STATE_6 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_120_PHASE_SHIFT );
        }
        else
        {
        }
        break;

      case STATE_6:
        if ( bPrevHallState == STATE_2 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_120_PHASE_SHIFT );
        }
        else if ( bPrevHallState == STATE_4 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT );
        }
        else
        {
        }
        break;

      case STATE_4:
        if ( bPrevHallState == STATE_6 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT );
        }
        else if ( bPrevHallState == STATE_5 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift );
        }
        else
        {
        }
        break;

      default:
        /* Bad hall sensor configutarion so update the speed reliability
          坏霍尔传感器配置更新速度可靠性 */
        pHandle->SensorIsReliable = false;

        break;
    }


#ifdef HALL_MTPA
    {
      pHandle->_Super.hElAngle = pHandle->MeasuredElAngle;
    }
#endif

    /* Discard first capture 丢弃第一次捕获*/
    if ( pHandle->FirstCapt == 0u )
    {
      pHandle->FirstCapt++;
      LL_TIM_IC_GetCaptureCH1( TIMx );
    }
    else
    {
      /* used to validate the average speed measurement 用于验证平均速度测量 */
      if ( pHandle->BufferFilled < pHandle->SpeedBufferSize )
      {
        pHandle->BufferFilled++;
      }

      /* Store the latest speed acquisition 存储最新的速度采集 */
      hHighSpeedCapture = LL_TIM_IC_GetCaptureCH1( TIMx );
      wCaptBuf = ( uint32_t )hHighSpeedCapture;
      hPrscBuf =  LL_TIM_GetPrescaler ( TIMx );

      /* Add the numbers of overflow to the counter  将溢出次数添加到计数器*/
      wCaptBuf += ( uint32_t )pHandle->OVFCounter * 0x10000uL;

      if ( pHandle->OVFCounter != 0u )
      {
        /* Adjust the capture using prescaler 使用预分频器调整捕获 */
        uint16_t hAux;
        hAux = hPrscBuf + 1u;
        wCaptBuf *= hAux;

        if ( pHandle->RatioInc )
        {
          pHandle->RatioInc = false;  /* Previous capture caused overflow 以前的捕获导致溢出*/
          /* Don't change prescaler (delay due to preload/update mechanism) 不要更改预分频器（由于预加载/更新机制导致的延迟） */
        }
        else
        {
          if ( LL_TIM_GetPrescaler ( TIMx ) < pHandle->HALLMaxRatio ) /* Avoid OVF w/ very low freq 避免OVF w /非常低的频率*/
          {
            LL_TIM_SetPrescaler ( TIMx, LL_TIM_GetPrescaler ( TIMx ) + 1 ); /* To avoid OVF during speed decrease 在速度降低期间避免OVF*/
            pHandle->RatioInc = true;   /* new prsc value updated at next capture only 新的prsc值仅在下次捕获时更新*/
          }
        }
      }
      else
      {
        /* If prsc preload reduced in last capture, store current register + 1 如果在上次捕获时prsc预加载减少，则存储当前寄存器+ 1*/
        if ( pHandle->RatioDec ) /* and don't decrease it again 并且不要再减少它*/
        {
          /* Adjust the capture using prescaler 使用预分频器调整捕获*/
          uint16_t hAux;
          hAux = hPrscBuf + 2u;
          wCaptBuf *= hAux;

          pHandle->RatioDec = false;
        }
        else  /* If prescaler was not modified on previous capture 如果在之前的捕获中未对预分频器进行修改*/
        {
          /* Adjust the capture using prescaler 使用预分频器调整捕获*/
          uint16_t hAux = hPrscBuf + 1u;
          wCaptBuf *= hAux;

          if ( hHighSpeedCapture < LOW_RES_THRESHOLD ) /* If capture range correct 如果捕获范围正确*/
          {
            if ( LL_TIM_GetPrescaler ( TIMx ) > 0u ) /* or prescaler cannot be further reduced 或预分频器不能进一步减少*/
            {
              LL_TIM_SetPrescaler ( TIMx, LL_TIM_GetPrescaler ( TIMx ) - 1 ); /* Increase accuracy by decreasing prsc 通过降低prsc来提高准确性*/
              /* Avoid decrementing again in next capt.(register preload delay) 避免在下一次捕获中再次递减。（注册预加载延迟）*/
              pHandle->RatioDec = true;
            }
          }
        }
      }

#if 0
      /* Store into the buffer 存入缓冲区*/
      /* Null Speed is detected, erase the buffer 检测到空速，擦除缓冲区 */
      if ( wCaptBuf > pHandle->MaxPeriod )
      {
        uint8_t bIndex;
        for ( bIndex = 0u; bIndex < pHandle->SpeedBufferSize; bIndex++ )
        {
          pHandle->SensorSpeed[bIndex]  = 0;
        }
        pHandle->BufferFilled = 0 ;
        pHandle->SpeedFIFOSetIdx = 1;
        pHandle->SpeedFIFOGetIdx = 0;
        /* Indicate new speed acquisitions 表示新的速度收购*/
        pHandle->NewSpeedAcquisition = 1;
        pHandle->ElSpeedSum = 0;
      }
      /* Filtering to fast speed... could be a glitch  ? 过滤到快速...可能是一个小故障？*/
      /* the HALL_MAX_PSEUDO_SPEED is temporary in the buffer, and never included in average computation
      HALL_MAX_PSEUDO_SPEED在缓冲区中是临时的，并且从不包含在平均计算中 */
      else
#endif
        if ( wCaptBuf < pHandle->MinPeriod )
        {
          pHandle->CurrentSpeed = HALL_MAX_PSEUDO_SPEED;
          pHandle->NewSpeedAcquisition = 0;
        }
        else
        {
          pHandle->ElSpeedSum -= pHandle->SensorSpeed[pHandle->SpeedFIFOIdx]; /* value we gonna removed from the accumulator我们要从累加器中删除的值 */
          if ( wCaptBuf >= pHandle->MaxPeriod )
          {
            pHandle->SensorSpeed[pHandle->SpeedFIFOIdx]  = 0;
          }
          else
          {
            pHandle->SensorSpeed[pHandle->SpeedFIFOIdx] = ( int16_t ) ( pHandle->PseudoFreqConv / wCaptBuf );
            pHandle->SensorSpeed[pHandle->SpeedFIFOIdx] *= pHandle->Direction;
            pHandle->ElSpeedSum += pHandle->SensorSpeed[pHandle->SpeedFIFOIdx];
          }
          /* Update pointers to speed buffer 更新指向速度缓冲区的指针*/
          pHandle->CurrentSpeed = pHandle->SensorSpeed[pHandle->SpeedFIFOIdx];
          pHandle->SpeedFIFOIdx++;
          if ( pHandle->SpeedFIFOIdx == pHandle->SpeedBufferSize )
          {
            pHandle->SpeedFIFOIdx = 0u;
          }
          /* Indicate new speed acquisitions 表示新的速度收购*/
          pHandle->NewSpeedAcquisition = 1;
        }
      /* Reset the number of overflow occurred 重置发生的溢出次数*/
      pHandle->OVFCounter = 0u;
    }
  }
  return MC_NULL;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
* @brief  Example of private method of the class HALL to implement an MC IRQ function
*         to be called when TIMx update event occurs
          类HALL的私有方法示例，用于实现在发生TIMx更新事件时要调用的MC IRQ函数
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
          pHandle：hall_speed_pos_fdbk组件的当前实例的处理程序
* @retval none
*/
void * HALL_TIMx_UP_IRQHandler( void * pHandleVoid )
{
  HALL_Handle_t * pHandle = ( HALL_Handle_t * ) pHandleVoid;
  TIM_TypeDef * TIMx = pHandle->TIMx;

  if ( pHandle->SensorIsReliable )
  {
    uint16_t hMaxTimerOverflow;
    /* an update event occured for this interrupt request generation 为此中断请求生成发生更新事件*/
    pHandle->OVFCounter++;

    hMaxTimerOverflow = ( uint16_t )( ( ( uint32_t )pHandle->HallTimeout * pHandle->OvfFreq )
                                      / ( ( LL_TIM_GetPrescaler ( TIMx ) + 1 ) * 1000u ) );
    if ( pHandle->OVFCounter >= hMaxTimerOverflow )
    {
      /* Set rotor speed to zero 将转子速度设置为零*/
      pHandle->_Super.hElSpeedDpp = 0;

      /* Reset the electrical angle according the hall sensor configuration 根据霍尔传感器配置重置电角度*/
      HALL_Init_Electrical_Angle( pHandle );

      /* Reset the overflow counter 重置溢出计数器*/
      pHandle->OVFCounter = 0u;


#if 1
      /* Reset the SensorSpeed buffer 重置SensorSpeed缓冲区 */
      uint8_t bIndex;
      for ( bIndex = 0u; bIndex < pHandle->SpeedBufferSize; bIndex++ )
      {
        pHandle->SensorSpeed[bIndex]  = 0;
      }
      pHandle->BufferFilled = 0 ;
      pHandle->CurrentSpeed = 0;
      pHandle->SpeedFIFOIdx = 1;
      pHandle->ElSpeedSum = 0;
#else
      pHandle->ElSpeedSum -= pHandle->SensorSpeed[pHandle->SpeedFIFOIdx];
      pHandle->SensorSpeed[pHandle->SpeedFIFOSetIdx]  = 0;
      pHandle->CurrentSpeed = 0;
      pHandle->SpeedFIFOIdx++;
      if ( pHandle->SpeedFIFOIdx == pHandle->SpeedBufferSize )
      {
        pHandle->SpeedFIFOIdx = 0u;
      }
#endif
    }
  }
  return MC_NULL;
}

/**
* @brief  Compute and returns the average rotor electrical speed express in dpp
          计算并返回以dpp表示的平均转子电速
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
          pHandle：hall_speed_pos_fdbk组件的当前实例的处理程序
* @retval int16_t the average rotor electrical speed express in dpp
          int16_t平均转子电速以dpp表示
*/
static int16_t HALL_CalcAvrgElSpeedDpp( HALL_Handle_t * pHandle )
{

  if ( pHandle->NewSpeedAcquisition == 1 )
  {

    if ( pHandle->BufferFilled < pHandle->SpeedBufferSize )
    {
      pHandle->AvrElSpeedDpp = ( int16_t )  pHandle->CurrentSpeed;
    }
    else
    {
      pHandle->AvrElSpeedDpp = ( int16_t )( pHandle->ElSpeedSum / ( int32_t )(
                                              pHandle->SpeedBufferSize ) ); /* Average value */
    }

    /* Clear new speed acquisitions flag 清除新的速度收购标志*/
    pHandle->NewSpeedAcquisition = 0;
  }

  return pHandle->AvrElSpeedDpp;
}

/**
* @brief  Read the logic level of the three Hall sensor and individuates in this
*         way the position of the rotor (+/- 30ï¿½). Electrical angle is then
*         initialized.
          读取三个霍尔传感器的逻辑电平，并以此方式个别化转子的位置（+/- 30°）。然后初始化电角度。
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval none
*/
static void HALL_Init_Electrical_Angle( HALL_Handle_t * pHandle )
{

  if ( pHandle->SensorPlacement == DEGREES_120 )
  {
    pHandle->HallState  = LL_GPIO_IsInputPinSet( pHandle->H3Port, pHandle->H3Pin ) << 2
                          | LL_GPIO_IsInputPinSet( pHandle->H2Port, pHandle->H2Pin ) << 1
                          | LL_GPIO_IsInputPinSet( pHandle->H1Port, pHandle->H1Pin );
  }
  else
  {
    pHandle->HallState  = ( LL_GPIO_IsInputPinSet( pHandle->H2Port, pHandle->H2Pin ) ^ 1 ) << 2
                          | LL_GPIO_IsInputPinSet( pHandle->H3Port, pHandle->H3Pin ) << 1
                          | LL_GPIO_IsInputPinSet( pHandle->H1Port, pHandle->H1Pin );
  }

  switch ( pHandle->HallState )
  {
    case STATE_5:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_1:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_60_PHASE_SHIFT +
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_3:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT +
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_2:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift - S16_120_PHASE_SHIFT -
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_6:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT -
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_4:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT / 2 );
      break;
    default:
      /* Bad hall sensor configutarion so update the speed reliability
        坏霍尔传感器配置更新速度可靠性*/
      pHandle->SensorIsReliable = false;
      break;
  }

  /* Initialize the measured angle 初始化测量的角度 */
  pHandle->MeasuredElAngle = pHandle->_Super.hElAngle;

}

/**
  * @brief  It could be used to set istantaneous information on rotor mechanical
  *         angle.
  *         Note: Mechanical angle management is not implemented in this
  *         version of Hall sensor class.
  它可用于设置转子机械角度的等静态信息。
  注意：霍尔传感器类的版本中未实现机械角度管理。
  * @param  pHandle pointer on related component instance
          相关组件实例上的pHandle指针
  * @param  hMecAngle istantaneous measure of rotor mechanical angle
            hMec角度瞬时测量转子机械角度
  * @retval none
  */
void HALL_SetMecAngle( HALL_Handle_t * pHandle, int16_t hMecAngle )
{
}


/**
  * @}
  */

/**
  * @}
  */

/** @} */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
