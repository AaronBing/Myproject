/**
  ******************************************************************************
  * @file    digital_output.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   此文件提供实现Motor Control SDK的Digital Output组件的固件功能:
  *
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "digital_output.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup DigitalOutput Digital Output
  * @brief digital output component of the Motor Control SDK
  *
  * @{
  */


/**
* @brief 因此，通过选择极性，它将数字输出设置为有效或无效
* @param pHandle数字输出组件的处理程序地址。
* @param OutputState_t新请求的状态
* @retval none
*/
void DOUT_SetOutputState( DOUT_handle_t * pHandle, DOutputState_t State )
{

  if ( State == ACTIVE )
  {
    if ( pHandle->bDOutputPolarity == DOutputActiveHigh )
    {
      LL_GPIO_SetOutputPin( pHandle->hDOutputPort, pHandle->hDOutputPin );
    }
    else
    {
      LL_GPIO_ResetOutputPin( pHandle->hDOutputPort, pHandle->hDOutputPin );
    }
  }
  else if ( pHandle->bDOutputPolarity == DOutputActiveHigh )
  {
    LL_GPIO_ResetOutputPin( pHandle->hDOutputPort, pHandle->hDOutputPin );
  }
  else
  {
    LL_GPIO_SetOutputPin( pHandle->hDOutputPort, pHandle->hDOutputPin );
  }
  pHandle->OutputState = State;
}

/**
* @brief It returns the state of the digital output
* @param pHandle pointer on related component instance
* @retval OutputState_t Digital output state (ACTIVE or INACTIVE)
*/
DOutputState_t DOUT_GetOutputState( DOUT_handle_t * pHandle )
{
  return ( pHandle->OutputState );
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
