/**
  ******************************************************************************
  * @file    digital_output.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   ���ļ��ṩʵ��Motor Control SDK��Digital Output����Ĺ̼�����:
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
* @brief ��ˣ�ͨ��ѡ���ԣ����������������Ϊ��Ч����Ч
* @param pHandle�����������Ĵ�������ַ��
* @param OutputState_t�������״̬
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
