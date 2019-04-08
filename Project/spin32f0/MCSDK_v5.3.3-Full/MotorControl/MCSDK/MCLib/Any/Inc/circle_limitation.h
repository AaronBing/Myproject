/**
  ******************************************************************************
  * @file    circle_limitation.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   ���ļ�����Motor Control SDK��Circle Limitation��������ж���ͺ���ԭ�͡�
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  * @ingroup CircleLimitation
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CIRCLELIMITATION_H
#define __CIRCLELIMITATION_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup CircleLimitation
  * @{
  */

/**
  * @brief  CircleLimitation component parameters definition
  */
typedef struct
{
  uint16_t MaxModule;               /**<  Բ�����������ģ�� */
  uint16_t Circle_limit_table[87];  /**<  Բ���Ʊ� */
  uint8_t  Start_index;             /**<  Բ���Ʊ�������ʼ */
} CircleLimitation_Handle_t;

/* Exported functions ------------------------------------------------------- */

Volt_Components Circle_Limitation( CircleLimitation_Handle_t * pHandle, Volt_Components Vqd );

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __Circle Limitation_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

