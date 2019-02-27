/**
  ******************************************************************************
  * @file    circle_limitation.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Circle Limitation component of the Motor Control SDK.
            此文件包含Motor Control SDK的 Circle Limitation组件的所有定义和函数原型。
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
  uint16_t MaxModule;               /**<  Circle limitation maximum allowed
                                         module */
  uint16_t Circle_limit_table[87];  /**<  Circle limitation table */
  uint8_t  Start_index;             /**<  Circle limitation table indexing
                                         start */
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
