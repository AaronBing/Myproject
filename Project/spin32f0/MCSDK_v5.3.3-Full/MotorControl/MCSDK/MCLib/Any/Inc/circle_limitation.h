/**
  ******************************************************************************
  * @file    circle_limitation.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   此文件包含Motor Control SDK的Circle Limitation组件的所有定义和函数原型。
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
  uint16_t MaxModule;               /**<  圆限制最大允许模块 */
  uint16_t Circle_limit_table[87];  /**<  圆限制表 */
  uint8_t  Start_index;             /**<  圆限制表索引开始 */
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

