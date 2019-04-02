/**
  ******************************************************************************
  * @file    mc_irq_handler.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          MC IRQ Handler component of the Motor Control SDK. This component is
  *          a temporary work around allowing to use the former IRQ Handler
  *          registration mechanism with both the old Classes and the new Components.
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
  * @ingroup MC_IRQ_HANDLER
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_IRQ_HANDLER_H
#define __MC_IRQ_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MC_IRQ_HANDLER
  * @{
  */

/* Exported constants --------------------------------------------------------*/
#define MC_IRQ_PWMNCURRFDBK_1   0u  /*!< Identifies the IRQ Handler for PWMnCurrFdbk, first instance.*/
#define MC_IRQ_PWMNCURRFDBK_2   1u  /*!< Identifies the IRQ Handler for PWMnCurrFdbk, second instance.*/
#define MC_IRQ_SPEEDNPOSFDBK_1  2u  /*!< Identifies the IRQ Handler for SpeednPosFdbk, first instance.*/
#define MC_IRQ_SPEEDNPOSFDBK_2  3u  /*!< Identifies the IRQ Handler for SpeednPosFdbk, second instance.*/

/** @brief Fake definition used avoid build issues with the users of the former MCIRQHandlerClass */
#define _CMCIRQ void *

/* Exported macros --------------------------------------------------------*/
/** @brief Bridges the Set_IRQ_Handler from the 4.3 implementation to the new and temporary MCIRQ_SetIrqHandler function
 *
 * This function registers @p Obj as the IRQ handler for the Interrupt identified with @p Id. @p Obj is both a pointer on
 * the instance of a class @b and a pointer a pointer on the interrupt handling function. This means that the first field
 * of the object's structure must be a pointer on the interrupt handling function. This hidden constraint only exists until
 * 4.3, not from 5.0 onwards.
 *
 * @b BEWARE: This macro is meant to be used by 4.3 code only. 5.0 code (components and Cockpit) shall NOT use it.
 */
#define Set_IRQ_Handler( Id, Obj ) MCIRQ_SetIrqHandler( (Id), *(MCIRQ_Handler_t *)(Obj), (void *)(Obj) )

/** @brief Bridges the Exec_IRQ_Handler from the 4.3 implementation to the new and temporary MCIRQ_ExecIrqHandler function
 *
 * This function executes the IRQ handler for the Interrupt identified with @p Id. The executed IRQ handler must have been
 * registered prior to calling this macro. See Set_IRQ_Handler for this.
 *
 * The executed IRQ Handling function is passed two parameters. The first one is a pointer on the object the function is
 * to work on, as registered with the Set_IRQ_Handler function. The second one is the uint8_t value @p Flag.
 *
 * @b BEWARE: This macro is meant to be used by 4.3 code only. 5.0 code (components and Cockpit) shall NOT use it.
 */
#define Exec_IRQ_Handler( Id, Flag ) MCIRQ_ExecIrqHandler( Id, Flag )

/* Exported types ------------------------------------------------------------*/

/**
* @brief Type of an Interrupt handling function
*
*  Interrupt handling functions accept two parameters:
*
* - a void pointer that is supposed  to contain either a pointer on the handle on
* the component the function operates on or a pointer on the object of which the
* function is a method. This pointer is registered with MCIRQ_SetIrqHandler().
* - a uint8_t flag that is passed to the MCIRQ_ExecIrqHandler() function.
*
* Interrupt handling functions return a void pointer.
*/
typedef void * ( *MCIRQ_Handler_t )( void *, uint8_t );

/* Exported functions ------------------------------------------------------- */

/* Sets the function to invoque when the interrupt identified by IrqId occurs. */
void MCIRQ_SetIrqHandler( uint8_t IrqId, MCIRQ_Handler_t Handler, void * Handle );

/* Executes the function registered for the interrupt identified by IrqId */
void * MCIRQ_ExecIrqHandler( uint8_t IrqId, uint8_t Flag );

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __MC_IRQ_HANDLER_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
