/**
  ******************************************************************************
  * @file    mc_irq_handler.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the MC IRQ Handler
  *          component of the Motor Control SDK.
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
#include "mc_irq_handler.h"
#include <stddef.h>

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MC_IRQ_HANDLER Motor Control IRQ Handler
  * @brief Performs registration and execution of Interrupts handlers used for Motor Control
  *
  *  This component is a temporary work around allowing to use the former IRQ Handler
  * registration mechanism with both the old Classes and the new Components.
  *
  *  The former mechanism is defined in the MCIRQHandlerClass and is basically doing
  * two things.
  *
  *  The first one is to register an Object and a method of that object to be executed
  * when an given interrupt occurs. Interrupt sources are identified by a number; 4
  * such sources are defined:
  *
  * - #MC_IRQ_PWMNCURRFDBK_1
  * - #MC_IRQ_PWMNCURRFDBK_2
  * - #MC_IRQ_SPEEDNPOSFDBK_1
  * - #MC_IRQ_SPEEDNPOSFDBK_2
  *
  *  The second thing it does is to provide a function that executes the Interrupt handler
  * registered for a given interrupt source.
  *
  *  The implementation of the MCIRQHandlerClass mandated that the first field of the
  * structure of the registered Object be a pointer on the interrupt handling function to
  * execute.
  *
  *  This constraint is unacceptable for Components and this is the reason why this work-around
  * is proposed.
  *
  *  It is only a work around as the actual interrupt handling method to be used on the 5.0.0
  * onwards will rely on Cube FW mechanisms and is still TBD in details.
  *
  *  This work around is designed to work both with remnants of the former 4.3 architecture and
  * with the new components from the 5.0 architecture. It minimizes the changes to be made in the
  * legacy 4.3 code by providing two macros named after the MCIRQHandlerClass methods:
  *
  * - #Set_IRQ_Handler
  * - #Exec_IRQ_Handler
  *
  *  And it provides two functions, similar to the former MCIRQHandlerClass's methods, but designed
  * to work with the new components.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/**
  * @brief Interrupt handling configuration structure
  *
  *  The #Handler field is a pointer on the Interrupt handling function, while the
  * #Handle field is a pointer that is passed as the first parameter to the interrupt
  * handling function when it is executed.
  */
typedef struct
{
  MCIRQ_Handler_t Handler;
  void      *      Handle;
} MCIRQ_HandlerConfigItem_t;

/* Private defines -----------------------------------------------------------*/
/** @brief Number of Interrupt handlers managed by the component */
#define MCIRQ_MAX_HANDLERS 4


/* Static variables ----------------------------------------------------------*/
/** @brief Table containing the Interrupt handling configurations */
static MCIRQ_HandlerConfigItem_t MCIRQ_Table[MCIRQ_MAX_HANDLERS];

/* Functions ---------------------------------------------------- */

/**
 * @brief Registers function @p Handler as the handler for the interrupt identified by @p IrqId.
 *
 *  @p Handle is also registered and passed as first argument to the @p Handler function when it
 * is executed.
 */
void MCIRQ_SetIrqHandler( uint8_t IrqId, MCIRQ_Handler_t Handler, void * Handle )
{
  if ( IrqId < MCIRQ_MAX_HANDLERS )
  {
    MCIRQ_Table[ IrqId ].Handler = Handler;
    MCIRQ_Table[ IrqId ].Handle  = Handle;
  }
}

/** @brief Executes the handler registered with identifier @p IrqId and returns its return value.
 *
 *  @p Flag is passed as second argument to the handler function, the first being the pointer
 * that was registered with it.
 */
void * MCIRQ_ExecIrqHandler( uint8_t IrqId, uint8_t Flag )
{
  void * ret_val = NULL;

  if ( IrqId < MCIRQ_MAX_HANDLERS )
  {
    ret_val = MCIRQ_Table[ IrqId ].Handler( MCIRQ_Table[ IrqId ].Handle, Flag );
  }

  return ret_val;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
