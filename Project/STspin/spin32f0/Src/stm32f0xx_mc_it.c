
/**
  ******************************************************************************
  * @file    stm32f0xx_mc_it.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt 
  *          service routine related to Motor Control for the STM32F0 Family.
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
  * @ingroup STM32F0xx_IRQ_Handlers
  */ 

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_config.h"
#include "mc_tasks.h"
#include "ui_task.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "stm32f0xx_ll_exti.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32F0xx_IRQ_Handlers STM32F0xx IRQ Handlers
  * @{
  */

/* USER CODE BEGIN PRIVATE */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY/1000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

/* Public prototypes of IRQ handlers called from assembly code ---------------*/
void CURRENT_REGULATION_IRQHandler(void);
void TIMx_UP_BRK_M1_IRQHandler(void);
void DMAx_R1_M1_IRQHandler(void);
void SPD_TIM_M1_IRQHandler(void);
void USART_IRQHandler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);
void EXTI0_1_IRQHandler (void);

/**
  * @brief  This function handles current regulation interrupt request.
  * @param  None
  * @retval None
  */
void CURRENT_REGULATION_IRQHandler(void)
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 0 */

  /* USER CODE END CURRENT_REGULATION_IRQn 0 */
  
  /* Clear Flags */
  DMA1->IFCR = (LL_DMA_ISR_GIF1|LL_DMA_ISR_TCIF1|LL_DMA_ISR_HTIF1);
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 1 */

  /* USER CODE END CURRENT_REGULATION_IRQn 1 */   
  
    TSK_HighFrequencyTask();          /*GUI, this section is present only if DAC is disabled*/
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 2 */

  /* USER CODE END CURRENT_REGULATION_IRQn 2 */   
}

/**
  * @brief  This function handles first motor TIMx Update, Break-in interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_UP_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 0 */   

  if(LL_TIM_IsActiveFlag_UPDATE(PWM_Handle_M1.pParams_str->TIMx) && LL_TIM_IsEnabledIT_UPDATE(PWM_Handle_M1.pParams_str->TIMx))
  {
    LL_TIM_ClearFlag_UPDATE(PWM_Handle_M1.pParams_str->TIMx);
    /* USER CODE BEGIN PWM_Update */

    /* USER CODE END PWM_Update */  
  }
  if(LL_TIM_IsActiveFlag_BRK(PWM_Handle_M1.pParams_str->TIMx) && LL_TIM_IsEnabledIT_BRK(PWM_Handle_M1.pParams_str->TIMx)) 
  {
    LL_TIM_ClearFlag_BRK(PWM_Handle_M1.pParams_str->TIMx);      
    F0XX_BRK_IRQHandler(&PWM_Handle_M1);
    /* USER CODE BEGIN Break */

    /* USER CODE END Break */ 
  }
  else 
  {
   /* No other interrupts are routed to this handler */
  }
  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 1 */   
}

/**
  * @brief  This function handles first motor DMAx TC interrupt request. 
  *         Required only for R1 with rep rate > 1
  * @param  None
  * @retval None
  */
void DMAx_R1_M1_IRQHandler(void)
{
  /* USER CODE BEGIN DMAx_R1_M1_IRQn 0 */

  /* USER CODE END DMAx_R1_M1_IRQn 0 */ 
  
  if (LL_DMA_IsActiveFlag_TC4(DMA1))
  {
    LL_DMA_ClearFlag_TC4(DMA1);
    /* USER CODE BEGIN DMAx_R1_M1_TC4 */

    /* USER CODE END DMAx_R1_M1_TC4 */     
  } 
  /* USER CODE BEGIN DMAx_R1_M1_IRQn 1 */

  /* USER CODE END DMAx_R1_M1_IRQn 1 */ 
}

/*Start here***********************************************************/
/*GUI, this section is present only if serial communication is enabled*/
/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  * @retval None
  */
void USART_IRQHandler(void)
{
  /* USER CODE BEGIN USART_IRQn 0 */

  /* USER CODE END USART_IRQn 0 */ 
  
  if (LL_USART_IsActiveFlag_RXNE(pUSART.USARTx))
  {
    uint16_t retVal;
    retVal = *(uint16_t*)(UFCP_RX_IRQ_Handler(&pUSART,LL_USART_ReceiveData8(pUSART.USARTx)));
    if (retVal == 1)
    {
      UI_SerialCommunicationTimeOutStart();
    }
    if (retVal == 2)
    {
      UI_SerialCommunicationTimeOutStop();
    }
  /* USER CODE BEGIN USART_RXNE */

  /* USER CODE END USART_RXNE   */ 	
	
  } 
  
  if (LL_USART_IsActiveFlag_TXE(pUSART.USARTx))
  {
    UFCP_TX_IRQ_Handler(&pUSART);
    /* USER CODE BEGIN USART_TXE */

    /* USER CODE END USART_TXE   */
  }
  
  if (LL_USART_IsActiveFlag_ORE(pUSART.USARTx)) /* Overrun error occurs */
  {
    /* Send Overrun message */
    UFCP_OVR_IRQ_Handler(&pUSART);
    LL_USART_ClearFlag_ORE(pUSART.USARTx); /* Clear overrun flag */
    UI_SerialCommunicationTimeOutStop();
    /* USER CODE BEGIN USART_ORE */

    /* USER CODE END USART_ORE   */   
  }
  
  /* USER CODE BEGIN USART_IRQn 1 */
  
  /* USER CODE END USART_IRQn 1 */

}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
 /* USER CODE BEGIN HardFault_IRQn 0 */

 /* USER CODE END HardFault_IRQn 0 */
  TSK_HardwareFaultTask();
  
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
    {
      if (LL_USART_IsActiveFlag_ORE(pUSART.USARTx)) /* Overrun error occurs */
      {
        /* Send Overrun message */
        UFCP_OVR_IRQ_Handler(&pUSART);
        LL_USART_ClearFlag_ORE(pUSART.USARTx); /* Clear overrun flag */
        UI_SerialCommunicationTimeOutStop();
      }
      
      if (LL_USART_IsActiveFlag_TXE(pUSART.USARTx))
      {   
        UFCP_TX_IRQ_Handler(&pUSART);
      }  
      
      if (LL_USART_IsActiveFlag_RXNE(pUSART.USARTx)) /* Valid data have been received */
      {
        uint16_t retVal;
        retVal = *(uint16_t*)(UFCP_RX_IRQ_Handler(&pUSART,LL_USART_ReceiveData8(pUSART.USARTx)));
        if (retVal == 1)
        {
          UI_SerialCommunicationTimeOutStart();
        }
        if (retVal == 2)
        {
          UI_SerialCommunicationTimeOutStop();
        }
      }
      else
      {
      }
    }  
  }
 /* USER CODE BEGIN HardFault_IRQn 1 */

 /* USER CODE END HardFault_IRQn 1 */

}

void SysTick_Handler(void)
{
#ifdef MC_HAL_IS_USED
static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  if (SystickDividerCounter == SYSTICK_DIVIDER)
  {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    SystickDividerCounter = 0;
  }
  SystickDividerCounter ++;  
#endif /* MC_HAL_IS_USED */

  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
    MC_RunMotorControlTasks();
  /* USER CODE BEGIN SysTick_IRQn 2 */
  /* USER CODE END SysTick_IRQn 2 */
}

/*GUI, this section is present only if start/stop button is enabled*/
/**
  * @brief  This function handles Button IRQ on PIN PF0.
  */
void EXTI0_1_IRQHandler (void)
{
/* USER CODE BEGIN START_STOP_BTN */
  LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_0);
  UI_HandleStartStopButton_cb ();
/* USER CODE END START_STOP_BTN */
}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
