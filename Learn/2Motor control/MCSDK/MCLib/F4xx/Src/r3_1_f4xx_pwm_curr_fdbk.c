/**
  ******************************************************************************
  * @file    r3_1_F4XX_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement current sensor
  *          class to be stantiated when the three shunts current sensing
  *          topology is used. It is specifically designed for STM32F302x8
  *          microcontrollers.
  *           + MCU peripheral and handle initialization function
  *           + three shunt current sesnsing
  *           + space vector modulation function
  *           + ADC sampling function
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
#include "r3_1_f4xx_pwm_curr_fdbk.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup r3_1_F4XX_pwm_curr_fdbk R3 1 ADC F30x PWM & Current Feedback
 *
 * @brief STM32F3, 1 ADC, 3-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F3 MCU, using a three
 * shunt resistors current sensing topology and only one ADC to acquire the current
 * values. This is typically the implementation to use with STM32F301xx and STM32F302xx
 * MCUs that only have one ADC peripheral.
 *
 * It computes the PWM duty cycles on each PWM period, applies them to the Motor phases and reads
 * the current flowing through the Motor phases. It is built on the @ref pwm_curr_fdbk component.
 *
 * Instances of this component are managed by a PWMC_R3_1_F4_Handle_t Handle structure that needs
 * to be initialized with the R3_1_F4XX_Init() prior to being used.
 *
 * Usually, the R3_1_F4XX_Init() function i the only function of this component that needs to be
 * called directly. Its other functions are usually invoked by functions of the @ref pwm_curr_fdbk
 * base component.
 *
 * @section r3_1_F4XX_periph_usage Peripheral usage
 * The PWMC_R3_1_F4 uses the following IPs of the STM32 MCU....
 *
 * * 1 Advanced Timer: 3 PWM channels, their output pins and optionally their complemented output
 * * 1 ADC: Three channels of the ADC are used. The ADC to choose can be triggered by the Timer....
 * * ...
 *
 * @todo: TODO: complete documentation.
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123        ((uint32_t)  0x00000555u)
#define NB_CONVERSIONS 16u

/* DIR bits of TIM1 CR1 register identification for correct check of Counting direction detection*/
#define DIR_MASK 0x0010u       /* binary value: 0000000000010000 */


/* Private typedef -----------------------------------------------------------*/


/** 
  * @brief  ADC Init structure definition  
  */
typedef struct
{
  uint8_t ADC_NbrOfInjecChannel;               /*!< Specifies the number of ADC channels that will be converted
                                                    using the sequencer for injected channel group.
                                                    This parameter must range from 1 to 4. */ 
  uint32_t ADC_InjecSequence1; 
  uint32_t ADC_InjecSequence2;
  uint32_t ADC_InjecSequence3;
  uint32_t ADC_InjecSequence4;                                            
}ADC_InjectedInitTypeDef;

/* Private function prototypes -----------------------------------------------*/
static void R3_1_F4XX_HFCurrentsCalibrationAB(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents);
static void R3_1_F4XX_HFCurrentsCalibrationC(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents);
static uint16_t R3_1_F4XX_WriteTIMRegisters(PWMC_Handle_t *pHdl, uint16_t hCCR4Reg);
static uint32_t SingleADC_InjectedConfig(ADC_TypeDef* ADCx,
                                         ADC_InjectedInitTypeDef* ADC_InjectedInitStruct);


/* Private functions ---------------------------------------------------------*/

/* Local redefinition of both LL_TIM_OC_EnablePreload & LL_TIM_OC_DisablePreload */
__STATIC_INLINE void __LL_TIM_OC_EnablePreload(TIM_TypeDef *TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register volatile uint32_t *pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1) + OFFSET_TAB_CCMRx[iChannel]));
  SET_BIT(*pReg, (TIM_CCMR1_OC1PE << SHIFT_TAB_OCxx[iChannel]));
}

__STATIC_INLINE void __LL_TIM_OC_DisablePreload(TIM_TypeDef *TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register volatile uint32_t *pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1) + OFFSET_TAB_CCMRx[iChannel]));
  CLEAR_BIT(*pReg, (TIM_CCMR1_OC1PE << SHIFT_TAB_OCxx[iChannel]));
}


/**
  * @brief  It initializes peripherals for current reading and PWM generation
  *         in three shunts configuration using STM32F302x8
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F4XX_Init(PWMC_R3_1_F4_Handle_t *pHandle)
{
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx  = pHandle->pParams_str->ADCx;
  ADC_InjectedInitTypeDef ADC_InjectedInitStruct;

  if ((uint32_t)pHandle == (uint32_t)&pHandle->_Super)
  {

    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
    LL_ADC_DisableIT_EOCS(ADCx);
    LL_ADC_ClearFlag_EOCS(ADCx);
    LL_ADC_DisableIT_JEOS(ADCx);
    LL_ADC_ClearFlag_JEOS(ADCx);

    /* disable main TIM counter to ensure
     * a synchronous start by TIM2 trigger */
    LL_TIM_DisableCounter(TIMx);
    LL_TIM_ClearFlag_BRK(TIMx);

    LL_TIM_EnableIT_BRK(TIMx);

    LL_ADC_Enable(ADCx);

    /* Configuration of ADC sequence of two currents for the future JSQR register setting*/
    ADC_InjectedInitStruct.ADC_NbrOfInjecChannel = 2u;

    /*AB currents sequence --------------------------------------------------------------------------------*/
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIaChannel;
    ADC_InjectedInitStruct.ADC_InjecSequence2 = pHandle->pParams_str->bIbChannel;

    pHandle->wADC_JSQR_phAB= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);

    /*BA currents sequence --------------------------------------------------------------------------------*/
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIbChannel;
    ADC_InjectedInitStruct.ADC_InjecSequence2 = pHandle->pParams_str->bIaChannel;

    pHandle->wADC_JSQR_phBA= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);

    /*AC currents sequence --------------------------------------------------------------------------------*/
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIaChannel;
    ADC_InjectedInitStruct.ADC_InjecSequence2 = pHandle->pParams_str->bIcChannel;

    pHandle->wADC_JSQR_phAC= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);

    /*CA currents sequence --------------------------------------------------------------------------------*/
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIcChannel;
    ADC_InjectedInitStruct.ADC_InjecSequence2 = pHandle->pParams_str->bIaChannel;

    pHandle->wADC_JSQR_phCA= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);

    /*BC currents sequence --------------------------------------------------------------------------------*/
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIbChannel;
    ADC_InjectedInitStruct.ADC_InjecSequence2 = pHandle->pParams_str->bIcChannel;

    pHandle->wADC_JSQR_phBC= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);

    /*CB currents sequence -------------------------------------------------------------------------------- */
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIcChannel;
    ADC_InjectedInitStruct.ADC_InjecSequence2 = pHandle->pParams_str->bIbChannel;

    pHandle->wADC_JSQR_phCB= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);
    /* ---------------------------------------------------------------------------------------------------- */

    /*conversion will be allowed at PWM switch on time */
    /* CubeMX overwrite that should be removed latter on */
    LL_ADC_INJ_StopConversionExtTrig(ADCx);
    LL_ADC_INJ_SetSequencerDiscont (ADCx, LL_ADC_INJ_SEQ_DISCONT_DISABLE );
    /* End of CubeMX overwrite */
    LL_ADC_EnableIT_JEOS(ADCx);

    pHandle->_Super.DTTest = 0u;
    pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;

  }
}

/**
  * @brief  It measures and stores into handler component variables the offset voltage on Ia and
  *         Ib current feedback analog channels when no current is flowing into the
  *         motor
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F4XX_CurrentReadingCalibration(PWMC_Handle_t *pHdl)
{
  uint16_t hCalibrationPeriodCounter;
  uint16_t hMaxPeriodsNumber;
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  pHandle->wPhaseAOffset = 0u;
  pHandle->wPhaseBOffset = 0u;
  pHandle->wPhaseCOffset = 0u;
  
  pHandle->bIndex=0u;
  
  /* It forces inactive level on TIMx CHy and CHyN */
  LL_TIM_CC_DisableChannel (TIMx, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
                                  LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
                                  LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N ); 
   
  /* Offset calibration for A & B phases */
  /* Change function to be executed in ADCx_ISR */ 
  pHandle->_Super.pFctGetPhaseCurrents = &R3_1_F4XX_HFCurrentsCalibrationAB;
  pHandle->_Super.pFctSetADCSampPointSect1 = &R3_1_F4XX_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect2 = &R3_1_F4XX_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect3 = &R3_1_F4XX_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect4 = &R3_1_F4XX_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect5 = &R3_1_F4XX_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect6 = &R3_1_F4XX_SetADCSampPointCalibration;
  
  pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;
  
  R3_1_F4XX_SwitchOnPWM(&pHandle->_Super);
  
  /* Wait for NB_CONVERSIONS to be executed */
  hMaxPeriodsNumber=(NB_CONVERSIONS+1u)*(((uint16_t)(pHandle->pParams_str->bRepetitionCounter)+1u)>>1);
  LL_TIM_ClearFlag_CC1(TIMx);
  hCalibrationPeriodCounter = 0u;
  while (pHandle->bIndex < NB_CONVERSIONS)
  {
    if (LL_TIM_IsActiveFlag_CC1(TIMx))
    {
      LL_TIM_ClearFlag_CC1(TIMx);
      hCalibrationPeriodCounter++;
      if (hCalibrationPeriodCounter >= hMaxPeriodsNumber)
      {
        if (pHandle->bIndex < NB_CONVERSIONS)
        {
          pHandle->_Super.SWerror = 1u;
          break;
        }
      }
    }
  }
  
  R3_1_F4XX_SwitchOffPWM(&pHandle->_Super);

  /* Offset calibration for C phase */
  /* Reset bIndex */
  pHandle->bIndex=0u;

  /* Change function to be executed in ADCx_ISR */ 
  pHandle->_Super.pFctGetPhaseCurrents = &R3_1_F4XX_HFCurrentsCalibrationC;

/* Change ADC1 JSQR to select phase C on JDR1      */
  pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phCA;
  
  R3_1_F4XX_SwitchOnPWM(&pHandle->_Super);
  
  /* Wait for NB_CONVERSIONS to be executed */
  LL_TIM_ClearFlag_CC1(TIMx);
  hCalibrationPeriodCounter = 0u;
  while (pHandle->bIndex < NB_CONVERSIONS)
  {
    if (LL_TIM_IsActiveFlag_CC1(TIMx))
    {
      LL_TIM_ClearFlag_CC1(TIMx);
      hCalibrationPeriodCounter++;
      if (hCalibrationPeriodCounter >= hMaxPeriodsNumber)
      {
        if (pHandle->bIndex < NB_CONVERSIONS)
        {
          pHandle->_Super.SWerror = 1u;
          break;
        }
      }
    }
  }
  
  R3_1_F4XX_SwitchOffPWM(&pHandle->_Super);
  
  /* The average is computed by divided by (NB_CONVERSIONS/2) because the actual
     value read by the ADC must be multiplied by 2 */
  pHandle->wPhaseAOffset = pHandle->wPhaseAOffset / (NB_CONVERSIONS/2);
  pHandle->wPhaseBOffset = pHandle->wPhaseBOffset / (NB_CONVERSIONS/2);
  pHandle->wPhaseCOffset = pHandle->wPhaseCOffset / (NB_CONVERSIONS/2);

  /* Change back function to be executed in ADCx_ISR */ 
  pHandle->_Super.pFctGetPhaseCurrents = &R3_1_F4XX_GetPhaseCurrents;
  pHandle->_Super.pFctSetADCSampPointSect1 = &R3_1_F4XX_SetADCSampPointSect1;
  pHandle->_Super.pFctSetADCSampPointSect2 = &R3_1_F4XX_SetADCSampPointSect2;
  pHandle->_Super.pFctSetADCSampPointSect3 = &R3_1_F4XX_SetADCSampPointSect3;
  pHandle->_Super.pFctSetADCSampPointSect4 = &R3_1_F4XX_SetADCSampPointSect4;
  pHandle->_Super.pFctSetADCSampPointSect5 = &R3_1_F4XX_SetADCSampPointSect5;
  pHandle->_Super.pFctSetADCSampPointSect6 = &R3_1_F4XX_SetADCSampPointSect6;
  
  /* To program the first samplig at the next switch on PWM */
  pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;

  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to 
     force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */  
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  
  LL_TIM_OC_SetCompareCH1 (TIMx,pHandle->Half_PWMPeriod);
  LL_TIM_OC_SetCompareCH2 (TIMx,pHandle->Half_PWMPeriod);
  LL_TIM_OC_SetCompareCH3 (TIMx,pHandle->Half_PWMPeriod);
  
  /* Enable TIMx preload */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  
  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  LL_TIM_CC_EnableChannel (TIMx, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
                                 LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
                                 LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N ); 
  pHandle->BrakeActionLock = false;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  It computes and return latest converted motor phase currents motor
  * @param pHdl: handler of the current instance of the PWM component
  * @retval Ia and Ib current in Curr_Components format
  */
void R3_1_F4XX_GetPhaseCurrents(PWMC_Handle_t *pHdl, Curr_Components* pStator_Currents)
{
  uint8_t bSector;
  int32_t wAux;
  uint16_t hReg1;
  uint16_t hReg2;
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;

  /* Clear the flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);
  
  hReg1 = LL_ADC_INJ_ReadConversionData12(pHandle->pParams_str->ADCx,LL_ADC_INJ_RANK_1)*2;
  hReg2 = LL_ADC_INJ_ReadConversionData12(pHandle->pParams_str->ADCx,LL_ADC_INJ_RANK_2)*2;
  
  bSector = (uint8_t)(pHandle->_Super.hSector);
  
  switch (bSector)
  {
  case SECTOR_4:
  case SECTOR_5:
    {
      /* Current on Phase C is not accessible     */
      
      /* Ia = PhaseAOffset - ADC converted value) */
      if(bSector == SECTOR_4)
      {
        wAux = (int32_t)(pHandle->wPhaseAOffset)-(int32_t)(hReg2);
      }
      else
      {
        wAux = (int32_t)(pHandle->wPhaseAOffset)-(int32_t)(hReg1);
      }
      
      /* Saturation of Ia */
      if (wAux < -INT16_MAX)
      {
        pStator_Currents->qI_Component1= -INT16_MAX;
      }
      else  if (wAux > INT16_MAX)
      {
        pStator_Currents->qI_Component1= INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1= (int16_t)wAux;
      }
      
      /* Ib = PhaseBOffset - ADC converted value) */
      if(bSector == SECTOR_4)
      {
        wAux = (int32_t)(pHandle->wPhaseBOffset)-(int32_t)(hReg1);
      }
      else
      {
        wAux = (int32_t)(pHandle->wPhaseBOffset)-(int32_t)(hReg2);
      }
      
      /* Saturation of Ib */
      if (wAux < -INT16_MAX)
      {
        pStator_Currents->qI_Component2= -INT16_MAX;
      }
      else  if (wAux > INT16_MAX)
      {
        pStator_Currents->qI_Component2= INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2= (int16_t)wAux;
      }
    }
    break;
    
  case SECTOR_6:
  case SECTOR_1:
    {
      /* Current on Phase A is not accessible     */
      
      /* Ib = PhaseBOffset - ADC converted value) */
      if(bSector == SECTOR_6)
      {
        wAux = (int32_t)(pHandle->wPhaseBOffset)-(int32_t)(hReg2);
      }
      else
      {
        wAux = (int32_t)(pHandle->wPhaseBOffset)-(int32_t)(hReg1);
      }
      
      /* Saturation of Ib */
      if (wAux < -INT16_MAX)
      {
        pStator_Currents->qI_Component2= -INT16_MAX;
      }
      else  if (wAux > INT16_MAX)
      {
        pStator_Currents->qI_Component2= INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2= (int16_t)wAux;
      }
      
      /* Ic = PhaseCOffset - ADC converted value) */
      /* Ia = -Ic -Ib */
      if(bSector == SECTOR_6)
      {
        wAux = (int32_t)(pHandle->wPhaseCOffset)-(int32_t)(hReg1);
      }
      else
      {
        wAux = (int32_t)(pHandle->wPhaseCOffset)-(int32_t)(hReg2);
      }
      
      wAux = -wAux - (int32_t)pStator_Currents->qI_Component2;
      
      /* Saturation of Ia */
      if (wAux> INT16_MAX)
      {
        pStator_Currents->qI_Component1 = INT16_MAX;
      }
      else  if (wAux <-INT16_MAX)
      {
        pStator_Currents->qI_Component1 = -INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1 = (int16_t)wAux;
      }
    }
    break;
    
  case SECTOR_2:
  case SECTOR_3:
    {
      /* Current on Phase B is not accessible     */
      
      /* Ia = PhaseAOffset - ADC converted value) */
      if(bSector == SECTOR_3)
      {
        wAux = (int32_t)(pHandle->wPhaseAOffset)-(int32_t)(hReg2);
      }
      else
      {
        wAux = (int32_t)(pHandle->wPhaseAOffset)-(int32_t)(hReg1);
      }
      
      /* Saturation of Ia */
      if (wAux < -INT16_MAX)
      {
        pStator_Currents->qI_Component1= -INT16_MAX;
      }
      else  if (wAux > INT16_MAX)
      {
        pStator_Currents->qI_Component1= INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1= (int16_t)wAux;
      }
      
      /* Ic = PhaseCOffset - ADC converted value) */
      /* Ib = -Ic -Ia */
      if(bSector == SECTOR_3)
      {
        wAux = (int32_t)(pHandle->wPhaseCOffset)-(int32_t)(hReg1);
      }
      else
      {
        wAux = (int32_t)(pHandle->wPhaseCOffset)-(int32_t)(hReg2);
      }
      
      wAux = -wAux -  (int32_t)pStator_Currents->qI_Component1;
      
      /* Saturation of Ib */
      if (wAux> INT16_MAX)
      {
        pStator_Currents->qI_Component2=INT16_MAX;
      }
      else  if (wAux <-INT16_MAX)
      {
        pStator_Currents->qI_Component2 = -INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2 = (int16_t)wAux;
      }
    }
    break;
    
  default:
    {
    }
    break;
  }
  pHandle->_Super.hIa = pStator_Currents->qI_Component1;
  pHandle->_Super.hIb = pStator_Currents->qI_Component2;
  pHandle->_Super.hIc = -pStator_Currents->qI_Component1 - pStator_Currents->qI_Component2;
}

/**
  * @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into wPhaseAOffset and
  *         wPhaseBOffset to compute the offset introduced in the current feedback
  *         network. It is requied to proper configure ADC inputs before to enable
  *         the offset computation.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval It always returns {0,0} in Curr_Components format
  */
static void R3_1_F4XX_HFCurrentsCalibrationAB(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents)
{  
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  /* Clear the flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);
  
  if (pHandle->bIndex < NB_CONVERSIONS)
  {
    pHandle->wPhaseAOffset += LL_ADC_INJ_ReadConversionData12 (pHandle->pParams_str->ADCx,LL_ADC_INJ_RANK_1);
    pHandle->wPhaseBOffset += LL_ADC_INJ_ReadConversionData12 (pHandle->pParams_str->ADCx,LL_ADC_INJ_RANK_2);
    pHandle->bIndex++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->qI_Component1 = 0;
  pStator_Currents->qI_Component2 = 0;
}

/**
  * @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into wPhaseCOffset
  *         to compute the offset introduced in the current feedback
  *         network. It is requied to proper configure ADC input before to enable
  *         the offset computation.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval It always returns {0,0} in Curr_Components format
  */
static void R3_1_F4XX_HFCurrentsCalibrationC(PWMC_Handle_t *pHdl, Curr_Components* pStator_Currents)
{
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  /* Clear the flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);
  
  if (pHandle->bIndex < NB_CONVERSIONS)
  {
    pHandle-> wPhaseCOffset += LL_ADC_INJ_ReadConversionData12(pHandle->pParams_str->ADCx,LL_ADC_INJ_RANK_1);
    pHandle->bIndex++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->qI_Component1 = 0;
  pStator_Currents->qI_Component2 = 0;
}

/**
  * @brief  It turns on low sides switches. This function is intended to be 
  *         used for charging boot capacitors of driving section. It has to be 
  *         called each motor start-up when using high voltage drivers
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F4XX_TurnOnLowSides(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = true;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  
  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1 (TIMx,0u);
  LL_TIM_OC_SetCompareCH2 (TIMx,0u);
  LL_TIM_OC_SetCompareCH3 (TIMx,0u);
  
  /* Wait until next update */
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0)
  {}
  
  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);
  
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    /* Enable signals activation */
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  return; 
}

/**
  * @brief  Enables PWM generation on the proper Timer peripheral acting on MOE bit
  * @param pHdl handler of the current instance of the PWM component
  */
void R3_1_F4XX_SwitchOnPWM(PWMC_Handle_t *pHdl)
{  
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0)
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  
  /* Set all duty to 50% */
  if (pHandle->_Super.RLDetectionMode == true)
  {
    LL_TIM_OC_SetCompareCH1 (TIMx,1u);
    pHandle->pParams_str->ADCx->JSQR = pHandle->wADC1_JSQR;
  }
  else
  {
    LL_TIM_OC_SetCompareCH1 (TIMx,(uint32_t)(pHandle->Half_PWMPeriod) >> 1);
  }
  LL_TIM_OC_SetCompareCH2 (TIMx,(uint32_t)(pHandle->Half_PWMPeriod) >> 1);
  LL_TIM_OC_SetCompareCH3 (TIMx,(uint32_t)(pHandle->Half_PWMPeriod) >> 1);
  LL_TIM_OC_SetCompareCH4 (TIMx,(uint32_t)(pHandle->Half_PWMPeriod) - 5u);
  
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0)
  {}
  
  /* if ADCx Injected conversions are started by CH4 it must be enabled */
  LL_TIM_CC_EnableChannel (TIMx, LL_TIM_CHANNEL_CH4);

  /* Main PWM Output Enable */
  LL_TIM_SetOffStates(TIMx, LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);
  LL_TIM_EnableAllOutputs(TIMx);
  
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u)
    {
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }
  /* We enable now the triggering of the ADC */
  LL_ADC_INJ_StartConversionExtTrig (pHandle->pParams_str->ADCx, LL_ADC_INJ_TRIG_EXT_RISING);
  pHandle->pParams_str->ADCx->JSQR = pHandle->wADC1_JSQR;

  return; 
}

/**
  * @brief  Disables PWM generation on the proper Timer peripheral acting on  MOE bit
  * @param pHdl handler of the current instance of the PWM component
  */
void R3_1_F4XX_SwitchOffPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  if (pHandle->BrakeActionLock == true)
  {
  }
  else
  {
    LL_TIM_SetOffStates(TIMx, LL_TIM_OSSI_DISABLE, LL_TIM_OSSR_DISABLE);
    
    if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }
  /* Triggering of the ADC is disable */
  LL_ADC_INJ_StopConversionExtTrig (pHandle->pParams_str->ADCx);
  LL_TIM_DisableAllOutputs (TIMx);
  
  return; 
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  writes into peripheral registers the new duty cycles and
  *        sampling point
  * @param pHdl: handler of the current instance of the PWM component
  * @param hCCR4Reg: new capture/compare register value.
  * @retval none
  */
static uint16_t R3_1_F4XX_WriteTIMRegisters(PWMC_Handle_t *pHdl, uint16_t hCCR4Reg)
{
  uint16_t hAux;
      
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  LL_TIM_OC_SetCompareCH1 (TIMx,pHandle->_Super.hCntPhA);
  LL_TIM_OC_SetCompareCH2 (TIMx,pHandle->_Super.hCntPhB);
  LL_TIM_OC_SetCompareCH3 (TIMx,pHandle->_Super.hCntPhC);
  
  __LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_SetCompareCH4 (TIMx,0xFFFFu);
  __LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_SetCompareCH4 (TIMx, hCCR4Reg);

  /* Limit for update event */
  /* Check the status flag. If an update event has occurred before to set new
  values of regs the FOC rate is too high */
  if (LL_TIM_IsActiveFlag_UPDATE(TIMx))
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  if (pHandle->_Super.SWerror == 1u)
  {
    hAux = MC_FOC_DURATION;
    pHandle->_Super.SWerror = 0u;
  }
  pHandle->pParams_str->ADCx->JSQR = pHandle->wADC1_JSQR;
  LL_ADC_INJ_StopConversionExtTrig(pHandle->pParams_str->ADCx);
  LL_ADC_INJ_StartConversionExtTrig(pHandle->pParams_str->ADCx,pHandle->ADC_TriggerEdge);
  /* By default set back the ADC trigger to rising edge */
  pHandle->ADC_TriggerEdge = (uint32_t )LL_ADC_INJ_TRIG_EXT_RISING;  

  return hAux;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling during calibration.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_1_F4XX_SetADCSampPointCalibration(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  pHandle->ADC_TriggerEdge = (uint32_t )LL_ADC_INJ_TRIG_EXT_RISING;
  
  return R3_1_F4XX_WriteTIMRegisters(&pHandle->_Super, (uint16_t)(pHandle->Half_PWMPeriod) - 1u);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling related to sector 1.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_1_F4XX_SetADCSampPointSect1(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;

  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  
  /* Check if sampling AB in the middle of PWM is possible */
  if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter)
  {
	hCntSmp = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_5; /* Dummy just for the GetPhaseCurrent */
    
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;
  }
  else
  { /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/
    /* Set JSQR register */
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phBC;
    
    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
    duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
    one with minimum complementary duty and one with variable complementary duty. In this case, phases 
    with variable complementary duty and with maximum duty are converted and the first will be always 
    the phase with variable complementary duty cycle */
	
	/* Crossing Point Searching */
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhB);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore; /* hTbefore = 2*Ts + Tc, where Ts = Sampling time of ADC, Tc = Conversion Time of ADC */
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter; /* hTafter = DT + max(Trise, Tnoise) */
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        { 
          /* It must be changed the trigger direction from positive to negative 
             to sample after middle of PWM*/
          pHandle->ADC_TriggerEdge = (uint32_t )LL_ADC_INJ_TRIG_EXT_FALLING;
          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }   
    return R3_1_F4XX_WriteTIMRegisters(&pHandle->_Super, hCntSmp);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling related to sector 2.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_1_F4XX_SetADCSampPointSect2(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  
  /* Check if sampling AB in the middle of PWM is possible */  
  
   if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter)

  {
    hCntSmp = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_5; /* Dummy just for the GetPhaseCurrent */
    
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;
  }
  else
  {
 /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/
 /* Set JSQR register */
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAC;
    
    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
    duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
    one with minimum complementary duty and one with variable complementary duty. In this case, phases 
    with variable complementary duty and with maximum duty are converted and the first will be always 
    the phase with variable complementary duty cycle */    
    
    /* Searching of sampling point to avoid noise inducted by a commutation of other phase's switch */
    
    /* Check if sampling of detected current phases in the middle of PWM is possible.
       It depends on if the complementary duty cycle of the phase having a minimum complementary duty cycle 
       is greater than the Tafter time because in this case its commutation is sufficiently distant from the 
       middle of PWM and it doesn't induct any noise to Shunt voltages of the others phases */
    
	/* Crossing Point Searching */ 
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhA);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore; /* hTbefore = 2*Ts + Tc, where Ts = Sampling time of ADC, Tc = Conversion Time of ADC  */
      }
      else
      {
        hCntSmp = pHandle->_Super. hCntPhB + pHandle->pParams_str->hTafter;  /* hTafter = DT + max(Trise, Tnoise) */
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* It must be changed the trigger direction from positive to negative 
             to sample after middle of PWM*/
          pHandle->ADC_TriggerEdge = (uint32_t )LL_ADC_INJ_TRIG_EXT_FALLING;

          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }    
  return R3_1_F4XX_WriteTIMRegisters(&pHandle->_Super, hCntSmp);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling related to sector 3.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_1_F4XX_SetADCSampPointSect3(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  
  /* Check if sampling AB in the middle of PWM is possible */
  if  ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter)
    {
    hCntSmp = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_5; /* Dummy just for the GetPhaseCurrent */

    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;
  }
  else
  {/* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/
    
    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
    duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
    one with minimum complementary duty and one with variable complementary duty. In this case, phases 
    with variable complementary duty and with maximum duty are converted and the first will be always 
    the phase with variable complementary duty cycle */ 
    /* Set JSQR register */
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phCA;
    /* Searching of sampling point to avoid noise inducted by a commutation of other phase's switch */
    
    /* Check if sampling of detected current phases in the middle of PWM is possible.
    It depends on if the complementary duty cycle of the phase having a minimum complementary duty cycle 
    is greater than the Tafter time because in this case its commutation is sufficiently distant from the 
    middle of PWM and it doesn't induct any noise to Shunt voltages of the others phases */
    
	/* Crossing Point Searching */
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhC);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore; /* hTbefore = 2*Ts + Tc, where Ts = Sampling time of ADC, Tc = Conversion Time of ADC  */
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhB + pHandle->pParams_str->hTafter; /* hTafter = DT + max(Trise, Tnoise) */
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* It must be changed the trigger direction from positive to negative 
             to sample after middle of PWM*/
          pHandle->ADC_TriggerEdge = (uint32_t )LL_ADC_INJ_TRIG_EXT_FALLING;
          
         hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }
    return R3_1_F4XX_WriteTIMRegisters(&pHandle->_Super, hCntSmp);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling related to sector 4.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_1_F4XX_SetADCSampPointSect4(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  
  /* Check if sampling AB in the middle of PWM is possible */
  if  ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter)
  {
    hCntSmp = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_5; /* Dummy just for the GetPhaseCurrent */
    
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/
    
    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
    duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
    one with minimum complementary duty and one with variable complementary duty. In this case, phases 
    with variable complementary duty and with maximum duty are converted and the first will be always 
    the phase with variable complementary duty cycle */ 
    /* Set JSQR register */
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phBA;
    /* Searching of sampling point to avoid noise inducted by a commutation of other phase's switch */
    
    /* Check if sampling of detected current phases in the middle of PWM is possible.
       It depends on if the complementary duty cycle of the phase having a minimum complementary duty cycle 
       is greater than the Tafter time because in this case its commutation is sufficiently distant from the 
       middle of PWM and it doesn't induct any noise to Shunt voltages of the others phases */    
    /* Crossing Point Searching */
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhB);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore; /* hTbefore = 2*Ts + Tc, where Ts = Sampling time of ADC, Tc = Conversion Time of ADC  */
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter; /* hTafter = DT + max(Trise, Tnoise) */
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* It must be changed the trigger direction from positive to negative 
             to sample after middle of PWM*/
          pHandle->ADC_TriggerEdge = (uint32_t )LL_ADC_INJ_TRIG_EXT_FALLING;
          
          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }
    
  return R3_1_F4XX_WriteTIMRegisters(&pHandle->_Super, hCntSmp);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling related to sector 5.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
*/
uint16_t R3_1_F4XX_SetADCSampPointSect5(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  /* Check if sampling AB in the middle of PWM is possible */
  if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter)
  {
    hCntSmp = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_5; /* Dummy just for the GetPhaseCurrent */

    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/
    
    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
    duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
    one with minimum complementary duty and one with variable complementary duty. In this case, phases 
    with variable complementary duty and with maximum duty are converted and the first will be always 
    the phase with variable complementary duty cycle */ 
  /* Set JSQR register */
     pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB; 
   /* Crossing Point Searching */
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhA);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore; /* hTbefore = 2*Ts + Tc, where Ts = Sampling time of ADC, Tc = Conversion Time of ADC  */
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter; /* hTafter = DT + max(Trise, Tnoise) */
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* It must be changed the trigger direction from positive to negative 
             to sample after middle of PWM*/
          pHandle->ADC_TriggerEdge = (uint32_t )LL_ADC_INJ_TRIG_EXT_FALLING;         
          
          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }
    
  return R3_1_F4XX_WriteTIMRegisters(&pHandle->_Super, hCntSmp);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling related to sector 6.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_1_F4XX_SetADCSampPointSect6(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  /* Check if sampling AB in the middle of PWM is possible */
  if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter)
  {
    hCntSmp = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_5; /* Dummy just for the GetPhaseCurrent */

    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/
    
    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
    duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
    one with minimum complementary duty and one with variable complementary duty. In this case, phases 
    with variable complementary duty and with maximum duty are converted and the first will be always 
    the phase with variable complementary duty cycle */ 

    /* Set JSQR register */
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phCB;   
	/* Crossing Point Searching */
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhC);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore; /* hTbefore = 2*Ts + Tc, where Ts = Sampling time of ADC, Tc = Conversion Time of ADC  */
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter; /* hTafter = DT + max(Trise, Tnoise) */
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* It must be changed the trigger direction from positive to negative 
             to sample after middle of PWM*/
          pHandle->ADC_TriggerEdge = (uint32_t )LL_ADC_INJ_TRIG_EXT_FALLING;
          
          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }
    return R3_1_F4XX_WriteTIMRegisters(&pHandle->_Super, hCntSmp);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  It contains the TIMx Update event interrupt
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void *R3_1_F4XX_TIMx_UP_IRQHandler(PWMC_R3_1_F4_Handle_t *pHandle)
{
  return pHandle;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  It contains the TIMx Break1 event interrupt
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void *R3_1_F4XX_BRK_IRQHandler(PWMC_R3_1_F4_Handle_t *pHandle)
{
 
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
     LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_v_pin);
     LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_u_pin);
     LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  pHandle->OverCurrentFlag = true;

  return &(pHandle->_Super.bMotor);
}

/**
  * @brief  It is used to check if an overcurrent occurred since last call.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
  *                  detected since last method call, MC_NO_FAULTS otherwise.
  */
uint16_t R3_1_F4XX_IsOverCurrentOccurred(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  uint16_t retVal = MC_NO_FAULTS;
  
  if (pHandle->OverVoltageFlag == true)
  {
    retVal = MC_OVER_VOLT;
    pHandle->OverVoltageFlag = false;
  }
  
  if (pHandle->OverCurrentFlag == true )
  {
    retVal |= MC_BREAK_IN;
    pHandle->OverCurrentFlag = false;
  }
  
  return retVal;
}


/**
  * @brief  It is used to disable the PWM mode during RL Detection Mode.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F4XX_RLDetectionModeEnable(PWMC_Handle_t *pHdl)
{
  ADC_InjectedInitTypeDef ADC_InjectedInitStruct;
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx = pHandle->pParams_str->ADCx;
  
  if (pHandle->_Super.RLDetectionMode == false)
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
    
    LL_TIM_OC_SetCompareCH1(TIMx, 0u);
    
    /*  Channel2 configuration */
    if ((pHandle->pParams_str->LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE);
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2);
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE);
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2);
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
    }
    else
    {
    }
    
    /*  Channel3 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM2);
    LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3N);
    
    
    /* Set Update as TRGO of TIM1 */
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_UPDATE);
		
   /* Configuration of ADC sequence of two Phase B current values for during RL Detection Mode*/     
   ADC_InjectedInitStruct.ADC_NbrOfInjecChannel =2u;
   
   /*Phase B currents sequence -----------------------------------------------*/
   ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIbChannel;
   ADC_InjectedInitStruct.ADC_InjecSequence2 = pHandle->pParams_str->bIbChannel;
   ADC_InjectedInitStruct.ADC_InjecSequence3 = 0u;
   ADC_InjectedInitStruct.ADC_InjecSequence4 = 0u;
	 
  /* ADCx Injected discontinuous mode activation.
   * This is important because permits to convert first current value of ADCx Injected Sequence at
   * the first Update-Trigger event and wait until the second Update-Trigger event happens to start 
   * the second ADCx Injected conversion, then only at the end of the second conversion JEOS Interrupt
   * event is generated.  
   */
   LL_ADC_INJ_SetSequencerDiscont(ADCx, LL_ADC_INJ_SEQ_DISCONT_1RANK);
	 
   /*NB: the following istruction doesn't write the JSQR register of ADCx but
   * writes into the variable of the pDVars_str structure Class the JSQR value that
   * will be used in the future functions */
   pHandle->wADC_JSQR_RL_Detection_phB = SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);
  }
  
  pHandle->_Super.pFctGetPhaseCurrents = &R3_1_F4XX_RLGetPhaseCurrents;
  pHandle->_Super.pFctTurnOnLowSides = &R3_1_F4XX_RLTurnOnLowSides;
  pHandle->_Super.pFctSwitchOnPwm = &R3_1_F4XX_RLSwitchOnPWM;
  pHandle->_Super.pFctSwitchOffPwm = &R3_1_F4XX_RLSwitchOffPWM;
  
  pHandle->_Super.RLDetectionMode = true;
}

/**
  * @brief  It is used to disable the PWM mode during RL Detection Mode.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F4XX_RLDetectionModeDisable(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  if (pHandle->_Super.RLDetectionMode == true)
  {
    /* Repetition Counter of TIM1 User value reactivation BEGIN*/
    
    /* The folowing while cycles ensure the identification of the positive counting mode of TIM1 
     * for correct reactivation of Repetition Counter value of TIM1.*/
    
    /* Wait the change of Counter Direction of TIM1 from Up-Direction to Down-Direction*/
    while ((TIMx->CR1 & DIR_MASK) == 0u)
    {
    }
    /* Wait the change of Counter Direction of TIM1 from Down-Direction to Up-Direction.*/
    while ((TIMx->CR1 & DIR_MASK) == DIR_MASK)
    {
    }
    
    /* TIM1 Repetition Counter reactivation to the User Value */
    TIMx->RCR = pHandle->pParams_str->bRepetitionCounter;
    /* Repetition Counter of TIM1 User value reactivation END*/
    
    
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
    
    if ((pHandle->pParams_str-> LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
    }
    else
    {
    }
    
    LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);
    
    /*  Channel2 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2);
    
    if ((pHandle->pParams_str-> LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
    }
    else
    {
    }
    
    LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);
    
    /*  Channel3 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3);
    
    if ((pHandle->pParams_str-> LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3N);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3N);
    }
    else
    {
    }
    
    LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);

    /* Set channel 4 as TRGO (Center TRIGGER - Overflow of TIM1)*/
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_OC4REF);
		
    /* ADCx Injected discontinuous mode disable */
    LL_ADC_INJ_SetSequencerDiscont(pHandle->pParams_str->ADCx,
                                   LL_ADC_INJ_SEQ_DISCONT_DISABLE);
       
    pHandle->_Super.pFctGetPhaseCurrents = &R3_1_F4XX_GetPhaseCurrents;
    pHandle->_Super.pFctTurnOnLowSides = &R3_1_F4XX_TurnOnLowSides;
    pHandle->_Super.pFctSwitchOnPwm = &R3_1_F4XX_SwitchOnPWM;
    pHandle->_Super.pFctSwitchOffPwm = &R3_1_F4XX_SwitchOffPWM;
    
    pHandle->_Super.RLDetectionMode = false;
  }
}

/**
  * @brief  It is used to set the PWM dutycycle during RL Detection Mode.
  * @param pHdl: handler of the current instance of the PWM component
  * @param  hDuty: duty cycle to be applied in uint16_t
  * @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR'
  *         otherwise. These error codes are defined in mc_type.h
  */
uint16_t R3_1_F4XX_RLDetectionModeSetDuty(PWMC_Handle_t *pHdl, uint16_t hDuty)
{
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  uint16_t hAux;
  
  uint32_t val = ((uint32_t)(pHandle->Half_PWMPeriod) * (uint32_t)(hDuty)) >> 16;
  pHandle->_Super.hCntPhA = (uint16_t)(val);
  
  /* JSQR ADCx resgister writing. The sequence configuration values are set into
   * the R3_1_F4XX_RLDetectionModeEnable function*/
  pHandle->pParams_str->ADCx->JSQR = pHandle->wADC_JSQR_RL_Detection_phB;

  /* TIM1 Channel 1 Duty Cycle configuration. 
   * In RL Detection mode only the Up-side device of Phase A are controlled 
   * while the Phase B up-side device is always open.*/
  pHandle->pParams_str->TIMx->CCR1 = pHandle->_Super.hCntPhA;
  
  /* Limit for update event */
  /* Check the status flag. If an update event has occurred before to set new
  values of regs the FOC rate is too high */
  if (LL_TIM_IsActiveFlag_UPDATE(pHandle->pParams_str->TIMx))
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  if (pHandle->_Super.SWerror == 1u)
  {
    hAux = MC_FOC_DURATION;
    pHandle->_Super.SWerror = 0u;
	  }
  return hAux;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  It computes and return latest converted motor phase currents motor
  *         during RL detection phase
  * @param pHdl: handler of the current instance of the PWM component
  * @retval Ia and Ib current in Curr_Components format
  */
void R3_1_F4XX_RLGetPhaseCurrents(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents)
{
  int32_t wAux;
  int16_t hCurrA = 0, hCurrB = 0;
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  
  /* Clear the flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);
  
  wAux = (int32_t)(pHandle->wPhaseBOffset)-(int32_t)(LL_ADC_INJ_ReadConversionData12(pHandle->pParams_str->ADCx,LL_ADC_INJ_RANK_1)*2);
  
  /* Check saturation */
  if (wAux > -INT16_MAX)
  {
    if (wAux < INT16_MAX)
    {
    }
    else
    {
      wAux = INT16_MAX;
    }
  }
  else
  {
    wAux = -INT16_MAX;
  }
  /* First value read of Phase B*/
  hCurrA = (int16_t)(wAux);                     
  
  wAux = (int32_t)(pHandle->wPhaseBOffset)-(int32_t)(LL_ADC_INJ_ReadConversionData12(pHandle->pParams_str->ADCx,LL_ADC_INJ_RANK_2)*2);
  
  /* Check saturation */
  if (wAux > -INT16_MAX)
  {
    if (wAux < INT16_MAX)
    {
    }
    else
    {
      wAux = INT16_MAX;
    }
  }
  else
  {
    wAux = -INT16_MAX;
  }
  /* Second value read of Phase B*/  
  hCurrB = (int16_t)(wAux);                   

  
  pStator_Currents->qI_Component1 = hCurrA;
  pStator_Currents->qI_Component2 = hCurrB;
}

/**
  * @brief  It turns on low sides switches. This function is intended to be 
  *         used for charging boot capacitors of driving section. It has to be 
  *         called each motor start-up when using high voltage drivers.
  *         This function is specific for RL detection phase.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F4XX_RLTurnOnLowSides(PWMC_Handle_t *pHdl)
{  
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  /*Turn on the phase A low side switch */
  TIMx->CCR1 = 0u;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  
  /* Wait until next update */
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0)
  {}
  
  /* Main PWM Output Enable */
  TIMx->BDTR |= TIM_BDTR_MOE;
  
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  return; 
}


/**
  * @brief  It enables PWM generation on the proper Timer peripheral
  *         This function is specific for RL detection phase.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F4XX_RLSwitchOnPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* The following while cycles ensure the identification of the negative counting mode of TIM1
   * for correct modification of Repetition Counter value of TIM1.*/
  /* Wait the change of Counter Direction of TIM1 from Down-Direction to Up-Direction*/
  while ((TIMx->CR1 & DIR_MASK) == DIR_MASK)
  {
  }

  /* Wait the change of Counter Direction of TIM1 from Up-Direction to Down-Direction*/
  while ((TIMx->CR1 & DIR_MASK) ==0u)
  {
  }
  /* Set Repetition counter to zero */
  TIMx->RCR = 0u;
  
  TIMx->CCR1 = 1u;
  TIMx->CCR4 = ( uint32_t )( pHandle->Half_PWMPeriod ) - 5u;
  /* JSQR ADCx resgister writing. The sequence configuration values are set into
   * the R3_1_F4XX_RLDetectionModeEnable function*/
  pHandle->pParams_str->ADCx->JSQR = pHandle->wADC_JSQR_RL_Detection_phB;
  
  LL_TIM_ClearFlag_UPDATE(TIMx); /* Clear flag to wait next update */
  
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0)
  {}
  
  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  TIMx->BDTR |= TIM_BDTR_MOE;
  
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u)
    {
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }

  LL_ADC_INJ_SetTriggerSource(pHandle->pParams_str->ADCx, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO);
  LL_ADC_INJ_StartConversionExtTrig(pHandle->pParams_str->ADCx, LL_ADC_INJ_TRIG_EXT_RISING);

  return; 
}


/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit
  *         This function is specific for RL detection phase.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F4XX_RLSwitchOffPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F4_Handle_t *pHandle = (PWMC_R3_1_F4_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
    
  /* Main PWM Output Disable */
  if (pHandle->BrakeActionLock == true)
  {
  }
  else
  {
    TIMx->BDTR &= ~((uint32_t)(LL_TIM_OSSI_ENABLE));
    
    if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }
  LL_TIM_DisableAllOutputs (TIMx);
  LL_ADC_DisableIT_JEOS(pHandle->pParams_str->ADCx);
  
  /* ADCx Injected conversions end interrupt enabling */
  LL_ADC_ClearFlag_JEOS(pHandle->pParams_str->ADCx);
  LL_ADC_EnableIT_JEOS(pHandle->pParams_str->ADCx);
  return;
}

/**
  * @brief  Initializes the ADCx peripheral according to the specified parameters
  *         in the ADC_InitStruct.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_InjectInitStruct: pointer to an ADC_InjecInitTypeDef structure that contains
  *         the configuration information for the specified ADC injected channel.
  * @retval None
  */
static uint32_t SingleADC_InjectedConfig(ADC_TypeDef* ADCx, ADC_InjectedInitTypeDef* ADC_InjectedInitStruct)
{
  uint32_t tmpreg1 = 0u;
  
  /*---------------------------- ADCx JSQR Configuration -----------------*/
  /* Configure ADCx: Injected channel sequence length and sequences
   */
  switch (ADC_InjectedInitStruct->ADC_NbrOfInjecChannel)
  {
  case 1:
    tmpreg1 = (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence1) << 15);
    break;
  case 2:
    tmpreg1 = (uint32_t)( (1u << 20 ) |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence1) << 10) |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence2) << 15)
                           );
    break;
  case 3:
    tmpreg1 = (uint32_t)( (2u << 20 ) |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence1) << 5) |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence2) << 10) |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence3) << 15)
                           );
    break;
  case 4:
    tmpreg1 = (uint32_t)( (3u << 20 ) |
                         (uint32_t) (ADC_InjectedInitStruct->ADC_InjecSequence1) |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence2) << 5) |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence3) << 10) |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence4) << 15)
                           );    
    break;  
  default:
    break;
  }
   return tmpreg1;  
}

/**
 * @brief  It turns on low sides switches and start ADC triggering.
 *         This function is specific for MP phase.
 * @param  pHandle Pointer on the target component instance
 * @retval none
 */
void RLTurnOnLowSidesAndStart( PWMC_Handle_t * pHdl )
{
  PWMC_R3_1_F4_Handle_t * pHandle = ( PWMC_R3_1_F4_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  LL_TIM_OC_SetCompareCH1 ( TIMx, 0x0u );
  LL_TIM_OC_SetCompareCH2 ( TIMx, 0x0u );
  LL_TIM_OC_SetCompareCH3 ( TIMx, 0x0u );

  TIMx->CCR4 = ( uint32_t )( pHandle->Half_PWMPeriod ) - 5u;

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE ;
  LL_TIM_EnableAllOutputs ( TIMx );

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }

  ADCx->JSQR = pHandle->wADC_JSQR_phAB;
  LL_ADC_INJ_StartConversionExtTrig(ADCx,LL_ADC_INJ_TRIG_EXT_RISING);

  return;
}


/**
 * @brief  It sets ADC sampling points.
 *         This function is specific for MP phase.
 * @param  pHandle Pointer on the target component instance
 * @retval none
 */
void RLSetADCSampPoint( PWMC_Handle_t * pHdl )
{
  PWMC_R3_1_F4_Handle_t * pHandle = ( PWMC_R3_1_F4_Handle_t * )pHdl;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;

  /* dummy sector setting to get correct Ia value */
  pHdl->hSector = SECTOR_5;

  ADCx->JSQR = pHandle->wADC_JSQR_phAB;
  LL_ADC_INJ_StartConversionExtTrig(ADCx,LL_ADC_INJ_TRIG_EXT_RISING);

  return;
}

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
