/**
  ******************************************************************************
  * @file    r1_g0xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement current sensor
  *          class to be stantiated when the single shunt current sensing
  *          topology is used. It is specifically designed for STM32F0XX
  *          microcontrollers and implements the successive sampling of two motor
  *          current using only one ADC.
  *           + MCU peripheral and handle initialization fucntion
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
#include "r1_g0xx_pwm_curr_fdbk.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup r1_g0XX_pwm_curr_fdbk
  * @brief PWM F0XX single shunt component of the Motor Control SDK
  *
  * @{
  */

/* Private Defines -----------------------------------------------------------*/

#define DR_OFFSET 0x40u
#define ADC1_DR_Address     ADC1_BASE + DR_OFFSET

#define NB_CONVERSIONS 16u

#define REGULAR         ((uint8_t)0u)
#define BOUNDARY_1      ((uint8_t)1u)  /* Two small, one big */
#define BOUNDARY_2      ((uint8_t)2u)  /* Two big, one small */
#define BOUNDARY_3      ((uint8_t)3u)  /* Three equal        */

#define INVERT_NONE 0u
#define INVERT_A 1u
#define INVERT_B 2u
#define INVERT_C 3u

#define SAMP_NO 0u
#define SAMP_IA 1u
#define SAMP_IB 2u
#define SAMP_IC 3u
#define SAMP_NIA 4u
#define SAMP_NIB 5u
#define SAMP_NIC 6u
#define SAMP_OLDA 7u
#define SAMP_OLDB 8u
#define SAMP_OLDC 9u

#define CH1NORMAL           0x0060u
#define CH2NORMAL           0x6000u
#define CH3NORMAL           0x0060u
#define CH4NORMAL           0x7000u

#define CCMR1_PRELOAD_DISABLE_MASK 0xF7F7u
#define CCMR2_PRELOAD_DISABLE_MASK 0xFFF7u

#define CCMR1_PRELOAD_ENABLE_MASK 0x0808u
#define CCMR2_PRELOAD_ENABLE_MASK 0x0008u


#define TIMxCCER_MASK_CH123              (TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|\
                                          TIM_CCER_CC1NE|TIM_CCER_CC2NE|TIM_CCER_CC3NE)
#define CC12_PRELOAD_ENABLE_MASK         (TIM_CCMR1_OC1PE|TIM_CCMR1_OC2PE)
#define CC3_PRELOAD_ENABLE_MASK          TIM_CCMR2_OC3PE
#define CC1_PRELOAD_DISABLE_MASK         ~TIM_CCMR1_OC1PE
#define CC2_PRELOAD_DISABLE_MASK         ~TIM_CCMR1_OC2PE
#define CC3_PRELOAD_DISABLE_MASK         ~TIM_CCMR2_OC3PE
#define TIMxCCR56_PRELOAD_DISABLE_MASK   ~(TIM_CCMR3_OC5PE|TIM_CCMR3_OC6PE)
#define TIMxCCR56_PRELOAD_ENABLE_MASK    (TIM_CCMR3_OC5PE|TIM_CCMR3_OC6PE)



/* DMA ENABLE mask */
#define CCR_ENABLE_Set          ((uint32_t)0x00000001u)
#define CCR_ENABLE_Reset        ((uint32_t)0xFFFFFFFEu)

#define CR2_JEXTSEL_Reset       ((uint32_t)0xFFFF8FFFu)
#define CR2_JEXTTRIG_Set        ((uint32_t)0x00008000u)
#define CR2_JEXTTRIG_Reset      ((uint32_t)0xFFFF7FFFu)

#define TIM_DMA_ENABLED_CC1 0x0200u
#define TIM_DMA_ENABLED_CC2 0x0400u
#define TIM_DMA_ENABLED_CC3 0x0800u

#define CR2_ADON_Set                ((uint32_t)0x00000001u)

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))
#define CR2_EXTTRIG_SWSTART_Set    ((uint32_t)0x00500000)

#define ADC1_CR2_EXTTRIG_SWSTART_BB 0x42248158u

#define ADCx_IRQn     ADC1_COMP_IRQn
#define TIMx_UP_IRQn  TIM1_BRK_UP_TRG_COM_IRQn

/* Constant values -----------------------------------------------------------*/
static const uint8_t REGULAR_SAMP_CUR1[6] = {SAMP_NIC,SAMP_NIC,SAMP_NIA,SAMP_NIA,SAMP_NIB,SAMP_NIB};
static const uint8_t REGULAR_SAMP_CUR2[6] = {SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA};
static const uint8_t BOUNDR1_SAMP_CUR2[6] = {SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA,SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR1[6] = {SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR2[6] = {SAMP_IC,SAMP_IA,SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC};

/* Private function prototypes -----------------------------------------------*/
static void R1G0XX_1ShuntMotorVarsRestart(PWMC_Handle_t *pHdl);
static void R1G0XX_TIMxInit(TIM_TypeDef* TIMx, PWMC_Handle_t *pHdl);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  It initializes TIM1, ADC, GPIO, DMA1 and NVIC for single shunt current
  *         reading configuration using STM32F0XX family.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1G0XX_Init(PWMC_R1_G0_Handle_t *pHandle)
{
  
  if ((uint32_t)pHandle == (uint32_t)&pHandle->_Super)
  {
    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
    LL_ADC_DisableIT_EOC( ADC1 );
    LL_ADC_ClearFlag_EOC( ADC1 );
    LL_ADC_DisableIT_EOS( ADC1 );
    LL_ADC_ClearFlag_EOS( ADC1 );
    /* DMA Event related to TIM1 Channel 4 */
    /* DMA1 Channel4 configuration ----------------------------------------------*/
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)pHandle->hDmaBuff);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)&(TIM1->CCR1));
    LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_4, 2u );

    /* ensable DMA1 Channel4 */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
    
    /* Debug feature, we froze the timer if the MCU is halted by the debugger*/
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DBGMCU);
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);
    /* End of debug feature */
    
    /* DMA Event related to TIM1 update */
    /* DMA1 Channel5 configuration ----------------------------------------------*/
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&(pHandle->wPreloadDisableActing));
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&TIM1->CCMR1);
    LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_5,1);

    /* enable DMA1 Channel5 */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);

    R1G0XX_TIMxInit(TIM1, &pHandle->_Super);


    /* DMA Event related to ADC conversion*/
    /* DMA channel configuration ----------------------------------------------*/
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)pHandle->hCurConv);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)ADC1_DR_Address);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2u);

    /* DMA1 channel 1 will be enabled after the CurrentReadingCalibration */


    /* Start calibration of ADC1 */
    LL_ADC_StartCalibration(ADC1);
    while(LL_ADC_IsCalibrationOnGoing(ADC1) == 1);
    (READ_BIT(ADC1->CR,ADC_CR_ADCAL) == RESET)?(LL_ADC_ReadReg(ADC1,DR)):(0);

    /* Enable ADC */
    LL_ADC_REG_SetSequencerConfigurable (ADC1, LL_ADC_REG_SEQ_FIXED);
    LL_ADC_REG_SetTriggerSource (ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
    LL_ADC_Enable(ADC1);
    

    /* Wait ADC Ready */
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1)==RESET)
    {}
    
    R1G0XX_1ShuntMotorVarsRestart(&pHandle->_Super);

    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    

    LL_TIM_EnableCounter(TIM1);

    pHandle->ADCRegularLocked=false; /* We allow ADC usage for regular conversion on Systick*/
    pHandle->_Super.DTTest = 0u;
    pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;

  }
}

/**
  * @brief  It initializes TIMx for PWM generation,
  *          active vector insertion and adc triggering.
  * @param  TIMx Timer to be initialized
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static void R1G0XX_TIMxInit(TIM_TypeDef* TIMx, PWMC_Handle_t *pHdl)
{

  PWMC_R1_G0_Handle_t *pHandle = (PWMC_R1_G0_Handle_t *)pHdl;

  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3);

  if ((pHandle->pParams_str->LowSideOutputs)== LS_PWM_TIMER)
  {
     LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
     LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
     LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3N);
  }

  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH5);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH6);

  LL_TIM_OC_SetDeadTime(TIMx, (pHandle->pParams_str->hDeadTime)/2u);
 
  pHandle->wPreloadDisableCC1 = TIMx->CCMR1 & CC1_PRELOAD_DISABLE_MASK;
  pHandle->wPreloadDisableCC2 = TIMx->CCMR1 & CC2_PRELOAD_DISABLE_MASK;
  pHandle->wPreloadDisableCC3 = TIMx->CCMR2 & CC3_PRELOAD_DISABLE_MASK;
}

/**
  * @brief  It stores into handler the voltage present on the
  *         current feedback analog channel when no current is flowin into the
  *         motor
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1G0XX_CurrentReadingCalibration(PWMC_Handle_t *pHdl)
{
  uint8_t bIndex = 0u;
  uint32_t wPhaseOffset = 0u;

  PWMC_R1_G0_Handle_t *pHandle = (PWMC_R1_G0_Handle_t *)pHdl;

  /* Set the CALIB flags to indicate the ADC calibration phase*/
  pHandle->hFlags |= CALIB;

  /* We forbid ADC usage for regular conversion on Systick*/
  pHandle->ADCRegularLocked=true; 
  /* ADC Channel config for current reading */
  LL_ADC_REG_SetSequencerChannels ( ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->hIChannel ));
  
  /* Disable DMA1 Channel1 */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  
  /* ADC Channel used for current reading are read 
  in order to get zero currents ADC values*/   
  while (bIndex< NB_CONVERSIONS)
  {     
    /* Software start of conversion */
    LL_ADC_REG_StartConversion(ADC1);
    
    /* Wait until end of regular conversion */
    while (LL_ADC_IsActiveFlag_EOC(ADC1)==RESET)
    {}    
    
    wPhaseOffset += LL_ADC_REG_ReadConversionData12(ADC1);
    bIndex++;
  }
  
  pHandle->hPhaseOffset = (uint16_t)(wPhaseOffset/NB_CONVERSIONS);
  
  /* Reset the CALIB flags to indicate the end of ADC calibartion phase*/
  pHandle->hFlags &= (~CALIB);
  
}

/**
  * @brief  Initialization of class members after each motor start
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static void R1G0XX_1ShuntMotorVarsRestart(PWMC_Handle_t *pHdl)
{
  PWMC_R1_G0_Handle_t *pHandle = (PWMC_R1_G0_Handle_t *)pHdl;
  
  /* Default value of DutyValues */
  pHandle->hCntSmp1 = (pHandle->Half_PWMPeriod >> 1) - pHandle->pParams_str->hTbefore;
  pHandle->hCntSmp2 = (pHandle->Half_PWMPeriod >> 1) + pHandle->pParams_str->hTafter;
  
  pHandle->bInverted_pwm_new=INVERT_NONE;
  pHandle->hFlags &= (~STBD3); /*STBD3 cleared*/
  

  /* After start value of dvDutyValues */
  pHandle->_Super.hCntPhA = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.hCntPhB = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.hCntPhC = pHandle->Half_PWMPeriod >> 1;
  
  /* Set the default previous value of Phase A,B,C current */
  pHandle->hCurrAOld=0;
  pHandle->hCurrBOld=0;

   /* After reset, value of DMA buffers for distortion*/
  pHandle->hDmaBuff[0] =  pHandle->Half_PWMPeriod + 1u;
  pHandle->hDmaBuff[1] =  pHandle->Half_PWMPeriod >> 1; /*dummy*/
  
  pHandle->BrakeActionLock = false;

   }

/**
  * @brief  It computes and return latest converted motor phase currents motor
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval Curr_Components Ia and Ib current in Curr_Components format
  */
void R1G0XX_GetPhaseCurrents(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents)
{  
  int32_t wAux;
  int16_t hCurrA = 0;
  int16_t hCurrB = 0;
  int16_t hCurrC = 0;
  uint8_t bCurrASamp = 0u;
  uint8_t bCurrBSamp = 0u;
  uint8_t bCurrCSamp = 0u;
  
  PWMC_R1_G0_Handle_t *pHandle = (PWMC_R1_G0_Handle_t *)pHdl;
  TIM1->CCMR1 |= CC12_PRELOAD_ENABLE_MASK;
  TIM1->CCMR2 |= CC3_PRELOAD_ENABLE_MASK; 
  
  /* Disabling the External triggering for ADCx*/
  LL_ADC_REG_SetTriggerSource (ADC1, LL_ADC_REG_TRIG_SOFTWARE);
  
  /* Reset the update flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE(TIM1);
  
  /* First sampling point */
  wAux = (int32_t)(pHandle->hCurConv[0]) - (int32_t)(pHandle->hPhaseOffset);
  
  /* Check saturation */
  wAux = (wAux > -INT16_MAX) ? ((wAux < INT16_MAX) ? wAux : INT16_MAX) : -INT16_MAX;
  
  switch (pHandle->sampCur1)
  {
  case SAMP_IA:
    hCurrA = (int16_t)(wAux);
    bCurrASamp = 1u;
    break;
  case SAMP_IB:
    hCurrB = (int16_t)(wAux);
    bCurrBSamp = 1u;
    break;
  case SAMP_IC:
    hCurrC = (int16_t)(wAux);
    bCurrCSamp = 1u;
    break;
  case SAMP_NIA:
    wAux = -wAux;
    hCurrA = (int16_t)(wAux);
    bCurrASamp = 1u;
    break;
  case SAMP_NIB:
    wAux = -wAux;
    hCurrB = (int16_t)(wAux);
    bCurrBSamp = 1u;
    break;
  case SAMP_NIC:
    wAux = -wAux;
    hCurrC = (int16_t)(wAux);
    bCurrCSamp = 1u;
    break;
  case SAMP_OLDA:
    hCurrA = pHandle->hCurrAOld;
    bCurrASamp = 1u;
    break;
  case SAMP_OLDB:
    hCurrB = pHandle->hCurrBOld;
    bCurrBSamp = 1u;
    break;
  default:
    break;
  }
  
  /* Second sampling point */
  wAux = (int32_t)(pHandle->hCurConv[1]) - (int32_t)(pHandle->hPhaseOffset);
  
  wAux = (wAux > -INT16_MAX) ? ((wAux < INT16_MAX) ? wAux : INT16_MAX) : -INT16_MAX;

  
  switch (pHandle->sampCur2)
  {
  case SAMP_IA:
    hCurrA = (int16_t)(wAux);
    bCurrASamp = 1u;
    break;
  case SAMP_IB:
    hCurrB = (int16_t)(wAux);
    bCurrBSamp = 1u;
    break;
  case SAMP_IC:
    hCurrC = (int16_t)(wAux);
    bCurrCSamp = 1u;
    break;
  case SAMP_NIA:
    wAux = -wAux; 
    hCurrA = (int16_t)(wAux);
    bCurrASamp = 1u;
    break;
  case SAMP_NIB:
    wAux = -wAux; 
    hCurrB = (int16_t)(wAux);
    bCurrBSamp = 1u;
    break;
  case SAMP_NIC:
    wAux = -wAux; 
    hCurrC = (int16_t)(wAux);
    bCurrCSamp = 1u;
    break;
  default:
    break;
  }
    
  /* Computation of the third value */
  if (bCurrASamp == 0u)
  {
    wAux = -((int32_t)(hCurrB)) -((int32_t)(hCurrC));
    
    /* Check saturation */
	wAux = (wAux > -INT16_MAX) ? ((wAux < INT16_MAX) ? wAux : INT16_MAX) : -INT16_MAX;
    
    hCurrA = (int16_t)wAux; 
  }
  if (bCurrBSamp == 0u)
  {
    wAux = -((int32_t)(hCurrA)) -((int32_t)(hCurrC));
    
    /* Check saturation */
	wAux = (wAux > -INT16_MAX) ? ((wAux < INT16_MAX) ? wAux : INT16_MAX) : -INT16_MAX;
    
    hCurrB = (int16_t)wAux;
  }
  if (bCurrCSamp == 0u)
  {
    wAux = -((int32_t)(hCurrA)) -((int32_t)(hCurrB));
    
    /* Check saturation */
    wAux = (wAux > -INT16_MAX) ? ((wAux < INT16_MAX) ? wAux : INT16_MAX) : -INT16_MAX;
    
    hCurrC = (int16_t)wAux;
  }
  
  /* hCurrA, hCurrB, hCurrC values are the sampled values */
    
  pHandle->hCurrAOld = hCurrA;
  pHandle->hCurrBOld = hCurrB;

  pStator_Currents->qI_Component1 = hCurrA;
  pStator_Currents->qI_Component2 = hCurrB;
  
  pHandle->_Super.hIa = pStator_Currents->qI_Component1;
  pHandle->_Super.hIb = pStator_Currents->qI_Component2;
  pHandle->_Super.hIc = -pStator_Currents->qI_Component1 - pStator_Currents->qI_Component2;

}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1G0XX_TurnOnLowSides(PWMC_Handle_t *pHdl)
{
  PWMC_R1_G0_Handle_t *pHandle = (PWMC_R1_G0_Handle_t *)pHdl;

  pHandle->_Super.bTurnOnLowSidesAction = true;

  TIM1->CCR1 = 0u;
  TIM1->CCR2 = 0u;
  TIM1->CCR3 = 0u;
  
  LL_TIM_ClearFlag_UPDATE(TIM1);
  while (LL_TIM_IsActiveFlag_UPDATE(TIM1) == RESET)
  {}
  
  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIM1);
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }
  return; 
}

/**
  * @brief  It enables PWM generation on the proper Timer peripheral acting on
  *         MOE bit, enaables the single shunt distortion and reset the TIM status
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1G0XX_SwitchOnPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R1_G0_Handle_t *pHandle = (PWMC_R1_G0_Handle_t *)pHdl;
  
  pHandle->_Super.bTurnOnLowSidesAction = false;
  
  /* enable break Interrupt */
  LL_TIM_ClearFlag_BRK(TIM1);
  LL_TIM_EnableIT_BRK(TIM1);

  LL_TIM_DisableDMAReq_CC4(TIM1);
  LL_TIM_DisableDMAReq_UPDATE(TIM1);
  LL_DMA_DisableChannel (DMA1, LL_DMA_CHANNEL_4);
  LL_DMA_DisableChannel (DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_SetDataLength (DMA1, LL_DMA_CHANNEL_4, 2);

  /* Enables the TIMx Preload on CC1 Register */
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  /* Enables the TIMx Preload on CC2 Register */
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  /* Enables the TIMx Preload on CC3 Register */
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);

  /* TIM output trigger 2 for ADC */
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_OC5_RISING_OC6_RISING);

  LL_TIM_ClearFlag_UPDATE(TIM1);
  while (LL_TIM_IsActiveFlag_UPDATE(TIM1)==RESET)
  {}
  LL_TIM_ClearFlag_UPDATE(TIM1);
 /* Set all duty to 50% */
  /* Set ch5 ch6 for triggering */
  /* Clear Update Flag */

  LL_TIM_OC_SetCompareCH1(TIM1,(uint32_t)(pHandle->Half_PWMPeriod >> 1));
  LL_TIM_OC_SetCompareCH2(TIM1,(uint32_t)(pHandle->Half_PWMPeriod >> 1));
  LL_TIM_OC_SetCompareCH3(TIM1,(uint32_t)(pHandle->Half_PWMPeriod >> 1));

  while (LL_TIM_IsActiveFlag_UPDATE(TIM1)==RESET)
  {}
  /* trick because the DMA is fired as soon as channel is enabled ...*/
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2u);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  
  /* Main PWM Output Enable */  
  LL_TIM_EnableAllOutputs(TIM1);
  
  /* TIM output trigger 2 for ADC */


  LL_TIM_OC_SetCompareCH5(TIM1,(((uint32_t)(pHandle->Half_PWMPeriod >> 1)) + (uint32_t)pHandle->pParams_str->hTafter));
  LL_TIM_OC_SetCompareCH6(TIM1,(uint32_t)(pHandle->Half_PWMPeriod - 1u));

  /* Main PWM Output Enable */
  LL_TIM_ClearFlag_UPDATE(TIM1);
  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_EnableDMAReq_CC4(TIM1);
  LL_DMA_EnableChannel (DMA1, LL_DMA_CHANNEL_4); 
  
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    if ((TIM1->CCER & TIMxCCER_MASK_CH123) != 0u)
    {
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
  }
  
  /* Enabling distortion for single shunt */
  pHandle->hFlags |= DSTEN;
  /* We forbid ADC usage for regular conversion on Systick*/
  pHandle->ADCRegularLocked=true; 
  return;
}

/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit, disables the single shunt distortion and reset the TIM status
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1G0XX_SwitchOffPWM(PWMC_Handle_t *pHdl)
{
  uint16_t hAux;

  PWMC_R1_G0_Handle_t *pHandle = (PWMC_R1_G0_Handle_t *)pHdl;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(TIM1);
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }
  
  /* Switch off the DMA from this point High frequency task is shut down*/
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  
  /* channel 5 and 6 Preload Disable */
  LL_TIM_OC_DisablePreload(TIM1, LL_TIM_CHANNEL_CH5);
  LL_TIM_OC_DisablePreload(TIM1, LL_TIM_CHANNEL_CH6);
  
  LL_TIM_OC_SetCompareCH5(TIM1,(uint32_t)(pHandle->Half_PWMPeriod + 1u));
  LL_TIM_OC_SetCompareCH6(TIM1,(uint32_t)(pHandle->Half_PWMPeriod + 1u));
  
    /* channel 5 and 6 Preload enable */
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH5);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH6);
  
  /* Disable TIMx DMA requests enable */
  LL_TIM_DisableDMAReq_CC4(TIM1);
  LL_TIM_DisableDMAReq_UPDATE(TIM1);

  /* Disable DMA channels*/
  LL_DMA_DisableChannel (DMA1, LL_DMA_CHANNEL_5);
  
  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE(TIM1);
  
  /* Disable break interrupt */
  LL_TIM_DisableIT_BRK(TIM1);
  
  /*Clear potential ADC Ongoing conversion*/
  if (LL_ADC_REG_IsConversionOngoing (ADC1))
  {
    LL_ADC_REG_StopConversion (ADC1);
    while ( LL_ADC_REG_IsConversionOngoing(ADC1))
    {
    }
  }
  LL_ADC_REG_SetTriggerSource (ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    
  /* Disabling distortion for single */
  pHandle->hFlags &= (~DSTEN);

  while (LL_TIM_IsActiveFlag_UPDATE(TIM1)==RESET)
  {}

  
  /* We allow ADC usage for regular conversion on Systick*/
  pHandle->ADCRegularLocked=false; 

  /* Set all duty to 50% */
  hAux = pHandle->Half_PWMPeriod >> 1;
  TIM1->CCR1 = hAux;
  TIM1->CCR2 = hAux;
  TIM1->CCR3 = hAux;    
    
  return; 
}

/**
  * @brief  Implementation of the single shunt algorithm to setup the
  *         TIM1 register and DMA buffers values for the next PWM period.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval uint16_t It returns MC_FOC_DURATION if the TIMx update occurs
  *          before the end of FOC algorithm else returns MC_NO_ERROR
  */
uint16_t R1G0XX_CalcDutyCycles(PWMC_Handle_t *pHdl)
{
  int16_t hDeltaDuty_0;
  int16_t hDeltaDuty_1;
  uint16_t hDutyV_0 = 0u;
  uint16_t hDutyV_1 = 0u;
  uint16_t hDutyV_2 = 0u;
  uint8_t bSector;
  uint8_t bStatorFluxPos;
  uint16_t hAux;

  PWMC_R1_G0_Handle_t *pHandle = (PWMC_R1_G0_Handle_t *)pHdl;
    
  bSector = (uint8_t)pHandle->_Super.hSector;
  
  if ((pHandle->hFlags & DSTEN) != 0u)
  { 
    switch (bSector)
    {
    case SECTOR_1:
      hDutyV_2 = pHandle->_Super.hCntPhA;
      hDutyV_1 = pHandle->_Super.hCntPhB;
      hDutyV_0 = pHandle->_Super.hCntPhC;
      break;
    case SECTOR_2:
      hDutyV_2 = pHandle->_Super.hCntPhB;
      hDutyV_1 = pHandle->_Super.hCntPhA;
      hDutyV_0 = pHandle->_Super.hCntPhC;
      break;
    case SECTOR_3:
      hDutyV_2 = pHandle->_Super.hCntPhB;
      hDutyV_1 = pHandle->_Super.hCntPhC;
      hDutyV_0 = pHandle->_Super.hCntPhA;
      break;
    case SECTOR_4:
      hDutyV_2 = pHandle->_Super.hCntPhC;
      hDutyV_1 = pHandle->_Super.hCntPhB;
      hDutyV_0 = pHandle->_Super.hCntPhA;
      break;
    case SECTOR_5:
      hDutyV_2 = pHandle->_Super.hCntPhC;
      hDutyV_1 = pHandle->_Super.hCntPhA;
      hDutyV_0 = pHandle->_Super.hCntPhB;
      break;
    case SECTOR_6:
      hDutyV_2 = pHandle->_Super.hCntPhA;
      hDutyV_1 = pHandle->_Super.hCntPhC;
      hDutyV_0 = pHandle->_Super.hCntPhB;
      break;
    default:
      break;
    }
    
    /* Compute delta duty */
    hDeltaDuty_0 = (int16_t)(hDutyV_1) - (int16_t)(hDutyV_0);
    hDeltaDuty_1 = (int16_t)(hDutyV_2) - (int16_t)(hDutyV_1);
    
    /* Check region */
    if ((uint16_t)hDeltaDuty_0<=pHandle->pParams_str->hTMin)
    {
      if ((uint16_t)hDeltaDuty_1<=pHandle->pParams_str->hTMin)
      {
        bStatorFluxPos = BOUNDARY_3;
      }
      else
      {
        bStatorFluxPos = BOUNDARY_2;
      }
    } 
    else 
    {
      if ((uint16_t)hDeltaDuty_1>pHandle->pParams_str->hTMin)
      {
        bStatorFluxPos = REGULAR;
      }
      else
      {
        bStatorFluxPos = BOUNDARY_1;
      }
    }
            
    if (bStatorFluxPos == REGULAR)
    {
      pHandle->bInverted_pwm_new = INVERT_NONE;
    }
    else if (bStatorFluxPos == BOUNDARY_1) /* Adjust the lower */
    {
      switch (bSector)
      {
      case SECTOR_5:
      case SECTOR_6:
        if (pHandle->_Super.hCntPhA - pHandle->pParams_str->hCHTMin - hDutyV_0 > pHandle->pParams_str->hTMin)
        {
          pHandle->bInverted_pwm_new = INVERT_A;
          pHandle->_Super.hCntPhA -=pHandle->pParams_str->hCHTMin;
          if (pHandle->_Super.hCntPhA < hDutyV_1)
          {
            hDutyV_1 = pHandle->_Super.hCntPhA;
          }
        }
        else
        {
          bStatorFluxPos = BOUNDARY_3;
          if ((pHandle->hFlags & STBD3) == 0u)
          {
            pHandle->bInverted_pwm_new = INVERT_A;
            pHandle->_Super.hCntPhA -=pHandle->pParams_str->hCHTMin;
            pHandle->hFlags |= STBD3;
          } 
          else
          {
            pHandle->bInverted_pwm_new = INVERT_B;
            pHandle->_Super.hCntPhB -=pHandle->pParams_str->hCHTMin;
            pHandle->hFlags &= (~STBD3);
          }
        }
        break;
      case SECTOR_2:
      case SECTOR_1:
        if (pHandle->_Super.hCntPhB - pHandle->pParams_str->hCHTMin - hDutyV_0 > pHandle->pParams_str->hTMin)
        {
          pHandle->bInverted_pwm_new = INVERT_B;
          pHandle->_Super.hCntPhB -=pHandle->pParams_str->hCHTMin;
          if (pHandle->_Super.hCntPhB < hDutyV_1)
          {
            hDutyV_1 = pHandle->_Super.hCntPhB;
          }
        }
        else
        {
          bStatorFluxPos = BOUNDARY_3;
          if ((pHandle->hFlags & STBD3) == 0u)
          {
            pHandle->bInverted_pwm_new = INVERT_A;
            pHandle->_Super.hCntPhA -=pHandle->pParams_str->hCHTMin;
            pHandle->hFlags |= STBD3;
          } 
          else
          {
            pHandle->bInverted_pwm_new = INVERT_B;
            pHandle->_Super.hCntPhB -=pHandle->pParams_str->hCHTMin;
            pHandle->hFlags &= (~STBD3);
          }
        }
        break;
      case SECTOR_4:
      case SECTOR_3:
        if (pHandle->_Super.hCntPhC - pHandle->pParams_str->hCHTMin - hDutyV_0 > pHandle->pParams_str->hTMin)
        {
          pHandle->bInverted_pwm_new = INVERT_C;
          pHandle->_Super.hCntPhC -=pHandle->pParams_str->hCHTMin;
          if (pHandle->_Super.hCntPhC < hDutyV_1)
          {
            hDutyV_1 = pHandle->_Super.hCntPhC;
          }
        }
        else
        {
          bStatorFluxPos = BOUNDARY_3;
          if ((pHandle->hFlags & STBD3) == 0u)
          {
            pHandle->bInverted_pwm_new = INVERT_A;
            pHandle->_Super.hCntPhA -=pHandle->pParams_str->hCHTMin;
            pHandle->hFlags |= STBD3;
          } 
          else
          {
            pHandle->bInverted_pwm_new = INVERT_B;
            pHandle->_Super.hCntPhB -=pHandle->pParams_str->hCHTMin;
            pHandle->hFlags &= (~STBD3);
          }
        }
        break;
      default:
        break;
      }
    }
    else if (bStatorFluxPos == BOUNDARY_2) /* Adjust the middler */
    {
      switch (bSector)
      {
      case SECTOR_4:
      case SECTOR_5: /* Invert B */
        pHandle->bInverted_pwm_new = INVERT_B;
        pHandle->_Super.hCntPhB -=pHandle->pParams_str->hCHTMin;
        if (pHandle->_Super.hCntPhB > 0xEFFFu)
        {
          pHandle->_Super.hCntPhB = 0u;
        }
        break;
      case SECTOR_2:
      case SECTOR_3: /* Invert A */
        pHandle->bInverted_pwm_new = INVERT_A;
        pHandle->_Super.hCntPhA -=pHandle->pParams_str->hCHTMin;
        if (pHandle->_Super.hCntPhA > 0xEFFFu)
        {
          pHandle->_Super.hCntPhA = 0u;
        }
        break;
      case SECTOR_6:
      case SECTOR_1: /* Invert C */
        pHandle->bInverted_pwm_new = INVERT_C;
        pHandle->_Super.hCntPhC -=pHandle->pParams_str->hCHTMin;
        if (pHandle->_Super.hCntPhC > 0xEFFFu)
        {
          pHandle->_Super.hCntPhC = 0u;
        }
        break;
      default:
        break;
      }
    }
    else
    {
      if ((pHandle->hFlags & STBD3) == 0u)
      {
        pHandle->bInverted_pwm_new = INVERT_A;
        pHandle->_Super.hCntPhA -=pHandle->pParams_str->hCHTMin;
        pHandle->hFlags |= STBD3;
      } 
      else
      {
        pHandle->bInverted_pwm_new = INVERT_B;
        pHandle->_Super.hCntPhB -=pHandle->pParams_str->hCHTMin;
        pHandle->hFlags &= (~STBD3);
      }
    }
        
    if (bStatorFluxPos == REGULAR) /* Regular zone */
    {
      /* First point */
  /*    if ((hDutyV_1 - hDutyV_0 - pHandle->pParams_str->hDeadTime)> pHandle->pParams_str->hMaxTrTs)
      {
        pHandle->hCntSmp1 = hDutyV_0 + hDutyV_1 + pHandle->pParams_str->hDeadTime;
        pHandle->hCntSmp1 >>= 1;
      }
      else
      { */
        pHandle->hCntSmp1 = hDutyV_1 - pHandle->pParams_str->hTbefore;
     /* }*/
      /* Second point */
    /*  if ((hDutyV_2 - hDutyV_1 - pHandle->pParams_str->hDeadTime)> pHandle->pParams_str->hMaxTrTs)
      {
        pHandle->hCntSmp2 = hDutyV_1 + hDutyV_2 + pHandle->pParams_str->hDeadTime;
        pHandle->hCntSmp2 >>= 1;
      }
      else
      {*/
        pHandle->hCntSmp2 = hDutyV_2 - pHandle->pParams_str->hTbefore;
    /*  }*/
    }
    
    if (bStatorFluxPos == BOUNDARY_1) /* Two small, one big */
    {      
      /* First point */
    /*  if ((hDutyV_1 - hDutyV_0 - pHandle->pParams_str->hDeadTime)> pHandle->pParams_str->hMaxTrTs)
      {
        pHandle->hCntSmp1 = hDutyV_0 + hDutyV_1 + pHandle->pParams_str->hDeadTime;
        pHandle->hCntSmp1 >>= 1;
      }
      else 
      { */
        pHandle->hCntSmp1 = hDutyV_1 - pHandle->pParams_str->hTbefore;
     /* }*/
      /* Second point */
      pHandle->hCntSmp2 = pHandle->Half_PWMPeriod - pHandle->pParams_str->hHTMin + pHandle->pParams_str->hTSample;
    }
    
    if (bStatorFluxPos == BOUNDARY_2) /* Two big, one small */
    {
      /* First point */
   /*   if ((hDutyV_2 - hDutyV_1 - pHandle->pParams_str->hDeadTime)>= pHandle->pParams_str->hMaxTrTs)
      {
        pHandle->hCntSmp1 = hDutyV_1 + hDutyV_2 + pHandle->pParams_str->hDeadTime;
        pHandle->hCntSmp1 >>= 1;
      }
      else
      { */
        pHandle->hCntSmp1 = hDutyV_2 - pHandle->pParams_str->hTbefore;
    /*  }*/
      /* Second point */
      pHandle->hCntSmp2 = pHandle->Half_PWMPeriod - pHandle->pParams_str->hHTMin + pHandle->pParams_str->hTSample;
    }
    
    if (bStatorFluxPos == BOUNDARY_3)  
    {
      /* First point */
      pHandle->hCntSmp1 = hDutyV_0-pHandle->pParams_str->hTbefore; /* Dummy trigger */
      /* Second point */
      pHandle->hCntSmp2 = pHandle->Half_PWMPeriod - pHandle->pParams_str->hHTMin + pHandle->pParams_str->hTSample;
    }
  }
  else
  {
    pHandle->bInverted_pwm_new = INVERT_NONE;
    bStatorFluxPos = REGULAR;
  }


 
  /* Update Timer Ch4 for active vector*/  
  /* Update Timer Ch 5,6 for ADC triggering and books the queue*/
  TIM1->CCMR3 &= TIMxCCR56_PRELOAD_DISABLE_MASK;
  TIM1->CCR5 = 0x0u;
  TIM1->CCR6 = 0xFFFFu;
  TIM1->CCMR3 |= TIMxCCR56_PRELOAD_ENABLE_MASK; 

  TIM1->CCR5 = pHandle->hCntSmp1;
  TIM1->CCR6 = pHandle->hCntSmp2;

    
 if (bStatorFluxPos == REGULAR)
  {
    LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_OC5_RISING_OC6_RISING);
    
	switch (pHandle->bInverted_pwm_new)
    {
      case INVERT_A:
        pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhA;

        break;
      case INVERT_B:
        pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhB;

        break;
      case INVERT_C:
        pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhC;

        break;
      default:
        break;
    }
  }
 else {
    /* disable DMA request update interrupt */
    LL_TIM_DisableDMAReq_UPDATE(TIM1);
    switch (pHandle->bInverted_pwm_new)
    {
      case INVERT_A:
        LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t) &(TIM1->CCR1));
        LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t) &(TIM1->CCMR1));        
        pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhA;
        pHandle->wPreloadDisableActing = pHandle->wPreloadDisableCC1;
        break;

      case INVERT_B:
        LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t) &(TIM1->CCR2));
        LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t) &(TIM1->CCMR1)); 
        pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhB;
        pHandle->wPreloadDisableActing = pHandle->wPreloadDisableCC2;
        break;

      case INVERT_C:
        LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t) &(TIM1->CCR3));
        LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t) &(TIM1->CCMR2)); 
        pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhC;
        pHandle->wPreloadDisableActing = pHandle->wPreloadDisableCC3;
        break;

      default:
        break;
    }    
    LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_OC5_RISING_OC6_FALLING);
    /*active vector*/
    LL_DMA_DisableChannel (DMA1, LL_DMA_CHANNEL_5);
    LL_DMA_SetDataLength (DMA1, LL_DMA_CHANNEL_5, 1);
    LL_DMA_EnableChannel (DMA1, LL_DMA_CHANNEL_5);

    /* enable DMA request update interrupt */
    LL_TIM_EnableDMAReq_UPDATE(TIM1);
 }
 
   /* Update Timer Ch 1,2,3 (These value are required before update event) */
  TIM1->CCR1 = pHandle->_Super.hCntPhA;
  TIM1->CCR2 = pHandle->_Super.hCntPhB;
  TIM1->CCR3 = pHandle->_Super.hCntPhC;
   
  /* Debug High frequency task duration
   * LL_GPIO_ResetOutputPin (GPIOB, LL_GPIO_PIN_3);
   */
  LL_ADC_REG_SetSequencerChannels ( ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->hIChannel ));
    /*check software error*/
  if (LL_TIM_IsActiveFlag_UPDATE(TIM1))
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
  
      
  /* The following instruction can be executed after Update handler 
     before the get phase current (Second EOC) */
      
  /* Set the current sampled */
   if (bStatorFluxPos == REGULAR) /* Regual zone */
  {
    pHandle->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pHandle->sampCur2 = REGULAR_SAMP_CUR2[bSector];
  }
  
  if (bStatorFluxPos == BOUNDARY_1) /* Two small, one big */
  {
    pHandle->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pHandle->sampCur2 = BOUNDR1_SAMP_CUR2[bSector];
  }
  
  if (bStatorFluxPos == BOUNDARY_2) /* Two big, one small */
  {
    pHandle->sampCur1 = BOUNDR2_SAMP_CUR1[bSector];
    pHandle->sampCur2 = BOUNDR2_SAMP_CUR2[bSector];
  }
  
  if (bStatorFluxPos == BOUNDARY_3)  
  {
    if (pHandle->bInverted_pwm_new == INVERT_A)
    {
      pHandle->sampCur1 = SAMP_OLDB;
      pHandle->sampCur2 = SAMP_IA;
    }
    if (pHandle->bInverted_pwm_new == INVERT_B)
    {
      pHandle->sampCur1 = SAMP_OLDA;
      pHandle->sampCur2 = SAMP_IB;
    }
  }
    
  /* Limit for the Get Phase current (Second EOC Handler) */
      
  return (hAux);
}

/**
  * @brief  R1_G0XX implement MC IRQ function TIMER Update
  * @param  this related object
  * @retval void* It returns always MC_NULL
  */
void R1G0XX_TIMx_UP_IRQHandler(PWMC_R1_G0_Handle_t *pHdl)
{ 

  LL_ADC_REG_SetTriggerSource (ADC1, LL_ADC_REG_TRIG_EXT_TIM1_TRGO2);
  LL_ADC_REG_StartConversion (ADC1);

}

/**
  * @brief  It contains the TIMx Break event interrupt connected to overcurrent.
  * @param  this related object
 * @retval none

  */
void* R1G0XX_OVERCURRENT_IRQHandler(PWMC_R1_G0_Handle_t *pHandle)
{ 
  if ( pHandle->BrakeActionLock == false )
  {
    if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
  }
  pHandle->OverCurrentFlag = true;
  
  return MC_NULL;
}

/**
  * @brief  It contains the TIMx Break event interrupt connected to overvoltage.
  * @param pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void * R1G0XX_OVERVOLTAGE_IRQHandler( PWMC_R1_G0_Handle_t * pHandle )
{
  pHandle->pParams_str->TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  pHandle->OverVoltageFlag = true;
  pHandle->BrakeActionLock = true;

  return &( pHandle->_Super.bMotor );
}

/**
  * @brief  It is used to check if an overcurrent occurred since last call.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
  *                  detected since last method call, MC_NO_FAULTS otherwise.
  */
uint16_t R1G0XX_IsOverCurrentOccurred(PWMC_Handle_t *pHdl)
{  
  PWMC_R1_G0_Handle_t *pHandle = (PWMC_R1_G0_Handle_t *)pHdl;
  uint16_t retVal = MC_NO_FAULTS;
  
  if ( pHandle->OverVoltageFlag == true )
  {
    retVal = MC_OVER_VOLT;
    pHandle->OverVoltageFlag = false;
  }
    
  if (pHandle->OverCurrentFlag == true)
  {
    retVal |= MC_BREAK_IN;
    pHandle->OverCurrentFlag = false;
  }
  return retVal;
}

/**
  * @}
  */
  
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
