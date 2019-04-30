
/**
  ******************************************************************************
  * @file    parameters_conversion.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file includes the proper Parameter conversion on the base
  *          of stdlib for the first drive
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_H
#define __PARAMETERS_CONVERSION_H

#include "pmsm_motor_parameters.h"
#include "parameters_conversion_g0xx.h"
#include "mc_math.h"

#define ADC_REFERENCE_VOLTAGE  3.30

/************************* CONTROL FREQUENCIES & DELAIES **********************/
#define TF_REGULATION_RATE 	(uint16_t) ((uint16_t)(PWM_FREQUENCY)/REGULATION_EXECUTION_RATE)
#define REP_COUNTER 			(uint16_t) ((REGULATION_EXECUTION_RATE *2u)-1u)

#define MEDIUM_FREQUENCY_TASK_RATE	(uint16_t)SPEED_LOOP_FREQUENCY_HZ

#define INRUSH_CURRLIMIT_DELAY_COUNTS  (uint16_t)(INRUSH_CURRLIMIT_DELAY_MS * \
                                  ((uint16_t)SPEED_LOOP_FREQUENCY_HZ)/1000u -1u)

#define SYS_TICK_FREQUENCY          2000
#define UI_TASK_FREQUENCY_HZ        10
#define SERIAL_COM_TIMEOUT_INVERSE  25
#define SERIAL_COM_ATR_TIME_MS 20

#define MF_TASK_OCCURENCE_TICKS  (SYS_TICK_FREQUENCY/SPEED_LOOP_FREQUENCY_HZ)-1u
#define UI_TASK_OCCURENCE_TICKS  (SYS_TICK_FREQUENCY/UI_TASK_FREQUENCY_HZ)-1u
#define SERIALCOM_TIMEOUT_OCCURENCE_TICKS (SYS_TICK_FREQUENCY/SERIAL_COM_TIMEOUT_INVERSE)-1u
#define SERIALCOM_ATR_TIME_TICKS (uint16_t)(((SYS_TICK_FREQUENCY * SERIAL_COM_ATR_TIME_MS) / 1000u) - 1u)

/************************* OBSERVER + PLL PARAMETERS **************************/
#define MAX_BEMF_VOLTAGE  (uint16_t)((MAX_APPLICATION_SPEED * 1.2 *\
                           MOTOR_VOLTAGE_CONSTANT*SQRT_2)/(1000u*SQRT_3))

/*max phase voltage, 0-peak Volts*/
#define MAX_VOLTAGE (int16_t)((ADC_REFERENCE_VOLTAGE/2)/BUS_ADC_CONV_RATIO) 

#define MAX_CURRENT (ADC_REFERENCE_VOLTAGE/(2*RSHUNT*AMPLIFICATION_GAIN))

#define C1 (int32_t)((((int16_t)F1)*RS)/(LS*TF_REGULATION_RATE))
#define C2 (int32_t) GAIN1
#define C3 (int32_t)((((int16_t)F1)*MAX_BEMF_VOLTAGE)/(LS*MAX_CURRENT*TF_REGULATION_RATE))
#define C4 (int32_t) GAIN2
#define C5 (int32_t)((((int16_t)F1)*MAX_VOLTAGE)/(LS*MAX_CURRENT*TF_REGULATION_RATE))

#define PERCENTAGE_FACTOR    (uint16_t)(VARIANCE_THRESHOLD*128u)      
#define OBS_MINIMUM_SPEED        (uint16_t) (OBS_MINIMUM_SPEED_RPM/6u)
#define HFI_MINIMUM_SPEED        (uint16_t) (HFI_MINIMUM_SPEED_RPM/6u)

/*********************** OBSERVER + CORDIC PARAMETERS *************************/
#define CORD_C1 (int32_t)((((int16_t)CORD_F1)*RS)/(LS*TF_REGULATION_RATE))
#define CORD_C2 (int32_t) CORD_GAIN1
#define CORD_C3 (int32_t)((((int16_t)CORD_F1)*MAX_BEMF_VOLTAGE)/(LS*MAX_CURRENT\
                                                           *TF_REGULATION_RATE))
#define CORD_C4 (int32_t) CORD_GAIN2
#define CORD_C5 (int32_t)((((int16_t)CORD_F1)*MAX_VOLTAGE)/(LS*MAX_CURRENT*\
                                                          TF_REGULATION_RATE))
#define CORD_PERCENTAGE_FACTOR    (uint16_t)(CORD_VARIANCE_THRESHOLD*128u)      
#define CORD_MINIMUM_SPEED        (uint16_t) (CORD_MINIMUM_SPEED_RPM/6u)

/**************************   VOLTAGE CONVERSIONS  ****************************/
#define BUS_ADC_CONV_RATIO       VBUS_PARTITIONING_FACTOR

#define OVERVOLTAGE_THRESHOLD_d   (uint16_t)(OV_VOLTAGE_THRESHOLD_V*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR))
#define UNDERVOLTAGE_THRESHOLD_d  (uint16_t)((UD_VOLTAGE_THRESHOLD_V*65535)/\
                                  ((uint16_t)(ADC_REFERENCE_VOLTAGE/\
                                                           BUS_ADC_CONV_RATIO)))
#define INT_SUPPLY_VOLTAGE          (uint16_t)(65536/ADC_REFERENCE_VOLTAGE)

#define DELTA_TEMP_THRESHOLD        (OV_TEMPERATURE_THRESHOLD_C- T0_C)
#define DELTA_V_THRESHOLD           (dV_dT * DELTA_TEMP_THRESHOLD)
#define OV_TEMPERATURE_THRESHOLD_d  ((V0_V + DELTA_V_THRESHOLD)*INT_SUPPLY_VOLTAGE)

#define DELTA_TEMP_HYSTERESIS        (OV_TEMPERATURE_HYSTERESIS_C)
#define DELTA_V_HYSTERESIS           (dV_dT * DELTA_TEMP_HYSTERESIS)
#define OV_TEMPERATURE_HYSTERESIS_d  (DELTA_V_HYSTERESIS*INT_SUPPLY_VOLTAGE)

/*************** Encoder Alignemnt ************************/
    
/* Encoder alignment */
#define ALIGNMENT_ANGLE_S16      (int16_t)  (ALIGNMENT_ANGLE_DEG*65536u/360u)

/*************** Timer for PWM generation & currenst sensing parameters  ******/
#define PWM_PERIOD_CYCLES (uint16_t)(ADV_TIM_CLK_MHz*\
                                      (unsigned long long)1000000u/((uint16_t)(PWM_FREQUENCY)))

#define DEADTIME_NS  SW_DEADTIME_NS

#define DEAD_TIME_ADV_TIM_CLK_MHz (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
#define DEAD_TIME_COUNTS_1  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/1000uL)

#if (DEAD_TIME_COUNTS_1 <= 255)
#define DEAD_TIME_COUNTS (uint16_t) DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS_1 <= 1008)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS_1 <= 2015)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS 510
#endif

#define DTCOMPCNT (uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz) / 2000)
#define TON_NS  500
#define TOFF_NS 500
#define TON  (uint16_t)((TON_NS * ADV_TIM_CLK_MHz)  / 2000)
#define TOFF (uint16_t)((TOFF_NS * ADV_TIM_CLK_MHz) / 2000)

#define MAX_TNTR_NS TRISE_NS

#define SAMPLING_TIME (uint16_t)(((uint16_t)(SAMPLING_TIME_NS) * ADV_TIM_CLK_MHz)/1000uL) 
#define TRISE (uint16_t)((((uint16_t)(TRISE_NS)) * ADV_TIM_CLK_MHz)/1000uL)
#define TDEAD (uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz)/1000uL)

#define TMIN (((uint16_t)(((DEADTIME_NS+((uint16_t)(TRISE_NS))+\
			 ((uint16_t)(SAMPLING_TIME_NS+TRIG_CONV_LATENCY_NS)))*ADV_TIM_CLK_MHz)/1000ul))+1)
#define HTMIN (uint16_t)(TMIN >> 1)
#define CHTMIN (uint16_t)(TMIN/(REGULATION_EXECUTION_RATE*2))
#define TAFTER ((uint16_t)(((DEADTIME_NS+((uint16_t)(TRISE_NS)))\
					   *ADV_TIM_CLK_MHz)/1000ul))

#define TBEFORE (((uint16_t)(((((uint16_t)(SAMPLING_TIME_NS+TRIG_CONV_LATENCY_NS)))\
                                           *ADV_TIM_CLK_MHz)/1000ul))+1)
                                           

#if (TRISE_NS > SAMPLING_TIME_NS)
#define MAX_TRTS (2 * TRISE)
#else
#define MAX_TRTS (2 * SAMPLING_TIME)
#endif

#define TNOISE (uint16_t)((((uint16_t)(TNOISE_NS)) * ADV_TIM_CLK_MHz)/1000uL)

#define MAX_TNTR_NS TRISE_NS

#define TW_AFTER ((uint16_t)(((DEADTIME_NS+MAX_TNTR_NS)*ADV_TIM_CLK_MHz)/1000ul))

/*************** PI divisor  ***************/
#define SP_KPDIV_LOG LOG2(128)
#define SP_KIDIV_LOG LOG2(16384)
#define SP_KDDIV_LOG LOG2(16)
#define TF_KPDIV_LOG LOG2(4096)
#define TF_KIDIV_LOG LOG2(16384)
#define TF_KDDIV_LOG LOG2(8192)
#define FW_KPDIV_LOG LOG2(32768)
#define FW_KIDIV_LOG LOG2(32768)
#define PLL_KPDIV     16384
#define PLL_KPDIV_LOG LOG2(PLL_KPDIV)
#define PLL_KIDIV     65535
#define PLL_KIDIV_LOG LOG2(PLL_KIDIV)
#define F1_LOG LOG2(8192)
#define F2_LOG LOG2(8192)
#define STO_FIFO_DEPTH_DPP_LOG LOG2(64)
#define CORD_FIFO_DEPTH_DPP_LOG LOG2(64)
#define HFI_PID_KPDIV_LOG LOG2(16384)
#define HFI_PID_KIDIV_LOG LOG2(32768)
 
/* USER CODE BEGIN virtual temperature */

#define M1_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE   25u
#define M1_TEMP_SW_FILTER_BW_FACTOR      250u
#define M1_VQD_SW_FILTER_BW_FACTOR       128u
#define M1_VQD_SW_FILTER_BW_FACTOR_LOG LOG2(M1_VQD_SW_FILTER_BW_FACTOR)

/* USER CODE END virtual temperature */
 
 
#define PQD_CONVERSION_FACTOR (int32_t)(( 1000 * 3 * ADC_REFERENCE_VOLTAGE ) /\
             ( 1.732 * RSHUNT * AMPLIFICATION_GAIN ))

#define USART_IRQHandler        USART3_IRQHandler

/****** Prepares the UI configurations according the MCconfxx settings ********/
#define COM_ENABLE | OPT_COM

#define DAC_ENABLE | OPT_DAC
#define DAC_OP_ENABLE | UI_CFGOPT_DAC

/* Motor 1 settings */
#define FW_ENABLE

#define DIFFTERM_ENABLE

/* Sensors setting */
#define MAIN_SCFG UI_SCODE_STO_PLL

#define AUX_SCFG 0x0

#define PLLTUNING_ENABLE

#define UI_CFGOPT_PFC_ENABLE

/******************************************************************************* 
  * UI configurations settings. It can be manually overwritten if special 
  * configuartion is required. 
*******************************************************************************/

/* Specific options of UI */
#define UI_CONFIG_M1 ( UI_CFGOPT_NONE DAC_OP_ENABLE FW_ENABLE DIFFTERM_ENABLE \
  | (MAIN_SCFG << MAIN_SCFG_POS) | (AUX_SCFG << AUX_SCFG_POS) | UI_CFGOPT_SETIDINSPDMODE PLLTUNING_ENABLE UI_CFGOPT_PFC_ENABLE | UI_CFGOPT_PLLTUNING)

#define UI_CONFIG_M2

#define ENABLE_START_STOP_BUTTON

#define DIN_ACTIVE_LOW Bit_RESET
#define DIN_ACTIVE_HIGH Bit_SET

 /*** Temporary bridge between workbench data model****/
#define PWM_TIM1	TIM1
#define PWM_TIM8  TIM8
#define HALL_TIM2 TIM2
#define HALL_TIM3 TIM3
#define HALL_TIM4 TIM4
#define HALL_TIM5 TIM5

#define ENC_TIM2 TIM2
#define ENC_TIM3 TIM3
#define ENC_TIM4 TIM4
#define ENC_TIM5 TIM5

#define DOUT_ACTIVE_HIGH   DOutputActiveHigh
#define DOUT_ACTIVE_LOW    DOutputActiveLow

#define START_INDEX     62
#define MAX_MODULE      32439   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*99%
#define MMITABLE {\
32635,32375,32121,31873,31631,31394,31162,30935,30714,30497,\
30284,30076,29872,29672,29574,29380,29190,29003,28820,28641,\
28464,28291,28122,27955,27791,27630,27471,27316,27163,27012,\
26864,26718,26575,26434,26295,26159,26024,25892,25761,25633,\
25569,25444,25320,25198,25078,24959,24842,24727,24613,24501,\
24391,24281,24174,24067,23963,23859,23757,23656,23556,23458,\
23361,23265,23170,23077,22984,22893\
}

#endif /*__PARAMETERS_CONVERSION_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
