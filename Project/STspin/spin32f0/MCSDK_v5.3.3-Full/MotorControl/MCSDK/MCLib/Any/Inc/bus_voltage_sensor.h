/**
  ******************************************************************************
  * @file    bus_voltage_sensor.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          BusVoltageSensor component of the Motor Control SDK.
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
  * @ingroup BusVoltageSensor
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUSVOLTAGESENSOR_H
#define __BUSVOLTAGESENSOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pwm_curr_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup BusVoltageSensor
  * @{
  */

/**
  * @brief  BusVoltageSensor handle definition
  */
typedef struct
{
  SensorType_t SensorType;    /*!< It contains the information about the type    它包含有关实例总线电压传感器对象类型的信息。它可以等于REAL_SENSOR或VIRTUAL_SENSO
                                   of instanced bus voltage sensor object.
                                   It can be equal to REAL_SENSOR or
                                   VIRTUAL_SENSOR */
  uint16_t ConversionFactor;  /*!< It is used to convert bus voltage from
                                   u16Volts into real Volts (V).
                                   1 u16Volt = 65536/hConversionFactor Volts
                                   For real sensors hConversionFactor it's
                                   equal to the product between the expected MCU
                                   voltage and the voltage sensing network
                                   attenuation. For virtual sensors it must
                                   be equal to 500 
									
								它用于将总线电压从u16Volts转换为实际伏特（V）。
 								1 u16Volt = 65536 / h转换因子电压
								对于真实传感器hConversionFactor它等于预期的MCU电压和电压感应网络衰减之间的乘积。
								对于虚拟传感器，它必须等于500
	
	
	*/

  uint16_t LatestConv;        /*!< It contains latest Vbus converted value
                                   expressed in u16Volts format 
								   它包含以u16Volts格式表示的最新Vbus转换值*/
  uint16_t AvBusVoltage_d;    /*!< It contains latest available average Vbus
                                   expressed in digit 
								   它包含以数字表示的最新可用平均Vbus */
  uint16_t FaultState;        /*!< It contains latest Fault code (MC_NO_ERROR,
                                   MC_OVER_VOLT or MC_UNDER_VOLT) 
								   它包含最新的故障代码（MC_NO_ERROR，MC_OVER_VOLT或MC_UNDER_VOLT）*/
} BusVoltageSensor_Handle_t;


/* Exported functions ------------------------------------------------------- */
uint16_t VBS_GetBusVoltage_d( BusVoltageSensor_Handle_t * pHandle );
uint16_t VBS_GetAvBusVoltage_d( BusVoltageSensor_Handle_t * pHandle );
uint16_t VBS_GetAvBusVoltage_V( BusVoltageSensor_Handle_t * pHandle );
uint16_t VBS_CheckVbus( BusVoltageSensor_Handle_t * pHandle );

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __BusVoltageSensor_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
