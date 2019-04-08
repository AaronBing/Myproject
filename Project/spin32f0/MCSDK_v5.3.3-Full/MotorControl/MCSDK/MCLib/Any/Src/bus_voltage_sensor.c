/**
  ******************************************************************************
  * @file    bus_voltage_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the BusVoltageSensor component of the Motor Control SDK.
  *
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "bus_voltage_sensor.h"


/** @addtogroup MCSDK
  * @{
  */

/** @defgroup BusVoltageSensor Bus Voltage Sensor
  * @brief Bus Voltage Sensor components of the Motor Control SDK
  *
  * 	提供两种总线电压传感器实现：
  *
  * - The @ref RDividerBusVoltageSensor "Resistor Divider Bus Voltage Sensor"电阻分压器母线电压传感器---》按照顾名思义运作
  * - The @ref VirtualBusVoltageSensor "Virtual Bus Voltage Sensor"虚拟总线电压传感器---》不进行测量，而是返回固定的应用程序定义值。
  *
  * @todo Document the Bus Voltage Sensor "module".
  *			记录总线电压传感器
  * @{
  */

/**
  * @brief  It return latest Vbus conversion result expressed in u16Volt     它返回以u16Volt表示的最新Vbus转换结果
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t           pHandle相关BusVoltageSensor_Handle_t的句柄
  * @retval uint16_t Latest Vbus conversion result in digit                uint16_t最新的Vbus转换结果为数字
  */
uint16_t VBS_GetBusVoltage_d( BusVoltageSensor_Handle_t * pHandle )
{
  return ( pHandle->LatestConv );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  It return latest averaged Vbus measurement expressed in u16Volt  它返回以u16Volt表示的最新平均Vbus测量值
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t   pHandle相关BusVoltageSensor_Handle_t的句柄
  * @retval uint16_t Latest averaged Vbus measurement in digit   uint16_t最新的平均Vbus测量值
  */
uint16_t VBS_GetAvBusVoltage_d( BusVoltageSensor_Handle_t * pHandle )
{
  return ( pHandle->AvBusVoltage_d );
}

/**
  * @brief  It return latest averaged Vbus measurement expressed in Volts  它返回以伏特表示的最新平均Vbus测量值
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t           pHandle相关BusVoltageSensor_Handle_t的句柄
  * @retval uint16_t Latest averaged Vbus measurement in Volts				uint16_t以伏特为单位的最新平均Vbus测量值		
  */
uint16_t VBS_GetAvBusVoltage_V( BusVoltageSensor_Handle_t * pHandle )
{
  uint32_t temp;

  temp = ( uint32_t )( pHandle->AvBusVoltage_d );
  temp *= pHandle->ConversionFactor;
  temp /= 65536u;

  return ( ( uint16_t )temp );
}

/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
  *         bus voltage and protection threshold values				它根据总线电压和保护阈值返回MC_OVER_VOLT，MC_UNDER_VOLT或MC_NO_ERROR
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t			pHandle相关BusVoltageSensor_Handle_t的句柄
  * @retval uint16_t Fault code error    uint16_t故障代码错误
  */
uint16_t VBS_CheckVbus( BusVoltageSensor_Handle_t * pHandle )
{
  return ( pHandle->FaultState );
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
