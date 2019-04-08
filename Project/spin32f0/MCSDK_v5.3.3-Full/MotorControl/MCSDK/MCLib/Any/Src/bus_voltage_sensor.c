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
  * 	�ṩ�������ߵ�ѹ������ʵ�֣�
  *
  * - The @ref RDividerBusVoltageSensor "Resistor Divider Bus Voltage Sensor"�����ѹ��ĸ�ߵ�ѹ������---�����չ���˼������
  * - The @ref VirtualBusVoltageSensor "Virtual Bus Voltage Sensor"�������ߵ�ѹ������---�������в��������Ƿ��ع̶���Ӧ�ó�����ֵ��
  *
  * @todo Document the Bus Voltage Sensor "module".
  *			��¼���ߵ�ѹ������
  * @{
  */

/**
  * @brief  It return latest Vbus conversion result expressed in u16Volt     ��������u16Volt��ʾ������Vbusת�����
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t           pHandle���BusVoltageSensor_Handle_t�ľ��
  * @retval uint16_t Latest Vbus conversion result in digit                uint16_t���µ�Vbusת�����Ϊ����
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
  * @brief  It return latest averaged Vbus measurement expressed in u16Volt  ��������u16Volt��ʾ������ƽ��Vbus����ֵ
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t   pHandle���BusVoltageSensor_Handle_t�ľ��
  * @retval uint16_t Latest averaged Vbus measurement in digit   uint16_t���µ�ƽ��Vbus����ֵ
  */
uint16_t VBS_GetAvBusVoltage_d( BusVoltageSensor_Handle_t * pHandle )
{
  return ( pHandle->AvBusVoltage_d );
}

/**
  * @brief  It return latest averaged Vbus measurement expressed in Volts  �������Է��ر�ʾ������ƽ��Vbus����ֵ
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t           pHandle���BusVoltageSensor_Handle_t�ľ��
  * @retval uint16_t Latest averaged Vbus measurement in Volts				uint16_t�Է���Ϊ��λ������ƽ��Vbus����ֵ		
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
  *         bus voltage and protection threshold values				���������ߵ�ѹ�ͱ�����ֵ����MC_OVER_VOLT��MC_UNDER_VOLT��MC_NO_ERROR
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t			pHandle���BusVoltageSensor_Handle_t�ľ��
  * @retval uint16_t Fault code error    uint16_t���ϴ������
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
