/**
  ******************************************************************************
  * @file    bus_voltage_sensor.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          BusVoltageSensor component of the Motor Control SDK.
            该文件包含Motor Control SDK的* BusVoltageSensor组件的所有定义和函数原型。
  ******************************************************************************
  * @attention
  *
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
  SensorType_t SensorType;    /*!< It contains the information about the type
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
                                   be equal to 500 */

  uint16_t LatestConv;        /*!< It contains latest Vbus converted value
                                   expressed in u16Volts format */
  uint16_t AvBusVoltage_d;    /*!< It contains latest available average Vbus
                                   expressed in digit */
  uint16_t FaultState;        /*!< It contains latest Fault code (MC_NO_ERROR,
                                   MC_OVER_VOLT or MC_UNDER_VOLT) */
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
