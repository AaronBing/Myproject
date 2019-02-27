/**
  ******************************************************************************
  * @file    ui_exported_functions.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions of UI exported functions.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UIEXPORTEDFUNCTIONS_H
#define __UIEXPORTEDFUNCTIONS_H

enum {
EF_UI_GetReg,
EF_UI_ExecSpeedRamp,
EF_UI_SetReg,
EF_UI_ExecCmd,
EF_UI_GetSelectedMCConfig,
EF_UI_SetRevupData,
EF_UI_GetRevupData,
EF_UI_DACChannelConfig,
EF_UI_SetCurrentReferences,
EF_UI_NUMBERS
};

typedef int32_t (*pUI_GetReg_t) (UI_Handle_t *pHandle,MC_Protocol_REG_t);
typedef bool (*pUI_ExecSpeedRamp_t)(UI_Handle_t *pHandle,int32_t,uint16_t);
typedef bool (*pUI_SetReg_t)(UI_Handle_t *pHandle,MC_Protocol_REG_t,int32_t);
typedef bool (*pUI_ExecCmd_t)(UI_Handle_t *pHandle,uint8_t);
typedef uint32_t (*pUI_GetSelectedMCConfig_t)(UI_Handle_t *pHandle);
typedef bool (*pUI_SetRevupData_t)(UI_Handle_t *pHandle,uint8_t,uint16_t,int16_t,int16_t);
typedef bool (*pUI_GetRevupData_t)(UI_Handle_t *pHandle,uint8_t,uint16_t*,int16_t*,int16_t*);
typedef void (*pUI_DACChannelConfig_t)(UI_Handle_t *pHandle,DAC_Channel_t,MC_Protocol_REG_t);
typedef void (*pUI_SetCurrentReferences_t)(UI_Handle_t *pHandle,int16_t,int16_t); 

#endif /*__UIEXPORTEDFUNCTIONS_H*/
/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
