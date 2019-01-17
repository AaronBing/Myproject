/***********************************************************
*	故障灯
*
*
*
************************************************************/
#include "extern.h"

u8 ErrorLedCnt;//led控制计次
u8 ErrorLedOnOff;//led开启

void ErrorLedCtrlFun(void);//控制led闪烁频率 故障指示



//-------------------------------------------------------------
void ErrorLedCtrlFun()
{//控制led闪烁频率 故障指示
  u16 cnt=0; 
	
   if ( FocFlagMotorSErrorElVol )
  {//短线一侧静态电流故障
    cnt = 1;
  }

  else if ( ErrorCircuit )
  {//电机相线有短路
    cnt = 3;
  }
  else if ( MotorSErrorPosFlag )
  {//短线一侧电机霍尔故障
    cnt = 4;
  }

  else if ( BatteryFlagError )
  {//电池故障标志
    cnt = 6;
  }
  else
  {
    cnt = 0;//不闪烁 无错误
  }
	
  if ( cnt )
  {
    if ( !ErrorLedOnOff )
    {
      ++ErrorLedCnt;//计数
      if ( ErrorLedCnt > cnt )
        ErrorLedCnt = 0;
    }
    if ( ErrorLedCnt >= cnt )
    {
      GPIO_ResetBits(GPIOB, 4);//LED
    }
			else if ( ErrorLedOnOff )
			{
				GPIO_SetBits(GPIOB, 4);//LED亮
			}
				else
				{
					GPIO_ResetBits(GPIOB, 4);//LED灭
				}
    ErrorLedOnOff ^= 1u;  //取反
  }
  else
  {
    GPIO_ResetBits(GPIOB, 4);//LED
  }
}
