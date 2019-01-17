/***********************************************************
*	喇叭
*
*
*
************************************************************/

#include "extern.h"



u8 BuzzFlagPowerOn;	//开关
u8 BuzzPowerOnType;	//开关类型
u8 BuzzRecoveryCnt;	//计数器恢复计数
u8 BatteryFlagLowBak;

u8 BuzzFlagunKnown1;//开机buzz完成
u8 BuzzFlagunKnown2;

u8 BuzzCnt;					//嗡鸣器计次
u8 BuzzTotal;				//
u8 BuzzOn;					//嗡鸣器开关
u16 BuzzType;				//鸣叫类型 out


void BuzzCtrlFun(void);
void BuzzFun(void);





//-------------------------------------------------------------
void BuzzCtrlFun()
{
//  signed int i; // 

	/********开机喇叭控制***********/
  if ( BuzzPowerOnType )
  {//可能是开机叫的声音
    if ( BuzzPowerOnType == 1 )
    {//开机
      BuzzType = 2;
      if ( BuzzRecoveryCnt >= 80 )
      {
        BuzzRecoveryCnt = 0;
        BuzzPowerOnType = 2;
      }
      else
      {
        ++BuzzRecoveryCnt;
      }
    }
    else if ( BuzzPowerOnType == 2 )
    {
      BuzzType = 3;
      if ( BuzzRecoveryCnt >= 80 )//原版的声音是80
      {
        BuzzRecoveryCnt = 0;
        BuzzPowerOnType = 3;
      }
      else
      {
        ++BuzzRecoveryCnt;
      }
    }
    else
    {
      BuzzType = 0;
      BuzzRecoveryCnt = 0;
      BuzzPowerOnType = 0;
      BuzzFlagPowerOn = 1;//开机标志 喇叭鸣叫完成
    }
  }
  else
  {
    BuzzType = 1;
    if ( BuzzRecoveryCnt >= 80 )//原版的声音是80
    {
      BuzzRecoveryCnt = 0;
      BuzzPowerOnType = 1;
    }
    else
    {
      ++BuzzRecoveryCnt;
    }
  }
}

//-------------------------------------------------------------
void BuzzFun()
{//控制嗡鸣器状态
  if ( BuzzType == 1 )
  {
    if ( !BuzzCnt )//报警计次为0
      BuzzTotal = 6;
  }
  else if ( BuzzType == 2 )
  {
    if ( !BuzzCnt )
      BuzzTotal = 5;
  }
  else if ( (BuzzType == 3) && (!BuzzCnt) )
  {
    BuzzTotal = 4;
  }
  if ( BuzzCnt >= BuzzTotal )
  {
    BuzzCnt = 0;
    if ( BuzzOn )
    {
      BuzzOn = 0;
      GPIO_SetBits(GPIOA, 16);//喇叭
    }
    else
    {
      BuzzOn = 1;
      GPIO_ResetBits(GPIOA, 16);
    }
  }
  else
  {
    ++BuzzCnt;
  }
}

