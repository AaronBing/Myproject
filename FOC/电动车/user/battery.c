/***********************************************************
*	电池电压
*
*
*
************************************************************/
#include "extern.h"


u8  BatteryFlagLowRunOut;//不知道的标志位
u8  BatteryFlagVolLower;
u16 BatteryValLowest=0xffff;
s16 BatteryDownCnt1;
u8  BatteryFlagError;//电压故障标志
u8  BatteryFlagVolLowest;
u16 BatteryVal1;// 电压值

u8  BatteryFlagPowerOnOut;//开机标志位，外部使用
u8  BatteryFlagLow;//电池电压低标志位
u8  BatteryFlagPowerOn;//开机标志位
u16 BatteryPowerOnCnt2;//开机计数器2
u16 BatteryFlagPowerOnCnt;//开机计数器
u32 BatteryAutoOffCnt;//自动关机
u16 BatteryPowerOnCnt;//开机计数器
u8 BatteryFlagLowCnt;//电池电压低计数器
u8 BatteryFlagHighCnt;//电池电压高计数器

void BatteryGetFun(void);
void BatteryCheckFun(void);//获取充电器状态
void BatteryCheckPowerOnFun(void);

//-------------------------------------------------------------
void BatteryGetFun()
{//电池电压
  u16 sum; 
  u8 i; 

	//电机2的电压
  sum = 0;
  for ( i = 0; i < 4; i = (u8)(i + 1) )
    sum += FocAdcBatteryBuf[i];
  BatteryVal1 = sum >> 2;//电压值平均
	
  if ( BatteryVal1 <= 720 )
  {//超出范围 关闭pwm
    BatteryFlagError = 1;//电池故障标志
    TIM1->BDTR &= 0x7FFFu;//关闭pwm
    TIM8->BDTR &= 0x7FFFu;//关闭pwm
  }
	//电压检测
//  if ( (((FocMotorSCurDiff + FocMotorLCurDiff) > 0) 
//		&& ((FocMotorSCurDiff + FocMotorLCurDiff) < 105))
//		&& (BatteryValLowest > BatteryVal1) )
    if ( ((FocMotorSCurDiff  > 0) 
		&& (FocMotorSCurDiff  < 105))
		&& (BatteryValLowest > BatteryVal1) )
  {//电流差
		//电池电量不停消耗 这里更新当前的电池电压值
    if ( BatteryDownCnt1 >= 80 )
    {
      BatteryDownCnt1 = 0;//计次
      BatteryValLowest = BatteryVal1;//最低电压
    }
    else
    {
      ++BatteryDownCnt1;
    }
  }
	if ( BatteryFlagLow )
	{//电压低
    BatteryValLowest = BatteryVal1;
	}
	//电压低
	if(BatteryValLowest < 1373)
	{//最低电压
		BatteryFlagVolLower = 1;
	}
	else
	{
		BatteryFlagVolLower = 0;
	}
	if(BatteryValLowest <= 1360)
	{//电压更低
		BatteryFlagVolLowest = 1;
	}
	else
	{
		BatteryFlagVolLowest = 0;
	}
	
  if((MotorSTimePole6 < 1350) 			 	
			&& (BatteryFlagVolLowest == 1)) //6次霍尔切换的总时间
	{
		BatteryFlagLowRunOut = 1;
	}
	else BatteryFlagLowRunOut = 0;
  
}


//-------------------------------------------------------------
void BatteryCheckFun()
{//电池检测
  if ( GPIO_ReadInputDataBit(GPIOA, 4096) )
  {//
    BatteryFlagLowCnt = 0;//电压低计次
    if ( BatteryFlagHighCnt >= 10 )
    {
      BatteryFlagLow = 0;
    }
    else
    {
      ++BatteryFlagHighCnt;//电压高计次
    }
  }
  else
  {
    BatteryFlagHighCnt = 0;
    if ( BatteryFlagLowCnt >= 10 )
    {
      BatteryFlagLow = 1;//电池电压低
    }
    else
    {
      ++BatteryFlagLowCnt;
    }
  }
}

//-------------------------------------------------------------
void BatteryCheckPowerOnFun()
{//开关检测
	u32 vol; 
  s32 i; 
//  s32 j;

  vol = (FocBootVolBuf[3] + FocBootVolBuf[2] + FocBootVolBuf[1] + FocBootVolBuf[0]) >> 2;
  
	if ( BatteryPowerOnCnt < 1000 )
    ++BatteryPowerOnCnt;
  if ( BatteryFlagPowerOn )
  {//开机后
		//------------检测电压------------------
    if ( vol <= 1240 )
    {
			//电压低
      if ( BatteryPowerOnCnt2 > 6 )
      {
        if ( BatteryPowerOnCnt >= 800 )
        {//电压检测循环计次
					//时间长关机，
          for ( i = 0; i < 200; i = (u16)(i + 1) )
          {
            while ( !FocTime1msFlag )
              ;
            FocTime1msFlag = 0;
          }
          GPIO_ResetBits(GPIOA, 32);//关机
        }
        else
        {//进入儿童模式
          BatteryPowerOnCnt = 800;
        }
      }
      BatteryPowerOnCnt2 = 0;
    }
    else if ( BatteryPowerOnCnt2 < 500 )
    {
      ++BatteryPowerOnCnt2;
    }
		//-----------自动关机-----------------
    if ( (MotorSFlagMove != 1) )
    {
      BatteryAutoOffCnt = 0;
    }
    else if ( 
			 (BatteryFlagError)				//电池故障
			|| MotorSErrorPosFlag		//			
			|| ErrorCircuit			
			|| FocFlagMotorSErrorElVol	//短线一侧静态电流故障			
				)					
    {//自动关机
      if ( BatteryAutoOffCnt >= 292968 )//无动作，关机 估计是10分钟
      {//故障循环计次
        GPIO_ResetBits(GPIOA, 32);//关机
      }
      else
      {
        BatteryAutoOffCnt++;//自动关机计数器
      }
    }
    else
    {
      BatteryAutoOffCnt = 0;//有动作，自动关机计数器清零
    }
  }
  else
	{
		if ( vol > 1240 )
		{
			//电压高 时间计数器
			if ( BatteryFlagPowerOnCnt < 1500 )
			{
				++BatteryFlagPowerOnCnt;
			}
//			else
//			{ 
//				RemoteFlagCorrectOut = 1;//按钮的时间长，进入校准
//			}
		}
		else if ( vol <= 1240 )
		{//否则开机
			BatteryFlagPowerOnCnt = 0;
			BatteryFlagPowerOn = 1;				//开始检测电压标志
			BatteryFlagPowerOnOut = 1;
		}
	}
}


