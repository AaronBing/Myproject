/***********************************************************
*	调控电机
*
*
*
************************************************************/
#include "extern.h"


u8  FocSelfCheckOK;//自检ok标志

//新增
u16 FocTime64usCnt;//TIM1_UP中断 1/15.625K = 64us 每64us中断一次
u16 FocGetAdcCnt;//获取adc转换次数
u8 	FocTime1msFlag;//16次TIM1_UP中断,1.024ms 约等于 1ms 延时

//长线电机一边的ad值
//u16 FocMotorLCurU; //U相电流值----经过运放放大的电压值
//u16 FocMotorLCurW; //W相电流值----经过运放放大的电压值
//u16 FocMotorLLoadBuf[4]={0};//母线电流检测  采集4次值,求平均值. 
u16 FocAdcBatteryBuf[4]={0};//电池电压检测  采集4次值,求平均值.
//短线电机一边的ad值
u16 FocMotorSCurU;
u16 FocMotorSCurW;
u16 FocMotorSLoadBuf[4];//母线电流检测  采集4次值,求平均值. 
u16 FocBootVolBuf[4];//开机电压检测 开机按键按下的时候

u8  FocFlagAdcOK;//采集4次adc完成 ok

//长线
//u8  FocMotorLCurVlotHigh;    //重负载????
//u8  FocFlagMotorLErrorElVol;//电机1电压故障 这里有修改
//u16 FocMotorLLoadRef;				//运放偏置电压
//s16 FocParaMotorLAttitude;	//电机转子角度????? --------陀螺仪的角度
//s16 FocMotorLCurDiff;		 //实际电流大小	 
//短线
u8  FocMotorSCurVlotHigh;
u8  FocFlagMotorSErrorElVol;//电机2电压故障
u16 FocMotorSLoadRef;				//
s16 FocParaMotorSAttitude;  
s16 FocMotorSCurDiff;

u16 FocMotorShPhaseAOffset;//短线端A相---静态偏置电流
u16 FocMotorShPhaseBOffset;//短线端C相---静态偏置电流
//u16 FocMotorLhPhaseAOffset;//长线端A相----静态偏置电流
//u16 FocMotorLhPhaseBOffset;//长线端B相----静态偏置电流

void FocSelfCheckingFun(void);
void FocVlotCmp1RegulationFun(void);//踏板速度调节
//-------------------------------------------------------------

void FocSelfCheckingFun()
{//安全检查 主要获取温漂 ok
  u16 i; // 
  u16 sum; //
  u16 sum1; // 

  MotorSTimePole6 = 0xffff;
	
//  BatteryFlagLow = 0;//电池电压低
//  BatteryFlagPowerOnOut = 0;
	
  for ( i = 0; i < 50; i = i + 1 ) //延时50ms
  {
    while ( !FocTime1msFlag );
    FocTime1msFlag = 0;
  }
	//--------开机检测---------------------
	i = FocBootVolBuf[0] + FocBootVolBuf[1];
	i += FocBootVolBuf[2];
	i += FocBootVolBuf[3];
  if ( (i >> 2) > 1240 )  //采集四次，取平均值
	{//开机检测
		GPIO_SetBits(GPIOA, 32);//开机
	}

  for ( i = 0; i < 380; i = (i + 1) )  //延时约380ms
  {
    while ( !FocTime1msFlag );
    FocTime1msFlag = 0;
  }

  sum = 0;
  for ( i = 0; i < 4; i = (u8)(i + 1) )
    sum = (u16)(FocMotorSLoadBuf[i] + sum);
  FocMotorSLoadRef = sum >> 2;//总线
  if ( (FocMotorSLoadRef >= 2234) || (FocMotorSLoadRef <= 1614) )
  {//超过范围 关闭pwm
    FocFlagMotorSErrorElVol = 1;//短线一侧静态电流故障
    TIM1->BDTR &= (~0x8000);//关闭pwm
  }

  //--------温漂---------(获取相电流偏置)-------------

  sum1 = 0;
  sum = 0;
  for ( i = 0; i < 16; i = (u8)(i + 1) )
  {
    sum1 += FocMotorSCurU;
    sum += FocMotorSCurW;
    while ( !FocTime1msFlag );
    FocTime1msFlag = 0;
  }
  FocMotorShPhaseAOffset = sum1 >> 4;
  FocMotorShPhaseBOffset = sum >> 4;
  if ( (FocMotorShPhaseAOffset > 2234)|| (FocMotorShPhaseAOffset < 1614)|| (FocMotorShPhaseBOffset > 2234)|| (FocMotorShPhaseBOffset < 1614) )
  {//超过范围 关闭pwm
    FocFlagMotorSErrorElVol = 1;
    TIM1->BDTR &= (~0x8000);//关闭pwm
  }
//  TIM1->BDTR &= (~0x8000);//关闭pwm
//  TIM8->BDTR &= (~0x8000);//关闭pwm
		
	FocSelfCheckOK = 1;//自检完成

}


//-------------------------------------------------------------
//中断
void TIM1_UP_IRQHandler(void)
{//time1溢出 控制电机主算法在此中断里进行
	//时间累计 定时器中断64us
  int torque_gas;
  int gas_faccelerate;
//  int attitude; 
//  int atat; 

	//-------------时间定时器------16*64us = 1024us ~ 1ms--------------
  if ( FocTime64usCnt >= 16 )
  {
    FocTime64usCnt = 0;
    FocTime1msFlag = 1;	//大概1ms定时标志
  }
  else
  {
    ++FocTime64usCnt;
  }
	//-------------获取adc计数------------------
	//每次中断检测一个----逐次检测(长线端母线电流，电池电压，短线端母线电流，开关机检测电压)
  if ( FocGetAdcCnt >= 3 )
  {
    FocFlagAdcOK = 1;//获取adc 4次完成
    FocGetAdcCnt = 0;
  }
  else
  {
    ++FocGetAdcCnt;
  }
	//-------------获取电角度-------------------(直接当库来用,后期了解)
  if ( FocSelfCheckOK )
  {//自检完成
    MotorSFetAngleFun();
  }
	
	//-------------获取电流L-------------------
//  FocMotorLCurU = ADC3->JDR1;//u
//  FocMotorLCurW = ADC3->JDR2;//w
//  FocMotorLLoadBuf[FocGetAdcCnt] = ADC3->JDR3;//总线电流
  FocAdcBatteryBuf[FocGetAdcCnt] = ADC3->JDR4;//电池电量缓冲数组	
	
	//-------------获取电流S-------------------
  FocMotorSCurU = ADC2->JDR1;//u相电流
  FocMotorSCurW = ADC2->JDR2;//w相电流
  FocMotorSLoadBuf[FocGetAdcCnt] = ADC2->JDR3;//总线电流  	
  FocBootVolBuf[FocGetAdcCnt] = ADC2->JDR4;//电压 开机检测按钮
//	FocAdcBatteryBuf[FocGetAdcCnt] = ADC3->JDR4;//电池电量缓冲数组
	
  MotorSCurr_ab.qI_Component2 = FocMotorShPhaseAOffset - FocMotorSCurU;   //A相实际电流
	
//B相实际电流, 实际测量到的是C相的电流，    由Ia+Ib+Ic=0;
//Ib = -(Ia+Ic) = -(MotorSCurr_ab.qI_Component2 +  FocMotorShPhaseBOffset - FocMotorSCurW )	
  MotorSCurr_ab.qI_Component1 = -(s16)(FocMotorShPhaseBOffset - FocMotorSCurW) 
																	- MotorSCurr_ab.qI_Component2;
  
	//-------------速度调节 -------------------(暂时当库来用，后期再研究)
	if ( FocParaMotorSAttitude < 0 )	//角度pid修正后的结果
    torque_gas = -FocParaMotorSAttitude;
  else
    torque_gas = FocParaMotorSAttitude;
  if ( MotorSAtatVolt_qd.qV_Component1 < 0 )
    gas_faccelerate = -MotorSAtatVolt_qd.qV_Component1;
  else
    gas_faccelerate = MotorSAtatVolt_qd.qV_Component1;
  if ( torque_gas <= gas_faccelerate )
  {
//    MotorSAtatVolt_qd.qV_Component1 = FocParaMotorSAttitude;  //这个是原来的。
	  MotorSAtatVolt_qd.qV_Component1 = 200; //这个是后面改的
  }
  else if ( !FocMotorSCurVlotHigh )
  {//负载不高
    if ( MotorSAtatVolt_qd.qV_Component1 >= FocParaMotorSAttitude )
    {
      if ( MotorSAtatVolt_qd.qV_Component1 > FocParaMotorSAttitude )
      {
        if ( MotorSAtatVolt_qd.qV_Component1 <= FocParaMotorSAttitude + 2 )
          --MotorSAtatVolt_qd.qV_Component1;
        else
          MotorSAtatVolt_qd.qV_Component1 -= 2;
      }
    }
    else if ( MotorSAtatVolt_qd.qV_Component1 + 2 >= FocParaMotorSAttitude )
    {
      ++MotorSAtatVolt_qd.qV_Component1;
    }
    else
    {
      MotorSAtatVolt_qd.qV_Component1 += 2;
    }
  }
	
  if ( FocSelfCheckOK )
  {//自检完成
		//---------控制电机S-------------------
    if ( MotorSFlagMove )
    {//车轮不移动后，积分归零
      MotorSInitParaFun();//参数初始化
    }
    else if ( (FocMotorSCurU < 3848)   //短线端U相电流
				&& (FocMotorSCurW < 3848) )     //短线端W相电流
    {
      MotorSClarkeFun();  //	Clarke坐标变换
      MotorSParkFun();		//  Park变换  计算角度参数等相关函数 
      MotorSFlowRegFun(); //负载流量的调节
    }
    MotorSRevParkFun();		//反park变换
    MotorSMotorCtrlFun(); //电机控制
		
  }

  TIM1->SR &= 0xFFFEu;
  TIM1->SR &= 0xFFFEu;
}


//-------------------------------------------------------------
// 应该不是负载调节，还在看。
// 可能是前馈电流

void FocVlotCmp1RegulationFun()
{//负载调节，总线电流调节
  u8 i; // 
  u32 val; // 
  u32 current; // 

	//---------motorS------------------
  val = 0;
  for ( i = 0; i < 4; i++ )
    val = (FocMotorSLoadBuf[i] + val);//总线电流
  current = val >> 2;
  FocMotorSCurDiff = current - FocMotorSLoadRef;
  if ( ((FocMotorSLoadRef + 717) >= current)
									&& (BatteryVal1 > 1032) )
  {//总线电流 电池电压 正常
    FocMotorSCurVlotHigh = 0;
  }
  else
  {//负载过大
    FocMotorSCurVlotHigh = 1;//负载大
    if ( MotorSAtatVolt_qd.qV_Component1 >= 0 )
      --MotorSAtatVolt_qd.qV_Component1;
    else
      ++MotorSAtatVolt_qd.qV_Component1;//负载调节
  }
}

