/***********************************************************
*	控制长线电机
*
*
*
************************************************************/

#include "extern.h"

//电机L
u8 MOTOR_PHASE_SEQUENCE[8]={0,2,4,3,6,1,5};//120度无刷电机 相位顺序表
Volt_Components 	MotorLAtatVolt_qd;// Vd   Vq
s16 MotorLCurrqd;
s16 MotorLFlowIntergral;
Curr_Components 	MotorLCurr_ab;// 当前相电流结构  Ia Ib 
Curr_Components  	MotorLCurrAlfaBeta; //iα  iβ
Volt_Components 	MotorLVol_ab;// 反PARK变换的  Vα  Vβ
Trig_Components  	MotorLVectorComponents; // 
/////////////////////////
u8 MotorLErrorPosFlag;//霍尔位置故障
u8 MotorLDirFlagLast;	//上一次霍尔改变的方向
u8 MotorLCurHall;			//当前霍尔
u8 MotorLLastHall;		//前次霍尔
u8 MotorLHallCnt;			//霍尔计数器

u8 MotorLFlagHallChange;	//霍尔切换标志位
u8 MotorLFlagPole6OK;			//霍尔6次完成ok
u16 MotorLTimePole=0xafc;	//霍尔切换一次
u16 MotorLPosCur;					//霍尔当前位置
u16 MotorLElectricalAngleBak;	//电角度相关
u16 MotorLUnknownPara1;
s16 MotorLEleAngleDiff;
u16 MotorLRollTimeLast;
u8 MotorLFlagShake;
u8 MotorLEleAnglePara1;
u8 MotorLPoleCnt;
u8 MotorLMoveCnt;
u8 MotorLFlagMove;
u16 MotorLTimePole6;//6次霍尔切换的总时间
u16 MotorLFlagDir;

u16 MotorLEleAngleParaA;
u16 MotorLEleAngleParaC;
u16 MotorLElectricalAngle;
u16 MotorLEleAngleParaD;
u16 MotorLEleAngleParaE;
u16 MotorLEleAngleParaF;
u16 MotorLEleAngleParaG;

u16 MotorLFinalEleAngle;
u16 MotorLTimePoleBuf[6];

void MotorLInitParaFun(void);
void MotorLFlowRegFun(void);
void MotorLClarkeFun(void);
void MotorLParkFun(void);
void MotorLRevParkFun(void);
void MotorLMotorCtrlFun(void);
void MotorLFetAngleFun(void); // weak
//长线一端的电机
//-------------------------------------------------------------
void MotorLFetAngleFun()
{//根据当前相位 获取电角度 64us执行一次
	s16 result;
  u32 sum; 
  signed int i; 
	s8 pos_diff;

  if ( MotorLTimePole < 2812 )//切换一个极的时间 小于180ms
    ++MotorLTimePole;
  if ( MotorLTimePole >= 2812 )//大于180ms 车轮停止
  {//换相时间过长 清除标志位
    MotorLTimePole6 			= 16872;//6次霍尔切换的总时间
    MotorLFlagHallChange = 0;		//霍尔切换
    MotorLFlagPole6OK 		= 0;		//车轮滚动一周标志位
    MotorLPoleCnt 			= 0;		//相序号
  }
  if ( (MotorLTimePole6 > 1013) //6次霍尔切换的总时间 > 1013 大概64ms
		|| ((5 * MotorLTimePole) > 1012) ) //5次换相的时间 > 1012 大概64ms
    MotorLEleAngleParaC = MotorLEleAngleParaG;//角度
	
	//----------电机是否移动------------------
  if ( (MotorLTimePole6 <= 1350) 		//6次霍尔切换的总时间 < 1013 大概86ms
		&& (6 * MotorLTimePole <= 1350) )//6次换相的时间 < 1013 大概86ms
  {
    if ( MotorLMoveCnt >= 6 )
    {
      MotorLFlagMove = 0;//车轮移动
      //ram_0C8 = 0;
    }
  }
  else
  {
    MotorLFlagMove = 1;//车轮未移动
    MotorLMoveCnt = 0;
  }
	
  if ( MotorLFlagDir )
  {
		//---------电机正向转、计算角度-----------------------
    if ( (MotorLPosCur == 6) && (MotorLEleAngleParaC != 0xffff) )
    {
      if ( MotorLFlagShake )
      {
        if ( (MotorLEleAngleParaC - MotorLUnknownPara1) <= MotorLElectricalAngleBak )
          MotorLElectricalAngleBak = MotorLEleAngleParaC;
        else
          MotorLElectricalAngleBak += MotorLUnknownPara1;
      }
      else
      {
        if ( (0xFFFF - MotorLUnknownPara1) < MotorLElectricalAngleBak )
          MotorLFlagShake = 1;
        MotorLElectricalAngleBak += MotorLUnknownPara1;
      }
    }
    else if ( MotorLEleAnglePara1 )
    {
      if ( (0xFFFF - MotorLUnknownPara1) < MotorLElectricalAngleBak )
        MotorLEleAnglePara1 = 0;
      MotorLElectricalAngleBak += MotorLUnknownPara1;
    }
    else if ( MotorLEleAngleParaC - MotorLUnknownPara1 <= MotorLElectricalAngleBak )
    {
      MotorLElectricalAngleBak = MotorLEleAngleParaC;
    }
    else
    {
      MotorLElectricalAngleBak += MotorLUnknownPara1;
    }
  }
  else
	{
		//---------电机反向转、计算角度-----------------------
		if ( (MotorLPosCur == 1) && MotorLEleAngleParaC )
		{
			if ( MotorLFlagShake )
			{
				if ( (MotorLEleAngleParaC + MotorLUnknownPara1) >= MotorLElectricalAngleBak )
					MotorLElectricalAngleBak = MotorLEleAngleParaC;
				else
					MotorLElectricalAngleBak -= MotorLUnknownPara1;
			}
			else
			{
				if ( MotorLElectricalAngleBak < (signed int)MotorLUnknownPara1 )
					MotorLFlagShake = 1;
				MotorLElectricalAngleBak -= MotorLUnknownPara1;
			}
		}
		else if ( MotorLEleAnglePara1 )
		{
			if ( MotorLElectricalAngleBak < (signed int)MotorLUnknownPara1 )
				MotorLEleAnglePara1 = 0;
			MotorLElectricalAngleBak -= MotorLUnknownPara1;
		}
		else if ( (MotorLEleAngleParaC + MotorLUnknownPara1) >= MotorLElectricalAngleBak )
		{
			MotorLElectricalAngleBak = MotorLEleAngleParaC;
		}
		else
		{
			MotorLElectricalAngleBak -= MotorLUnknownPara1;
		}
	}
	//-----------霍尔-------------------
  MotorLCurHall = (GPIOB->IDR >> 5) & 7;//霍尔状态
  if ( MotorLCurHall && (MotorLCurHall != 7) )
  {//霍尔状态正确 相位正确
    MotorLHallCnt = 0;//清零
  }
  else if ( MotorLHallCnt < 30 )
  {
    ++MotorLHallCnt;//检测霍尔状态 计数器 
    MotorLCurHall = MotorLLastHall;//霍尔状态是上一个状态
  }
	//霍尔状态切换 
  if ( MotorLCurHall != MotorLLastHall )
  {
    MotorLEleAnglePara1 = 0;
    MotorLFlagHallChange = 1;//霍尔切换标志位
		//-------判断方向 电机正转反转------------
    pos_diff = MOTOR_PHASE_SEQUENCE[MotorLCurHall] - MotorLPosCur;
    if ( (pos_diff == 1) || (pos_diff == -5) )
    {
      MotorLFlagDir = 1;//方向标志位
    }
    else if ( (pos_diff == -1) || (pos_diff == 5) )
		{
      MotorLFlagDir = 0;//方向
    }
    if ( MotorLDirFlagLast != MotorLFlagDir )
    {//电机相反方向转
      MotorLTimePole6 = 16872;	//6次霍尔切换的总时间
      MotorLFlagHallChange = 0;	//霍尔切换标志位
      MotorLFlagPole6OK = 0;			//车轮滚动一周标志位
      MotorLPoleCnt = 0;				//相序号
    }
    MotorLDirFlagLast = MotorLFlagDir;//前一次的方向
		//--------移动计数器++----------------
    if ( MotorLMoveCnt < 6 )
      ++MotorLMoveCnt;
		
		//--------霍尔检测切换的时间----------------
    MotorLTimePoleBuf[MotorLPoleCnt] = MotorLTimePole;
    MotorLTimePole = 0;//清零
		
    if ( MotorLFlagPole6OK )
    {//霍尔6次切换完成
      sum = 0;
      for ( i = 0; i < 6; i = (u8)(i + 1) )
        sum += MotorLTimePoleBuf[i];//6次霍尔切换的总时间
      MotorLTimePole6 = sum;				//6次霍尔切换的总时间
    }
    else
    {//没有完成6次切换完成 进入
      if ( MotorLFlagHallChange )
      {//霍尔切换
				//6次霍尔切换的总时间，为6*本次霍尔切换的时间
        MotorLTimePole6 = 6 * MotorLTimePoleBuf[MotorLPoleCnt];
      }
      else
      {//霍尔没有切换、故障、恢复寄存器
        MotorLTimePole6 = 16872;	//6次霍尔切换的总时间
        MotorLFlagMove = 1;				//车轮未移动
      }
      if ( MotorLPoleCnt == 5 )
        MotorLFlagPole6OK = 1;		//霍尔6次切换完成
    }
    if ( MotorLPoleCnt >= 5 )
      MotorLPoleCnt = 0;
    else
      ++MotorLPoleCnt;//霍尔切换计次
		
    if ( MotorLTimePole6 < 46 )		//6次霍尔切换的总时间 最低限制 最高速限制
      MotorLTimePole6 = 46;
		
		//-------获取当前霍尔位置-----------------
    MotorLPosCur = MOTOR_PHASE_SEQUENCE[MotorLCurHall];
    MotorLLastHall = MotorLCurHall;//前一次霍尔状态备份
    //motorL_phase_bak_172 = MotorLCurHall;
		
    if ( MotorLPosCur == 0 )
    {//霍尔错误 关闭pwm输出
      MotorLErrorPosFlag = 1;
      TIM8->BDTR &= 0x7FFFu;//关闭pwm
    }//长线一侧
    else
    {
			//----------获取电角度------------------
      if ( MotorLFlagDir )
      {//方向 正向
				switch ( MotorLPosCur )//霍尔位置
				{
					case 1u://位置1
						MotorLElectricalAngle = 0x1555;
						MotorLEleAngleParaA = 0;
						MotorLEleAngleParaE = 0xEC16;
						MotorLEleAngleParaD = 5097;
						MotorLEleAngleParaG = 10922;
						MotorLEleAngleParaF = 16019;
						if ( !MotorLFlagShake )
							MotorLEleAnglePara1 = 1;
						break;
					case 2u://位置2
						MotorLElectricalAngle = 0x3FFF;
						MotorLEleAngleParaA = 0x2AAA;
						MotorLEleAngleParaE = 5825;
						MotorLEleAngleParaD = 16019;
						MotorLEleAngleParaG = 21845;
						MotorLEleAngleParaF = 26942;
						break;
					case 3u://位置3
						MotorLElectricalAngle = 0x6AAA;
						MotorLEleAngleParaA = 0x5555;
						MotorLEleAngleParaE = 16748;
						MotorLEleAngleParaD = 26942;
						MotorLEleAngleParaG = 0x8000;
						MotorLEleAngleParaF = 0x93E9;
						break;
					case 4u://位置4
						MotorLElectricalAngle = 0x9555;
						MotorLEleAngleParaA = 0x8000;
						MotorLEleAngleParaE = 27671;
						MotorLEleAngleParaD = 0x93E9;
						MotorLEleAngleParaG = 0xAAAA;
						MotorLEleAngleParaF = 0xBE93;
						break;
					case 5u://位置5
						MotorLElectricalAngle = 0xBFFF;
						MotorLEleAngleParaA = 0xAAAA;
						MotorLEleAngleParaE = 0x96C1;
						MotorLEleAngleParaD = 0xBE93;
						MotorLEleAngleParaG = 0xD555;
						MotorLEleAngleParaF = 0xE93E;
						break;
					case 6u://位置6
						MotorLElectricalAngle = 0xEAAA;
						MotorLEleAngleParaA = 0xD555;
						MotorLEleAngleParaE = 0xC16C;
						MotorLEleAngleParaD = 0xE93E;
						MotorLEleAngleParaG = 0xFFFF;
						MotorLEleAngleParaF = 5097;
						break;
					case 0u:
						break;
				}
        MotorLEleAngleDiff = MotorLEleAngleParaA - MotorLElectricalAngleBak;
      }
      else
      {
				switch ( MotorLPosCur )
				{
					case 1u://位置1
						MotorLElectricalAngle = 0x1555;//电角度
						MotorLEleAngleParaA = 0x2AAA;
						MotorLEleAngleParaE = 5825;
						MotorLEleAngleParaD = 16019;
						MotorLEleAngleParaG = 0;
						MotorLEleAngleParaF = 0xEC16;
						break;
					case 2u://位置2
						MotorLElectricalAngle = 0x3FFF;
						MotorLEleAngleParaA = 0x5555;
						MotorLEleAngleParaE = 16748;
						MotorLEleAngleParaD = 26942;
						MotorLEleAngleParaG = 10922;
						MotorLEleAngleParaF = 5825;
						break;
					case 3u://位置3
						MotorLElectricalAngle = 0x6AAA;
						MotorLEleAngleParaA = 0x8000;
						MotorLEleAngleParaE = 27671;
						MotorLEleAngleParaD = 0x93E9;
						MotorLEleAngleParaG = 21845;
						MotorLEleAngleParaF = 16748;
						break;
					case 4u://位置4
						MotorLElectricalAngle = 0x9555;
						MotorLEleAngleParaA = 0xAAAA;
						MotorLEleAngleParaE = 0x96C1;
						MotorLEleAngleParaD = 0xBE93;
						MotorLEleAngleParaG = 0x8000;
						MotorLEleAngleParaF = 27671;
						break;
					case 5u://位置5
						MotorLElectricalAngle = 0xBFFF;
						MotorLEleAngleParaA = 0xD555;
						MotorLEleAngleParaE = 0xC16C;
						MotorLEleAngleParaD = 0xE93E;
						MotorLEleAngleParaG = 0xAAAA;
						MotorLEleAngleParaF = 0x96C1;
						break;
					case 6u://位置6
						MotorLElectricalAngle = 0xEAAA;
						MotorLEleAngleParaA = 0xFFFF;
						MotorLEleAngleParaE = 0xEC16;
						MotorLEleAngleParaD = 5097;
						MotorLEleAngleParaG = 0xD555;
						MotorLEleAngleParaF = 0xC16C;
						if ( !MotorLFlagShake )
							MotorLEleAnglePara1 = 1;
						break;
					case 0u:
						break;
				}
        MotorLEleAngleDiff = MotorLElectricalAngleBak - MotorLEleAngleParaA;
      }
      if ( (MotorLTimePole6 > 1013) || (MotorLFlagPole6OK==0) )
      {//6次霍尔时间、没有完成6次切换
        MotorLElectricalAngleBak = MotorLEleAngleParaA;
        MotorLEleAngleParaC = MotorLEleAngleParaG;
        MotorLEleAngleDiff = 0;
        MotorLEleAnglePara1 = 0;
      }
      else
      {
        MotorLEleAngleParaC = MotorLEleAngleParaF;
        if ( MotorLFlagDir )
        {//正向
          if ( MotorLPosCur == 1 )
          {//霍尔位置1
            if ( MotorLFlagShake )
            {
              if ( MotorLElectricalAngleBak > (signed int)MotorLEleAngleParaD )
              {
                MotorLElectricalAngleBak = MotorLEleAngleParaA;
                MotorLEleAngleDiff = 0;
                MotorLEleAnglePara1 = 0;
              }
            }
            else if ( MotorLElectricalAngleBak < (signed int)MotorLEleAngleParaE )
            {
              MotorLElectricalAngleBak = MotorLEleAngleParaA;
              MotorLEleAngleDiff = 0;
              MotorLEleAnglePara1 = 0;
            }
          }
          else
          {
            if ( (MotorLPosCur == 6) && ((signed int)MotorLTimePole6 > 1013) )
              MotorLEleAngleParaC = 0xffff;
            if ( (MotorLElectricalAngleBak > (signed int)MotorLEleAngleParaD) || (MotorLElectricalAngleBak < (signed int)MotorLEleAngleParaE) )
            {
              MotorLElectricalAngleBak = MotorLEleAngleParaA;
              MotorLEleAngleDiff = 0;
              MotorLEleAnglePara1 = 0;
            }
          }
        }
        else
				{//反向
					if ( MotorLPosCur == 6 )
					{//霍尔位置6
						if ( MotorLFlagShake )
						{
							if ( MotorLElectricalAngleBak < (signed int)MotorLEleAngleParaE )
							{
								MotorLElectricalAngleBak = MotorLEleAngleParaA;
								MotorLEleAngleDiff = 0;
								MotorLEleAnglePara1 = 0;
							}
						}
						else if ( MotorLElectricalAngleBak > (signed int)MotorLEleAngleParaD )
						{
							MotorLElectricalAngleBak = MotorLEleAngleParaA;
							MotorLEleAngleDiff = 0;
							MotorLEleAnglePara1 = 0;
						}
					}
					else
					{
						if ( (MotorLPosCur == 1) && ((signed int)MotorLTimePole6 > 1013) )
							MotorLEleAngleParaC = 0;
						if ( (MotorLElectricalAngleBak > (signed int)MotorLEleAngleParaD) || (MotorLElectricalAngleBak < (signed int)MotorLEleAngleParaE) )
						{
							MotorLElectricalAngleBak = MotorLEleAngleParaA;
							MotorLEleAngleDiff = 0;
							MotorLEleAnglePara1 = 0;
						}
					}
				}
      }
    }
    if ( MotorLTimePole6 <= 1013 )//霍尔6次时间
      result = (MotorLTimePole6 - MotorLRollTimeLast) >> 3;
    else
      result = 0;
    MotorLUnknownPara1 = (MotorLEleAngleDiff + 0xFFFF + ((MotorLTimePole6 + result) >> 1))
																/ (MotorLTimePole6 + result);
    MotorLFlagShake = 0;
    MotorLRollTimeLast = MotorLTimePole6;//6次霍尔切换的总时间 备份
  }
	//-------最终电角度-----------------
  if ( MotorLFlagMove )//未移动
    MotorLElectricalAngleBak = MotorLElectricalAngle;//
  MotorLFinalEleAngle = MotorLElectricalAngleBak;
  if ( MotorLFlagMove )
  {//车轮未移动
    MotorLFinalEleAngle += 5916;//角度增量
  }
  else if ( MotorLFlagDir )
  {//正向
    MotorLFinalEleAngle += 6735;//
  }
  else
  {//反向
    MotorLFinalEleAngle += 5097;
  }
}

//-------------------------------------------------------------
void MotorLInitParaFun()
{
  MotorLFlowIntergral = 0;
  MotorLCurrqd = 0;
  MotorLAtatVolt_qd.qV_Component2 = 0;
}

//-------------------------------------------------------------
void MotorLFlowRegFun()
{//负载流量调节 输出参考电压vds 
	s16 q_comp1_val; //
  s16 diff; //
  s32 rst; // 

  q_comp1_val = -MotorLCurrqd;
  if ( q_comp1_val > 0 )
    diff = 1;
  else if ( q_comp1_val < 0 )
		diff = -1;
	else
		diff = 0;
	
  MotorLFlowIntergral += diff;
  rst = MotorLFlowIntergral + (q_comp1_val >> 2);
  if ( rst < -0x4000 )
  {
    rst = -16384;
    MotorLFlowIntergral = 0xC000;
  }
  else if ( rst > 0x4000 )
  {
    rst = 0x4000;
    MotorLFlowIntergral = 0x4000;
  }
  MotorLAtatVolt_qd.qV_Component2 = rst >> 6;// 除以/64
}

//-------------------------------------------------------------
void MotorLClarkeFun()
{//坐标变换 
  MotorLCurrAlfaBeta.qI_Component1 = MotorLCurr_ab.qI_Component1;
	MotorLCurrAlfaBeta.qI_Component2 = 18918 * (MotorLCurr_ab.qI_Component1 
										+ 2 * MotorLCurr_ab.qI_Component2) >> 15;
}

//-------------------------------------------------------------
void MotorLParkFun()
{
	s32 result; //

  result = (MotorLCurrAlfaBeta.qI_Component1 * MotorLVectorComponents.hCos
						+ MotorLCurrAlfaBeta.qI_Component2 * MotorLVectorComponents.hSin)
						>> 15;
  MotorLCurrqd = (20317 * MotorLCurrqd 
															+ 12451 * result) 
															>> 15;
}

//-------------------------------------------------------------
void MotorLRevParkFun()
{//反park变换 	
  MotorLVectorComponents.hSin = get_sin(MotorLFinalEleAngle);
  MotorLVectorComponents.hCos = get_cos(MotorLFinalEleAngle);
  MotorLVol_ab.qV_Component2 = 
		(MotorLAtatVolt_qd.qV_Component2 * MotorLVectorComponents.hCos 
		- MotorLAtatVolt_qd.qV_Component1 * MotorLVectorComponents.hSin) >> 15;
  MotorLVol_ab.qV_Component1 = 
		(MotorLAtatVolt_qd.qV_Component1 * MotorLVectorComponents.hCos 
		+ MotorLAtatVolt_qd.qV_Component2 * MotorLVectorComponents.hSin) >> 15;
}

//-------------------------------------------------------------
//SVPWM
void MotorLMotorCtrlFun()
{// 控制pwm wZ wY wX有可能不对（错位） 其他的是对的
  s16 hTimePhA; 
	s16 hTimePhB;
	s16 hTimePhC; 
	
  u16 bSector; 
	
  s16 wX; 
  s16 wZ; 
  s16 wY;

	//Clarke逆变换
	wX = MotorLVol_ab.qV_Component1;
  wZ = (s16)-(MotorLVol_ab.qV_Component1 + 
										((3547 * MotorLVol_ab.qV_Component2) >> 11));
  wY = (s16)(((3547 * MotorLVol_ab.qV_Component2) >> 11) 
										- MotorLVol_ab.qV_Component1);
	
	
	bSector = 0;
  if ( wX > 0 )
    bSector = 1;
  if ( wY > 0 )
    bSector = (u16)(bSector + 2);
  if ( wZ > 0 )
    bSector = (u16)(bSector + 4);
	
	wX = (255 *       wX       ) >> 10;
  wZ = (255 * ( ( -wZ ) >> 1)) >> 10;
  wY = (255 * ( ( -wY ) >> 1)) >> 10;
  
	switch ( bSector )
	{
		case 0:
			hTimePhA = 255;
			hTimePhB = 255;
			hTimePhC = 255;
			break;
		case 1:
			hTimePhA = (s16)(255 - wY + wZ);
			hTimePhB = (s16)(wY + 255 + wZ);
			hTimePhC = (s16)(255 - wY - wZ);
			break;
		case 2:
			hTimePhA = (s16)(255 - wX + wZ);
			hTimePhB = (s16)(255 - wZ + wX);
			hTimePhC = (s16)(255 - wZ - wX);
			break;
		case 3:
			hTimePhA = (s16)(255 - wY + wX);
			hTimePhB = (s16)(wY + 255 + wX);
			hTimePhC = (s16)(255 - wX + wY);
			break;
		case 4:
			hTimePhA = (s16)(255 - wY + wX);
			hTimePhB = (s16)(wY + 255 + wX);
			hTimePhC = (s16)(255 - wX + wY);
			break;
		case 5:
			hTimePhA = (s16)(255 - wX + wZ);
			hTimePhB = (s16)(255 - wZ + wX);
			hTimePhC = (s16)(255 - wZ - wX);
			break;
		case 6:
			hTimePhA = (s16)(255 - wY + wZ);
			hTimePhB = (s16)(wY + 255 + wZ);
			hTimePhC = (s16)(255 - wY - wZ);
			break;
		default:
			break;
	}
  if ( hTimePhA < 0 )
    (hTimePhA) = 0;
  if ( hTimePhB < 0 )
    (hTimePhB) = 0;
  if ( hTimePhC < 0 )
    (hTimePhC) = 0;
  TIM8->CCR1 = hTimePhA;
  TIM8->CCR2 = hTimePhB;
  TIM8->CCR3 = hTimePhC;
}

