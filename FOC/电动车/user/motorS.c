/***********************************************************
*	控制短线电机
*
*
*
************************************************************/

#include "extern.h"

u8 MOTOR_PHASE_SEQUENCE[8]={0,2,4,3,6,1,5};//120度无刷电机 相位顺序表
//电机S
Volt_Components MotorSAtatVolt_qd;//
s16 MotorSCurrqd;//负载调节park变换
s16 MotorSFlowIntergral;
Curr_Components MotorSCurr_ab;// 当前相电流结构
Curr_Components  MotorSCurrAlfaBeta; // 
Volt_Components MotorSVol_ab;//
Trig_Components  MotorSVectorComponents; // 

/////////////////////////
u8 MotorSErrorPosFlag; //霍尔位置故障
u8 MotorSDirFlagLast;  //上一次霍尔改变的方向
u8 MotorSCurHall;  //当前霍尔
u8 MotorSLastHall; //前次霍尔
u8 MotorSHallCnt;  //霍尔计数器

u8 MotorSFlagHallChange;//霍尔切换标志位
u8 MotorSFlagPole6OK;   //霍尔6次完成ok
u16 MotorSTimePole=0xafc; //霍尔切换一次
u16 MotorSPosCur;					//霍尔当前位置
u16 MotorSElectricalAngleBak; 
u16 MotorSUnknownPara1;//不知道的参数
s16 MotorSEleAngleDiff;
u16 MotorSRollTimeLast;
u8 MotorSFlagShake;//可能是方向
u8 MotorSEleAnglePara1;
u8 MotorSPoleCnt; //相序号
u8 MotorSMoveCnt; //移动计数
s16 MotorSFlagMove;// 电机是否移动标志位
u16 MotorSTimePole6; //6次霍尔切换的总时间
u16 MotorSFlagDir;//电机方向标志

u16 MotorSEleAngleParaA;
u16 MotorSEleAngleParaC;
u16 MotorSElectricalAngle;
u16 MotorSEleAngleParaD;
u16 MotorSEleAngleParaE;
u16 MotorSEleAngleParaF;
u16 MotorSEleAngleParaG;

u16 MotorSTimePoleBuf[6];//角度数组

u16 MotorSFinalEleAngle;

void MotorSFetAngleFun(void);
void MotorSInitParaFun(void);
void MotorSFlowRegFun(void);
void MotorSClarkeFun(void);
void MotorSParkFun(void);
void MotorSRevParkFun(void);
void MotorSMotorCtrlFun(void);

//-------------------------------------------------------------
void MotorSFetAngleFun()
{//根据当前相位 获取需要调整的角度
	s16 		result;
	u32 		sum; // 
  signed int i; // 
	s8 			pos_diff;
	
  if ( MotorSTimePole < 2812 )
    ++MotorSTimePole;
  if ( MotorSTimePole >= 2812 )
  {
    MotorSTimePole6 			= 16872;
    MotorSFlagHallChange 	= 0;
    MotorSFlagPole6OK 		= 0;
    MotorSPoleCnt 				= 0;
  }
  if ( ((signed int)MotorSTimePole6 > 1013) || (5 * MotorSTimePole > 1012) )
    MotorSEleAngleParaC = MotorSEleAngleParaG;
  if ( ((signed int)MotorSTimePole6 <= 1350) && (6 * MotorSTimePole <= 1350) )
  {
    if ( (signed int)MotorSMoveCnt >= 6 )
    {
      MotorSFlagMove = 0;
//      ram_0C6 = 0;
    }
  }
  else
  {
    MotorSFlagMove = 1;
    MotorSMoveCnt = 0;
  }
  if ( MotorSFlagDir )
  {
    if ( (MotorSPosCur == 6) && (MotorSEleAngleParaC != 0xffff) )
    {
      if ( MotorSFlagShake )
      {
        if ( (MotorSEleAngleParaC - MotorSUnknownPara1) <= MotorSElectricalAngleBak )
          MotorSElectricalAngleBak = MotorSEleAngleParaC;
        else
          MotorSElectricalAngleBak += MotorSUnknownPara1;
      }
      else
      {
        if ( (0xFFFF - MotorSUnknownPara1) < MotorSElectricalAngleBak )
          MotorSFlagShake = 1;
        MotorSElectricalAngleBak += MotorSUnknownPara1;
      }
    }
    else if ( MotorSEleAnglePara1 )
    {
      if ( (0xFFFF - MotorSUnknownPara1) < MotorSElectricalAngleBak )
        MotorSEleAnglePara1 = 0;
      MotorSElectricalAngleBak += MotorSUnknownPara1;
    }
    else if ( (MotorSEleAngleParaC - MotorSUnknownPara1) <= MotorSElectricalAngleBak )
    {
      MotorSElectricalAngleBak = MotorSEleAngleParaC;
    }
    else
    {
      MotorSElectricalAngleBak += MotorSUnknownPara1;
    }
  }
  else if ( (MotorSPosCur == 1) && MotorSEleAngleParaC )
  {
    if ( MotorSFlagShake )
    {
      if ( (MotorSEleAngleParaC + MotorSUnknownPara1) >= MotorSElectricalAngleBak )
        MotorSElectricalAngleBak = MotorSEleAngleParaC;
      else
        MotorSElectricalAngleBak -= MotorSUnknownPara1;
    }
    else
    {
      if ( MotorSElectricalAngleBak < MotorSUnknownPara1 )
        MotorSFlagShake = 1;
      MotorSElectricalAngleBak -= MotorSUnknownPara1;
    }
  }
  else if ( MotorSEleAnglePara1 )
  {
    if ( MotorSElectricalAngleBak < MotorSUnknownPara1 )
      MotorSEleAnglePara1 = 0;
    MotorSElectricalAngleBak -= MotorSUnknownPara1;
  }
  else if ( (MotorSEleAngleParaC + MotorSUnknownPara1) >= MotorSElectricalAngleBak )
  {
    MotorSElectricalAngleBak = MotorSEleAngleParaC;
  }
  else
  {
    MotorSElectricalAngleBak -= MotorSUnknownPara1;
  }
  MotorSCurHall = (GPIOC->IDR >> 10) & 7;
  if ( MotorSCurHall && (MotorSCurHall != 7) )
  {
    MotorSHallCnt = 0;
  }
  else if ( MotorSHallCnt < 30 )
  {
    ++MotorSHallCnt;
    MotorSCurHall = MotorSLastHall;
  }
	//读取sa sb sc状态 电机霍尔状态
  if ( MotorSCurHall != MotorSLastHall )//如果hall改变
  {
    MotorSEleAnglePara1 = 0;
    MotorSFlagHallChange = 1;
    pos_diff = MOTOR_PHASE_SEQUENCE[MotorSCurHall] - MotorSPosCur;//角度差
    if ( (pos_diff != 1) && (pos_diff != -5) )
    {
      if ( (pos_diff == -1) || (pos_diff == 5) )
        MotorSFlagDir = 0;//电机方向标志 正反转
    }
    else
    {
      MotorSFlagDir = 1;//电机方向标志 正反转
    }
    if ( MotorSDirFlagLast != MotorSFlagDir )
    {
      MotorSTimePole6 = 16872;
      MotorSFlagHallChange = 0;
      MotorSFlagPole6OK = 0;
      MotorSPoleCnt = 0;
    }
    MotorSDirFlagLast = MotorSFlagDir;
    if ( MotorSMoveCnt < 6 )
      ++MotorSMoveCnt;
    MotorSTimePoleBuf[MotorSPoleCnt] = MotorSTimePole;
    MotorSTimePole = 0;
    if ( MotorSFlagPole6OK )
    {
      sum = 0;
      for ( i = 0; i < 6; i = (u8)(i + 1) )
        sum += MotorSTimePoleBuf[i];
      MotorSTimePole6 = sum;
    }
    else
    {
      if ( MotorSFlagHallChange )
      {
        MotorSTimePole6 = 6 * MotorSTimePoleBuf[MotorSPoleCnt];
      }
      else
      {
        MotorSTimePole6 = 16872;
        MotorSFlagMove = 1;
      }
      if ( MotorSPoleCnt == 5 )
        MotorSFlagPole6OK = 1;
    }
    if ( MotorSPoleCnt >= 5 )
      MotorSPoleCnt = 0;
    else
      ++MotorSPoleCnt;
    if ( MotorSTimePole6 < 46 )
      MotorSTimePole6 = 46;
    MotorSPosCur = MOTOR_PHASE_SEQUENCE[MotorSCurHall];
    MotorSLastHall = MotorSCurHall;
    //motorS_phase_last_bak_100 = MotorSCurHall;
    if ( MotorSPosCur ==0 )
    {
      MotorSErrorPosFlag = 1;//电机位置错误标志
      TIM1->BDTR &= 0x7FFFu;//关闭pwm
    }
    else
    {
      if ( MotorSFlagDir )
      {
        switch ( MotorSPosCur )
        {
          case 1u:
            MotorSElectricalAngle = 5461;
            MotorSEleAngleParaA = 0;
            MotorSEleAngleParaE = 0xEC16;
            MotorSEleAngleParaD = 5097;
            MotorSEleAngleParaG = 10922;
            MotorSEleAngleParaF = 16019;
            if ( !MotorSFlagShake )
              MotorSEleAnglePara1 = 1;
            break;
          case 2u:
            MotorSElectricalAngle = 0x3FFF;
            MotorSEleAngleParaA = 10922;
            MotorSEleAngleParaE = 5825;
            MotorSEleAngleParaD = 16019;
            MotorSEleAngleParaG = 21845;
            MotorSEleAngleParaF = 26942;
            break;
          case 3u:
            MotorSElectricalAngle = 27306;
            MotorSEleAngleParaA = 21845;
            MotorSEleAngleParaE = 16748;
            MotorSEleAngleParaD = 26942;
            MotorSEleAngleParaG = 0x8000;
            MotorSEleAngleParaF = 0x93E9;
            break;
          case 4u:
            MotorSElectricalAngle = 0x9555;
            MotorSEleAngleParaA = 0x8000;
            MotorSEleAngleParaE = 27671;
            MotorSEleAngleParaD = 0x93E9;
            MotorSEleAngleParaG = 0xAAAA;
            MotorSEleAngleParaF = 0xBE93;
            break;
          case 5u:
            MotorSElectricalAngle = 0xBFFF;
            MotorSEleAngleParaA = 0xAAAA;
            MotorSEleAngleParaE = 0x96C1;
            MotorSEleAngleParaD = 0xBE93;
            MotorSEleAngleParaG = 0xD555;
            MotorSEleAngleParaF = 0xE93E;
            break;
          case 6u:
            MotorSElectricalAngle = 0xEAAA;
            MotorSEleAngleParaA = 0xD555;
            MotorSEleAngleParaE = 0xC16C;
            MotorSEleAngleParaD = 0xE93E;
            MotorSEleAngleParaG = 0xFFFF;
            MotorSEleAngleParaF = 5097;
            break;
          case 0u:
            break;
        }
        MotorSEleAngleDiff = MotorSEleAngleParaA - MotorSElectricalAngleBak;
      }
      else
      {
        switch ( MotorSPosCur )
        {
          case 1u:
            MotorSElectricalAngle = 5461;
            MotorSEleAngleParaA = 10922;
            MotorSEleAngleParaE = 5825;
            MotorSEleAngleParaD = 16019;
            MotorSEleAngleParaG = 0;
            MotorSEleAngleParaF = 0xEC16;
            break;
          case 2u:
            MotorSElectricalAngle = 0x3FFF;
            MotorSEleAngleParaA = 21845;
            MotorSEleAngleParaE = 16748;
            MotorSEleAngleParaD = 26942;
            MotorSEleAngleParaG = 10922;
            MotorSEleAngleParaF = 5825;
            break;
          case 3u:
            MotorSElectricalAngle = 27306;
            MotorSEleAngleParaA = 0x8000;
            MotorSEleAngleParaE = 27671;
            MotorSEleAngleParaD = 0x93E9;
            MotorSEleAngleParaG = 21845;
            MotorSEleAngleParaF = 16748;
            break;
          case 4u:
            MotorSElectricalAngle = 0x9555;
            MotorSEleAngleParaA = 0xAAAA;
            MotorSEleAngleParaE = 0x96C1;
            MotorSEleAngleParaD = 0xBE93;
            MotorSEleAngleParaG = 0x8000;
            MotorSEleAngleParaF = 27671;
            break;
          case 5u:
            MotorSElectricalAngle = 0xBFFF;
            MotorSEleAngleParaA = 0xD555;
            MotorSEleAngleParaE = 0xC16C;
            MotorSEleAngleParaD = 0xE93E;
            MotorSEleAngleParaG = 0xAAAA;
            MotorSEleAngleParaF = 0x96C1;
            break;
          case 6u:
            MotorSElectricalAngle = 0xEAAA;
            MotorSEleAngleParaA = 0xFFFF;
            MotorSEleAngleParaE = 0xEC16;
            MotorSEleAngleParaD = 5097;
            MotorSEleAngleParaG = 0xD555;
            MotorSEleAngleParaF = 0xC16C;
            if ( !MotorSFlagShake )
              MotorSEleAnglePara1 = 1;
            break;
          case 0u:
            break;
        }
        MotorSEleAngleDiff = MotorSElectricalAngleBak - MotorSEleAngleParaA;
      }
      if ( (MotorSTimePole6 > 1013) || (MotorSFlagPole6OK==0) )
      {
        MotorSElectricalAngleBak = MotorSEleAngleParaA;
        MotorSEleAngleParaC = MotorSEleAngleParaG;
        MotorSEleAngleDiff = 0;
        MotorSEleAnglePara1 = 0;
      }
      else
      {
        MotorSEleAngleParaC = MotorSEleAngleParaF;
        if ( MotorSFlagDir )
        {
          if ( MotorSPosCur == 1 )
          {
            if ( MotorSFlagShake )
            {
              if ( MotorSElectricalAngleBak > MotorSEleAngleParaD )
              {
                MotorSElectricalAngleBak = MotorSEleAngleParaA;
                MotorSEleAngleDiff = 0;
                MotorSEleAnglePara1 = 0;
              }
            }
            else if ( MotorSElectricalAngleBak < MotorSEleAngleParaE )
            {
              MotorSElectricalAngleBak = MotorSEleAngleParaA;
              MotorSEleAngleDiff = 0;
              MotorSEleAnglePara1 = 0;
            }
          }
          else
          {
            if ( (MotorSPosCur == 6) && (MotorSTimePole6 > 1013) )
              MotorSEleAngleParaC = 0xffff;
            if ( (MotorSElectricalAngleBak > MotorSEleAngleParaD) || (MotorSElectricalAngleBak < (signed int)MotorSEleAngleParaE) )
            {
              MotorSElectricalAngleBak = MotorSEleAngleParaA;
              MotorSEleAngleDiff = 0;
              MotorSEleAnglePara1 = 0;
            }
          }
        }
        else if ( MotorSPosCur == 6 )
        {
          if ( MotorSFlagShake )
          {
            if ( MotorSElectricalAngleBak < MotorSEleAngleParaE )
            {
              MotorSElectricalAngleBak = MotorSEleAngleParaA;
              MotorSEleAngleDiff = 0;
              MotorSEleAnglePara1 = 0;
            }
          }
          else if ( MotorSElectricalAngleBak > MotorSEleAngleParaD )
          {
            MotorSElectricalAngleBak = MotorSEleAngleParaA;
            MotorSEleAngleDiff = 0;
            MotorSEleAnglePara1 = 0;
          }
        }
        else
        {
          if ( (MotorSPosCur == 1) && (MotorSTimePole6 > 1013) )
            MotorSEleAngleParaC = 0;
          if ( (MotorSElectricalAngleBak > MotorSEleAngleParaD) || (MotorSElectricalAngleBak < (signed int)MotorSEleAngleParaE) )
          {
            MotorSElectricalAngleBak = MotorSEleAngleParaA;
            MotorSEleAngleDiff = 0;
            MotorSEleAnglePara1 = 0;
          }
        }
      }
    }
    if ( MotorSTimePole6 <= 1013 )
      result = (s16)(MotorSTimePole6 - MotorSRollTimeLast);
    else
      result = 0;
    MotorSUnknownPara1 = (MotorSEleAngleDiff + 0xFFFF + ((MotorSTimePole6 + result) >> 1)) / ((unsigned int)MotorSTimePole6 + result);
    MotorSFlagShake = 0;
    MotorSRollTimeLast = MotorSTimePole6;
  }
  if ( MotorSFlagMove )
    MotorSElectricalAngleBak = MotorSElectricalAngle;
  MotorSFinalEleAngle = MotorSElectricalAngleBak;//角度
  if ( MotorSFlagMove )
  {
    MotorSFinalEleAngle += 5916;//这几个值，有变化
  }
  else if ( MotorSFlagDir )
  {
    MotorSFinalEleAngle += 6735;
  }
  else
  {
    MotorSFinalEleAngle += 5097;
  }
}

//-------------------------------------------------------------
void MotorSInitParaFun()
{//参数初始化
  MotorSFlowIntergral = 0;//负载累积参数
  MotorSCurrqd = 0;
  MotorSAtatVolt_qd.qV_Component2 = 0;
}

//-------------------------------------------------------------
void MotorSFlowRegFun()
{//负载电流调节 输出参考电压vds
	s16 curq_d; //
  s16 diff; //
  s16 val; //

  curq_d = -MotorSCurrqd;
  if ( curq_d > 0 )
  {
    diff = 1;
  }
  else if ( curq_d < 0 )
	{
		diff = -1;
  }
	else
	{
		diff = 0;
  }
	//
  MotorSFlowIntergral += diff;//负载正负积分
  val = MotorSFlowIntergral + (curq_d >> 2);
  if ( val < -0x4000 )
  {
    val = -16384;
    MotorSFlowIntergral = -16384;
  }
  else if ( val > 0x4000 )
  {
    val = 0x4000;
    MotorSFlowIntergral = 0x4000;
  }
  MotorSAtatVolt_qd.qV_Component2 = val >> 6;
}

//-------------------------------------------------------------
void MotorSClarkeFun()
{//坐标变换 	Clarke坐标变换
	//改大了 一卡一卡的 对速度没用太大影响
  MotorSCurrAlfaBeta.qI_Component1 = MotorSCurr_ab.qI_Component1;
	
	//根号3 = 1.732     2^15/18918 = 1.732
  MotorSCurrAlfaBeta.qI_Component2 = 18918 * (MotorSCurr_ab.qI_Component1 
									+ 2 * MotorSCurr_ab.qI_Component2) >> 15;
}

//-------------------------------------------------------------
void MotorSParkFun()
{//Park变换
	s32 result; //

  result = (MotorSCurrAlfaBeta.qI_Component1 * MotorSVectorComponents.hCos
					+ MotorSCurrAlfaBeta.qI_Component2 * MotorSVectorComponents.hSin) >> 15;
  MotorSCurrqd = (20317 * MotorSCurrqd
														+ 12451 * result) >> 15;   //Id = 0 转化得到的
	
}

//-------------------------------------------------------------
void MotorSRevParkFun()
{// 反park变换		
  MotorSVectorComponents.hSin = get_sin(MotorSFinalEleAngle);
  MotorSVectorComponents.hCos = get_cos(MotorSFinalEleAngle);
  MotorSVol_ab.qV_Component2 = 
		(MotorSAtatVolt_qd.qV_Component2 * MotorSVectorComponents.hCos 
		- MotorSAtatVolt_qd.qV_Component1 * MotorSVectorComponents.hSin) >> 15;
  MotorSVol_ab.qV_Component1 = 
		(MotorSAtatVolt_qd.qV_Component1 * MotorSVectorComponents.hCos 
		+ MotorSAtatVolt_qd.qV_Component2 * MotorSVectorComponents.hSin) >> 15;
}


//SVPWM-------------------------------------------------------------
void MotorSMotorCtrlFun(void)
{// 控制pwm wZ wY wX有可能不对（错位） 其他的是对的
  s16 hTimePhA; // 
	s16 hTimePhB;
	s16 hTimePhC; //
	
  u16 bSector; // 扇区
  s16 wX; // 
  s16 wZ; // 
  s16 wY; // 
	
  //Clarke逆变换
	wX = MotorSVol_ab.qV_Component1;
  wZ = (s16)-(MotorSVol_ab.qV_Component1 + 
									((3547 * MotorSVol_ab.qV_Component2) >> 11));
	wY = (s16)(((3547 * MotorSVol_ab.qV_Component2) >> 11) 
										- MotorSVol_ab.qV_Component1);
	
  bSector = 0;
  if ( wX > 0 )
    bSector = 1;
  if ( wY > 0 )
    bSector = (u16)(bSector + 2);
  if ( wZ > 0 )
    bSector = (u16)(bSector + 4);
	
  wX = (255 *      wX     ) >> 10;
  wZ = (255 * ((-wZ) >> 1)) >> 10;
  wY = (255 * ((-wY) >> 1)) >> 10;
  
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
  TIM1->CCR1 = hTimePhA;
  TIM1->CCR2 = hTimePhB;
  TIM1->CCR3 = hTimePhC;
}


