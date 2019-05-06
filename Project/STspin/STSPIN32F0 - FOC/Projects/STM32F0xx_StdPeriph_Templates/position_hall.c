//=============================================================================
#include 	"main.h"

//=============================================================================
int16_t 	MotorFlagMove;
uint16_t 	MotorFinalEleAngle;

//-----------------------------------------------------------------------------
//u8 MOTOR_PHASE_SEQUENCE[7] = {0,2,4,3,6,1,5}; //120度无刷电机 相位顺序表

u8 MOTOR_PHASE_SEQUENCE[7] = {0,3,5,4,1,2,6}; //阳工的轮子

//u8 MOTOR_PHASE_SEQUENCE[7] = {0,1,3,2,5,6,4};

//u8 MOTOR_PHASE_SEQUENCE[7] = {5,7,6,4,0,1,3}; //60 相位顺序表

u16 MotorTimePoleBuf[6]; //角度数组

u16 MotorTimePole = 2812;
u16 MotorTimePole6;    //6次霍尔切换的总时间
u8 MotorFlagPole6OK;   //霍尔6次完成ok
u8 MotoSFlagHallChange;//霍尔切换标志位
u8 MotorPoleCnt;       //相序号
 
u16 MotorElectricalAngle;		//电角度总值
u16 MotorEleAngleParaA;			//电角度参数
u16 MotorEleAngleParaC;
u16 MotorEleAngleParaD;
u16 MotorEleAngleParaE;
u16 MotorEleAngleParaF;
u16 MotorEleAngleParaG;

u8 MotorMoveCnt;  
u16 MotorFlagDir; 
u16 MotorPosCur; 			//位置
u8 MotorFlagShake;
u16 MotorUnknownPara1;
u16 MotorElectricalAngleBak;
u8 MotorEleAnglePara1;		//电角度参数1
u8 MotorCurHall;   			//6种hall状态
u8 MotorHallCnt;  
u8 MotorLastHall; 			//最后一次hall
u8 MotorDirFlagLast;  
u8 MotorErrorPosFlag; 
s16 MotorEleAngleDiff;
u16 MotorRollTimeLast;

u8 MotorFlagHallChange;			//hall变换标志位

//=============================================================================
void MotorFetAngleFun (void)
{
	s16 	result;
	u32 	sum;  
	signed int i;  
	s8 		pos_diff;
	
	if ( MotorTimePole < 2812 )			//是否到达单次换向
		++MotorTimePole;
	if ( MotorTimePole >= 2812 )		//第一次就运行到这里 	16872/2812=6
	{
		MotorTimePole6 			= 16872;				
		MotorFlagHallChange 	= 0;
		MotorFlagPole6OK 		= 0;
		MotorPoleCnt 			= 0;
	}
	
	if ( ((signed int)MotorTimePole6 > 1013) || (5 * MotorTimePole > 1012) )		
		MotorEleAngleParaC = MotorEleAngleParaG;
	
	if ( ((signed int)MotorTimePole6 <= 1350) && (6 * MotorTimePole <= 1350) )
	{
		if ( (signed int)MotorMoveCnt >= 6 )
		{
			MotorFlagMove = 0;
		}
	}
	else
	{
		MotorFlagMove = 1;				//1表示要移动？？
		MotorMoveCnt  = 0;
	}
	if ( MotorFlagDir )
	{
		if ( (MotorPosCur == 6) && (MotorEleAngleParaC != 0xffff) )
		{
			if ( MotorFlagShake )
			{
				if ( (MotorEleAngleParaC - MotorUnknownPara1) <= MotorElectricalAngleBak )
					MotorElectricalAngleBak = MotorEleAngleParaC;
				else
					MotorElectricalAngleBak += MotorUnknownPara1;
			}
			else
			{
				if ( (0xFFFF - MotorUnknownPara1) < MotorElectricalAngleBak )
					MotorFlagShake = 1;
				
				MotorElectricalAngleBak += MotorUnknownPara1;
			}
		}
		else if ( MotorEleAnglePara1 )         //判断SVPWM
		{
			if ( (0xFFFF - MotorUnknownPara1) < MotorElectricalAngleBak )
				MotorEleAnglePara1 = 0;
			
			MotorElectricalAngleBak += MotorUnknownPara1;
		}
		else if ( (MotorEleAngleParaC - MotorUnknownPara1) <= MotorElectricalAngleBak )
		{
			MotorElectricalAngleBak = MotorEleAngleParaC;
		}
		else
		{
			MotorElectricalAngleBak += MotorUnknownPara1;
		}
	}
	else if ( (MotorPosCur == 1) && MotorEleAngleParaC )
	{
		if ( MotorFlagShake )
		{
			if ( (MotorEleAngleParaC + MotorUnknownPara1) >= MotorElectricalAngleBak )
				MotorElectricalAngleBak = MotorEleAngleParaC;
			else
				MotorElectricalAngleBak -= MotorUnknownPara1;
		}
		else
		{
			if ( MotorElectricalAngleBak < MotorUnknownPara1 )
				MotorFlagShake = 1;
			
			MotorElectricalAngleBak -= MotorUnknownPara1;
		}
	}
	else if ( MotorEleAnglePara1 )
	{
		if ( MotorElectricalAngleBak < MotorUnknownPara1 )
			MotorEleAnglePara1 = 0;
		
		MotorElectricalAngleBak -= MotorUnknownPara1;
	}
	else if ( (MotorEleAngleParaC + MotorUnknownPara1) >= MotorElectricalAngleBak )
	{
		MotorElectricalAngleBak = MotorEleAngleParaC;
	}
	else
	{
		MotorElectricalAngleBak -= MotorUnknownPara1;
	}
	
//-------------------------猜测从这里开始------------------	
	
	
	MotorCurHall = GPIOA->IDR  & 7;//   GPIO port input data register,           A0,A1,A2读取值
	
	if ( MotorCurHall && (MotorCurHall != 7) )      //排除0和第七种状态
	{
		MotorHallCnt = 0;
	}
	else if ( MotorHallCnt < 30 )
	{
		++MotorHallCnt;
		MotorCurHall = MotorLastHall;
	}
//-----------------------------------------------
	if ( MotorCurHall != MotorLastHall )
	{
		MotorEleAnglePara1 = 0;
		MotorFlagHallChange = 1;
		pos_diff = MOTOR_PHASE_SEQUENCE[MotorCurHall] - MotorPosCur;//非常重要的一句话
		if ( (pos_diff != 1) && (pos_diff != -5) )
		{
			if ( (pos_diff == -1) || (pos_diff == 5) )
				MotorFlagDir = 0;
		}
		else
		{
			MotorFlagDir = 1;
		}
		
		if ( MotorDirFlagLast != MotorFlagDir )
		{
			MotorTimePole6 = 16872;
			MotorFlagHallChange = 0;
			MotorFlagPole6OK = 0;
			MotorPoleCnt = 0;
		}
		
		MotorDirFlagLast = MotorFlagDir;             //
		
		if ( MotorMoveCnt < 6 )
			++MotorMoveCnt;
		
		MotorTimePoleBuf[MotorPoleCnt] = MotorTimePole;
		MotorTimePole = 0;
		
		if ( MotorFlagPole6OK )
		{
			sum = 0;
			for ( i = 0; i < 6; i = (u8)(i + 1) )
				sum += MotorTimePoleBuf[i];
			MotorTimePole6 = sum;
		}
		else
		{
			if ( MotorFlagHallChange )
			{
				MotorTimePole6 = 6 * MotorTimePoleBuf[MotorPoleCnt];
			}
			else
			{
				MotorTimePole6 = 16872;
				MotorFlagMove = 1;			//表示有偏转
			}
			if ( MotorPoleCnt == 5 )
				MotorFlagPole6OK = 1;
		}
		
		if ( MotorPoleCnt >= 5 )
			MotorPoleCnt = 0;
		else
			++MotorPoleCnt;
		
		if ( MotorTimePole6 < 46 )
			MotorTimePole6 = 46;
		
		MotorPosCur = MOTOR_PHASE_SEQUENCE[MotorCurHall];
		MotorLastHall = MotorCurHall;

		if ( MotorPosCur ==0 )
		{
  			Ctl.State=MOTOR_FAILURE;
			//MotorErrorPosFlag = 1;			//错误标志位
			TIM1->BDTR &= 0x7FFF;			//关闭pwm
		}
		else
		{
			if ( MotorFlagDir )
			{
				switch ( MotorPosCur )
				{
					case 1u:
					MotorElectricalAngle = 5461;
					MotorEleAngleParaA = 0;
					MotorEleAngleParaE = 0xEC16;
					MotorEleAngleParaD = 5097;
					MotorEleAngleParaG = 10922;
					MotorEleAngleParaF = 16019;
					if ( !MotorFlagShake )
					  MotorEleAnglePara1 = 1;
					break;
				  case 2u:
					MotorElectricalAngle = 0x3FFF;
					MotorEleAngleParaA = 10922;
					MotorEleAngleParaE = 5825;
					MotorEleAngleParaD = 16019;
					MotorEleAngleParaG = 21845;
					MotorEleAngleParaF = 26942;
					break;
				  case 3u:
					MotorElectricalAngle = 27306;
					MotorEleAngleParaA = 21845;
					MotorEleAngleParaE = 16748;
					MotorEleAngleParaD = 26942;
					MotorEleAngleParaG = 0x8000;
					MotorEleAngleParaF = 0x93E9;
					break;
				  case 4u:
					MotorElectricalAngle = 0x9555;
					MotorEleAngleParaA = 0x8000;
					MotorEleAngleParaE = 27671;
					MotorEleAngleParaD = 0x93E9;
					MotorEleAngleParaG = 0xAAAA;
					MotorEleAngleParaF = 0xBE93;
					break;
				  case 5u:
					MotorElectricalAngle = 0xBFFF;
					MotorEleAngleParaA = 0xAAAA;
					MotorEleAngleParaE = 0x96C1;
					MotorEleAngleParaD = 0xBE93;
					MotorEleAngleParaG = 0xD555;
					MotorEleAngleParaF = 0xE93E;
					break;
				  case 6u:
					MotorElectricalAngle = 0xEAAA;
					MotorEleAngleParaA = 0xD555;
					MotorEleAngleParaE = 0xC16C;
					MotorEleAngleParaD = 0xE93E;
					MotorEleAngleParaG = 0xFFFF;
					MotorEleAngleParaF = 5097;
					break;
				  case 0u:
					break;
				}
				MotorEleAngleDiff = MotorEleAngleParaA - MotorElectricalAngleBak;
			}
			else
			{
				switch ( MotorPosCur )
				{
				  case 1u:
					MotorElectricalAngle = 5461;
					MotorEleAngleParaA = 10922;
					MotorEleAngleParaE = 5825;
					MotorEleAngleParaD = 16019;
					MotorEleAngleParaG = 0;
					MotorEleAngleParaF = 0xEC16;
					break;
				  case 2u:
					MotorElectricalAngle = 0x3FFF;
					MotorEleAngleParaA = 21845;
					MotorEleAngleParaE = 16748;
					MotorEleAngleParaD = 26942;
					MotorEleAngleParaG = 10922;
					MotorEleAngleParaF = 5825;
					break;
				  case 3u:
					MotorElectricalAngle = 27306;
					MotorEleAngleParaA = 0x8000;
					MotorEleAngleParaE = 27671;
					MotorEleAngleParaD = 0x93E9;
					MotorEleAngleParaG = 21845;
					MotorEleAngleParaF = 16748;
					break;
				  case 4u:
					MotorElectricalAngle = 0x9555;
					MotorEleAngleParaA = 0xAAAA;
					MotorEleAngleParaE = 0x96C1;
					MotorEleAngleParaD = 0xBE93;
					MotorEleAngleParaG = 0x8000;
					MotorEleAngleParaF = 27671;
					break;
				  case 5u:
					MotorElectricalAngle = 0xBFFF;
					MotorEleAngleParaA = 0xD555;
					MotorEleAngleParaE = 0xC16C;
					MotorEleAngleParaD = 0xE93E;
					MotorEleAngleParaG = 0xAAAA;
					MotorEleAngleParaF = 0x96C1;
					break;
				  case 6u:
					MotorElectricalAngle = 0xEAAA;
					MotorEleAngleParaA = 0xFFFF;
					MotorEleAngleParaE = 0xEC16;
					MotorEleAngleParaD = 5097;
					MotorEleAngleParaG = 0xD555;
					MotorEleAngleParaF = 0xC16C;
					if ( !MotorFlagShake )
					  MotorEleAnglePara1 = 1;
					break;
				  case 0u:
					break;
				}
				MotorEleAngleDiff = MotorElectricalAngleBak - MotorEleAngleParaA;
			}
			if ( (MotorTimePole6 > 1013) || (MotorFlagPole6OK==0) )
			{
				MotorElectricalAngleBak = MotorEleAngleParaA;
				MotorEleAngleParaC = MotorEleAngleParaG;
				MotorEleAngleDiff = 0;
				MotorEleAnglePara1 = 0;
			}
			else
			{
				MotorEleAngleParaC = MotorEleAngleParaF;
				if ( MotorFlagDir )
				{
					if ( MotorPosCur == 1 )
					{
						if ( MotorFlagShake )
						{
							if ( MotorElectricalAngleBak > MotorEleAngleParaD )
							{
								MotorElectricalAngleBak = MotorEleAngleParaA;
								MotorEleAngleDiff = 0;
								MotorEleAnglePara1 = 0;
							}
						}
						else if ( MotorElectricalAngleBak < MotorEleAngleParaE )
						{
							MotorElectricalAngleBak = MotorEleAngleParaA;
							MotorEleAngleDiff = 0;
							MotorEleAnglePara1 = 0;
						}
					}
					else
					{
						if ( (MotorPosCur == 6) && (MotorTimePole6 > 1013) )
							MotorEleAngleParaC = 0xffff;
						if (   (MotorElectricalAngleBak > MotorEleAngleParaD) 
							|| (MotorElectricalAngleBak < (signed int)MotorEleAngleParaE) )
						{
							MotorElectricalAngleBak = MotorEleAngleParaA;
							MotorEleAngleDiff = 0;
							MotorEleAnglePara1 = 0;
						}
					}
				}
				else if ( MotorPosCur == 6 )
				{
					if ( MotorFlagShake )
					{
						if ( MotorElectricalAngleBak < MotorEleAngleParaE )
						{
							MotorElectricalAngleBak = MotorEleAngleParaA;
							MotorEleAngleDiff = 0;
							MotorEleAnglePara1 = 0;
						}
					}
					else if ( MotorElectricalAngleBak > MotorEleAngleParaD )
					{
						MotorElectricalAngleBak = MotorEleAngleParaA;
						MotorEleAngleDiff = 0;
						MotorEleAnglePara1 = 0;
					}
				}
				else
				{
					if ( (MotorPosCur == 1) && (MotorTimePole6 > 1013) )
						MotorEleAngleParaC = 0;
					if (   (MotorElectricalAngleBak > MotorEleAngleParaD) 
						|| (MotorElectricalAngleBak < (signed int)MotorEleAngleParaE) )
					{
						MotorElectricalAngleBak = MotorEleAngleParaA;
						MotorEleAngleDiff = 0;
						MotorEleAnglePara1 = 0;
					}
				}
			}
		}
		
		if ( MotorTimePole6 <= 1013 )
			result = (s16)(MotorTimePole6 - MotorRollTimeLast);
		else
			result = 0;
		
		MotorUnknownPara1 = (MotorEleAngleDiff + 0xFFFF + ((MotorTimePole6 + result) >> 1)) 
		                  / ((unsigned int)MotorTimePole6 + result);
		MotorFlagShake = 0;
		MotorRollTimeLast = MotorTimePole6;
	}
	
	if ( MotorFlagMove )
		MotorElectricalAngleBak = MotorElectricalAngle;
	
	MotorFinalEleAngle = MotorElectricalAngleBak;
	
	if ( MotorFlagMove )
	{
		MotorFinalEleAngle += 5916;
	}
	else if ( MotorFlagDir )
	{
		MotorFinalEleAngle += 6735;
	}
	else
	{
		MotorFinalEleAngle += 5097;
	}
}
//=============================================================================
