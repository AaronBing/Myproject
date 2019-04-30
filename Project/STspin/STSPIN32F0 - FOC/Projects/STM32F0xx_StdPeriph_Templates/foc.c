


#include "main.h"
extern uint16_t 	MotorFinalEleAngle;			//外部来的，那这个是最新的电角度吗
Curr_Components 	MotorCurr_ab;
Volt_Components 	MotorAtatVolt_qd;
Curr_Components  	MotorCurrAlfaBeta;
Trig_Components  	MotorVectorComponents;
Volt_Components 	MotorVol_ab;
int16_t 	MotorFlowIntergral;  
int16_t 	MotorCurrqd;         
void MotorInitParaFun (void)
{
	MotorFlowIntergral             = 0;
	MotorCurrqd                    = 0;
	MotorAtatVolt_qd.qV_Component2 = 0;
}
void MotorClarkeFun (void)
{ 
  MotorCurrAlfaBeta.qI_Component1 = MotorCurr_ab.qI_Component1;	
  MotorCurrAlfaBeta.qI_Component2 = 18918 * (MotorCurr_ab.qI_Component1 
									    + 2 * MotorCurr_ab.qI_Component2) >> 15;
}
//=============================================================================
void MotorParkFun (void)
{ 
	s32 	result; 
	result = (MotorCurrAlfaBeta.qI_Component1 * MotorVectorComponents.hCos
			+ MotorCurrAlfaBeta.qI_Component2 * MotorVectorComponents.hSin) >> 15;
	MotorCurrqd = (20317 * MotorCurrqd + 12451 * result) >> 15; 
}
//=============================================================================
void MotorFlowRegFun (void)
{
	s16 	curq_d; 
	s16 	diff; 
	s16 	val; 
	curq_d = -MotorCurrqd;
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
	MotorFlowIntergral += diff;	
	val = MotorFlowIntergral + (curq_d >> 2);
	if ( val < -0x4000 )
	{
		val = -16384;
		MotorFlowIntergral = -16384;
	}
	else if ( val > 0x4000 )
	{
		val = 0x4000;
		MotorFlowIntergral = 0x4000;
	}	
	MotorAtatVolt_qd.qV_Component2 = val >> 6;
}
//=============================================================================
void MotorRevParkFun (void)
{		
	MotorVectorComponents.hSin = get_sin(MotorFinalEleAngle);
	MotorVectorComponents.hCos = get_cos(MotorFinalEleAngle);
	
	MotorVol_ab.qV_Component2 = 
		 (MotorAtatVolt_qd.qV_Component2 * MotorVectorComponents.hCos 
		- MotorAtatVolt_qd.qV_Component1 * MotorVectorComponents.hSin) >> 15;
	
	MotorVol_ab.qV_Component1 = 
		 (MotorAtatVolt_qd.qV_Component1 * MotorVectorComponents.hCos 
		+ MotorAtatVolt_qd.qV_Component2 * MotorVectorComponents.hSin) >> 15;
}
//=============================================================================
void MotorCtrlFun (void)  
{
	s16 	hTimePhA; 
	s16 	hTimePhB;
	s16 	hTimePhC; 	
	u16 bSector; 	
	s16 wX;  
	s16 wZ;  
	s16 wY; 	
	wX = MotorVol_ab.qV_Component1;
	wY = (s16)(((3547 * MotorVol_ab.qV_Component2) >> 11) - MotorVol_ab.qV_Component1);
	wZ = (s16)-(MotorVol_ab.qV_Component1 + ((3547 * MotorVol_ab.qV_Component2) >> 11));	
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
	TIM1->CCR1 = hTimePhA;		//TIM capture/compare register 1,
	TIM1->CCR2 = hTimePhB;
	TIM1->CCR3 = hTimePhC;
}

