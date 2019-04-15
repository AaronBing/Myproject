


#include "main.h"
extern  uint32_t	FocTime1msFlag;
//extern  uint16_t 	FocBootVolBuf[4];
//extern  uint16_t	FocMotorLoadBuf[4];
extern  uint16_t	FocMotorCurA;
extern  uint16_t	FocMotorCurB;
uint32_t	FocSelfCheckOK;
uint16_t 	FocMotorLoadRef;
uint32_t  	FocFlagMotorErrorElVol;
uint16_t 	FocMotorPhaseAOffset;
uint16_t 	FocMotorPhaseBOffset;
void board_self_test (void)
{
	uint16_t 	i;
	uint16_t 	sum; 
	uint16_t 	sum1; 
	for (i = 0; i < 50; i++)    //所以这个是50ms延迟？
	{
		while ( !FocTime1msFlag );
		FocTime1msFlag = 0;
	}
	
	
	sum1 = 0;
	sum  = 0;	
	for ( i = 0; i < 16; i++ )		//累加16次，
	{
		sum1 += FocMotorCurA;	
		sum  += FocMotorCurB;			
		while ( !FocTime1msFlag );
		FocTime1msFlag = 0;
	}
	FocMotorPhaseAOffset = sum1 >> 4;	
	FocMotorPhaseBOffset = sum  >> 4;		
	if ( (FocMotorPhaseAOffset > 2234) 		//基准电流不在1614-2234这个范围内
      || (FocMotorPhaseAOffset < 1614) 
      || (FocMotorPhaseBOffset > 2234) 
      || (FocMotorPhaseBOffset < 1614) )
	{
		
//		Ctl.State=MOTOR_FAILURE;
//		Ctl.Error= E_CURR;			//基准电流保护
//		
		//TIM1->BDTR &= (~0x8000);		//关闭pwm
	}else{
		Ctl.State=MOTOR_OPENLOOP;
	}
	FocSelfCheckOK = 1;
}
