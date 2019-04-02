


#include "main.h"
extern  uint32_t	FocTime1msFlag;
extern  uint16_t 	FocBootVolBuf[4];
extern  uint16_t	FocMotorLoadBuf[4];
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
	for (i = 0; i < 50; i++)
	{
		while ( !FocTime1msFlag );
		FocTime1msFlag = 0;
	}
	sum1 = 0;
	sum  = 0;	
	for ( i = 0; i < 16; i++ )
	{
		sum1 += FocMotorCurA;	
		sum  += FocMotorCurB;			
		while ( !FocTime1msFlag );
		FocTime1msFlag = 0;
	}
	FocMotorPhaseAOffset = sum1 >> 4;	
	FocMotorPhaseBOffset = sum  >> 4;		
	if ( (FocMotorPhaseAOffset > 2234) 
      || (FocMotorPhaseAOffset < 1614) 
      || (FocMotorPhaseBOffset > 2234) 
      || (FocMotorPhaseBOffset < 1614) )
	{
		FocFlagMotorErrorElVol = 1;
		TIM1->BDTR &= (~0x8000);
	}	
	FocSelfCheckOK = 1;
}
