
#include "stm32f0xx.h"
#include "stdint.h"


#define u32 unsigned int 
#define u16 unsigned short 
#define s32 int 
#define s16 short int 
#define u8  unsigned short 
#define s8  short int
//=============================================================================
typedef struct 
{
	s16 	qI_Component1;
	s16 	qI_Component2;
} Curr_Components;

typedef struct 
{
	s16 	qV_Component1;
	s16 	qV_Component2;
} Volt_Components; 
							
typedef struct
{
	s16 	hCos;
	s16 	hSin;
} Trig_Components;

//==============================================================================
void HardwareInit(void);
void board_self_test (void);

void MotorInitParaFun (void);
void MotorClarkeFun (void);
void MotorParkFun (void);
void MotorFlowRegFun (void);
void MotorRevParkFun (void);
void MotorCtrlFun (void);

s16 get_sin (u16 a1); 
s16 get_cos (u32 a1);

void MotorFetAngleFun (void);
