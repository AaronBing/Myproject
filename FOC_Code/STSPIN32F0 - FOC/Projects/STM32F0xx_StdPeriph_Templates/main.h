
#include "stm32f0xx.h"
#include "stdint.h"

typedef unsigned char u8;
typedef unsigned short int u16;
typedef unsigned int u32;
typedef signed char s8;
typedef signed short int s16;
typedef signed int s32;
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
void device_init (void);
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
