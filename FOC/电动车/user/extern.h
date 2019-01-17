#ifndef __EXTERN_H
#define __EXTERN_H		
#include "stm32f10x.h"	
#include "math.h"


typedef struct 
{
  s16 qI_Component1;
  s16 qI_Component2;
} Curr_Components;    

typedef struct 
{
  s16 qV_Component1;
  s16 qV_Component2;
} Volt_Components;   
							
typedef struct
{
  s16 hCos;
  s16 hSin;
} Trig_Components;	
		   		   

/**********main***************/
extern u8 BatteryFlagLowRunOut;
extern u8 BatteryFlagVolLower;
extern u8 BatteryFlagError;//电压故障标志
extern u8 BatteryFlagVolLowest;
extern u16 BatteryVal1;// 电压值
extern u8 FocSelfCheckOK;//自检ok标志
//extern u8 RemoteFlagCorrectOut;//校准
extern u8 BatteryFlagPowerOnOut;

extern u8 FocTime1msFlag;//
extern u8 ErrorCircuit;
//电压电流寄存器
extern u16 FocAdcBatteryBuf[4];//可能是电压值的缓冲
extern u16 FocBootVolBuf[4];//可能是电压值的缓冲

//extern u8 RemoteFlagCorrectbeepOut;
extern u8 BatteryFlagLow;
//extern u8 RemoteFlagLockOut;//遥控器外部使用的锁机状态	
//extern u8 RemoteFlagAlarm;

//extern u16 BuzzType;
extern u8 FocFlagAdcOK;//获取adc ok
extern u8 FocFlagMotorLErrorElVol;//电机1电压故障
extern s16 FocParaMotorLAttitude;//调节参数1
//extern s16 FocMotorLCurDiff;
extern u8 FocFlagMotorSErrorElVol;//电机2电压故障
extern s16 FocParaMotorSAttitude;
extern s16 FocMotorSCurDiff;

extern u8 MOTOR_PHASE_SEQUENCE[8];

extern u8 MotorSErrorPosFlag;
extern s16 MotorSFlagMove;
extern u16 MotorSTimePole6;
extern u16 MotorSFlagDir;//电机方向标志
/////////////////////////
//电机1
extern Volt_Components MotorSAtatVolt_qd;//104 反park变换 最后赋值给pwm
extern Curr_Components MotorSCurr_ab;//10C 当前相电流结构

/***********motor2.c***************/
//电机2
extern Volt_Components MotorLAtatVolt_qd;//124
extern Curr_Components MotorLCurr_ab;//12C 当前相电流结构
//


//extern u8 UsartFlagMpuLErrorCnt;
//extern u8 UsartFlagMpuSErrorCnt;
//extern s16 UsartGyroZCtrl;//s16
//extern s16 UsartMpuSGyroZ;//16
//extern u8 UsartError_angle;//姿态板角度过大

//extern u8 RemoteFlagChildOut;
//extern u8 RemoteFlagChildbeepOut;

//extern float UsartMpuLFinalVal;
//extern float UsartMpuSFinalVal;
//extern u8 UsartMpuPHOL;

//extern u8 UsartStatemotorSOk;
//extern u8 UsartStatemotorLOk;
//extern u8 UsartFlagLowAndPhoOn;//充电时 踩下踏板
//extern u8 UsartErrorOverAngle;//角度过大
//extern u8 UsartFlagOverSpeed;



//extern u8 RemoteFlagCorrectOut;//校准

/**********charge.h***************/
extern void BatteryGetFun(void);
extern void BatteryCheckFun(void);//获取充电器状态
extern void BatteryCheckPowerOnFun(void);//休眠检测
/**********v_remote.h***************/
//extern void RemoteCtrlFun(void);
/**********attitude.h***************/
//extern void AttCorAngleFun(void);
//extern int AttMotorSPidFun(void);//motorS角度pid算法
//extern int AttMotorLPidFun(void);//motorL角度pid算法
/**********algorithm.h***************/
extern void FocSelfCheckingFun(void);
extern void FocVlotCmp1RegulationFun(void);//踏板速度调节
/**********triangle***************/
extern s16 get_sin(u16); // weak
extern s16 get_cos(u32 a1);

/**********motor1***************/
extern void MotorSFetAngleFun(void);
extern void MotorSInitParaFun(void);
extern void MotorSFlowRegFun(void);
extern void MotorSClarkeFun(void);
extern void MotorSParkFun(void);
extern void MotorSRevParkFun(void);
extern void MotorSMotorCtrlFun(void);

/**********buzz***************/
//extern void BuzzCtrlFun(void);
//extern void BuzzFun(void);

/**********led***************/
//extern void ErrorLedCtrlFun(void);//控制led闪烁频率 故障指示


/**********InitFun***************/
extern void InitFun(void);



#endif
