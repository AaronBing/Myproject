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
extern u8 BatteryFlagError;//��ѹ���ϱ�־
extern u8 BatteryFlagVolLowest;
extern u16 BatteryVal1;// ��ѹֵ
extern u8 FocSelfCheckOK;//�Լ�ok��־
//extern u8 RemoteFlagCorrectOut;//У׼
extern u8 BatteryFlagPowerOnOut;

extern u8 FocTime1msFlag;//
extern u8 ErrorCircuit;
//��ѹ�����Ĵ���
extern u16 FocAdcBatteryBuf[4];//�����ǵ�ѹֵ�Ļ���
extern u16 FocBootVolBuf[4];//�����ǵ�ѹֵ�Ļ���

//extern u8 RemoteFlagCorrectbeepOut;
extern u8 BatteryFlagLow;
//extern u8 RemoteFlagLockOut;//ң�����ⲿʹ�õ�����״̬	
//extern u8 RemoteFlagAlarm;

//extern u16 BuzzType;
extern u8 FocFlagAdcOK;//��ȡadc ok
extern u8 FocFlagMotorLErrorElVol;//���1��ѹ����
extern s16 FocParaMotorLAttitude;//���ڲ���1
//extern s16 FocMotorLCurDiff;
extern u8 FocFlagMotorSErrorElVol;//���2��ѹ����
extern s16 FocParaMotorSAttitude;
extern s16 FocMotorSCurDiff;

extern u8 MOTOR_PHASE_SEQUENCE[8];

extern u8 MotorSErrorPosFlag;
extern s16 MotorSFlagMove;
extern u16 MotorSTimePole6;
extern u16 MotorSFlagDir;//��������־
/////////////////////////
//���1
extern Volt_Components MotorSAtatVolt_qd;//104 ��park�任 ���ֵ��pwm
extern Curr_Components MotorSCurr_ab;//10C ��ǰ������ṹ

/***********motor2.c***************/
//���2
extern Volt_Components MotorLAtatVolt_qd;//124
extern Curr_Components MotorLCurr_ab;//12C ��ǰ������ṹ
//


//extern u8 UsartFlagMpuLErrorCnt;
//extern u8 UsartFlagMpuSErrorCnt;
//extern s16 UsartGyroZCtrl;//s16
//extern s16 UsartMpuSGyroZ;//16
//extern u8 UsartError_angle;//��̬��Ƕȹ���

//extern u8 RemoteFlagChildOut;
//extern u8 RemoteFlagChildbeepOut;

//extern float UsartMpuLFinalVal;
//extern float UsartMpuSFinalVal;
//extern u8 UsartMpuPHOL;

//extern u8 UsartStatemotorSOk;
//extern u8 UsartStatemotorLOk;
//extern u8 UsartFlagLowAndPhoOn;//���ʱ ����̤��
//extern u8 UsartErrorOverAngle;//�Ƕȹ���
//extern u8 UsartFlagOverSpeed;



//extern u8 RemoteFlagCorrectOut;//У׼

/**********charge.h***************/
extern void BatteryGetFun(void);
extern void BatteryCheckFun(void);//��ȡ�����״̬
extern void BatteryCheckPowerOnFun(void);//���߼��
/**********v_remote.h***************/
//extern void RemoteCtrlFun(void);
/**********attitude.h***************/
//extern void AttCorAngleFun(void);
//extern int AttMotorSPidFun(void);//motorS�Ƕ�pid�㷨
//extern int AttMotorLPidFun(void);//motorL�Ƕ�pid�㷨
/**********algorithm.h***************/
extern void FocSelfCheckingFun(void);
extern void FocVlotCmp1RegulationFun(void);//̤���ٶȵ���
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
//extern void ErrorLedCtrlFun(void);//����led��˸Ƶ�� ����ָʾ


/**********InitFun***************/
extern void InitFun(void);



#endif
