/***********************************************************
*	���ص��
*
*
*
************************************************************/
#include "extern.h"


u8  FocSelfCheckOK;//�Լ�ok��־

//����
u16 FocTime64usCnt;//TIM1_UP�ж� 1/15.625K = 64us ÿ64us�ж�һ��
u16 FocGetAdcCnt;//��ȡadcת������
u8 	FocTime1msFlag;//16��TIM1_UP�ж�,1.024ms Լ���� 1ms ��ʱ

//���ߵ��һ�ߵ�adֵ
//u16 FocMotorLCurU; //U�����ֵ----�����˷ŷŴ�ĵ�ѹֵ
//u16 FocMotorLCurW; //W�����ֵ----�����˷ŷŴ�ĵ�ѹֵ
//u16 FocMotorLLoadBuf[4]={0};//ĸ�ߵ������  �ɼ�4��ֵ,��ƽ��ֵ. 
u16 FocAdcBatteryBuf[4]={0};//��ص�ѹ���  �ɼ�4��ֵ,��ƽ��ֵ.
//���ߵ��һ�ߵ�adֵ
u16 FocMotorSCurU;
u16 FocMotorSCurW;
u16 FocMotorSLoadBuf[4];//ĸ�ߵ������  �ɼ�4��ֵ,��ƽ��ֵ. 
u16 FocBootVolBuf[4];//������ѹ��� �����������µ�ʱ��

u8  FocFlagAdcOK;//�ɼ�4��adc��� ok

//����
//u8  FocMotorLCurVlotHigh;    //�ظ���????
//u8  FocFlagMotorLErrorElVol;//���1��ѹ���� �������޸�
//u16 FocMotorLLoadRef;				//�˷�ƫ�õ�ѹ
//s16 FocParaMotorLAttitude;	//���ת�ӽǶ�????? --------�����ǵĽǶ�
//s16 FocMotorLCurDiff;		 //ʵ�ʵ�����С	 
//����
u8  FocMotorSCurVlotHigh;
u8  FocFlagMotorSErrorElVol;//���2��ѹ����
u16 FocMotorSLoadRef;				//
s16 FocParaMotorSAttitude;  
s16 FocMotorSCurDiff;

u16 FocMotorShPhaseAOffset;//���߶�A��---��̬ƫ�õ���
u16 FocMotorShPhaseBOffset;//���߶�C��---��̬ƫ�õ���
//u16 FocMotorLhPhaseAOffset;//���߶�A��----��̬ƫ�õ���
//u16 FocMotorLhPhaseBOffset;//���߶�B��----��̬ƫ�õ���

void FocSelfCheckingFun(void);
void FocVlotCmp1RegulationFun(void);//̤���ٶȵ���
//-------------------------------------------------------------

void FocSelfCheckingFun()
{//��ȫ��� ��Ҫ��ȡ��Ư ok
  u16 i; // 
  u16 sum; //
  u16 sum1; // 

  MotorSTimePole6 = 0xffff;
	
//  BatteryFlagLow = 0;//��ص�ѹ��
//  BatteryFlagPowerOnOut = 0;
	
  for ( i = 0; i < 50; i = i + 1 ) //��ʱ50ms
  {
    while ( !FocTime1msFlag );
    FocTime1msFlag = 0;
  }
	//--------�������---------------------
	i = FocBootVolBuf[0] + FocBootVolBuf[1];
	i += FocBootVolBuf[2];
	i += FocBootVolBuf[3];
  if ( (i >> 2) > 1240 )  //�ɼ��ĴΣ�ȡƽ��ֵ
	{//�������
		GPIO_SetBits(GPIOA, 32);//����
	}

  for ( i = 0; i < 380; i = (i + 1) )  //��ʱԼ380ms
  {
    while ( !FocTime1msFlag );
    FocTime1msFlag = 0;
  }

  sum = 0;
  for ( i = 0; i < 4; i = (u8)(i + 1) )
    sum = (u16)(FocMotorSLoadBuf[i] + sum);
  FocMotorSLoadRef = sum >> 2;//����
  if ( (FocMotorSLoadRef >= 2234) || (FocMotorSLoadRef <= 1614) )
  {//������Χ �ر�pwm
    FocFlagMotorSErrorElVol = 1;//����һ�ྲ̬��������
    TIM1->BDTR &= (~0x8000);//�ر�pwm
  }

  //--------��Ư---------(��ȡ�����ƫ��)-------------

  sum1 = 0;
  sum = 0;
  for ( i = 0; i < 16; i = (u8)(i + 1) )
  {
    sum1 += FocMotorSCurU;
    sum += FocMotorSCurW;
    while ( !FocTime1msFlag );
    FocTime1msFlag = 0;
  }
  FocMotorShPhaseAOffset = sum1 >> 4;
  FocMotorShPhaseBOffset = sum >> 4;
  if ( (FocMotorShPhaseAOffset > 2234)|| (FocMotorShPhaseAOffset < 1614)|| (FocMotorShPhaseBOffset > 2234)|| (FocMotorShPhaseBOffset < 1614) )
  {//������Χ �ر�pwm
    FocFlagMotorSErrorElVol = 1;
    TIM1->BDTR &= (~0x8000);//�ر�pwm
  }
//  TIM1->BDTR &= (~0x8000);//�ر�pwm
//  TIM8->BDTR &= (~0x8000);//�ر�pwm
		
	FocSelfCheckOK = 1;//�Լ����

}


//-------------------------------------------------------------
//�ж�
void TIM1_UP_IRQHandler(void)
{//time1��� ���Ƶ�����㷨�ڴ��ж������
	//ʱ���ۼ� ��ʱ���ж�64us
  int torque_gas;
  int gas_faccelerate;
//  int attitude; 
//  int atat; 

	//-------------ʱ�䶨ʱ��------16*64us = 1024us ~ 1ms--------------
  if ( FocTime64usCnt >= 16 )
  {
    FocTime64usCnt = 0;
    FocTime1msFlag = 1;	//���1ms��ʱ��־
  }
  else
  {
    ++FocTime64usCnt;
  }
	//-------------��ȡadc����------------------
	//ÿ���жϼ��һ��----��μ��(���߶�ĸ�ߵ�������ص�ѹ�����߶�ĸ�ߵ��������ػ�����ѹ)
  if ( FocGetAdcCnt >= 3 )
  {
    FocFlagAdcOK = 1;//��ȡadc 4�����
    FocGetAdcCnt = 0;
  }
  else
  {
    ++FocGetAdcCnt;
  }
	//-------------��ȡ��Ƕ�-------------------(ֱ�ӵ�������,�����˽�)
  if ( FocSelfCheckOK )
  {//�Լ����
    MotorSFetAngleFun();
  }
	
	//-------------��ȡ����L-------------------
//  FocMotorLCurU = ADC3->JDR1;//u
//  FocMotorLCurW = ADC3->JDR2;//w
//  FocMotorLLoadBuf[FocGetAdcCnt] = ADC3->JDR3;//���ߵ���
  FocAdcBatteryBuf[FocGetAdcCnt] = ADC3->JDR4;//��ص�����������	
	
	//-------------��ȡ����S-------------------
  FocMotorSCurU = ADC2->JDR1;//u�����
  FocMotorSCurW = ADC2->JDR2;//w�����
  FocMotorSLoadBuf[FocGetAdcCnt] = ADC2->JDR3;//���ߵ���  	
  FocBootVolBuf[FocGetAdcCnt] = ADC2->JDR4;//��ѹ ������ⰴť
//	FocAdcBatteryBuf[FocGetAdcCnt] = ADC3->JDR4;//��ص�����������
	
  MotorSCurr_ab.qI_Component2 = FocMotorShPhaseAOffset - FocMotorSCurU;   //A��ʵ�ʵ���
	
//B��ʵ�ʵ���, ʵ�ʲ���������C��ĵ�����    ��Ia+Ib+Ic=0;
//Ib = -(Ia+Ic) = -(MotorSCurr_ab.qI_Component2 +  FocMotorShPhaseBOffset - FocMotorSCurW )	
  MotorSCurr_ab.qI_Component1 = -(s16)(FocMotorShPhaseBOffset - FocMotorSCurW) 
																	- MotorSCurr_ab.qI_Component2;
  
	//-------------�ٶȵ��� -------------------(��ʱ�������ã��������о�)
	if ( FocParaMotorSAttitude < 0 )	//�Ƕ�pid������Ľ��
    torque_gas = -FocParaMotorSAttitude;
  else
    torque_gas = FocParaMotorSAttitude;
  if ( MotorSAtatVolt_qd.qV_Component1 < 0 )
    gas_faccelerate = -MotorSAtatVolt_qd.qV_Component1;
  else
    gas_faccelerate = MotorSAtatVolt_qd.qV_Component1;
  if ( torque_gas <= gas_faccelerate )
  {
//    MotorSAtatVolt_qd.qV_Component1 = FocParaMotorSAttitude;  //�����ԭ���ġ�
	  MotorSAtatVolt_qd.qV_Component1 = 200; //����Ǻ���ĵ�
  }
  else if ( !FocMotorSCurVlotHigh )
  {//���ز���
    if ( MotorSAtatVolt_qd.qV_Component1 >= FocParaMotorSAttitude )
    {
      if ( MotorSAtatVolt_qd.qV_Component1 > FocParaMotorSAttitude )
      {
        if ( MotorSAtatVolt_qd.qV_Component1 <= FocParaMotorSAttitude + 2 )
          --MotorSAtatVolt_qd.qV_Component1;
        else
          MotorSAtatVolt_qd.qV_Component1 -= 2;
      }
    }
    else if ( MotorSAtatVolt_qd.qV_Component1 + 2 >= FocParaMotorSAttitude )
    {
      ++MotorSAtatVolt_qd.qV_Component1;
    }
    else
    {
      MotorSAtatVolt_qd.qV_Component1 += 2;
    }
  }
	
  if ( FocSelfCheckOK )
  {//�Լ����
		//---------���Ƶ��S-------------------
    if ( MotorSFlagMove )
    {//���ֲ��ƶ��󣬻��ֹ���
      MotorSInitParaFun();//������ʼ��
    }
    else if ( (FocMotorSCurU < 3848)   //���߶�U�����
				&& (FocMotorSCurW < 3848) )     //���߶�W�����
    {
      MotorSClarkeFun();  //	Clarke����任
      MotorSParkFun();		//  Park�任  ����ǶȲ�������غ��� 
      MotorSFlowRegFun(); //���������ĵ���
    }
    MotorSRevParkFun();		//��park�任
    MotorSMotorCtrlFun(); //�������
		
  }

  TIM1->SR &= 0xFFFEu;
  TIM1->SR &= 0xFFFEu;
}


//-------------------------------------------------------------
// Ӧ�ò��Ǹ��ص��ڣ����ڿ���
// ������ǰ������

void FocVlotCmp1RegulationFun()
{//���ص��ڣ����ߵ�������
  u8 i; // 
  u32 val; // 
  u32 current; // 

	//---------motorS------------------
  val = 0;
  for ( i = 0; i < 4; i++ )
    val = (FocMotorSLoadBuf[i] + val);//���ߵ���
  current = val >> 2;
  FocMotorSCurDiff = current - FocMotorSLoadRef;
  if ( ((FocMotorSLoadRef + 717) >= current)
									&& (BatteryVal1 > 1032) )
  {//���ߵ��� ��ص�ѹ ����
    FocMotorSCurVlotHigh = 0;
  }
  else
  {//���ع���
    FocMotorSCurVlotHigh = 1;//���ش�
    if ( MotorSAtatVolt_qd.qV_Component1 >= 0 )
      --MotorSAtatVolt_qd.qV_Component1;
    else
      ++MotorSAtatVolt_qd.qV_Component1;//���ص���
  }
}

