/***********************************************************
*	��ص�ѹ
*
*
*
************************************************************/
#include "extern.h"


u8  BatteryFlagLowRunOut;//��֪���ı�־λ
u8  BatteryFlagVolLower;
u16 BatteryValLowest=0xffff;
s16 BatteryDownCnt1;
u8  BatteryFlagError;//��ѹ���ϱ�־
u8  BatteryFlagVolLowest;
u16 BatteryVal1;// ��ѹֵ

u8  BatteryFlagPowerOnOut;//������־λ���ⲿʹ��
u8  BatteryFlagLow;//��ص�ѹ�ͱ�־λ
u8  BatteryFlagPowerOn;//������־λ
u16 BatteryPowerOnCnt2;//����������2
u16 BatteryFlagPowerOnCnt;//����������
u32 BatteryAutoOffCnt;//�Զ��ػ�
u16 BatteryPowerOnCnt;//����������
u8 BatteryFlagLowCnt;//��ص�ѹ�ͼ�����
u8 BatteryFlagHighCnt;//��ص�ѹ�߼�����

void BatteryGetFun(void);
void BatteryCheckFun(void);//��ȡ�����״̬
void BatteryCheckPowerOnFun(void);

//-------------------------------------------------------------
void BatteryGetFun()
{//��ص�ѹ
  u16 sum; 
  u8 i; 

	//���2�ĵ�ѹ
  sum = 0;
  for ( i = 0; i < 4; i = (u8)(i + 1) )
    sum += FocAdcBatteryBuf[i];
  BatteryVal1 = sum >> 2;//��ѹֵƽ��
	
  if ( BatteryVal1 <= 720 )
  {//������Χ �ر�pwm
    BatteryFlagError = 1;//��ع��ϱ�־
    TIM1->BDTR &= 0x7FFFu;//�ر�pwm
    TIM8->BDTR &= 0x7FFFu;//�ر�pwm
  }
	//��ѹ���
//  if ( (((FocMotorSCurDiff + FocMotorLCurDiff) > 0) 
//		&& ((FocMotorSCurDiff + FocMotorLCurDiff) < 105))
//		&& (BatteryValLowest > BatteryVal1) )
    if ( ((FocMotorSCurDiff  > 0) 
		&& (FocMotorSCurDiff  < 105))
		&& (BatteryValLowest > BatteryVal1) )
  {//������
		//��ص�����ͣ���� ������µ�ǰ�ĵ�ص�ѹֵ
    if ( BatteryDownCnt1 >= 80 )
    {
      BatteryDownCnt1 = 0;//�ƴ�
      BatteryValLowest = BatteryVal1;//��͵�ѹ
    }
    else
    {
      ++BatteryDownCnt1;
    }
  }
	if ( BatteryFlagLow )
	{//��ѹ��
    BatteryValLowest = BatteryVal1;
	}
	//��ѹ��
	if(BatteryValLowest < 1373)
	{//��͵�ѹ
		BatteryFlagVolLower = 1;
	}
	else
	{
		BatteryFlagVolLower = 0;
	}
	if(BatteryValLowest <= 1360)
	{//��ѹ����
		BatteryFlagVolLowest = 1;
	}
	else
	{
		BatteryFlagVolLowest = 0;
	}
	
  if((MotorSTimePole6 < 1350) 			 	
			&& (BatteryFlagVolLowest == 1)) //6�λ����л�����ʱ��
	{
		BatteryFlagLowRunOut = 1;
	}
	else BatteryFlagLowRunOut = 0;
  
}


//-------------------------------------------------------------
void BatteryCheckFun()
{//��ؼ��
  if ( GPIO_ReadInputDataBit(GPIOA, 4096) )
  {//
    BatteryFlagLowCnt = 0;//��ѹ�ͼƴ�
    if ( BatteryFlagHighCnt >= 10 )
    {
      BatteryFlagLow = 0;
    }
    else
    {
      ++BatteryFlagHighCnt;//��ѹ�߼ƴ�
    }
  }
  else
  {
    BatteryFlagHighCnt = 0;
    if ( BatteryFlagLowCnt >= 10 )
    {
      BatteryFlagLow = 1;//��ص�ѹ��
    }
    else
    {
      ++BatteryFlagLowCnt;
    }
  }
}

//-------------------------------------------------------------
void BatteryCheckPowerOnFun()
{//���ؼ��
	u32 vol; 
  s32 i; 
//  s32 j;

  vol = (FocBootVolBuf[3] + FocBootVolBuf[2] + FocBootVolBuf[1] + FocBootVolBuf[0]) >> 2;
  
	if ( BatteryPowerOnCnt < 1000 )
    ++BatteryPowerOnCnt;
  if ( BatteryFlagPowerOn )
  {//������
		//------------����ѹ------------------
    if ( vol <= 1240 )
    {
			//��ѹ��
      if ( BatteryPowerOnCnt2 > 6 )
      {
        if ( BatteryPowerOnCnt >= 800 )
        {//��ѹ���ѭ���ƴ�
					//ʱ�䳤�ػ���
          for ( i = 0; i < 200; i = (u16)(i + 1) )
          {
            while ( !FocTime1msFlag )
              ;
            FocTime1msFlag = 0;
          }
          GPIO_ResetBits(GPIOA, 32);//�ػ�
        }
        else
        {//�����ͯģʽ
          BatteryPowerOnCnt = 800;
        }
      }
      BatteryPowerOnCnt2 = 0;
    }
    else if ( BatteryPowerOnCnt2 < 500 )
    {
      ++BatteryPowerOnCnt2;
    }
		//-----------�Զ��ػ�-----------------
    if ( (MotorSFlagMove != 1) )
    {
      BatteryAutoOffCnt = 0;
    }
    else if ( 
			 (BatteryFlagError)				//��ع���
			|| MotorSErrorPosFlag		//			
			|| ErrorCircuit			
			|| FocFlagMotorSErrorElVol	//����һ�ྲ̬��������			
				)					
    {//�Զ��ػ�
      if ( BatteryAutoOffCnt >= 292968 )//�޶������ػ� ������10����
      {//����ѭ���ƴ�
        GPIO_ResetBits(GPIOA, 32);//�ػ�
      }
      else
      {
        BatteryAutoOffCnt++;//�Զ��ػ�������
      }
    }
    else
    {
      BatteryAutoOffCnt = 0;//�ж������Զ��ػ�����������
    }
  }
  else
	{
		if ( vol > 1240 )
		{
			//��ѹ�� ʱ�������
			if ( BatteryFlagPowerOnCnt < 1500 )
			{
				++BatteryFlagPowerOnCnt;
			}
//			else
//			{ 
//				RemoteFlagCorrectOut = 1;//��ť��ʱ�䳤������У׼
//			}
		}
		else if ( vol <= 1240 )
		{//���򿪻�
			BatteryFlagPowerOnCnt = 0;
			BatteryFlagPowerOn = 1;				//��ʼ����ѹ��־
			BatteryFlagPowerOnOut = 1;
		}
	}
}


