/***********************************************************
*	���ϵ�
*
*
*
************************************************************/
#include "extern.h"

u8 ErrorLedCnt;//led���Ƽƴ�
u8 ErrorLedOnOff;//led����

void ErrorLedCtrlFun(void);//����led��˸Ƶ�� ����ָʾ



//-------------------------------------------------------------
void ErrorLedCtrlFun()
{//����led��˸Ƶ�� ����ָʾ
  u16 cnt=0; 
	
   if ( FocFlagMotorSErrorElVol )
  {//����һ�ྲ̬��������
    cnt = 1;
  }

  else if ( ErrorCircuit )
  {//��������ж�·
    cnt = 3;
  }
  else if ( MotorSErrorPosFlag )
  {//����һ������������
    cnt = 4;
  }

  else if ( BatteryFlagError )
  {//��ع��ϱ�־
    cnt = 6;
  }
  else
  {
    cnt = 0;//����˸ �޴���
  }
	
  if ( cnt )
  {
    if ( !ErrorLedOnOff )
    {
      ++ErrorLedCnt;//����
      if ( ErrorLedCnt > cnt )
        ErrorLedCnt = 0;
    }
    if ( ErrorLedCnt >= cnt )
    {
      GPIO_ResetBits(GPIOB, 4);//LED
    }
			else if ( ErrorLedOnOff )
			{
				GPIO_SetBits(GPIOB, 4);//LED��
			}
				else
				{
					GPIO_ResetBits(GPIOB, 4);//LED��
				}
    ErrorLedOnOff ^= 1u;  //ȡ��
  }
  else
  {
    GPIO_ResetBits(GPIOB, 4);//LED
  }
}
