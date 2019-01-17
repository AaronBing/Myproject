/***********************************************************
*	����
*
*
*
************************************************************/

#include "extern.h"



u8 BuzzFlagPowerOn;	//����
u8 BuzzPowerOnType;	//��������
u8 BuzzRecoveryCnt;	//�������ָ�����
u8 BatteryFlagLowBak;

u8 BuzzFlagunKnown1;//����buzz���
u8 BuzzFlagunKnown2;

u8 BuzzCnt;					//�������ƴ�
u8 BuzzTotal;				//
u8 BuzzOn;					//����������
u16 BuzzType;				//�������� out


void BuzzCtrlFun(void);
void BuzzFun(void);





//-------------------------------------------------------------
void BuzzCtrlFun()
{
//  signed int i; // 

	/********�������ȿ���***********/
  if ( BuzzPowerOnType )
  {//�����ǿ����е�����
    if ( BuzzPowerOnType == 1 )
    {//����
      BuzzType = 2;
      if ( BuzzRecoveryCnt >= 80 )
      {
        BuzzRecoveryCnt = 0;
        BuzzPowerOnType = 2;
      }
      else
      {
        ++BuzzRecoveryCnt;
      }
    }
    else if ( BuzzPowerOnType == 2 )
    {
      BuzzType = 3;
      if ( BuzzRecoveryCnt >= 80 )//ԭ���������80
      {
        BuzzRecoveryCnt = 0;
        BuzzPowerOnType = 3;
      }
      else
      {
        ++BuzzRecoveryCnt;
      }
    }
    else
    {
      BuzzType = 0;
      BuzzRecoveryCnt = 0;
      BuzzPowerOnType = 0;
      BuzzFlagPowerOn = 1;//������־ �����������
    }
  }
  else
  {
    BuzzType = 1;
    if ( BuzzRecoveryCnt >= 80 )//ԭ���������80
    {
      BuzzRecoveryCnt = 0;
      BuzzPowerOnType = 1;
    }
    else
    {
      ++BuzzRecoveryCnt;
    }
  }
}

//-------------------------------------------------------------
void BuzzFun()
{//����������״̬
  if ( BuzzType == 1 )
  {
    if ( !BuzzCnt )//�����ƴ�Ϊ0
      BuzzTotal = 6;
  }
  else if ( BuzzType == 2 )
  {
    if ( !BuzzCnt )
      BuzzTotal = 5;
  }
  else if ( (BuzzType == 3) && (!BuzzCnt) )
  {
    BuzzTotal = 4;
  }
  if ( BuzzCnt >= BuzzTotal )
  {
    BuzzCnt = 0;
    if ( BuzzOn )
    {
      BuzzOn = 0;
      GPIO_SetBits(GPIOA, 16);//����
    }
    else
    {
      BuzzOn = 1;
      GPIO_ResetBits(GPIOA, 16);
    }
  }
  else
  {
    ++BuzzCnt;
  }
}

