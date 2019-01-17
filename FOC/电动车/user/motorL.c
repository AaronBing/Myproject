/***********************************************************
*	���Ƴ��ߵ��
*
*
*
************************************************************/

#include "extern.h"

//���L
u8 MOTOR_PHASE_SEQUENCE[8]={0,2,4,3,6,1,5};//120����ˢ��� ��λ˳���
Volt_Components 	MotorLAtatVolt_qd;// Vd   Vq
s16 MotorLCurrqd;
s16 MotorLFlowIntergral;
Curr_Components 	MotorLCurr_ab;// ��ǰ������ṹ  Ia Ib 
Curr_Components  	MotorLCurrAlfaBeta; //i��  i��
Volt_Components 	MotorLVol_ab;// ��PARK�任��  V��  V��
Trig_Components  	MotorLVectorComponents; // 
/////////////////////////
u8 MotorLErrorPosFlag;//����λ�ù���
u8 MotorLDirFlagLast;	//��һ�λ����ı�ķ���
u8 MotorLCurHall;			//��ǰ����
u8 MotorLLastHall;		//ǰ�λ���
u8 MotorLHallCnt;			//����������

u8 MotorLFlagHallChange;	//�����л���־λ
u8 MotorLFlagPole6OK;			//����6�����ok
u16 MotorLTimePole=0xafc;	//�����л�һ��
u16 MotorLPosCur;					//������ǰλ��
u16 MotorLElectricalAngleBak;	//��Ƕ����
u16 MotorLUnknownPara1;
s16 MotorLEleAngleDiff;
u16 MotorLRollTimeLast;
u8 MotorLFlagShake;
u8 MotorLEleAnglePara1;
u8 MotorLPoleCnt;
u8 MotorLMoveCnt;
u8 MotorLFlagMove;
u16 MotorLTimePole6;//6�λ����л�����ʱ��
u16 MotorLFlagDir;

u16 MotorLEleAngleParaA;
u16 MotorLEleAngleParaC;
u16 MotorLElectricalAngle;
u16 MotorLEleAngleParaD;
u16 MotorLEleAngleParaE;
u16 MotorLEleAngleParaF;
u16 MotorLEleAngleParaG;

u16 MotorLFinalEleAngle;
u16 MotorLTimePoleBuf[6];

void MotorLInitParaFun(void);
void MotorLFlowRegFun(void);
void MotorLClarkeFun(void);
void MotorLParkFun(void);
void MotorLRevParkFun(void);
void MotorLMotorCtrlFun(void);
void MotorLFetAngleFun(void); // weak
//����һ�˵ĵ��
//-------------------------------------------------------------
void MotorLFetAngleFun()
{//���ݵ�ǰ��λ ��ȡ��Ƕ� 64usִ��һ��
	s16 result;
  u32 sum; 
  signed int i; 
	s8 pos_diff;

  if ( MotorLTimePole < 2812 )//�л�һ������ʱ�� С��180ms
    ++MotorLTimePole;
  if ( MotorLTimePole >= 2812 )//����180ms ����ֹͣ
  {//����ʱ����� �����־λ
    MotorLTimePole6 			= 16872;//6�λ����л�����ʱ��
    MotorLFlagHallChange = 0;		//�����л�
    MotorLFlagPole6OK 		= 0;		//���ֹ���һ�ܱ�־λ
    MotorLPoleCnt 			= 0;		//�����
  }
  if ( (MotorLTimePole6 > 1013) //6�λ����л�����ʱ�� > 1013 ���64ms
		|| ((5 * MotorLTimePole) > 1012) ) //5�λ����ʱ�� > 1012 ���64ms
    MotorLEleAngleParaC = MotorLEleAngleParaG;//�Ƕ�
	
	//----------����Ƿ��ƶ�------------------
  if ( (MotorLTimePole6 <= 1350) 		//6�λ����л�����ʱ�� < 1013 ���86ms
		&& (6 * MotorLTimePole <= 1350) )//6�λ����ʱ�� < 1013 ���86ms
  {
    if ( MotorLMoveCnt >= 6 )
    {
      MotorLFlagMove = 0;//�����ƶ�
      //ram_0C8 = 0;
    }
  }
  else
  {
    MotorLFlagMove = 1;//����δ�ƶ�
    MotorLMoveCnt = 0;
  }
	
  if ( MotorLFlagDir )
  {
		//---------�������ת������Ƕ�-----------------------
    if ( (MotorLPosCur == 6) && (MotorLEleAngleParaC != 0xffff) )
    {
      if ( MotorLFlagShake )
      {
        if ( (MotorLEleAngleParaC - MotorLUnknownPara1) <= MotorLElectricalAngleBak )
          MotorLElectricalAngleBak = MotorLEleAngleParaC;
        else
          MotorLElectricalAngleBak += MotorLUnknownPara1;
      }
      else
      {
        if ( (0xFFFF - MotorLUnknownPara1) < MotorLElectricalAngleBak )
          MotorLFlagShake = 1;
        MotorLElectricalAngleBak += MotorLUnknownPara1;
      }
    }
    else if ( MotorLEleAnglePara1 )
    {
      if ( (0xFFFF - MotorLUnknownPara1) < MotorLElectricalAngleBak )
        MotorLEleAnglePara1 = 0;
      MotorLElectricalAngleBak += MotorLUnknownPara1;
    }
    else if ( MotorLEleAngleParaC - MotorLUnknownPara1 <= MotorLElectricalAngleBak )
    {
      MotorLElectricalAngleBak = MotorLEleAngleParaC;
    }
    else
    {
      MotorLElectricalAngleBak += MotorLUnknownPara1;
    }
  }
  else
	{
		//---------�������ת������Ƕ�-----------------------
		if ( (MotorLPosCur == 1) && MotorLEleAngleParaC )
		{
			if ( MotorLFlagShake )
			{
				if ( (MotorLEleAngleParaC + MotorLUnknownPara1) >= MotorLElectricalAngleBak )
					MotorLElectricalAngleBak = MotorLEleAngleParaC;
				else
					MotorLElectricalAngleBak -= MotorLUnknownPara1;
			}
			else
			{
				if ( MotorLElectricalAngleBak < (signed int)MotorLUnknownPara1 )
					MotorLFlagShake = 1;
				MotorLElectricalAngleBak -= MotorLUnknownPara1;
			}
		}
		else if ( MotorLEleAnglePara1 )
		{
			if ( MotorLElectricalAngleBak < (signed int)MotorLUnknownPara1 )
				MotorLEleAnglePara1 = 0;
			MotorLElectricalAngleBak -= MotorLUnknownPara1;
		}
		else if ( (MotorLEleAngleParaC + MotorLUnknownPara1) >= MotorLElectricalAngleBak )
		{
			MotorLElectricalAngleBak = MotorLEleAngleParaC;
		}
		else
		{
			MotorLElectricalAngleBak -= MotorLUnknownPara1;
		}
	}
	//-----------����-------------------
  MotorLCurHall = (GPIOB->IDR >> 5) & 7;//����״̬
  if ( MotorLCurHall && (MotorLCurHall != 7) )
  {//����״̬��ȷ ��λ��ȷ
    MotorLHallCnt = 0;//����
  }
  else if ( MotorLHallCnt < 30 )
  {
    ++MotorLHallCnt;//������״̬ ������ 
    MotorLCurHall = MotorLLastHall;//����״̬����һ��״̬
  }
	//����״̬�л� 
  if ( MotorLCurHall != MotorLLastHall )
  {
    MotorLEleAnglePara1 = 0;
    MotorLFlagHallChange = 1;//�����л���־λ
		//-------�жϷ��� �����ת��ת------------
    pos_diff = MOTOR_PHASE_SEQUENCE[MotorLCurHall] - MotorLPosCur;
    if ( (pos_diff == 1) || (pos_diff == -5) )
    {
      MotorLFlagDir = 1;//�����־λ
    }
    else if ( (pos_diff == -1) || (pos_diff == 5) )
		{
      MotorLFlagDir = 0;//����
    }
    if ( MotorLDirFlagLast != MotorLFlagDir )
    {//����෴����ת
      MotorLTimePole6 = 16872;	//6�λ����л�����ʱ��
      MotorLFlagHallChange = 0;	//�����л���־λ
      MotorLFlagPole6OK = 0;			//���ֹ���һ�ܱ�־λ
      MotorLPoleCnt = 0;				//�����
    }
    MotorLDirFlagLast = MotorLFlagDir;//ǰһ�εķ���
		//--------�ƶ�������++----------------
    if ( MotorLMoveCnt < 6 )
      ++MotorLMoveCnt;
		
		//--------��������л���ʱ��----------------
    MotorLTimePoleBuf[MotorLPoleCnt] = MotorLTimePole;
    MotorLTimePole = 0;//����
		
    if ( MotorLFlagPole6OK )
    {//����6���л����
      sum = 0;
      for ( i = 0; i < 6; i = (u8)(i + 1) )
        sum += MotorLTimePoleBuf[i];//6�λ����л�����ʱ��
      MotorLTimePole6 = sum;				//6�λ����л�����ʱ��
    }
    else
    {//û�����6���л���� ����
      if ( MotorLFlagHallChange )
      {//�����л�
				//6�λ����л�����ʱ�䣬Ϊ6*���λ����л���ʱ��
        MotorLTimePole6 = 6 * MotorLTimePoleBuf[MotorLPoleCnt];
      }
      else
      {//����û���л������ϡ��ָ��Ĵ���
        MotorLTimePole6 = 16872;	//6�λ����л�����ʱ��
        MotorLFlagMove = 1;				//����δ�ƶ�
      }
      if ( MotorLPoleCnt == 5 )
        MotorLFlagPole6OK = 1;		//����6���л����
    }
    if ( MotorLPoleCnt >= 5 )
      MotorLPoleCnt = 0;
    else
      ++MotorLPoleCnt;//�����л��ƴ�
		
    if ( MotorLTimePole6 < 46 )		//6�λ����л�����ʱ�� ������� ���������
      MotorLTimePole6 = 46;
		
		//-------��ȡ��ǰ����λ��-----------------
    MotorLPosCur = MOTOR_PHASE_SEQUENCE[MotorLCurHall];
    MotorLLastHall = MotorLCurHall;//ǰһ�λ���״̬����
    //motorL_phase_bak_172 = MotorLCurHall;
		
    if ( MotorLPosCur == 0 )
    {//�������� �ر�pwm���
      MotorLErrorPosFlag = 1;
      TIM8->BDTR &= 0x7FFFu;//�ر�pwm
    }//����һ��
    else
    {
			//----------��ȡ��Ƕ�------------------
      if ( MotorLFlagDir )
      {//���� ����
				switch ( MotorLPosCur )//����λ��
				{
					case 1u://λ��1
						MotorLElectricalAngle = 0x1555;
						MotorLEleAngleParaA = 0;
						MotorLEleAngleParaE = 0xEC16;
						MotorLEleAngleParaD = 5097;
						MotorLEleAngleParaG = 10922;
						MotorLEleAngleParaF = 16019;
						if ( !MotorLFlagShake )
							MotorLEleAnglePara1 = 1;
						break;
					case 2u://λ��2
						MotorLElectricalAngle = 0x3FFF;
						MotorLEleAngleParaA = 0x2AAA;
						MotorLEleAngleParaE = 5825;
						MotorLEleAngleParaD = 16019;
						MotorLEleAngleParaG = 21845;
						MotorLEleAngleParaF = 26942;
						break;
					case 3u://λ��3
						MotorLElectricalAngle = 0x6AAA;
						MotorLEleAngleParaA = 0x5555;
						MotorLEleAngleParaE = 16748;
						MotorLEleAngleParaD = 26942;
						MotorLEleAngleParaG = 0x8000;
						MotorLEleAngleParaF = 0x93E9;
						break;
					case 4u://λ��4
						MotorLElectricalAngle = 0x9555;
						MotorLEleAngleParaA = 0x8000;
						MotorLEleAngleParaE = 27671;
						MotorLEleAngleParaD = 0x93E9;
						MotorLEleAngleParaG = 0xAAAA;
						MotorLEleAngleParaF = 0xBE93;
						break;
					case 5u://λ��5
						MotorLElectricalAngle = 0xBFFF;
						MotorLEleAngleParaA = 0xAAAA;
						MotorLEleAngleParaE = 0x96C1;
						MotorLEleAngleParaD = 0xBE93;
						MotorLEleAngleParaG = 0xD555;
						MotorLEleAngleParaF = 0xE93E;
						break;
					case 6u://λ��6
						MotorLElectricalAngle = 0xEAAA;
						MotorLEleAngleParaA = 0xD555;
						MotorLEleAngleParaE = 0xC16C;
						MotorLEleAngleParaD = 0xE93E;
						MotorLEleAngleParaG = 0xFFFF;
						MotorLEleAngleParaF = 5097;
						break;
					case 0u:
						break;
				}
        MotorLEleAngleDiff = MotorLEleAngleParaA - MotorLElectricalAngleBak;
      }
      else
      {
				switch ( MotorLPosCur )
				{
					case 1u://λ��1
						MotorLElectricalAngle = 0x1555;//��Ƕ�
						MotorLEleAngleParaA = 0x2AAA;
						MotorLEleAngleParaE = 5825;
						MotorLEleAngleParaD = 16019;
						MotorLEleAngleParaG = 0;
						MotorLEleAngleParaF = 0xEC16;
						break;
					case 2u://λ��2
						MotorLElectricalAngle = 0x3FFF;
						MotorLEleAngleParaA = 0x5555;
						MotorLEleAngleParaE = 16748;
						MotorLEleAngleParaD = 26942;
						MotorLEleAngleParaG = 10922;
						MotorLEleAngleParaF = 5825;
						break;
					case 3u://λ��3
						MotorLElectricalAngle = 0x6AAA;
						MotorLEleAngleParaA = 0x8000;
						MotorLEleAngleParaE = 27671;
						MotorLEleAngleParaD = 0x93E9;
						MotorLEleAngleParaG = 21845;
						MotorLEleAngleParaF = 16748;
						break;
					case 4u://λ��4
						MotorLElectricalAngle = 0x9555;
						MotorLEleAngleParaA = 0xAAAA;
						MotorLEleAngleParaE = 0x96C1;
						MotorLEleAngleParaD = 0xBE93;
						MotorLEleAngleParaG = 0x8000;
						MotorLEleAngleParaF = 27671;
						break;
					case 5u://λ��5
						MotorLElectricalAngle = 0xBFFF;
						MotorLEleAngleParaA = 0xD555;
						MotorLEleAngleParaE = 0xC16C;
						MotorLEleAngleParaD = 0xE93E;
						MotorLEleAngleParaG = 0xAAAA;
						MotorLEleAngleParaF = 0x96C1;
						break;
					case 6u://λ��6
						MotorLElectricalAngle = 0xEAAA;
						MotorLEleAngleParaA = 0xFFFF;
						MotorLEleAngleParaE = 0xEC16;
						MotorLEleAngleParaD = 5097;
						MotorLEleAngleParaG = 0xD555;
						MotorLEleAngleParaF = 0xC16C;
						if ( !MotorLFlagShake )
							MotorLEleAnglePara1 = 1;
						break;
					case 0u:
						break;
				}
        MotorLEleAngleDiff = MotorLElectricalAngleBak - MotorLEleAngleParaA;
      }
      if ( (MotorLTimePole6 > 1013) || (MotorLFlagPole6OK==0) )
      {//6�λ���ʱ�䡢û�����6���л�
        MotorLElectricalAngleBak = MotorLEleAngleParaA;
        MotorLEleAngleParaC = MotorLEleAngleParaG;
        MotorLEleAngleDiff = 0;
        MotorLEleAnglePara1 = 0;
      }
      else
      {
        MotorLEleAngleParaC = MotorLEleAngleParaF;
        if ( MotorLFlagDir )
        {//����
          if ( MotorLPosCur == 1 )
          {//����λ��1
            if ( MotorLFlagShake )
            {
              if ( MotorLElectricalAngleBak > (signed int)MotorLEleAngleParaD )
              {
                MotorLElectricalAngleBak = MotorLEleAngleParaA;
                MotorLEleAngleDiff = 0;
                MotorLEleAnglePara1 = 0;
              }
            }
            else if ( MotorLElectricalAngleBak < (signed int)MotorLEleAngleParaE )
            {
              MotorLElectricalAngleBak = MotorLEleAngleParaA;
              MotorLEleAngleDiff = 0;
              MotorLEleAnglePara1 = 0;
            }
          }
          else
          {
            if ( (MotorLPosCur == 6) && ((signed int)MotorLTimePole6 > 1013) )
              MotorLEleAngleParaC = 0xffff;
            if ( (MotorLElectricalAngleBak > (signed int)MotorLEleAngleParaD) || (MotorLElectricalAngleBak < (signed int)MotorLEleAngleParaE) )
            {
              MotorLElectricalAngleBak = MotorLEleAngleParaA;
              MotorLEleAngleDiff = 0;
              MotorLEleAnglePara1 = 0;
            }
          }
        }
        else
				{//����
					if ( MotorLPosCur == 6 )
					{//����λ��6
						if ( MotorLFlagShake )
						{
							if ( MotorLElectricalAngleBak < (signed int)MotorLEleAngleParaE )
							{
								MotorLElectricalAngleBak = MotorLEleAngleParaA;
								MotorLEleAngleDiff = 0;
								MotorLEleAnglePara1 = 0;
							}
						}
						else if ( MotorLElectricalAngleBak > (signed int)MotorLEleAngleParaD )
						{
							MotorLElectricalAngleBak = MotorLEleAngleParaA;
							MotorLEleAngleDiff = 0;
							MotorLEleAnglePara1 = 0;
						}
					}
					else
					{
						if ( (MotorLPosCur == 1) && ((signed int)MotorLTimePole6 > 1013) )
							MotorLEleAngleParaC = 0;
						if ( (MotorLElectricalAngleBak > (signed int)MotorLEleAngleParaD) || (MotorLElectricalAngleBak < (signed int)MotorLEleAngleParaE) )
						{
							MotorLElectricalAngleBak = MotorLEleAngleParaA;
							MotorLEleAngleDiff = 0;
							MotorLEleAnglePara1 = 0;
						}
					}
				}
      }
    }
    if ( MotorLTimePole6 <= 1013 )//����6��ʱ��
      result = (MotorLTimePole6 - MotorLRollTimeLast) >> 3;
    else
      result = 0;
    MotorLUnknownPara1 = (MotorLEleAngleDiff + 0xFFFF + ((MotorLTimePole6 + result) >> 1))
																/ (MotorLTimePole6 + result);
    MotorLFlagShake = 0;
    MotorLRollTimeLast = MotorLTimePole6;//6�λ����л�����ʱ�� ����
  }
	//-------���յ�Ƕ�-----------------
  if ( MotorLFlagMove )//δ�ƶ�
    MotorLElectricalAngleBak = MotorLElectricalAngle;//
  MotorLFinalEleAngle = MotorLElectricalAngleBak;
  if ( MotorLFlagMove )
  {//����δ�ƶ�
    MotorLFinalEleAngle += 5916;//�Ƕ�����
  }
  else if ( MotorLFlagDir )
  {//����
    MotorLFinalEleAngle += 6735;//
  }
  else
  {//����
    MotorLFinalEleAngle += 5097;
  }
}

//-------------------------------------------------------------
void MotorLInitParaFun()
{
  MotorLFlowIntergral = 0;
  MotorLCurrqd = 0;
  MotorLAtatVolt_qd.qV_Component2 = 0;
}

//-------------------------------------------------------------
void MotorLFlowRegFun()
{//������������ ����ο���ѹvds 
	s16 q_comp1_val; //
  s16 diff; //
  s32 rst; // 

  q_comp1_val = -MotorLCurrqd;
  if ( q_comp1_val > 0 )
    diff = 1;
  else if ( q_comp1_val < 0 )
		diff = -1;
	else
		diff = 0;
	
  MotorLFlowIntergral += diff;
  rst = MotorLFlowIntergral + (q_comp1_val >> 2);
  if ( rst < -0x4000 )
  {
    rst = -16384;
    MotorLFlowIntergral = 0xC000;
  }
  else if ( rst > 0x4000 )
  {
    rst = 0x4000;
    MotorLFlowIntergral = 0x4000;
  }
  MotorLAtatVolt_qd.qV_Component2 = rst >> 6;// ����/64
}

//-------------------------------------------------------------
void MotorLClarkeFun()
{//����任 
  MotorLCurrAlfaBeta.qI_Component1 = MotorLCurr_ab.qI_Component1;
	MotorLCurrAlfaBeta.qI_Component2 = 18918 * (MotorLCurr_ab.qI_Component1 
										+ 2 * MotorLCurr_ab.qI_Component2) >> 15;
}

//-------------------------------------------------------------
void MotorLParkFun()
{
	s32 result; //

  result = (MotorLCurrAlfaBeta.qI_Component1 * MotorLVectorComponents.hCos
						+ MotorLCurrAlfaBeta.qI_Component2 * MotorLVectorComponents.hSin)
						>> 15;
  MotorLCurrqd = (20317 * MotorLCurrqd 
															+ 12451 * result) 
															>> 15;
}

//-------------------------------------------------------------
void MotorLRevParkFun()
{//��park�任 	
  MotorLVectorComponents.hSin = get_sin(MotorLFinalEleAngle);
  MotorLVectorComponents.hCos = get_cos(MotorLFinalEleAngle);
  MotorLVol_ab.qV_Component2 = 
		(MotorLAtatVolt_qd.qV_Component2 * MotorLVectorComponents.hCos 
		- MotorLAtatVolt_qd.qV_Component1 * MotorLVectorComponents.hSin) >> 15;
  MotorLVol_ab.qV_Component1 = 
		(MotorLAtatVolt_qd.qV_Component1 * MotorLVectorComponents.hCos 
		+ MotorLAtatVolt_qd.qV_Component2 * MotorLVectorComponents.hSin) >> 15;
}

//-------------------------------------------------------------
//SVPWM
void MotorLMotorCtrlFun()
{// ����pwm wZ wY wX�п��ܲ��ԣ���λ�� �������ǶԵ�
  s16 hTimePhA; 
	s16 hTimePhB;
	s16 hTimePhC; 
	
  u16 bSector; 
	
  s16 wX; 
  s16 wZ; 
  s16 wY;

	//Clarke��任
	wX = MotorLVol_ab.qV_Component1;
  wZ = (s16)-(MotorLVol_ab.qV_Component1 + 
										((3547 * MotorLVol_ab.qV_Component2) >> 11));
  wY = (s16)(((3547 * MotorLVol_ab.qV_Component2) >> 11) 
										- MotorLVol_ab.qV_Component1);
	
	
	bSector = 0;
  if ( wX > 0 )
    bSector = 1;
  if ( wY > 0 )
    bSector = (u16)(bSector + 2);
  if ( wZ > 0 )
    bSector = (u16)(bSector + 4);
	
	wX = (255 *       wX       ) >> 10;
  wZ = (255 * ( ( -wZ ) >> 1)) >> 10;
  wY = (255 * ( ( -wY ) >> 1)) >> 10;
  
	switch ( bSector )
	{
		case 0:
			hTimePhA = 255;
			hTimePhB = 255;
			hTimePhC = 255;
			break;
		case 1:
			hTimePhA = (s16)(255 - wY + wZ);
			hTimePhB = (s16)(wY + 255 + wZ);
			hTimePhC = (s16)(255 - wY - wZ);
			break;
		case 2:
			hTimePhA = (s16)(255 - wX + wZ);
			hTimePhB = (s16)(255 - wZ + wX);
			hTimePhC = (s16)(255 - wZ - wX);
			break;
		case 3:
			hTimePhA = (s16)(255 - wY + wX);
			hTimePhB = (s16)(wY + 255 + wX);
			hTimePhC = (s16)(255 - wX + wY);
			break;
		case 4:
			hTimePhA = (s16)(255 - wY + wX);
			hTimePhB = (s16)(wY + 255 + wX);
			hTimePhC = (s16)(255 - wX + wY);
			break;
		case 5:
			hTimePhA = (s16)(255 - wX + wZ);
			hTimePhB = (s16)(255 - wZ + wX);
			hTimePhC = (s16)(255 - wZ - wX);
			break;
		case 6:
			hTimePhA = (s16)(255 - wY + wZ);
			hTimePhB = (s16)(wY + 255 + wZ);
			hTimePhC = (s16)(255 - wY - wZ);
			break;
		default:
			break;
	}
  if ( hTimePhA < 0 )
    (hTimePhA) = 0;
  if ( hTimePhB < 0 )
    (hTimePhB) = 0;
  if ( hTimePhC < 0 )
    (hTimePhC) = 0;
  TIM8->CCR1 = hTimePhA;
  TIM8->CCR2 = hTimePhB;
  TIM8->CCR3 = hTimePhC;
}

