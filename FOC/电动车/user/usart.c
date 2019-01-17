/***********************************************************
*	ͨѶ
* ���ߡ����ߵ���Ľ��պͷ��ͣ��Լ���⸱��״̬
*
*
************************************************************/
#include "extern.h"


u8 UsartLRemoteVal;
u8 UsartSRemoteVal;
u8 UsartStateLTurn;//ת���״̬
u8 UsartStateSTurn;
u8 UsartStateBatteryBak2;
u8 UsartStateBatteryBak;
u8 UsartStateBatteryLow;
u8 UsartState_state_8;
u8 UsartStateOK;
u8 UsartStateOverAngleBak;
u8 UsartFlagMpuLErrorCnt;
u8 UsartFlagMpuSErrorCnt;

s16 UsartGyroZCtrl;// z�����
s16 UsartMpuSGyroZ;// z�����ݣ���ȡ����һ�ߵ�

u8 UsartError_angle;//��̬��Ƕȹ���
u8 UsartLCnt;
u8 UsartMpuPHOLCnt;
u8 UsartLRecBuf[8];

u16 UsartMpuLTimeOut;
u8 UsartLRemoteValBak;
float UsartMpuLValLast;
float UsartMpuLVal;
u8 UsartSCnt;
u8 UsartMpuPHOSCnt;
u8 UsartSRecBuf[8];//ͨѶ3����
u32 UsartMpuSTimeOut;
u8 UsartSRemoteValBak;
u8 UsartLSendBuf[6];
u8 UsartStateBattery2;
u8 UsartStateBattery1;
u8 UsartSSendBuf[6];
u8 UsartLSendCnt;
u8 UsartSSendCnt;
u8 UsartMpuData[2];
float UsartMpuLFinalVal;//��̬�����ĽǶ�
float UsartMpuSValLast;
float UsartMpuSFinalVal;//��̬�崫���ĽǶ�
u8 UsartMpuLError;
u8 UsartMpuSError;
u8 UsartMpuPHOL;
u8 UsartMpuPHOS;



u8 UsartStatemotorSOk;//���������־ out
u8 UsartStatemotorLOk;//
u8 UsartFlagLowAndPhoOn;//���ʱ ����̤��
u8 UsartErrorOverAngle;
u8 UsartFlagOverSpeed;
u8 UsartMpuPHOLLast;
u8 UsartMpuPHOSLast;
u8 UsartFlagMpuPHOSFirst;
u8 UsartFlagMpuPHOLFirst;


void UsartLGetFun(void);//��ȡ����ͨѶ
void UsartSGetFun(void);//��ȡ����ͨѶ
void UsartSendFun(void);//����
void UsartGetStateCtrlLed(void);//��ȡ��ǰ״̬���͸�С�壬����led
void UsartCheckAttitudeFun(void);//��鸱��
void UsartCheckSpeedFun(void);


//-------------------------------------------------------------
void UsartLGetFun()
{//����USART2���� ����
	//�޸����Ϊ��ƥ���鶯΢����
	u16 	v_data; 
	float f_val;
  s16		v_mpu;

  if ( USART2->SR & 0x20 )
  {
    v_data = USART2->DR;
    UsartLRecBuf[UsartLCnt] = v_data;
    ++UsartLCnt;
    //if ( v_data & 0x100 )
    //{
      if ( UsartLCnt == 8 )
      {//�����������
				//ң������ȡ
				if( (UsartLRecBuf[0] & 0xf) == UsartLRemoteValBak)
				{
					UsartLRemoteVal = UsartLRecBuf[0] & 0xf;//ң����
				}
				UsartLRemoteValBak = UsartLRecBuf[0] & 0xf;//����
				//----------��̬�Ƕ�-------------
        UsartMpuData[0] = UsartLRecBuf[2];
        UsartMpuData[1] = UsartLRecBuf[3];
        v_mpu = *(s16*)UsartMpuData;
        UsartMpuData[0] = UsartLRecBuf[4];
        UsartMpuData[1] = UsartLRecBuf[5];
        if ( v_mpu == *(s16*)UsartMpuData )
        {
					//���νǶ�����һ�νǶȾ�ֵ
					f_val = v_mpu / 100.0f;
					UsartMpuLVal = f_val;
					UsartMpuLVal = UsartMpuLVal / 2.0f;
					UsartMpuLVal = UsartMpuLValLast / 2.0f + UsartMpuLVal;
					UsartMpuLValLast = f_val;
					UsartMpuLFinalVal = -UsartMpuLVal;//�������� ��Ϊ��װ����ͬ ����һ��һ��
					//----------���----------------
					if ( UsartLRecBuf[1] == 0x55 )
					{
						UsartMpuPHOL = 1;//��翪��״̬
						UsartMpuPHOLCnt = 0;
					}
					else if ( UsartMpuPHOLCnt >= 8 )
					{
						UsartMpuPHOL = 0;//����
					}
					else
					{
						++UsartMpuPHOLCnt;
					}
				
				}
        else
        {
          UsartFlagMpuLErrorCnt = 1;//ͨѶ����
        }
				UsartLCnt = 0;
      }
			else if((UsartLCnt == 1) && ((UsartLRecBuf[0] & 0xf0) != 0xc0))
			{
				UsartLCnt = 0;
			}
			else if(UsartLCnt == 2)
			{
				if((UsartLRecBuf[1] != 0x55) && (UsartLRecBuf[1] != 0xaa))
				{
					UsartLCnt = 0;
				}
			}
    //}
			UsartMpuLTimeOut = 0;//��ʱ������
  }
  else if ( UsartMpuLTimeOut >= 30000 )//�����˳�ʱʱ��
  {//���������й���
    UsartLRemoteVal = 0;//ң������ֵ
    UsartMpuLError = 1;	//����һ�ั�����ӹ���
    TIM1->BDTR &= 0x7FFFu;	//�ر�pwm
    TIM8->BDTR &= 0x7FFFu;	//�ر�pwm
  }
  else
  {
    ++UsartMpuLTimeOut;
  }
}

//-------------------------------------------------------------
void UsartSGetFun()
{//����
	//20170419�޸�ƥ���鶯΢����
	u16 	v_data; 
	float f_val;
  s16 	v_mpu; 

  if ( USART3->SR & 0x20 )
  {
    v_data = USART3->DR;
    UsartSRecBuf[UsartSCnt] = v_data;
    ++UsartSCnt;
		
    //if ( v_data & 0x100 )
    //{//��ʼ��ַ �ĳ�8λ���ˣ�����ȥ��
		if ( UsartSCnt == 8 )
		{
			//ң����
			if ( (UsartSRecBuf[0] & 0xF) == UsartSRemoteValBak )
			{
				UsartSRemoteVal = UsartSRecBuf[0] & 0xF;
			}
			UsartSRemoteValBak = UsartSRecBuf[0] & 0xF;
			//��̬��Ƕ�
			UsartMpuData[0] = UsartSRecBuf[2];
			UsartMpuData[1] = UsartSRecBuf[3];
			v_mpu = *(s16*)UsartMpuData;
			UsartMpuData[0] = UsartSRecBuf[4];
			UsartMpuData[1] = UsartSRecBuf[5];
			if ( v_mpu == *(s16*)UsartMpuData )
			{//ͨѶ�������ֽ�������ͬ ��ȷ��Ϊ�µĽǶ�
				f_val = v_mpu / 100.0f;
				UsartMpuSFinalVal = f_val;
				UsartMpuSFinalVal = UsartMpuSFinalVal / 2.0f;
				UsartMpuSFinalVal = UsartMpuSValLast / 2.0f + UsartMpuSFinalVal;
				UsartMpuSValLast = f_val;
				
				//��翪��
				if ( UsartSRecBuf[1] == 0x55 )
				{//��̤����
					UsartMpuPHOS = 1;
					UsartMpuPHOSCnt = 0;
				}
				else if ( UsartMpuPHOSCnt >= 8 )
				{
					UsartMpuPHOS = 0;
				}
				else
				{
					++UsartMpuPHOSCnt;
				}
				//ֻ���˶���һ�˵�������z��У׼����Ƕ�
				if(UsartSRecBuf[6] == UsartSRecBuf[7])
				{
					UsartMpuSGyroZ = UsartSRecBuf[6] - 0x58;
				}
			}
			else
			{
				UsartFlagMpuSErrorCnt = 1;
			}
      UsartSCnt = 0;
      //}
    }
		else if((UsartSCnt == 1) && ((UsartSRecBuf[0]&0xf0) != 0xc0))
		{//0xc0��ʼ
				UsartSCnt = 0;
		}
		else if(UsartSCnt == 2)
		{
			if((UsartSRecBuf[1] != 0x55) && (UsartSRecBuf[1] != 0xaa))
			{
				UsartSCnt = 0;
			}
		}
		UsartMpuSTimeOut = 0;
  }
  else if ( UsartMpuSTimeOut >= 30000 )
  {
    UsartSRemoteVal = 0;	//ң����
    UsartMpuSError = 1;		//����һ�ั�����ӹ���
    TIM1->BDTR &= 0x7FFFu;		//�ر�pwm
    TIM8->BDTR &= 0x7FFFu;		//�ر�pwm
  }
  else
  {
    ++UsartMpuSTimeOut;
  }
}

//-------------------------------------------------------------
void UsartSendFun()
{//ͨѶ����
	//�����巢��״̬ ��ӵ����������ԭ�滹��û�е�20170323
	//MotorSFlagDir ����
	//MotorSFlagMove 0�ƶ� 1δ�ƶ�
	//MotorLFlagDir ����
	//MotorLFlagMove 0�ƶ� 1δ�ƶ�
	
	//-----����--------------------------
  if ( UsartLSendCnt >= 5 )
  {
    UsartLSendCnt = 0;
    UsartLSendBuf[0] = 0;
    UsartLSendBuf[1] = 0;
    UsartLSendBuf[2] = 0;
    UsartLSendBuf[3] = 0;
    UsartLSendBuf[4] = 0;
    UsartLSendBuf[5] = 0;
    if ( UsartStateSTurn )//ת��״̬ ����led
      UsartLSendBuf[0] |= 1u;
    if ( UsartStateBatteryBak2 )//���״̬
      UsartLSendBuf[0] |= 2u;
    if ( UsartStateBatteryBak )//���״̬
      UsartLSendBuf[0] |= 4u;
    if ( UsartStateBatteryLow )//��ѹ�͵�ʱ����¹��
      UsartLSendBuf[0] |= 8u;
    if ( BatteryFlagLow )//��ص�ѹ��
      UsartLSendBuf[0] |= 0x10u;
    if ( RemoteFlagLockOut )//ң�����ⲿʹ�õ�����״̬
      UsartLSendBuf[0] |= 0x20u;
    if ( RemoteFlagCorrectOut )//У׼
      UsartLSendBuf[3] = 40;
		//UsartLSendBuf[4] = MotorSFlagDir;
		//UsartLSendBuf[5] = !MotorSFlagMove;
  }
  else
  {
    ++UsartLSendCnt;
  }
	switch ( UsartLSendCnt )
	{
		case 0:
			USART2->DR = UsartLSendBuf[0] | 0x100;//֡ͷ
			break;
		case 1u:
			USART2->DR = UsartLSendBuf[1];
			break;
		case 2u:
			USART2->DR = UsartLSendBuf[2];
			break;
		case 3u:
			USART2->DR = UsartLSendBuf[3];
			break;
		case 4u:
			USART2->DR = UsartLSendBuf[4];
			break;
		case 5u:
			USART2->DR = UsartLSendBuf[5];
			break;
	}
	
	//-----����--------------------------
  if ( (signed int)UsartSSendCnt >= 5 )
  {
    UsartSSendCnt = 0;
    UsartSSendBuf[0] = 0;
    UsartSSendBuf[1] = 0;
    UsartSSendBuf[2] = 0;
    UsartSSendBuf[3] = 0;
    UsartSSendBuf[4] = 0;
    UsartSSendBuf[5] = 0;
    if ( UsartStateLTurn )//ת��״̬ ����led
      UsartSSendBuf[0] |= 1u;
    if ( UsartState_state_8 )
      UsartSSendBuf[0] |= 2u;
    if ( UsartStateOK )//�������ok��û���ϣ�����������ת
      UsartSSendBuf[0] |= 4u;
    if ( UsartStateOverAngleBak )//�й���
      UsartSSendBuf[0] |= 8u;
    if ( BatteryFlagLow )//��ص�ѹ��
      UsartSSendBuf[0] |= 0x10u;
    if ( RemoteFlagLockOut )//ң�����ⲿʹ�õ�����״̬
      UsartSSendBuf[0] |= 0x20u;
    if ( MotorSErrorPosFlag )
    {
      UsartSSendBuf[2] = 4;
    }
    else if ( MotorLErrorPosFlag )
    {//���߻�������
      UsartSSendBuf[2] = 5;
    }
    else if ( FocFlagMotorSErrorElVol )
    {//����һ�ྲ̬��������
      UsartSSendBuf[2] = 1;
    }
    else if ( FocFlagMotorLErrorElVol )
    {//����һ�ྲ̬��������
      UsartSSendBuf[2] = 2;
    }
    else if ( ErrorCircuit )
    {//��������ж�·
      UsartSSendBuf[2] = 3;
    }
    else if ( BatteryFlagError )
    {//��ѹ���ϱ�־
      UsartSSendBuf[2] = 6;
    }
    else if ( UsartMpuLError )
    {//����һ�ั�����ӹ���
      UsartSSendBuf[2] = 7;
    }
    else if ( UsartMpuSError )
    {//����һ�ั�����ӹ���
      UsartSSendBuf[2] = 8;
    }
    if ( RemoteFlagCorrectOut )//У׼
      UsartSSendBuf[3] = 40;
		//UsartSSendBuf[4] = MotorLFlagDir;
		//UsartSSendBuf[5] = !MotorLFlagMove;
  }
  else
  {
    ++UsartSSendCnt;
  }
	switch ( UsartSSendCnt )
	{
		case 0u:
			USART3->DR = UsartSSendBuf[0] | 0x100;
			break;
		case 1u:
			USART3->DR = UsartSSendBuf[1];
			break;
		case 2u:
			USART3->DR = UsartSSendBuf[2];
			break;
		case 3u:
			USART3->DR = UsartSSendBuf[3];
			break;
		case 4u:
			USART3->DR = UsartSSendBuf[4];
			break;
		case 5u:
			USART3->DR = UsartSSendBuf[5];
			break;
	}
}

//-------------------------------------------------------------
void UsartGetStateCtrlLed()
{//��ȡ��ǰ״̬���͸�С��
  if ( IDCal6 == IDCal8 )
  {//��������
		//--------ת��ʱ��ĵ�-----------------------------
    if ( (MotorLTimePole6 <= 1350) || (MotorSTimePole6 <= 1350) )
    {//������������ʱ��С��Ҳ���ǵ������һ�ߵ�����ٶȣ��������
      if ( MotorSFlagDir == MotorLFlagDir )
      {//����ͬ
        if ( MotorSFlagDir == MotorLFlagDir )
        {//����ͬ
					//���ߵ��
					if(MotorLTimePole6 < 1350 ) UsartStateLTurn = 1;//������ת���ʱ���
					else UsartStateLTurn = 0;
					if(MotorSTimePole6 < 1350 ) UsartStateSTurn = 1;
					else UsartStateSTurn = 0;
        }
        else
        {//������ͬ
          UsartStateLTurn = 0;
          UsartStateSTurn = 0;
        }
      }
			else//���������ͬ
			{
				if( MotorLTimePole6 > ( MotorSTimePole6 + 88 ) )
				{
          UsartStateLTurn = 1;//�����ٶȿ�
          UsartStateSTurn = 0;
				}	
				else if ( MotorSTimePole6 > ( MotorLTimePole6 + 88 ) )
        {
          UsartStateLTurn = 0;
          UsartStateSTurn = 1;//�����ٶȿ�
        }
        else
        {
          UsartStateLTurn = 0;
          UsartStateSTurn = 0;
        }
			}
    }
    else
    {
      UsartStateLTurn = 0;
      UsartStateSTurn = 0;
    }
		//--------��ѹ��-----------------------------
    if ( BatteryFlagLow )//��ص�ѹ��
    {//��ѹ��
      UsartStateBattery1 = 0;
      if ( UsartFlagLowAndPhoOn )
      {//��ѹ�͵�ʱ�� ���¹��
        UsartStateBatteryLow = 1;//��ѹ�͵�ʱ����¹��
        UsartStateBattery2 = 0;
      }
      else
      {//
        UsartStateBatteryLow = 0;//�������
        if ( BatteryVal1 >= 1634 )
          UsartStateBattery2 = 1;
        else
          UsartStateBattery2 ^= 1u;
      }
    }
    else
    {//�ǳ��״̬
      UsartStateBattery2 = 0;
      if ( BatteryFlagVolLowest )
      {//��ѹ����
        UsartStateBatteryLow = 1;
        UsartStateBattery1 = 0;//��ѹ״̬
      }
      else
      {//��ѹ����
        UsartStateBatteryLow = 0;//
        if ( BatteryFlagVolLower == 0)
        {
          UsartStateBattery1 = 1;
        }
        else if ( BatteryFlagVolLower )
				{
					UsartStateBattery1 ^= 1u;
				}
				else
				{
          UsartStateBattery1 = 1;
        }
      }
    }
    UsartStateBatteryBak = UsartStateBattery1;//���״̬
    UsartStateBatteryBak2 = UsartStateBattery2;
    UsartStateOverAngleBak = 0;
    UsartStateOK = 0;
    if ( UsartErrorOverAngle )//�Ƕȹ���
    {
      UsartStateOverAngleBak = 1;
    }
    else if ( UsartStatemotorLOk || UsartStatemotorSOk )
    {
      UsartStateOK = 1;
    }
  }
}

//-------------------------------------------------------------
void UsartCheckAttitudeFun()
{//��鸱��
  if ( (BatteryFlagLow 			== 1) 	//��ص�ѹ��
		|| (BatteryFlagPowerOnOut 		== 0)		//��ѹ�ͱ�־ 
		|| (RemoteFlagCorrectOut 	== 1) 	//У׼��
		|| (RemoteFlagLockOut 			== 1) 	//ң�����ⲿʹ�õ�����״̬	
		|| (IDFlagWrite3 				== 0) )	//id���ܳ���
  {//������������
    UsartFlagMpuPHOSFirst = 0;
    TIM1->BDTR &= 0x7FFFu;//�ر�pwm
    UsartStatemotorLOk = 0;	//�޷�����
    UsartFlagMpuPHOLFirst = 0;
    TIM8->BDTR &= 0x7FFFu;//�ر�pwm
    UsartStatemotorSOk = 0;//�رյ�� 
		return;
  }
	//----------����--------------------
	if ( (UsartMpuPHOS == 1) && (UsartMpuPHOSLast==0) )
	{//����״α����� ��ִ��һ��
		UsartFlagMpuPHOSFirst = 0;//�տ���С��
	}
	else if ( (UsartMpuPHOSLast != 1) || (UsartMpuPHOS != 1) )
	{//���û�б�����
		if ( (UsartFlagMpuPHOSFirst == 1) && (!UsartMpuPHOL) )
		{//�����¹�� �ر�һֻpwm ��ִ��һ��
			UsartFlagMpuPHOSFirst = 0;
			TIM1->BDTR &= 0x7FFFu;//�ر�pwm
			UsartStatemotorLOk = 0;
			UsartErrorOverAngle = 0;
		}
	}
	else if ( !UsartFlagMpuPHOSFirst )
	{//��ִ��һ��
		if ( (MotorLErrorPosFlag)//��������
			|| (MotorSErrorPosFlag)
			|| (UsartMpuSError)
			|| (UsartMpuLError)//����һ�ั�����ӹ���
			|| (FocFlagMotorLErrorElVol)
			|| (FocFlagMotorSErrorElVol)
			|| (ErrorCircuit)			//��������ж�·
			|| (BatteryFlagError)
			|| (((UsartMpuSFinalVal < -15.0f) || (UsartMpuSFinalVal >= 15.0f)) //�Ƕȳ�����Χ
					&& (UsartStatemotorSOk == 0)) )
		{//�й���
			if( ( UsartMpuSFinalVal <= -15.0f ) || ( UsartMpuSFinalVal >= 15.0f ) )//�Ƕȳ�����Χ
			{
				if ( !UsartStatemotorSOk )//tim8�ر�
					UsartErrorOverAngle = 1;//�Ƕȹ���
			}
		}
		else
		{//û����������
			UsartFlagMpuPHOSFirst = 1;//��λ
			if ( !UsartStatemotorLOk )//�ر���tim1 ���¿���
				TIM1->BDTR |= 0x8000u;
			if ( !UsartStatemotorSOk )//�ر���tim8 ���¿���
				TIM8->BDTR |= 0x8000u;
			UsartStatemotorLOk = 1;//
			UsartErrorOverAngle = 0;
			UsartError_angle = 0;
		}
	}
	UsartMpuPHOSLast = UsartMpuPHOS;
	//----------����--------------------
	if ( (UsartMpuPHOL == 1) && (UsartMpuPHOLLast==0) )
	{
		UsartFlagMpuPHOLFirst = 0;//�״�
	}
	else if ( (UsartMpuPHOLLast != 1) || (UsartMpuPHOL != 1) )
	{
		if ( (UsartFlagMpuPHOLFirst == 1) && (!UsartMpuPHOS) )
		{//�ر�һֻpwm
			UsartFlagMpuPHOLFirst = 0;
			TIM8->BDTR &= 0x7FFFu;//�ر�pwm
			UsartStatemotorSOk = 0;
			UsartErrorOverAngle = 0;
		}
	}
	else if ( !UsartFlagMpuPHOLFirst )
	{
		if ( (MotorLErrorPosFlag)//��������
			|| (MotorSErrorPosFlag)
			|| (UsartMpuSError)
			|| (UsartMpuLError)
			|| (FocFlagMotorLErrorElVol)
			|| (FocFlagMotorSErrorElVol)
			|| (ErrorCircuit)			//��������ж�·
			|| (BatteryFlagError)
			|| ((((UsartMpuLFinalVal <= -15.0f)) || ((UsartMpuLFinalVal >= 15.0f))) 
			&& (UsartStatemotorLOk == 0)) )
		{//�й���
			if ( (UsartMpuLFinalVal <= -15.0f) || (UsartMpuLFinalVal >= 15.0f) )
			{//�Ƕȳ�����Χ
				if ( !UsartStatemotorLOk )
					UsartErrorOverAngle = 1;//�Ƕȳ�����Χ
			}
		}
		else
		{
			UsartFlagMpuPHOLFirst = 1;
			if ( !UsartStatemotorLOk )
				TIM1->BDTR |= 0x8000u;
			if ( !UsartStatemotorSOk )
				TIM8->BDTR |= 0x8000u;
			UsartStatemotorSOk = 1;
			UsartErrorOverAngle = 0;
			UsartError_angle = 0;//��̬��Ƕȹ���
		}
	}
	if ( (!UsartMpuPHOS) && (!UsartMpuPHOL) )
		UsartErrorOverAngle = 0;//��綼û�б����µ�ʱ�򣬽Ƕȹ���ı�־λ����
	UsartMpuPHOLLast = UsartMpuPHOL;
}



//-------------------------------------------------------------
void UsartCheckSpeedFun()
{
	//-------��ѹ�͵�ʱ�������Ƿ񱻰���-----------------
	if(BatteryFlagLow 											 //��ص�ѹ��
		&& (UsartMpuPHOS || UsartMpuPHOL)) //���û�в���
	{
		UsartFlagLowAndPhoOn = 1;	//��ص�ѹ�ص�ʱ���类����
	}
	else UsartFlagLowAndPhoOn = 0;
	//------------���ٱ���------------------------
  if ( MotorSFlagDir == MotorLFlagDir )
  {//�����ͬ�� û�г��ٱ���
    UsartFlagOverSpeed = 0;
  }
  else if ( ((MotorLTimePole6 + MotorSTimePole6) / 2) >= 168 )
  {
    UsartFlagOverSpeed = 0;//
  }
  else
  {//���ٱ���
    UsartFlagOverSpeed = 1;
  }
}


