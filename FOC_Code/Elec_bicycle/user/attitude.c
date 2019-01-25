/***********************************************************
*	��̬�Ƕȵ�����
*
*
*
************************************************************/
#include "extern.h"
#define PARA_PID_P		70.0f//70.0f
#define PID_PARA_KD		30.0f//30.0f
	

float AttMotorLAngle;//���������Ƕ� motor2
float AttMotorSAngle;//���������Ƕ� motor1

float AttDiffLast;
float AttMotorLCorparaAngle;//��������
float AttMotorSCorparaAngle;
float AttMotorSIntegral;//motor1������
float AttMotorLIntegral;//motor2������
float AttPidKi;//
float AttMotorSAngleLast;//��һ���Ƕ�
float AttMotorLEleAngleLast;//��һ���Ƕ�
float AttDirOffset;//



float attGetDiroffset(float acce, float offset);
void 	AttCorAngleFun(void);			//�����Ƕ�
int 	AttMotorSPidFun(void);		//motorS�Ƕ�pid�㷨
int 	AttMotorLPidFun(void);		//motorL�Ƕ�pid�㷨


//-------------------------------------------------------------
float attGetDiroffset(float acce, float offset)
{//֮ǰ�����ʱ�� ����ֵ��angle_dot_1DC��ֵ���� ���������Ӵ�
	//����� û������������
	//���㷽��ƫ��
	float f_diff;
	float f_frst;
	f_diff = acce - offset;
	f_frst = 0.002f * f_diff + 0.00003f * AttDiffLast;//ƫ���ǰһ��ƫ��
	AttDiffLast = f_diff;
	return f_frst;
}

//-------------------------------------------------------------
void AttCorAngleFun()
{//�Ƕ�����
	float f_acceler; 
  float f_MotorLEleAngle_cor; 
  float f_motorS_angle_cor; 
  float f_motorL_angle_final;
  float f_motorS_angle_final;
	u16 	v_roll_time;//������������ʱ�� �������޸�
	
	//-----------����ƫ�����--------------
  if ( ((MotorSFlagDir == 0) && (MotorLFlagDir == 1)) 
		|| ((MotorSFlagDir == 1) && (MotorLFlagDir == 0)) )
  {//���ͬ�� ��ǰ�������
		//v0 �߿��ʱ�� ���� �߿��Զ�̧ͷ
		//ƽʱתȦ��ʱ�� ���Ƿ��ֻ����һ�� ����ǰ��һ�� ���ֵ�ı�����
		//��ǰ�ٶ�̫����� ���Ժ��� �����ٶ�Ҳ̫�� ����һ������ǰ��
		//�����ǰ�����������
    v_roll_time = (MotorLTimePole6 + MotorSTimePole6) >> 1;//���λ���������ʱ��
    if ( (v_roll_time < 168) || BatteryFlagLowRunOut )
    {
      f_acceler = 5.8f;
    }
    else if ( v_roll_time < 253 )//�������޸ģ�֮ǰ��406 
    {
			//f_acceler = (405 - v_roll_time) * 0.0244725738f;////�ٶȴ����12�����ʱ�� ̧ͷ 5.8/(405-168) = 0.0244725738f
			f_acceler = (253 - v_roll_time) * 0.068235f;// 5.8/(253-168) = 0.068235f
    }
    else
    {
      f_acceler = 0;
    }
		/*//�Ƿ���6�����ٶȵ�ʱ��̧ͷ
		v_roll_time = (MotorLTimePole6 + MotorSTimePole6) >> 1;//���λ���������ʱ��
    if ( ((signed int)v_roll_time < 84) || BatteryFlagLowRunOut )
    {//
      f_acceler = 5.8f;
    }
    else if ( v_roll_time < 203 )
    {//����ֵ�ļ��� 5.8/(202-84)
			f_acceler = (202 - v_roll_time) * 0.04915254237288135f;//Ϊʲô�������� ����ֵ������
    }
    else
    {
      f_acceler = 0;
    }*/
  }
  else
  {
    f_acceler = 0;
    v_roll_time = 0xFFFF;
  }
  if ( (MotorSFlagDir == 1) && (MotorLFlagDir == 0) )
		f_acceler = -f_acceler;//����ǰ ���������
	//--------------��������ٶ�--------------
	AttDirOffset = attGetDiroffset(f_acceler, AttDirOffset) + AttDirOffset;//���㷽��ƫ��
	f_MotorLEleAngle_cor = UsartMpuLFinalVal + AttDirOffset;//�����Ƕ�����
	f_motorS_angle_cor = UsartMpuSFinalVal + AttDirOffset;
	//----������ ������z��------------------
	if ( (MotorSFlagDir == MotorLFlagDir)//������Ť����ʱ��
				|| MotorSFlagMove || MotorLFlagMove) 
  {
		if ( UsartGyroZCtrl < 0 )
    {
			++UsartGyroZCtrl;
    }
    else if ( UsartGyroZCtrl > 0 )
    {
      --UsartGyroZCtrl;
    }
	}
	else
	{//z�ᣬһ��һ��ı�
		if ( UsartGyroZCtrl > UsartMpuSGyroZ )
		{
			--UsartGyroZCtrl;
		}
		else if ( UsartGyroZCtrl < UsartMpuSGyroZ )
		{
			++UsartGyroZCtrl;
		}
	}
	//------�Ƕȳ���----------------------
  if ( (UsartMpuLFinalVal > 80.0f)//ͨѶ�������ĽǶ�
    || (UsartMpuLFinalVal < -80.0f)
    || (UsartMpuSFinalVal > 80.0f)
    || (UsartMpuSFinalVal < -80.0f) )
  {
    TIM1->BDTR &= ~0x8000;//�ر�pwm
    TIM8->BDTR &= ~0x8000;//�ر�pwm
    UsartError_angle = 1;//��̬��Ƕȹ���
  }
	//-------���ߵ���Ƕ�����---------------
  if ( UsartStatemotorSOk == 0)
  {//���ߵ��û������������� ���ߵ����������ǵ�ǰ�Ƕ�
    AttMotorLCorparaAngle = -UsartMpuLFinalVal;//������������
  }
  else if(AttMotorLCorparaAngle > 0.02f)
	{
		AttMotorLCorparaAngle = AttMotorLCorparaAngle - 0.02f;
	}
	else if( AttMotorLCorparaAngle < -0.02f )
	{
		AttMotorLCorparaAngle = AttMotorLCorparaAngle + 0.02f;
	}
	else
	{
		AttMotorLCorparaAngle = 0;
	}
	f_motorL_angle_final = f_MotorLEleAngle_cor + AttMotorLCorparaAngle;//������ĽǶ�
	
	//-------���ߵ���Ƕ�����---------------
  if ( UsartStatemotorLOk == 0)
  {//���ߵ��û������������� ���ߵ�������������ǵ�ǰ�Ƕ�
    AttMotorSCorparaAngle = -UsartMpuSFinalVal;
  }
  else if ( AttMotorSCorparaAngle >  0.02f)
	{
		AttMotorSCorparaAngle = AttMotorSCorparaAngle - 0.02f;
	}
  else if ( AttMotorSCorparaAngle < -0.02f )
	{
		AttMotorSCorparaAngle = AttMotorSCorparaAngle + 0.02f;
	}
	else
	{
		AttMotorSCorparaAngle = 0;
	}
  f_motorS_angle_final = f_motorS_angle_cor + AttMotorSCorparaAngle;
	
	/**********��ͯģʽ�£�����Ƕȵ�������Ҫƫ��**************/
  if ( (RemoteFlagChildOut == 1) //��ͯģʽ
		&& UsartStatemotorLOk 
		&& UsartStatemotorSOk )//���������������״���� �������ֽǶ�
  {//�����������ֽǶ� �����Ƕ�С�ĳ���
		if( ( f_motorL_angle_final < 0 ) && (f_motorS_angle_final < 0 ) )
		{//�Ƕ�ͬ�� ȫ����ת �����Ƕ�С�ĳ��� �ӿ��ٶ�
			if( f_motorL_angle_final > f_motorS_angle_final)
			{//motor1�Ƕȴ���motor2�Ƕ� Ϊ��Э�������� ��������motor2�Ƕ� ����ͬ��
				f_motorS_angle_final = (f_motorS_angle_final - f_motorL_angle_final ) * 0.6f + f_motorL_angle_final;
			}
			else if(f_motorL_angle_final < f_motorS_angle_final)
			{
				f_motorL_angle_final = (f_motorL_angle_final - f_motorS_angle_final) * 0.6f + f_motorS_angle_final;
			}
		}
		else if( ( f_motorL_angle_final > 0 ) && ( f_motorS_angle_final > 0 ) )
		{//�Ƕ�ͬ�� ȫ����ת �����Ƕȴ������ ��С�ٶ�
			if( f_motorL_angle_final < f_motorS_angle_final)
			{
				f_motorS_angle_final = (f_motorS_angle_final - f_motorL_angle_final) * 0.6f + f_motorL_angle_final;
			}
			else if(f_motorL_angle_final > f_motorS_angle_final)
			{
				f_motorL_angle_final = (f_motorL_angle_final - f_motorS_angle_final) * 0.6f + f_motorS_angle_final;
			}
		}
		else
		{//��������ת���෴ �Ƕȼ�С �𵽼��ٵ�����
				f_motorL_angle_final = f_motorL_angle_final * 0.6f;
				f_motorS_angle_final = f_motorS_angle_final * 0.6f;
		}
  }
  AttMotorLAngle = f_motorL_angle_final;//���������Ƕ�
  AttMotorSAngle = f_motorS_angle_final;
}

int AttMotorSPidFun()
{//pid�㷨 �ǶȻ�
	float kd;
	s16 val;
	if(UsartStatemotorLOk == 0)
	{//���ߵ��û��ת��������� �����������Ϊ0
		AttMotorSAngleLast = 0;//��һ������
		AttMotorLIntegral = 0;//���ֲ���
		kd = 0;//pid kd����
		val = 0;//pid����ֵ
		FocParaMotorSAttitude = 0;
	}
	else
	{
		kd = (AttMotorSAngle - AttMotorSAngleLast) * PID_PARA_KD;//��ǰһ���ĽǶȲ�
		AttMotorSIntegral = AttMotorSIntegral + 0.3f * AttMotorSAngle;//���޸�0.2�ĳ�0.3
		if(AttMotorSIntegral > 1023.0f)
		{
			AttMotorSIntegral = 1023.0f;
		}
		else if(AttMotorSIntegral < -1023.0f)
		{
			AttMotorSIntegral = -1023.0f;
		}
		//�����ط������޸ĵ�PARA_PID_P����
		val = PARA_PID_P * AttMotorSAngle + AttPidKi + kd;
		//-----����----------------
		val = val - UsartGyroZCtrl;
		//-------------------------
		if( val > 1024 )
		{
			val = 1023;
		}
		if( val < -1024 )
		{
			val = -1023;
		}
		AttMotorSAngleLast = AttMotorSAngle;//��һ���Ƕ�
	}
	return val;
}

int AttMotorLPidFun()
{//pid�㷨 �ǶȻ�
	float kd;
	s16 val;
	if(UsartStatemotorSOk == 0)
	{//����û��������״̬�� �����������Ϊ0
		AttMotorLEleAngleLast = 0;//��һ������
		AttMotorLIntegral = 0;//pid����
		kd = 0;
		val = 0;
		FocParaMotorLAttitude = 0;
	}
	else
	{//���������������״���� �������ֽǶ�
		kd = (AttMotorLAngle - AttMotorLEleAngleLast) * PID_PARA_KD;//kd����
		AttMotorLIntegral = AttMotorLIntegral + 0.3f * AttMotorLAngle;//motor2������
		//���ֳ���
		if(AttMotorLIntegral > 1023.0f)
		{
			AttMotorLIntegral = 1023.0f;
		}
		else if(AttMotorLIntegral < -1023.0f)
		{
			AttMotorLIntegral = -1023.0f;
		}
		//������������Ļ��ֱȽ� ����ki
		if(( AttMotorLIntegral > 0 ) && ( AttMotorSIntegral > 0 ))
		{
			if( AttMotorLIntegral > AttMotorSIntegral )
			{
				AttPidKi = AttMotorSIntegral;//ѡȡ����С����ki
			}
			else
			{
				AttPidKi = AttMotorLIntegral;
			}
		}
		else if(( AttMotorLIntegral < 0 ) && ( AttMotorSIntegral < 0 ))
		{
			if( AttMotorLIntegral > AttMotorSIntegral )
			{
				AttPidKi = AttMotorLIntegral;//ѡȡ���ִ����ki
			}
			else
			{
				AttPidKi = AttMotorSIntegral;
			}
		}
		else
		{
			AttPidKi = 0;
		}
		//pid��ʽ
		val = PARA_PID_P * AttMotorLAngle + AttPidKi + kd;
		//-----����----------------
		val = val + UsartGyroZCtrl;
		//-------------------------
		if( val >= 1024 )
		{
			val = 1023;
		}
		if( val <= -1024 )
		{
			val = -1023;
		}
		//������һ��ֵ
		AttMotorLEleAngleLast = AttMotorLAngle;//����
	}
	return val;
}

