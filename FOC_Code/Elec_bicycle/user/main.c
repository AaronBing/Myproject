/*************************************************************************
* �µ�gd��ŤŤ������
* ͨѶ��8����������z��Ĵ��룬�����Ť������
* ����������st�Ŀ⣬ע��������gd�ϵ�
* ��ʱѡ����103c8����Ϊc8��±ȽϷ��㣬�õ�ʱ�򣬼ǵøĻ�103rc
**************************************************************************/
#include "extern.h"

//������
u8 WorkCnt;//��ȡ��ѹ�ƴ�
u8 PollCnt;//���Ƽ�����

u8 ErrorCircuit;//���߶�·


//-------------------------------------------------------------
int main()
{
//  s16 attitudeL; 
//  s16 attitudeS; 

	InitFun();					//��ʼ��
	FocSelfCheckingFun();		//�Լ�
	
	while ( 1 )
	{
		//--------��·���-------------
		if ( TIM1->SR & 0x80 )			//״̬�Ĵ���
		{
			ErrorCircuit = 1;			//��������ж�·
			TIM1->SR = 0;
			TIM1->BDTR &= 0x7FFFu;		//�ر�pwm
			TIM8->BDTR &= 0x7FFFu;		//�ر�pwm
		}

		if ( FocFlagAdcOK )
		{//��ȡadc 4�����
			FocVlotCmp1RegulationFun();			//���ص���
			FocFlagAdcOK = 0;
		}
		if ( FocTime1msFlag )
		{
			FocTime1msFlag = 0;
//			++WorkCnt;		//��ȡ����
//			if(WorkCnt < 2)
//			{
//			  FocParaMotorSAttitude = 200;
//			  //TIM1->BDTR |= 0x8000u;
//			}
//			else 
//			{
//			  WorkCnt = 0;			
//			  BatteryGetFun();	//��ؼ��				
//			  BatteryCheckPowerOnFun();	//���߼��			 
//			  BatteryCheckFun();	//��ص�ѹ
//			}
		}
	}
}


