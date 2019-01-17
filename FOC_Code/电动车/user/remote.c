/***************************************************************************
* ң��������1���ݵ���ֵ��->2����������
* ң������1��ŤŤ��->2��,��2����->1��,��3��������->4��,��4������ƽ��->8��,
* ��1+2У׼->3��,1+3->6,1+5->10,2+3->5(��̬�಻������5����),
* 2+4->9,3+4->12
****************************************************************************/
#include "extern.h"


#define REMOTE_UNLOCK				1//����
#define REMOTE_LOCK					2//��
#define REMOTE_CORRECT			3//У׼
#define REMOTE_ALARM				4//����
#define REMOTE_CHILD 			  8//��ͯģʽ


u8 RemoteFlagCorrectOut;//У׼ out
u8 RemoteFlagCorrectbeepOut;//buzz������� out
u8 RemoteFlagLockOut;//�ⲿʹ�� out
u8 RemoteFlagAlarm;//�ⲿʹ�� out

u8 RemoteFlagChildOut;//�ⲿ��ʹ�ã���ͯģʽ out
u8 RemoteFlagChildbeepOut;//��ͯģʽ�л�beep��һ���� out

u8 RemoteRunCnt;
u8 RemoteFlagChild;
u8 RemoteRecoveryCnt;
u8 RemoteFlagLock;
u8 RemoteCloseAlarmCnt;
u8 RemoteFlagCorrect;
u16 RemoteCorrectCnt;
u16 RemoteCorrectCnt2;



void RemoteCtrlFun(void);

//-------------------------------------------------------------
void RemoteCtrlFun()
{//ң����
	int v_remote; // 
	
	//ң���������ߵ����ߣ����߶��ߵ�����
  v_remote = UsartLRemoteVal;//ң��
  if ( !UsartLRemoteVal )
    v_remote = UsartSRemoteVal;
	
	//����������
  if ( v_remote == REMOTE_ALARM )
  {
    RemoteFlagAlarm = 1;
    RemoteCloseAlarmCnt = 0;
  }
  else if ( (signed int)RemoteCloseAlarmCnt >= 80 )//�رձ���������
  {
    RemoteFlagAlarm = 0;//�رձ���
  }
  else
  {
    ++RemoteCloseAlarmCnt;
  }
	//
  if ( ((!UsartMpuPHOS) && (!UsartMpuPHOL) 
						&&(!UsartStatemotorLOk) && (!UsartStatemotorSOk)) 
				|| RemoteFlagLockOut )
  {//������̬�岻������̤��ʱ�򣬻���������״̬�����Բ���ң����
    switch ( v_remote )
    {
      case REMOTE_LOCK://����
        if ( (signed int)RemoteRunCnt > 8 )
        {
          if ( !RemoteFlagLock )
          {
            RemoteFlagLock = 1;//����־λ
            RemoteFlagLockOut ^= 1u;//ң�����ⲿʹ�õ�����״̬
          }
        }
        else
        {
          ++RemoteRunCnt;
        }
        RemoteRecoveryCnt = 0;//�ָ�������������
        break;
      case REMOTE_UNLOCK://����
        if ( (signed int)RemoteRunCnt > 8 )
        {
          if ( !RemoteFlagLock )
          {
            RemoteFlagLock = 1;
            RemoteFlagLockOut = 0;//ң�����ⲿʹ�õ�����״̬
          }
        }
        else
        {
          ++RemoteRunCnt;
        }
        RemoteRecoveryCnt = 0;
        break;
      case REMOTE_CHILD://��ͯģʽ
        if ( (signed int)RemoteRunCnt > 10 )
        {
          if ( !RemoteFlagChild )
          {
            RemoteFlagChildOut ^= 1u;//�ⲿʹ�ã���ͯģʽ
            RemoteFlagChildbeepOut = 1;//��ͯģʽ�л�beep��һ����
            RemoteFlagChild = 1;
          }
        }
        else
        {
          ++RemoteRunCnt;
        }
        RemoteRecoveryCnt = 0;
        break;
      case REMOTE_CORRECT://У׼
        if ( (signed int)RemoteCorrectCnt >= 1500 )
        {
          RemoteCorrectCnt = 0;
          RemoteFlagCorrect = 1;//У׼����
          RemoteFlagCorrectOut = 1;//У׼
          RemoteFlagCorrectbeepOut = 0;
        }
        else
        {
          ++RemoteCorrectCnt;
        }
        RemoteRecoveryCnt = 0;
        break;
      default:
        if ( (signed int)RemoteRecoveryCnt >= 180 )
        {//�ָ����еı�־λ
          RemoteRecoveryCnt = 0;
          RemoteFlagChild = 0;
          RemoteFlagLock = 0;
        }
        else
        {
          ++RemoteRecoveryCnt;
        }
        RemoteCorrectCnt = 0;
        RemoteRunCnt = 0;
        break;
    }
		//-----У׼---------------------
    if ( RemoteFlagCorrectOut )
    {//У׼
      if ( (signed int)RemoteCorrectCnt2 >= 80 )
      {
        if ( RemoteFlagCorrect )
        {//У׼
          RemoteFlagCorrectOut = 0;//У׼���
          RemoteFlagCorrectbeepOut = 0;//�� У׼beep��һ��
        }
      }
      else
      {
        ++RemoteCorrectCnt2;
      }
    }
    else
    {
      RemoteCorrectCnt2 = 0;
    }
  }
}
