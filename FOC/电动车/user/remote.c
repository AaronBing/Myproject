/***************************************************************************
* 遥控器按键1传递的数值是->2，其他如下
* 遥控器（1锁扭扭车->2）,（2解锁->1）,（3铃铛报警->4）,（4闪电自平衡->8）,
* （1+2校准->3）,1+3->6,1+5->10,2+3->5(姿态班不允许发送5出来),
* 2+4->9,3+4->12
****************************************************************************/
#include "extern.h"


#define REMOTE_UNLOCK				1//解锁
#define REMOTE_LOCK					2//锁
#define REMOTE_CORRECT			3//校准
#define REMOTE_ALARM				4//报警
#define REMOTE_CHILD 			  8//儿童模式


u8 RemoteFlagCorrectOut;//校准 out
u8 RemoteFlagCorrectbeepOut;//buzz鸣叫完成 out
u8 RemoteFlagLockOut;//外部使用 out
u8 RemoteFlagAlarm;//外部使用 out

u8 RemoteFlagChildOut;//外部会使用，儿童模式 out
u8 RemoteFlagChildbeepOut;//儿童模式切换beep的一声响 out

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
{//遥控器
	int v_remote; // 
	
	//遥控器，长线电机这边，或者短线电机这边
  v_remote = UsartLRemoteVal;//遥控
  if ( !UsartLRemoteVal )
    v_remote = UsartSRemoteVal;
	
	//铃铛，警报
  if ( v_remote == REMOTE_ALARM )
  {
    RemoteFlagAlarm = 1;
    RemoteCloseAlarmCnt = 0;
  }
  else if ( (signed int)RemoteCloseAlarmCnt >= 80 )//关闭报警计数器
  {
    RemoteFlagAlarm = 0;//关闭报警
  }
  else
  {
    ++RemoteCloseAlarmCnt;
  }
	//
  if ( ((!UsartMpuPHOS) && (!UsartMpuPHOL) 
						&&(!UsartStatemotorLOk) && (!UsartStatemotorSOk)) 
				|| RemoteFlagLockOut )
  {//两个姿态板不允许被踩踏的时候，或者锁机的状态，可以操作遥控器
    switch ( v_remote )
    {
      case REMOTE_LOCK://锁车
        if ( (signed int)RemoteRunCnt > 8 )
        {
          if ( !RemoteFlagLock )
          {
            RemoteFlagLock = 1;//锁标志位
            RemoteFlagLockOut ^= 1u;//遥控器外部使用的锁机状态
          }
        }
        else
        {
          ++RemoteRunCnt;
        }
        RemoteRecoveryCnt = 0;//恢复计数器，清零
        break;
      case REMOTE_UNLOCK://解锁
        if ( (signed int)RemoteRunCnt > 8 )
        {
          if ( !RemoteFlagLock )
          {
            RemoteFlagLock = 1;
            RemoteFlagLockOut = 0;//遥控器外部使用的锁机状态
          }
        }
        else
        {
          ++RemoteRunCnt;
        }
        RemoteRecoveryCnt = 0;
        break;
      case REMOTE_CHILD://儿童模式
        if ( (signed int)RemoteRunCnt > 10 )
        {
          if ( !RemoteFlagChild )
          {
            RemoteFlagChildOut ^= 1u;//外部使用，儿童模式
            RemoteFlagChildbeepOut = 1;//儿童模式切换beep的一声响
            RemoteFlagChild = 1;
          }
        }
        else
        {
          ++RemoteRunCnt;
        }
        RemoteRecoveryCnt = 0;
        break;
      case REMOTE_CORRECT://校准
        if ( (signed int)RemoteCorrectCnt >= 1500 )
        {
          RemoteCorrectCnt = 0;
          RemoteFlagCorrect = 1;//校准开启
          RemoteFlagCorrectOut = 1;//校准
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
        {//恢复所有的标志位
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
		//-----校准---------------------
    if ( RemoteFlagCorrectOut )
    {//校准
      if ( (signed int)RemoteCorrectCnt2 >= 80 )
      {
        if ( RemoteFlagCorrect )
        {//校准
          RemoteFlagCorrectOut = 0;//校准完成
          RemoteFlagCorrectbeepOut = 0;//再 校准beep响一声
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
