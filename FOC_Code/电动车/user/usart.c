/***********************************************************
*	通讯
* 长线、短线电机的接收和发送，以及检测副板状态
*
*
************************************************************/
#include "extern.h"


u8 UsartLRemoteVal;
u8 UsartSRemoteVal;
u8 UsartStateLTurn;//转弯的状态
u8 UsartStateSTurn;
u8 UsartStateBatteryBak2;
u8 UsartStateBatteryBak;
u8 UsartStateBatteryLow;
u8 UsartState_state_8;
u8 UsartStateOK;
u8 UsartStateOverAngleBak;
u8 UsartFlagMpuLErrorCnt;
u8 UsartFlagMpuSErrorCnt;

s16 UsartGyroZCtrl;// z轴控制
s16 UsartMpuSGyroZ;// z轴数据，仅取短线一边的

u8 UsartError_angle;//姿态板角度过大
u8 UsartLCnt;
u8 UsartMpuPHOLCnt;
u8 UsartLRecBuf[8];

u16 UsartMpuLTimeOut;
u8 UsartLRemoteValBak;
float UsartMpuLValLast;
float UsartMpuLVal;
u8 UsartSCnt;
u8 UsartMpuPHOSCnt;
u8 UsartSRecBuf[8];//通讯3数据
u32 UsartMpuSTimeOut;
u8 UsartSRemoteValBak;
u8 UsartLSendBuf[6];
u8 UsartStateBattery2;
u8 UsartStateBattery1;
u8 UsartSSendBuf[6];
u8 UsartLSendCnt;
u8 UsartSSendCnt;
u8 UsartMpuData[2];
float UsartMpuLFinalVal;//姿态传来的角度
float UsartMpuSValLast;
float UsartMpuSFinalVal;//姿态板传来的角度
u8 UsartMpuLError;
u8 UsartMpuSError;
u8 UsartMpuPHOL;
u8 UsartMpuPHOS;



u8 UsartStatemotorSOk;//电机启动标志 out
u8 UsartStatemotorLOk;//
u8 UsartFlagLowAndPhoOn;//充电时 踩下踏板
u8 UsartErrorOverAngle;
u8 UsartFlagOverSpeed;
u8 UsartMpuPHOLLast;
u8 UsartMpuPHOSLast;
u8 UsartFlagMpuPHOSFirst;
u8 UsartFlagMpuPHOLFirst;


void UsartLGetFun(void);//获取长线通讯
void UsartSGetFun(void);//获取短线通讯
void UsartSendFun(void);//发送
void UsartGetStateCtrlLed(void);//获取当前状态发送给小板，控制led
void UsartCheckAttitudeFun(void);//检查副板
void UsartCheckSpeedFun(void);


//-------------------------------------------------------------
void UsartLGetFun()
{//接收USART2数据 长线
	//修改这里，为了匹配灵动微副板
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
      {//接收数据完毕
				//遥控器获取
				if( (UsartLRecBuf[0] & 0xf) == UsartLRemoteValBak)
				{
					UsartLRemoteVal = UsartLRecBuf[0] & 0xf;//遥控器
				}
				UsartLRemoteValBak = UsartLRecBuf[0] & 0xf;//备份
				//----------姿态角度-------------
        UsartMpuData[0] = UsartLRecBuf[2];
        UsartMpuData[1] = UsartLRecBuf[3];
        v_mpu = *(s16*)UsartMpuData;
        UsartMpuData[0] = UsartLRecBuf[4];
        UsartMpuData[1] = UsartLRecBuf[5];
        if ( v_mpu == *(s16*)UsartMpuData )
        {
					//本次角度与上一次角度均值
					f_val = v_mpu / 100.0f;
					UsartMpuLVal = f_val;
					UsartMpuLVal = UsartMpuLVal / 2.0f;
					UsartMpuLVal = UsartMpuLValLast / 2.0f + UsartMpuLVal;
					UsartMpuLValLast = f_val;
					UsartMpuLFinalVal = -UsartMpuLVal;//两个板子 因为安装方向不同 所以一正一负
					//----------光电----------------
					if ( UsartLRecBuf[1] == 0x55 )
					{
						UsartMpuPHOL = 1;//光电开关状态
						UsartMpuPHOLCnt = 0;
					}
					else if ( UsartMpuPHOLCnt >= 8 )
					{
						UsartMpuPHOL = 0;//连接
					}
					else
					{
						++UsartMpuPHOLCnt;
					}
				
				}
        else
        {
          UsartFlagMpuLErrorCnt = 1;//通讯出错
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
			UsartMpuLTimeOut = 0;//超时计数器
  }
  else if ( UsartMpuLTimeOut >= 30000 )//增加了超时时间
  {//副板连接有故障
    UsartLRemoteVal = 0;//遥控器的值
    UsartMpuLError = 1;	//长线一侧副板连接故障
    TIM1->BDTR &= 0x7FFFu;	//关闭pwm
    TIM8->BDTR &= 0x7FFFu;	//关闭pwm
  }
  else
  {
    ++UsartMpuLTimeOut;
  }
}

//-------------------------------------------------------------
void UsartSGetFun()
{//短线
	//20170419修改匹配灵动微副板
	u16 	v_data; 
	float f_val;
  s16 	v_mpu; 

  if ( USART3->SR & 0x20 )
  {
    v_data = USART3->DR;
    UsartSRecBuf[UsartSCnt] = v_data;
    ++UsartSCnt;
		
    //if ( v_data & 0x100 )
    //{//起始地址 改成8位的了，这里去掉
		if ( UsartSCnt == 8 )
		{
			//遥控器
			if ( (UsartSRecBuf[0] & 0xF) == UsartSRemoteValBak )
			{
				UsartSRemoteVal = UsartSRecBuf[0] & 0xF;
			}
			UsartSRemoteValBak = UsartSRecBuf[0] & 0xF;
			//姿态板角度
			UsartMpuData[0] = UsartSRecBuf[2];
			UsartMpuData[1] = UsartSRecBuf[3];
			v_mpu = *(s16*)UsartMpuData;
			UsartMpuData[0] = UsartSRecBuf[4];
			UsartMpuData[1] = UsartSRecBuf[5];
			if ( v_mpu == *(s16*)UsartMpuData )
			{//通讯上两个字节数据相同 既确认为新的角度
				f_val = v_mpu / 100.0f;
				UsartMpuSFinalVal = f_val;
				UsartMpuSFinalVal = UsartMpuSFinalVal / 2.0f;
				UsartMpuSFinalVal = UsartMpuSValLast / 2.0f + UsartMpuSFinalVal;
				UsartMpuSValLast = f_val;
				
				//光电开关
				if ( UsartSRecBuf[1] == 0x55 )
				{//踩踏开关
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
				//只用了短线一端的陀螺仪z轴校准电机角度
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
		{//0xc0开始
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
    UsartSRemoteVal = 0;	//遥控器
    UsartMpuSError = 1;		//短线一侧副板连接故障
    TIM1->BDTR &= 0x7FFFu;		//关闭pwm
    TIM8->BDTR &= 0x7FFFu;		//关闭pwm
  }
  else
  {
    ++UsartMpuSTimeOut;
  }
}

//-------------------------------------------------------------
void UsartSendFun()
{//通讯函数
	//给副板发送状态 后加的这个，所以原版还是没有的20170323
	//MotorSFlagDir 方向
	//MotorSFlagMove 0移动 1未移动
	//MotorLFlagDir 方向
	//MotorLFlagMove 0移动 1未移动
	
	//-----长线--------------------------
  if ( UsartLSendCnt >= 5 )
  {
    UsartLSendCnt = 0;
    UsartLSendBuf[0] = 0;
    UsartLSendBuf[1] = 0;
    UsartLSendBuf[2] = 0;
    UsartLSendBuf[3] = 0;
    UsartLSendBuf[4] = 0;
    UsartLSendBuf[5] = 0;
    if ( UsartStateSTurn )//转弯状态 控制led
      UsartLSendBuf[0] |= 1u;
    if ( UsartStateBatteryBak2 )//电池状态
      UsartLSendBuf[0] |= 2u;
    if ( UsartStateBatteryBak )//电池状态
      UsartLSendBuf[0] |= 4u;
    if ( UsartStateBatteryLow )//电压低的时候踩下光电
      UsartLSendBuf[0] |= 8u;
    if ( BatteryFlagLow )//电池电压低
      UsartLSendBuf[0] |= 0x10u;
    if ( RemoteFlagLockOut )//遥控器外部使用的锁机状态
      UsartLSendBuf[0] |= 0x20u;
    if ( RemoteFlagCorrectOut )//校准
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
			USART2->DR = UsartLSendBuf[0] | 0x100;//帧头
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
	
	//-----短线--------------------------
  if ( (signed int)UsartSSendCnt >= 5 )
  {
    UsartSSendCnt = 0;
    UsartSSendBuf[0] = 0;
    UsartSSendBuf[1] = 0;
    UsartSSendBuf[2] = 0;
    UsartSSendBuf[3] = 0;
    UsartSSendBuf[4] = 0;
    UsartSSendBuf[5] = 0;
    if ( UsartStateLTurn )//转弯状态 控制led
      UsartSSendBuf[0] |= 1u;
    if ( UsartState_state_8 )
      UsartSSendBuf[0] |= 2u;
    if ( UsartStateOK )//连个电机ok，没故障，可以正常运转
      UsartSSendBuf[0] |= 4u;
    if ( UsartStateOverAngleBak )//有故障
      UsartSSendBuf[0] |= 8u;
    if ( BatteryFlagLow )//电池电压低
      UsartSSendBuf[0] |= 0x10u;
    if ( RemoteFlagLockOut )//遥控器外部使用的锁机状态
      UsartSSendBuf[0] |= 0x20u;
    if ( MotorSErrorPosFlag )
    {
      UsartSSendBuf[2] = 4;
    }
    else if ( MotorLErrorPosFlag )
    {//长线霍尔故障
      UsartSSendBuf[2] = 5;
    }
    else if ( FocFlagMotorSErrorElVol )
    {//短线一侧静态电流故障
      UsartSSendBuf[2] = 1;
    }
    else if ( FocFlagMotorLErrorElVol )
    {//长线一侧静态电流故障
      UsartSSendBuf[2] = 2;
    }
    else if ( ErrorCircuit )
    {//电机相线有短路
      UsartSSendBuf[2] = 3;
    }
    else if ( BatteryFlagError )
    {//电压故障标志
      UsartSSendBuf[2] = 6;
    }
    else if ( UsartMpuLError )
    {//长线一侧副板连接故障
      UsartSSendBuf[2] = 7;
    }
    else if ( UsartMpuSError )
    {//短线一侧副板连接故障
      UsartSSendBuf[2] = 8;
    }
    if ( RemoteFlagCorrectOut )//校准
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
{//获取当前状态发送给小板
  if ( IDCal6 == IDCal8 )
  {//加密正常
		//--------转弯时候的灯-----------------------------
    if ( (MotorLTimePole6 <= 1350) || (MotorSTimePole6 <= 1350) )
    {//六个霍尔换相时间小，也就是电机任意一边电机有速度，电机启动
      if ( MotorSFlagDir == MotorLFlagDir )
      {//方向不同
        if ( MotorSFlagDir == MotorLFlagDir )
        {//方向不同
					//长线电机
					if(MotorLTimePole6 < 1350 ) UsartStateLTurn = 1;//可能是转弯的时候吧
					else UsartStateLTurn = 0;
					if(MotorSTimePole6 < 1350 ) UsartStateSTurn = 1;
					else UsartStateSTurn = 0;
        }
        else
        {//方向相同
          UsartStateLTurn = 0;
          UsartStateSTurn = 0;
        }
      }
			else//电机方向相同
			{
				if( MotorLTimePole6 > ( MotorSTimePole6 + 88 ) )
				{
          UsartStateLTurn = 1;//短线速度快
          UsartStateSTurn = 0;
				}	
				else if ( MotorSTimePole6 > ( MotorLTimePole6 + 88 ) )
        {
          UsartStateLTurn = 0;
          UsartStateSTurn = 1;//长线速度快
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
		//--------电压低-----------------------------
    if ( BatteryFlagLow )//电池电压低
    {//电压低
      UsartStateBattery1 = 0;
      if ( UsartFlagLowAndPhoOn )
      {//电压低的时候 踩下光电
        UsartStateBatteryLow = 1;//电压低的时候踩下光电
        UsartStateBattery2 = 0;
      }
      else
      {//
        UsartStateBatteryLow = 0;//正常充电
        if ( BatteryVal1 >= 1634 )
          UsartStateBattery2 = 1;
        else
          UsartStateBattery2 ^= 1u;
      }
    }
    else
    {//非充电状态
      UsartStateBattery2 = 0;
      if ( BatteryFlagVolLowest )
      {//电压更低
        UsartStateBatteryLow = 1;
        UsartStateBattery1 = 0;//电压状态
      }
      else
      {//电压不低
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
    UsartStateBatteryBak = UsartStateBattery1;//电池状态
    UsartStateBatteryBak2 = UsartStateBattery2;
    UsartStateOverAngleBak = 0;
    UsartStateOK = 0;
    if ( UsartErrorOverAngle )//角度过大
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
{//检查副板
  if ( (BatteryFlagLow 			== 1) 	//电池电压低
		|| (BatteryFlagPowerOnOut 		== 0)		//电压低标志 
		|| (RemoteFlagCorrectOut 	== 1) 	//校准中
		|| (RemoteFlagLockOut 			== 1) 	//遥控器外部使用的锁机状态	
		|| (IDFlagWrite3 				== 0) )	//id加密出错
  {//不能正常运行
    UsartFlagMpuPHOSFirst = 0;
    TIM1->BDTR &= 0x7FFFu;//关闭pwm
    UsartStatemotorLOk = 0;	//无法运行
    UsartFlagMpuPHOLFirst = 0;
    TIM8->BDTR &= 0x7FFFu;//关闭pwm
    UsartStatemotorSOk = 0;//关闭电机 
		return;
  }
	//----------短线--------------------
	if ( (UsartMpuPHOS == 1) && (UsartMpuPHOSLast==0) )
	{//光电首次被按下 仅执行一次
		UsartFlagMpuPHOSFirst = 0;//刚开启小板
	}
	else if ( (UsartMpuPHOSLast != 1) || (UsartMpuPHOS != 1) )
	{//光电没有被按下
		if ( (UsartFlagMpuPHOSFirst == 1) && (!UsartMpuPHOL) )
		{//不按下光电 关闭一只pwm 仅执行一次
			UsartFlagMpuPHOSFirst = 0;
			TIM1->BDTR &= 0x7FFFu;//关闭pwm
			UsartStatemotorLOk = 0;
			UsartErrorOverAngle = 0;
		}
	}
	else if ( !UsartFlagMpuPHOSFirst )
	{//仅执行一次
		if ( (MotorLErrorPosFlag)//霍尔故障
			|| (MotorSErrorPosFlag)
			|| (UsartMpuSError)
			|| (UsartMpuLError)//长线一侧副板连接故障
			|| (FocFlagMotorLErrorElVol)
			|| (FocFlagMotorSErrorElVol)
			|| (ErrorCircuit)			//电机相线有短路
			|| (BatteryFlagError)
			|| (((UsartMpuSFinalVal < -15.0f) || (UsartMpuSFinalVal >= 15.0f)) //角度超出范围
					&& (UsartStatemotorSOk == 0)) )
		{//有故障
			if( ( UsartMpuSFinalVal <= -15.0f ) || ( UsartMpuSFinalVal >= 15.0f ) )//角度超出范围
			{
				if ( !UsartStatemotorSOk )//tim8关闭
					UsartErrorOverAngle = 1;//角度过大
			}
		}
		else
		{//没有其他故障
			UsartFlagMpuPHOSFirst = 1;//置位
			if ( !UsartStatemotorLOk )//关闭了tim1 重新开启
				TIM1->BDTR |= 0x8000u;
			if ( !UsartStatemotorSOk )//关闭了tim8 重新开启
				TIM8->BDTR |= 0x8000u;
			UsartStatemotorLOk = 1;//
			UsartErrorOverAngle = 0;
			UsartError_angle = 0;
		}
	}
	UsartMpuPHOSLast = UsartMpuPHOS;
	//----------长线--------------------
	if ( (UsartMpuPHOL == 1) && (UsartMpuPHOLLast==0) )
	{
		UsartFlagMpuPHOLFirst = 0;//首次
	}
	else if ( (UsartMpuPHOLLast != 1) || (UsartMpuPHOL != 1) )
	{
		if ( (UsartFlagMpuPHOLFirst == 1) && (!UsartMpuPHOS) )
		{//关闭一只pwm
			UsartFlagMpuPHOLFirst = 0;
			TIM8->BDTR &= 0x7FFFu;//关闭pwm
			UsartStatemotorSOk = 0;
			UsartErrorOverAngle = 0;
		}
	}
	else if ( !UsartFlagMpuPHOLFirst )
	{
		if ( (MotorLErrorPosFlag)//霍尔故障
			|| (MotorSErrorPosFlag)
			|| (UsartMpuSError)
			|| (UsartMpuLError)
			|| (FocFlagMotorLErrorElVol)
			|| (FocFlagMotorSErrorElVol)
			|| (ErrorCircuit)			//电机相线有短路
			|| (BatteryFlagError)
			|| ((((UsartMpuLFinalVal <= -15.0f)) || ((UsartMpuLFinalVal >= 15.0f))) 
			&& (UsartStatemotorLOk == 0)) )
		{//有故障
			if ( (UsartMpuLFinalVal <= -15.0f) || (UsartMpuLFinalVal >= 15.0f) )
			{//角度超出范围
				if ( !UsartStatemotorLOk )
					UsartErrorOverAngle = 1;//角度超出范围
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
			UsartError_angle = 0;//姿态板角度过大
		}
	}
	if ( (!UsartMpuPHOS) && (!UsartMpuPHOL) )
		UsartErrorOverAngle = 0;//光电都没有被按下的时候，角度过大的标志位清零
	UsartMpuPHOLLast = UsartMpuPHOL;
}



//-------------------------------------------------------------
void UsartCheckSpeedFun()
{
	//-------电压低的时候检测光电是否被按下-----------------
	if(BatteryFlagLow 											 //电池电压低
		&& (UsartMpuPHOS || UsartMpuPHOL)) //光电没有踩下
	{
		UsartFlagLowAndPhoOn = 1;	//电池电压地的时候光电被踩下
	}
	else UsartFlagLowAndPhoOn = 0;
	//------------超速报警------------------------
  if ( MotorSFlagDir == MotorLFlagDir )
  {//电机不同向 没有超速报警
    UsartFlagOverSpeed = 0;
  }
  else if ( ((MotorLTimePole6 + MotorSTimePole6) / 2) >= 168 )
  {
    UsartFlagOverSpeed = 0;//
  }
  else
  {//超速报警
    UsartFlagOverSpeed = 1;
  }
}


