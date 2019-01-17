/***********************************************************
*	姿态角度的修正
*
*
*
************************************************************/
#include "extern.h"
#define PARA_PID_P		70.0f//70.0f
#define PID_PARA_KD		30.0f//30.0f
	

float AttMotorLAngle;//最终修正角度 motor2
float AttMotorSAngle;//最终修正角度 motor1

float AttDiffLast;
float AttMotorLCorparaAngle;//修正参数
float AttMotorSCorparaAngle;
float AttMotorSIntegral;//motor1积分项
float AttMotorLIntegral;//motor2积分项
float AttPidKi;//
float AttMotorSAngleLast;//上一个角度
float AttMotorLEleAngleLast;//上一个角度
float AttDirOffset;//



float attGetDiroffset(float acce, float offset);
void 	AttCorAngleFun(void);			//修正角度
int 	AttMotorSPidFun(void);		//motorS角度pid算法
int 	AttMotorLPidFun(void);		//motorL角度pid算法


//-------------------------------------------------------------
float attGetDiroffset(float acce, float offset)
{//之前错误的时候 返回值和angle_dot_1DC赋值反了 现象是轮子打滑
	//改完后 没用这种现象了
	//计算方向偏差
	float f_diff;
	float f_frst;
	f_diff = acce - offset;
	f_frst = 0.002f * f_diff + 0.00003f * AttDiffLast;//偏差和前一次偏差
	AttDiffLast = f_diff;
	return f_frst;
}

//-------------------------------------------------------------
void AttCorAngleFun()
{//角度修正
	float f_acceler; 
  float f_MotorLEleAngle_cor; 
  float f_motorS_angle_cor; 
  float f_motorL_angle_final;
  float f_motorS_angle_final;
	u16 	v_roll_time;//六个霍尔换相时间 这里有修改
	
	//-----------方向偏差计算--------------
  if ( ((MotorSFlagDir == 0) && (MotorLFlagDir == 1)) 
		|| ((MotorSFlagDir == 1) && (MotorLFlagDir == 0)) )
  {//电机同向 向前或者向后
		//v0 走快的时候 减速 走快自动抬头
		//平时转圈得时候 总是发现会后仰一下 或者前倾一下 这个值改变这里
		//往前速度太快减速 所以后仰 往后速度也太快 所以一个轮子前仰
		//造成了前倾后仰得问题
    v_roll_time = (MotorLTimePole6 + MotorSTimePole6) >> 1;//六次霍尔换相总时间
    if ( (v_roll_time < 168) || BatteryFlagLowRunOut )
    {
      f_acceler = 5.8f;
    }
    else if ( v_roll_time < 253 )//这里有修改，之前是406 
    {
			//f_acceler = (405 - v_roll_time) * 0.0244725738f;////速度大概是12公里的时候 抬头 5.8/(405-168) = 0.0244725738f
			f_acceler = (253 - v_roll_time) * 0.068235f;// 5.8/(253-168) = 0.068235f
    }
    else
    {
      f_acceler = 0;
    }
		/*//是否是6公里速度的时候抬头
		v_roll_time = (MotorLTimePole6 + MotorSTimePole6) >> 1;//六次霍尔换相总时间
    if ( ((signed int)v_roll_time < 84) || BatteryFlagLowRunOut )
    {//
      f_acceler = 5.8f;
    }
    else if ( v_roll_time < 203 )
    {//浮点值的计算 5.8/(202-84)
			f_acceler = (202 - v_roll_time) * 0.04915254237288135f;//为什么这样计算 浮点值哪来的
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
		f_acceler = -f_acceler;//车向前 或者是向后
	//--------------修正电机速度--------------
	AttDirOffset = attGetDiroffset(f_acceler, AttDirOffset) + AttDirOffset;//计算方向偏差
	f_MotorLEleAngle_cor = UsartMpuLFinalVal + AttDirOffset;//矫正角度数据
	f_motorS_angle_cor = UsartMpuSFinalVal + AttDirOffset;
	//----新增的 陀螺仪z轴------------------
	if ( (MotorSFlagDir == MotorLFlagDir)//可能是扭动的时候
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
	{//z轴，一点一点改变
		if ( UsartGyroZCtrl > UsartMpuSGyroZ )
		{
			--UsartGyroZCtrl;
		}
		else if ( UsartGyroZCtrl < UsartMpuSGyroZ )
		{
			++UsartGyroZCtrl;
		}
	}
	//------角度超限----------------------
  if ( (UsartMpuLFinalVal > 80.0f)//通讯传上来的角度
    || (UsartMpuLFinalVal < -80.0f)
    || (UsartMpuSFinalVal > 80.0f)
    || (UsartMpuSFinalVal < -80.0f) )
  {
    TIM1->BDTR &= ~0x8000;//关闭pwm
    TIM8->BDTR &= ~0x8000;//关闭pwm
    UsartError_angle = 1;//姿态板角度过大
  }
	//-------长线电机角度修正---------------
  if ( UsartStatemotorSOk == 0)
  {//短线电机没有启动的情况下 长线的修正参数是当前角度
    AttMotorLCorparaAngle = -UsartMpuLFinalVal;//重新修正数据
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
	f_motorL_angle_final = f_MotorLEleAngle_cor + AttMotorLCorparaAngle;//修正后的角度
	
	//-------短线电机角度修正---------------
  if ( UsartStatemotorLOk == 0)
  {//长线电机没有启动的情况下 短线电机的修正参数是当前角度
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
	
	/**********儿童模式下，电机角度的修正，要偏软**************/
  if ( (RemoteFlagChildOut == 1) //儿童模式
		&& UsartStatemotorLOk 
		&& UsartStatemotorSOk )//两个电机都启动的状况下 修正车轮角度
  {//根据两个车轮角度 修正角度小的车轮
		if( ( f_motorL_angle_final < 0 ) && (f_motorS_angle_final < 0 ) )
		{//角度同向 全部正转 修正角度小的车轮 加快速度
			if( f_motorL_angle_final > f_motorS_angle_final)
			{//motor1角度大于motor2角度 为了协调两个轮 重新设置motor2角度 以下同理
				f_motorS_angle_final = (f_motorS_angle_final - f_motorL_angle_final ) * 0.6f + f_motorL_angle_final;
			}
			else if(f_motorL_angle_final < f_motorS_angle_final)
			{
				f_motorL_angle_final = (f_motorL_angle_final - f_motorS_angle_final) * 0.6f + f_motorS_angle_final;
			}
		}
		else if( ( f_motorL_angle_final > 0 ) && ( f_motorS_angle_final > 0 ) )
		{//角度同向 全部反转 修正角度大得轮子 减小速度
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
		{//两个车轮转向相反 角度减小 起到减速的作用
				f_motorL_angle_final = f_motorL_angle_final * 0.6f;
				f_motorS_angle_final = f_motorS_angle_final * 0.6f;
		}
  }
  AttMotorLAngle = f_motorL_angle_final;//最终修正角度
  AttMotorSAngle = f_motorS_angle_final;
}

int AttMotorSPidFun()
{//pid算法 角度环
	float kd;
	s16 val;
	if(UsartStatemotorLOk == 0)
	{//长线电机没有转动的情况下 相关修正参数为0
		AttMotorSAngleLast = 0;//上一个参数
		AttMotorLIntegral = 0;//积分参数
		kd = 0;//pid kd参数
		val = 0;//pid返回值
		FocParaMotorSAttitude = 0;
	}
	else
	{
		kd = (AttMotorSAngle - AttMotorSAngleLast) * PID_PARA_KD;//与前一个的角度差
		AttMotorSIntegral = AttMotorSIntegral + 0.3f * AttMotorSAngle;//有修改0.2改成0.3
		if(AttMotorSIntegral > 1023.0f)
		{
			AttMotorSIntegral = 1023.0f;
		}
		else if(AttMotorSIntegral < -1023.0f)
		{
			AttMotorSIntegral = -1023.0f;
		}
		//几个地方都是修改的PARA_PID_P数据
		val = PARA_PID_P * AttMotorSAngle + AttPidKi + kd;
		//-----新增----------------
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
		AttMotorSAngleLast = AttMotorSAngle;//上一个角度
	}
	return val;
}

int AttMotorLPidFun()
{//pid算法 角度环
	float kd;
	s16 val;
	if(UsartStatemotorSOk == 0)
	{//短线没有启动的状态下 相关修正参数为0
		AttMotorLEleAngleLast = 0;//上一个参数
		AttMotorLIntegral = 0;//pid积分
		kd = 0;
		val = 0;
		FocParaMotorLAttitude = 0;
	}
	else
	{//两个电机都启动的状况下 修正车轮角度
		kd = (AttMotorLAngle - AttMotorLEleAngleLast) * PID_PARA_KD;//kd参数
		AttMotorLIntegral = AttMotorLIntegral + 0.3f * AttMotorLAngle;//motor2积分项
		//积分超限
		if(AttMotorLIntegral > 1023.0f)
		{
			AttMotorLIntegral = 1023.0f;
		}
		else if(AttMotorLIntegral < -1023.0f)
		{
			AttMotorLIntegral = -1023.0f;
		}
		//根据两个电机的积分比较 调整ki
		if(( AttMotorLIntegral > 0 ) && ( AttMotorSIntegral > 0 ))
		{
			if( AttMotorLIntegral > AttMotorSIntegral )
			{
				AttPidKi = AttMotorSIntegral;//选取积分小的做ki
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
				AttPidKi = AttMotorLIntegral;//选取积分大的做ki
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
		//pid公式
		val = PARA_PID_P * AttMotorLAngle + AttPidKi + kd;
		//-----新增----------------
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
		//备份上一个值
		AttMotorLEleAngleLast = AttMotorLAngle;//备份
	}
	return val;
}

