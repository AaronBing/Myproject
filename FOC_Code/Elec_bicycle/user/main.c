/*************************************************************************
* 新的gd的扭扭车主板
* 通讯是8个，增加了z轴的代码，解决了扭死现象
* 本代码用了st的库，注意是用在gd上的
* 暂时选择了103c8，因为c8软仿比较方便，用的时候，记得改回103rc
**************************************************************************/
#include "extern.h"

//电池相关
u8 WorkCnt;//获取电压计次
u8 PollCnt;//控制计数器

u8 ErrorCircuit;//相线短路


//-------------------------------------------------------------
int main()
{
//  s16 attitudeL; 
//  s16 attitudeS; 

	InitFun();					//初始化
	FocSelfCheckingFun();		//自检
	
	while ( 1 )
	{
		//--------短路检测-------------
		if ( TIM1->SR & 0x80 )			//状态寄存器
		{
			ErrorCircuit = 1;			//电机相线有短路
			TIM1->SR = 0;
			TIM1->BDTR &= 0x7FFFu;		//关闭pwm
			TIM8->BDTR &= 0x7FFFu;		//关闭pwm
		}

		if ( FocFlagAdcOK )
		{//获取adc 4次完成
			FocVlotCmp1RegulationFun();			//负载调节
			FocFlagAdcOK = 0;
		}
		if ( FocTime1msFlag )
		{
			FocTime1msFlag = 0;
//			++WorkCnt;		//获取次数
//			if(WorkCnt < 2)
//			{
//			  FocParaMotorSAttitude = 200;
//			  //TIM1->BDTR |= 0x8000u;
//			}
//			else 
//			{
//			  WorkCnt = 0;			
//			  BatteryGetFun();	//电池检测				
//			  BatteryCheckPowerOnFun();	//休眠检测			 
//			  BatteryCheckFun();	//电池电压
//			}
		}
	}
}


