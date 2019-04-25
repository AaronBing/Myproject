
#include "main.h"


uint32_t MsCnt=0;
uint16_t sumA=0,sumB=0;
uint16_t sumCnt=0;

extern uint32_t	FocSelfCheckOK;
extern int16_t 	MotorFlagMove;
extern Curr_Components 		MotorCurr_ab;
extern Volt_Components 		MotorAtatVolt_qd;
extern uint16_t 	FocMotorPhaseAOffset;
extern uint16_t 	FocMotorPhaseBOffset;
extern uint16_t 	MotorFinalEleAngle;
extern uint16_t     RegularConvData_Tab[5];
extern void TX_data(void);
uint16_t	vadc_pwroff_bat;
uint16_t	vadc_pwron_bat;
uint16_t	vadc_ibus;
uint16_t	vadc_ia, vadc_ib, vadc_ic;
uint32_t	FocTime64usCnt; 
uint32_t	FocTime1msFlag;  
//uint32_t	MsFlag; 		//

uint32_t	FocGetAdcCnt;
uint32_t	FocFlagAdcOK;
//uint16_t 	FocBootVolBuf[4];
uint16_t	FocAdcBatteryBuf[4] = {0};
//uint16_t    FocMotorLoadBuf[4] = {0};
uint16_t 	FocSpeedRef=0;
uint16_t	FocMotorCurA;
uint16_t	FocMotorCurB;
uint16_t   FocMotorRef;

//--------------------速度反馈
extern uint16_t FocSpeedout;//最终期望的速度
uint16_t FocSpeednow=0;		//当前设定的速度
uint8_t DelaySpeed = 0;


uint8_t cnt = 0;
uint8_t res_old = 0;
uint8_t res_old1 = 0;
uint8_t res;
uint8_t USART_RX_BUF[25];     
u8 Yibiao_Set, fuzhiwc_flg = 0;
u8 CiGangShu;		//极对数？
u16 LunJing;      //轮径？
u16 SpeedPWM = 0;		//调速？
u16 SpeedPWM1=0;
u16 SpeedPWM2=0;
u8 Yibiao_CheckBit;		//仪表
u8 zero_run_flg;

uint8_t filter_cnt = 0;  //滤波计数
uint16_t filter_pwm[5];
uint16_t TiaoSu=0;
uint16_t X;

extern uint8_t send_data[16];
extern void Uart_Buff_Clear (void);

//uint8_t pwm_filter(void) 
//{
////	uint16_t filter_min;
//	uint16_t sum;
//	uint16_t filter_max;
//	for(filter_cnt = 0;filter_cnt < 5;filter_cnt ++)
//	{
//		 filter_pwm[filter_cnt] = SpeedPWM;
//		filter_max = filter_pwm[0];
//		if( filter_pwm[filter_cnt] >  filter_max)	
//			filter_max = filter_pwm[filter_cnt];
//		sum += filter_pwm[filter_cnt+1];
//		if(filter_cnt == 5)
//		{
//			filter_cnt = 0;
//			sum = 0;
//			TiaoSu = (sum>>2);
//		}
//	}
//	return 	TiaoSu;
//}

void ADC1_IRQHandler (void)
{
	ADC_ClearFlag (ADC1, ADC_FLAG_EOC);		//End of conversion flag
	GPIO_SetBits (GPIOC, GPIO_Pin_13);      //PC13 设置电平干什么？
	if (++FocTime64usCnt >= 16)				// 16*64=1024  约等于1ms
	{
		FocTime64usCnt = 0;
		//MsCnt++;
		
		if(DelaySpeed<3)				//缓慢加速的阈值
			DelaySpeed++;
		else{
			if(FocSpeednow<FocSpeedout)
				FocSpeednow++;
			else if((FocSpeednow>FocSpeedout)&&((FocSpeednow>1)))
				FocSpeednow--;
			DelaySpeed=0;
		}
		
		FocTime1msFlag = 1;
		
//		if(Ctl.State == MOTOR_STOP)
//		{
//			if(sumCnt<16)		//累加16次，
//			{
//				sumA += FocMotorCurA;	
//				sumB += FocMotorCurB;		
//				sumCnt++;
//			}
//			else
//			{
//				FocMotorPhaseAOffset = sumA >> 4;	
//				FocMotorPhaseBOffset = sumB >> 4;		
//				if ( (FocMotorPhaseAOffset > 2234) 		//基准电流不在1614-2234这个范围内
//					|| (FocMotorPhaseAOffset < 1614) 
//					|| (FocMotorPhaseBOffset > 2234) 
//					|| (FocMotorPhaseBOffset < 1614) )
//				{
//					Ctl.State=MOTOR_FAILURE;
//					Ctl.Error= E_CURR;			//基准电流保护
//					
//				}else{
//					Ctl.State=MOTOR_OPENLOOP;
//				}
//			}
//		}
		
		
		
	}
	if (++FocGetAdcCnt >= 3)			//采集3次
	{
		FocGetAdcCnt = 0;
		
		FocFlagAdcOK = 1;
	}
	
	//当自测通过后，开始测试hall角度，有位置偏转后开始输出
	if (FocSelfCheckOK)					//自测通过
	{
		MotorFetAngleFun ();				//====================================运行 电机角度检测
	}
	//FocMotorLoadBuf[FocGetAdcCnt]   = RegularConvData_Tab[0];  	  	//过流
	
	FocMotorRef					    = RegularConvData_Tab[0]; 
	FocMotorCurB 					= RegularConvData_Tab[1];	    //B相
	FocMotorCurA 					= RegularConvData_Tab[2];		//A相
	FocSpeedRef 				    = RegularConvData_Tab[3]>>2;       //电位器
	FocAdcBatteryBuf[FocGetAdcCnt]  = RegularConvData_Tab[4];		//母线电压
	MotorCurr_ab.qI_Component1 =    FocMotorPhaseAOffset - FocMotorCurA;			//先进行自测，再到这里输出自测的差值
	MotorCurr_ab.qI_Component2 = 	FocMotorPhaseBOffset - FocMotorCurB;
	MotorAtatVolt_qd.qV_Component1 = FocSpeednow;			//SpeedPWM;之前是从窗口获得初值	
	
	//自测通过，并且有hall角度变动后，开始输出
	if ( FocSelfCheckOK )
	{
		if ( MotorFlagMove )			//检测到启动位置
		{	
			MotorInitParaFun();
		}
		else if (  (FocMotorCurA <5000) 				//当两相的电流都小于   3848
				&& (FocMotorCurB < 5000) )
		{
			MotorClarkeFun();		//克拉克，帕克变换
			MotorParkFun();		
			MotorFlowRegFun();
		}
		MotorRevParkFun();			//反park变化
 		MotorCtrlFun();
	}
}
void TIM1_CC_IRQHandler (void)
{
	
	if (TIM1->SR & 0X0010)		//TIM status register,      			
	{		
		ADC_StartOfConversion(ADC1);		//开始连续采	

//		if(RegularConvData_Tab[0] > 310) 		//DMA里存储的adc采样值，		320次大于310
//		{										//过流保护的值
//			X ++;
//			if(X > 100)
//			TIM1->BDTR &= (~0x8000);	//TIM break and dead-time register,   关掉pwm输出
//			Ctl.State=MOTOR_FAILURE;
//			//TIM1->BDTR |= (0x8000);
//		}
//		else  X = 0;
//		
//		if(MsFlag > 50)
//			MsFlag = 0;
		
		TIM1->SR &= 0XFFEF;			//置位
	}	
}
void Uart_Buff_Clear (void)  
{
    u8 uart_data;
    for(uart_data = 0; uart_data < 25; uart_data++)
    {
        USART_RX_BUF[uart_data] = 0;
    }
}

void USART1_IRQHandler (void)   
{	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 		
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);						
		res = USART_ReceiveData(USART1); 				
        if((res_old == 0) && (res == 1))
        {
            if(cnt)
            {
                cnt = 0;
            }
            res_old = res;
            USART_RX_BUF[cnt] = res;
        }
        if((res_old == 1) && (res_old1 == 0) && (res == 20))
        {
            res_old1 = res;
            USART_RX_BUF[cnt] = res;
        }

        if((USART_RX_BUF[0] != 1) || (USART_RX_BUF[1] != 20))
        {
            if(cnt == 1)
            {
                res_old = 0;
                res_old1 = 0;
                cnt = 0;
                Uart_Buff_Clear();
            }
        }
        if((USART_RX_BUF[0] == 1) && (USART_RX_BUF[1] == 20))
        {
            if(cnt > 1)
            {
                USART_RX_BUF[cnt] = res;
            }
        }
        cnt++;
        if(cnt == 20)
        {
			filter_cnt ++;
            cnt = 0;
            res_old = 0;
            res_old1 = 0;
        }		
		/*	0地址				0x01
			1帧长				20
			2命令号				0x01
			3驱动方式			1-3				0助力驱动，1电驱动，2助力驱动和电驱动同时
			4助力设定			0-15			
			5控制器控制设定		
			6测速磁钢数			
			7轮径		0.1英寸
			8
			9助力灵敏度
			10助力启动强度
			11限速专用hall
			12限速值
			13控制器限流值
			14控制器欠压值
			15控制器欠压值
			16转把调速PWM掌控比
			17转把调速PWM掌控比
			18	控制器设定2
			19	助力磁钢盘类型选择
			20	校验和
			
			
		
		
		*/
		if(USART_RX_BUF[0]==0x01)		
		{
		 if((USART_RX_BUF[19]))
		 {
			 Yibiao_Set=USART_RX_BUF[5];
			 CiGangShu=USART_RX_BUF[6];
			 LunJing=(USART_RX_BUF[7]<<8)+USART_RX_BUF[8];
			 SpeedPWM=(USART_RX_BUF[16]<<8)+USART_RX_BUF[17];
			 Yibiao_CheckBit=USART_RX_BUF[19];
			 fuzhiwc_flg = 1;			 
			 if(USART_RX_BUF[5] & 0x40)
			 {
				 zero_run_flg = 1;
			 }
			 else
			 {
				 zero_run_flg = 0;
			 }			 
		 }
		}		
    }		
}


