
#include "main.h"

extern uint32_t	FocSelfCheckOK;
extern int16_t 	MotorFlagMove;
extern Curr_Components 		MotorCurr_ab;
extern Volt_Components 		MotorAtatVolt_qd;
extern uint16_t 	FocMotorPhaseAOffset;
extern uint16_t 	FocMotorPhaseBOffset;
extern uint16_t 	MotorFinalEleAngle;
extern uint16_t     RegularConvData_Tab[4];
extern void TX_data(void);
uint16_t	vadc_pwroff_bat;
uint16_t	vadc_pwron_bat;
uint16_t	vadc_ibus;
uint16_t	vadc_ia, vadc_ib, vadc_ic;
uint32_t	FocTime64usCnt; 
uint32_t	FocTime1msFlag;  
uint32_t	MsFlag; 
uint32_t	FocGetAdcCnt;
uint32_t	FocFlagAdcOK;
uint16_t 	FocBootVolBuf[4];
uint16_t	FocAdcBatteryBuf[4] = {0};
uint16_t    FocMotorLoadBuf[4] = {0};
uint16_t	FocMotorCurA;
uint16_t	FocMotorCurB;
uint8_t cnt = 0;
uint8_t res_old = 0;
uint8_t res_old1 = 0;
uint8_t res;
uint8_t USART_RX_BUF[25];     
u8 Yibiao_Set, fuzhiwc_flg = 0;
u8 CiGangShu;
u16 LunJing;
u16 TiaoSuPWM = 0;
u16 TiaoSuPWM1=0;
u16 TiaoSuPWM2=0;
u8 Yibiao_CheckBit;
u8 zero_run_flg;

uint8_t filter_cnt = 0;  //ÂË²¨¼ÆÊý
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
//		 filter_pwm[filter_cnt] = TiaoSuPWM;
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
	ADC_ClearFlag (ADC1, ADC_FLAG_EOC);
	GPIO_SetBits (GPIOC, GPIO_Pin_13);
	if (++FocTime64usCnt >= 16)
	{
		FocTime64usCnt = 0;
		MsFlag ++;
		FocTime1msFlag = 1;
	}
	if (++FocGetAdcCnt >= 3)
	{
		FocGetAdcCnt = 0;
		
		FocFlagAdcOK = 1;
	}
	if (FocSelfCheckOK)
	{
		MotorFetAngleFun ();
	}
	FocMotorLoadBuf[FocGetAdcCnt]   = RegularConvData_Tab[0];  	
	FocMotorCurB 					= RegularConvData_Tab[1];	
	FocMotorCurA 					= RegularConvData_Tab[2];		
	FocAdcBatteryBuf[FocGetAdcCnt]  = RegularConvData_Tab[3];     
	MotorCurr_ab.qI_Component1 =    FocMotorPhaseAOffset - FocMotorCurA;
	MotorCurr_ab.qI_Component2 = 	FocMotorPhaseBOffset - FocMotorCurB;
	MotorAtatVolt_qd.qV_Component1 = 300;//TiaoSuPWM;
	if ( FocSelfCheckOK )
	{
		if ( MotorFlagMove )
		{	
			MotorInitParaFun();
		}
		else if (  (FocMotorCurA < 3848) 
				&& (FocMotorCurB < 3848) )
		{
			MotorClarkeFun();
			MotorParkFun();
			MotorFlowRegFun();
		}
		MotorRevParkFun();
		MotorCtrlFun();
	}
}
void TIM1_CC_IRQHandler (void)
{
	
	if (TIM1->SR & 0X0010)					
	{		
		ADC_StartOfConversion(ADC1);	

		if(RegularConvData_Tab[0] > 310) 
		{
			X ++;
			if(X > 320)
			TIM1->BDTR &= (~0x8000);
		}
		else  X = 0;
		
		if(MsFlag > 50)
			MsFlag = 0;				
		TIM1->SR &= 0XFFEF;
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
		if(USART_RX_BUF[0]==0x01)
		{
		 if((USART_RX_BUF[19]))
		 {
			 Yibiao_Set=USART_RX_BUF[5];
			 CiGangShu=USART_RX_BUF[6];
			 LunJing=(USART_RX_BUF[7]<<8)+USART_RX_BUF[8];
			 TiaoSuPWM=(USART_RX_BUF[16]<<8)+USART_RX_BUF[17];
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


