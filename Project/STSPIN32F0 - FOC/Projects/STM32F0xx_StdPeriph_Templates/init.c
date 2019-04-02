
#include "main.h"

#define ADC1_DR_Address	   0x40012440
uint16_t RegularConvData_Tab[4];

void rcc_init (void)
{

}

void port_init (void)
{
		GPIO_InitTypeDef  	GPIO_InitStructure;
    
		/*初始化外设时钟*/
		RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOF, ENABLE);
		RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOC, ENABLE);  
		RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOB, ENABLE); 
		RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOA, ENABLE);

		/* F6,F7 输出	Push-pull*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;   
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOF, &GPIO_InitStructure);			//初始化写入
		GPIO_SetBits(GPIOF,GPIO_Pin_6|GPIO_Pin_7);  //F6,7 = 1 
	
	
		/* A0,A1,A2 	 输入模式*/
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2; 
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;	    	
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (GPIOA, &GPIO_InitStructure);			//初始化写入
	
		/* A6  输出 Push-pull*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;   
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);    //初始化写入
}

void pwm_init (void)
{
	
		GPIO_InitTypeDef 			GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  			TIM_OCInitStructure;
    TIM_BDTRInitTypeDef 		TIM_BDTRInitStruct;
	
		/*初始化外设时钟*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB , ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15 ; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; 
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);    
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_2); 
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_2);	
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_2);	
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12,GPIO_AF_2); 
	
		TIM_TimeBaseStructure.TIM_Period        	= 512; 
		TIM_TimeBaseStructure.TIM_Prescaler     	= 2; 
		TIM_TimeBaseStructure.TIM_ClockDivision 	= 0; 
		TIM_TimeBaseStructure.TIM_CounterMode 		= TIM_CounterMode_CenterAligned2;  
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);  
		TIM_BDTRInitStruct.TIM_OSSRState 		= TIM_OSSRState_Enable;
		TIM_BDTRInitStruct.TIM_OSSIState 		= TIM_OSSIState_Enable;
		TIM_BDTRInitStruct.TIM_LOCKLevel 		= TIM_LOCKLevel_OFF;
		TIM_BDTRInitStruct.TIM_DeadTime        	= 24;		
		TIM_BDTRInitStruct.TIM_Break           	= TIM_Break_Enable;  
		TIM_BDTRInitStruct.TIM_BreakPolarity   	= TIM_BreakPolarity_High;
		TIM_BDTRInitStruct.TIM_AutomaticOutput 	= TIM_AutomaticOutput_Disable;
		TIM_BDTRConfig( TIM1, &TIM_BDTRInitStruct);   
		TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1; 
		TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable; 
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Enable; 
		TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High; 
		TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
		TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;	
		TIM_OCInitStructure.TIM_Pulse = 0;
		TIM_OC1Init(TIM1, &TIM_OCInitStructure);  
		TIM_OCInitStructure.TIM_Pulse = 0; 
		TIM_OC2Init(TIM1, &TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_Pulse = 0; 
		TIM_OC3Init(TIM1, &TIM_OCInitStructure);		
		TIM_OCInitStructure.TIM_Pulse = 440;
		TIM_OC4Init(TIM1, &TIM_OCInitStructure);
		TIM1->CCER |= 0x04 | 0x40 |0x400|0x1000;
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);    
		TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);   
		TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable); 
		TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable); 	
		TIM_ARRPreloadConfig(TIM1, ENABLE);	
		TIM_ClearFlag(TIM1, TIM_IT_CC4);
		TIM_ITConfig (TIM1, TIM_IT_CC4, ENABLE);
		TIM_CtrlPWMOutputs(TIM1, ENABLE);
		TIM_Cmd(TIM1, ENABLE);
}
//==============================================================================
void adc_init (void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure; 
	DMA_InitTypeDef		DMA_InitStructure;
	ADC_InitTypeDef  	ADC_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);  
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;							
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 4;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);	
	DMA_Cmd(DMA1_Channel1, ENABLE);	
	ADC_InitStructure.ADC_ContinuousConvMode   = ENABLE;
	ADC_InitStructure.ADC_DataAlign 		   = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv 	   = ADC_ExternalTrigConv_T3_TRGO;
	ADC_InitStructure.ADC_Resolution 		   = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanDirection 	   = ADC_ScanDirection_Upward;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_ChannelConfig(ADC1, ADC_Channel_3 , ADC_SampleTime_1_5Cycles); 
	ADC_ChannelConfig(ADC1, ADC_Channel_4 , ADC_SampleTime_1_5Cycles); 
	ADC_ChannelConfig(ADC1, ADC_Channel_5,  ADC_SampleTime_1_5Cycles); 
	ADC_ChannelConfig(ADC1, ADC_Channel_9 , ADC_SampleTime_1_5Cycles); 	
	ADC_GetCalibrationFactor(ADC1);
	ADC_DMACmd(ADC1, ENABLE);	
	ADC_Cmd(ADC1, ENABLE); 	
	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);	
}
//==============================================================================
void nvic_init (void)
{
    NVIC_InitTypeDef 			NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel 			= TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority 	= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd 		= ENABLE;         
	NVIC_Init(&NVIC_InitStructure);  
    NVIC_InitStructure.NVIC_IRQChannel 			= ADC1_COMP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority 	= 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd 		= ENABLE;         
	NVIC_Init(&NVIC_InitStructure);   	
	NVIC_InitStructure.NVIC_IRQChannel 			= USART1_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPriority	= 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd 		= ENABLE;     	
	NVIC_Init(&NVIC_InitStructure); 
}
void usart_init (void)
{

	USART_InitTypeDef   	USART_InitStructure;
	GPIO_InitTypeDef 		GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB , ENABLE);  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;   
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;  
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);		
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_0); 
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_0);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);	
	USART_ITConfig( USART1, USART_IT_RXNE, ENABLE); 
	USART_Cmd( USART1, ENABLE );
}
void i2c_init (void)
{
	
}

void HardwareInit (void)
{
		uint16_t	i;
		i = 20000;
		while (i--);
	
		rcc_init ();
		port_init ();
		pwm_init ();
		adc_init ();
		nvic_init ();
		usart_init ();
		i2c_init ();	
}
