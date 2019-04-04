
#include "main.h"

#define ADC1_DR_Address	   0x40012440
uint16_t RegularConvData_Tab[4];

void rcc_init (void)
{

}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
		GPIO_InitTypeDef  	GPIO_InitStructure;
    
		/*初始化外设时钟*/
		RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOF, ENABLE);
		RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOC, ENABLE);  
		RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOB, ENABLE); 
		RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOA, ENABLE);

		/* PF6,PF7 输出	Push-pull*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;   
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOF, &GPIO_InitStructure);			
		GPIO_SetBits(GPIOF,GPIO_Pin_6|GPIO_Pin_7);  //F6,7 = 1 
	
	
		/* PA0,PA1,PA2 	 输入模式  用于hall检测 */
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2; 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;	    	
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init (GPIOA, &GPIO_InitStructure);			
	
		/* PA6  输出 Push-pull  	PWM,ADC 	*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;   
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);    
}

static void pwm_init (void)
{
	
		GPIO_InitTypeDef 			GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  			TIM_OCInitStructure;
		TIM_BDTRInitTypeDef 		TIM_BDTRInitStruct;
	
		/*初始化外设时钟*/
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB , ENABLE);  
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
		/*PA8,PA9,PA10  复用功能 Push-pull 		TIM1 LS*/
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
		/*PB13,PB14,PB15  复用功能 Push-pull 	TIM1 HS*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15 ; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);	 
		
		/*PB12  输入 上拉*/
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; 
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);   

		
		/*PA8,PA9,PA10 */
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_2);   //  TIM2, TIM1, EVENTOUT, TIM16, TIM17,   上面使能了tim1
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_2);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_2);

		/*PB13,PB14,PB15 */
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_2);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_2);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_2);	
		//PB12
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_2);   //比较器
	
		
		TIM_TimeBaseStructure.TIM_Period        	= 512; //自动重载计数周期值  21.33us
		TIM_TimeBaseStructure.TIM_Prescaler     	= 2; 	//分频系数    24mhz
		TIM_TimeBaseStructure.TIM_ClockDivision 	= 0; 		
		TIM_TimeBaseStructure.TIM_CounterMode 		= TIM_CounterMode_CenterAligned2;  
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 

		/*断路 死区*/
		TIM_BDTRInitStruct.TIM_OSSRState 		= TIM_OSSRState_Enable;  //运行
		TIM_BDTRInitStruct.TIM_OSSIState 		= TIM_OSSIState_Enable;  //关闭
		TIM_BDTRInitStruct.TIM_LOCKLevel 		= TIM_LOCKLevel_OFF;     //锁存
		TIM_BDTRInitStruct.TIM_DeadTime        	= 24;		
		TIM_BDTRInitStruct.TIM_Break           	= TIM_Break_Enable;      //断路控制
		TIM_BDTRInitStruct.TIM_BreakPolarity   	= TIM_BreakPolarity_High; //断路输入极性
		TIM_BDTRInitStruct.TIM_AutomaticOutput 	= TIM_AutomaticOutput_Disable;  //自动输出
		TIM_BDTRConfig( TIM1, &TIM_BDTRInitStruct);  

		/*输出比较*/
		TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1; //比较输出模式
		TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable; //比较输出使能
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Enable; //比较互补输出使能
		TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;   //输出极性
		TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;  //互补输出极性
		TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Set;		// 空闲状态下比较输出状态
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;	// 空闲状态下比较互补输出状态
		TIM_OCInitStructure.TIM_Pulse = 0;	//脉冲宽度
		TIM_OC1Init(TIM1, &TIM_OCInitStructure);  
		
		
		TIM_OCInitStructure.TIM_Pulse = 0; 
		TIM_OC2Init(TIM1, &TIM_OCInitStructure);
		
		TIM_OCInitStructure.TIM_Pulse = 0; 
		TIM_OC3Init(TIM1, &TIM_OCInitStructure);		
		TIM_OCInitStructure.TIM_Pulse = 440;
		TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //
		
		
		TIM1->CCER |= 0x04 | 0x40 |0x400|0x1000; //使能tim1比较寄存器
		
		//TIMx_CCRx寄存器能够在任何时候通过软件进行更新以控制波形
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);    //开启/禁止TIMx_CCR1寄存器的预装载功能
		TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);   
		TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable); 
		TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable); 	
		
		TIM_ARRPreloadConfig(TIM1, ENABLE);	//允许或禁止在定时器工作时向ARR的缓冲器中写入新值，
		TIM_ClearFlag(TIM1, TIM_IT_CC4);
		TIM_ITConfig (TIM1, TIM_IT_CC4, ENABLE);//
		TIM_CtrlPWMOutputs(TIM1, ENABLE);
		TIM_Cmd(TIM1, ENABLE);//使能 TIMx
}
//==============================================================================
static void adc_init (void)
{
	//初始化变量
	GPIO_InitTypeDef 	GPIO_InitStructure; 
	DMA_InitTypeDef		DMA_InitStructure;
	ADC_InitTypeDef  	ADC_InitStructure;
	
	/*初始化外设时钟*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);  
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	/*PA3,PA4,PA5  Analog In/Out Mode */
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*PB1  Analog In/Out Mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;							
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;  //
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;  //
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//单向传输
	DMA_InitStructure.DMA_BufferSize = 4;			//4*16bit
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //设置DMA的外设递增模式，如果DMA选用的通道（CHx）有多个外设连接，需要使用外设递增模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;			//DMA访问多个内存参数
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//设置DMA在访问时每次操作的数据长度   16bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//连续不断的循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  //DMA优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA的2个memory中的变量互相访问的
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);	//初始化
	DMA_Cmd(DMA1_Channel1, ENABLE);	//使能
	
	
	ADC_InitStructure.ADC_ContinuousConvMode   = ENABLE;		//使能连续转换
	ADC_InitStructure.ADC_DataAlign 		   = ADC_DataAlign_Right;//右对齐
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测
	ADC_InitStructure.ADC_ExternalTrigConv 	   = ADC_ExternalTrigConv_T3_TRGO;//数模转换外部通道tim3 为触发源
	ADC_InitStructure.ADC_Resolution 		   = ADC_Resolution_12b;  //12bit 分辨率
	ADC_InitStructure.ADC_ScanDirection 	   = ADC_ScanDirection_Upward;//扫描通道方向
	ADC_Init(ADC1, &ADC_InitStructure);
	
	
	ADC_ChannelConfig(ADC1, ADC_Channel_3 , ADC_SampleTime_1_5Cycles); //采样延时
	ADC_ChannelConfig(ADC1, ADC_Channel_4 , ADC_SampleTime_1_5Cycles); 
	ADC_ChannelConfig(ADC1, ADC_Channel_5,  ADC_SampleTime_1_5Cycles); 
	ADC_ChannelConfig(ADC1, ADC_Channel_9 , ADC_SampleTime_1_5Cycles); 	
	ADC_GetCalibrationFactor(ADC1);         //校准adc
	ADC_DMACmd(ADC1, ENABLE);				//关联DMA
	ADC_Cmd(ADC1, ENABLE); 	
	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);	
}
//==============================================================================
static void nvic_init (void)
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
static void usart_init (void)
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
		MX_GPIO_Init();
		pwm_init ();
		adc_init ();
		nvic_init ();
		usart_init ();
		i2c_init ();	
}
