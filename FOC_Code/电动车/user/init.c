/***********************************************************
*	初始化
*
*
*
************************************************************/
#include "stm32f10x.h"	

void InitPortFun(void);
void InitPWMLFun(void);
void InitPWMSFun(void);
void InitNvicFun(void);
void InitADCFun(void);
void InitRccFun(void);
//void InitUsartFun(void);
void InitFun(void);


//-------------------------------------------------------------
void InitPortFun()
{//ok
	//控制电机PA8 PB13 PA9 PB14 PA10 PB15  (TIM1  ------- L 左边电机)
	//控制电机PC7 PB0  PB1 PC8  PC6  PA7   (TIM8  ------- R 右边电机)
	//PC2 	母线电压检测  电池ad检测
	//PA1  	检测按键是否按下  (开机按键)  input
	//PA5   开机按键按下之后，PA5维持电源导通  output
	
	//PC0 	从运放输出--------母线电流检测------右
	//PC1 	从运放输出--------母线电流检测------左
	//PA6 	TIM8_BKIN
	//PB12 	TIM1_BKIN
	//PA0 PC3  u v电流检测  采集了下桥MOS管的内阻
	//PC4 PC5  u v电流相  	采集了下桥MOS管的内阻 (另外一个电机的)
	//PA4 	喇叭
  GPIO_InitTypeDef type;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|
						RCC_APB2Periph_GPIOB|
						RCC_APB2Periph_GPIOA|
						RCC_APB2Periph_AFIO, ENABLE);//使能或者失能 APB2 外设时钟
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//改变指定管脚的映射 JTAG-DP 禁止 + SW-DP 使能
  
	/**************************/
	//USART PA3 PA2 
	//充电器接口检测 PA12
	type.GPIO_Pin = 72;//PA3 PA6 
  type.GPIO_Speed = GPIO_Speed_50MHz;
  type.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &type);
	
  type.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;//PA0 PA1 ad输入
  type.GPIO_Speed = GPIO_Speed_50MHz;
  type.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &type);
	
  type.GPIO_Pin = 1924;//PA2、7、8、9、10
  type.GPIO_Speed = GPIO_Speed_50MHz;
  type.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &type);
	
  type.GPIO_Pin = 24624;//PA4、5、13、14
  type.GPIO_Speed = GPIO_Speed_50MHz;
  type.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &type);
	
  type.GPIO_Pin = 0x9800;//PA11 12  15
  type.GPIO_Speed = GPIO_Speed_2MHz;
  type.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &type);
	
  type.GPIO_Pin = 4096;//PA12
  type.GPIO_Speed = GPIO_Speed_2MHz;
  type.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &type);
	/**************************/
	//USART PB10 PB11
	//LED PB2
	//HALL_L PB5 6 7
  type.GPIO_Pin = 792;//PB3\4\8\9
  type.GPIO_Speed = GPIO_Speed_2MHz;
  type.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOB, &type);
	
  type.GPIO_Pin = 4;//PB2
  type.GPIO_Speed = GPIO_Speed_50MHz;
  type.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &type);
	
  type.GPIO_Pin = 6368;//PB5\6\7\11\12
  type.GPIO_Speed = GPIO_Speed_50MHz;
  type.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &type);
	
  type.GPIO_Pin = 0xE403;//PB0\1\10\13\14\15
  type.GPIO_Speed = GPIO_Speed_50MHz;
  type.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &type);
	
	/**************************/
	//HALL_R PC10 11 12
  type.GPIO_Pin = 7168;//PC10\11\12
  type.GPIO_Speed = GPIO_Speed_50MHz;
  type.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &type);
	
  type.GPIO_Pin = 63;//PC0\1\2\3\4\5
  type.GPIO_Speed = GPIO_Speed_50MHz;
  type.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &type);
	
  type.GPIO_Pin = 960;//PC6\7\8\9
  type.GPIO_Speed = GPIO_Speed_50MHz;
  type.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &type);
	
  type.GPIO_Pin = 0xE000;//PC13\14\15
  type.GPIO_Speed = GPIO_Speed_2MHz;
  type.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOC, &type);
	//没用
  type.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;//PD0\1
  type.GPIO_Speed = GPIO_Speed_2MHz;
  type.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOD, &type);
}

////-------------------------------------------------------------
//void InitPWMLFun()
//{//参见初始化短线电机的pwm配置
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);//使能或者失能 APB2 外设时钟
//  TIM8->CR1 = 196;
//  TIM8->CR2 = 0x2A21;
//  TIM8->DIER = 0x81;
//  TIM8->CCMR1 = 0x6868;
//  TIM8->CCMR2 = 0x3868;
//	TIM8->CCER = 0x888;		
//  TIM8->PSC = 3;
//  TIM8->ARR = 512;
//	TIM8->RCR = 0;
//  TIM8->CCR1 = 0;
//  TIM8->CCR2 = 0;
//  TIM8->CCR3 = 0;
//  TIM8->CCR4 = 0;
//  TIM8->BDTR = 0x9020;
//  TIM8->CCR1 = 0;
//  TIM8->CCR2 = 0;
//  TIM8->CCR3 = 0;
//  TIM8->CCR4 = 510;
//  TIM8->CCER |= 0x1555;
//  TIM8->EGR = 33;
//  TIM8->RCR = 1;
//  TIM8->CR1 |= 1;
//}

//-------------------------------------------------------------
void InitPWMSFun()
{//初始化短线电机的pwm控制
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//使能或者失能 APB2 外设时钟
  TIM1->CR1 = 0xc4;			//timx_arr寄存器被装入缓冲器、向上计数器、
												//发生更新事件时，计数器不停止
												//溢出的时候产生中断
												//禁止uev事件
												//禁止计数器
  TIM1->CR2 = 0x2A21;		//
												//输出空闲状态3，oc3n输出，见ois1n位
												//输出空闲状态2，oc2n输出，见 ois1n位
												//如果moe=0，死区后oc1n=0
												//如果moe=0，实现了oc1n，则死区后oc1=1
												//timx_ch1引脚引入ti1输入
												//更新事件被选入触发trgo
												//如果捕获、比较控制位是预装载的、只能通过设置com位更新他们
												//ccxne\ocxm位是预装载的；他们只在设置了com位后被更新，互补输出的通道起作用
  TIM1->DIER = 0x81;		//中断控制
												//禁止dma
												//允许刹车中断
												//禁止触发中断
												//禁止com中断
												//禁止捕获比较1、2、3、4中断
												//允许更新中断
  TIM1->CCMR1 = 0x6868;
												//开启oc2，pwm模式，向上计数时，timx_cnt<timx_ccr1，通道为有效电平，否则无效
												//oc2pe输出比较2预装载使能
												//cc2通道被配置为输出
												//开启oc1，pwm模式，向上计数时，timx_cnt<timx_ccr1，通道为有效电平，否则无效
												//开启timx_ccr1寄存器的预装载功能
												//cc1通道被配置为输出
  TIM1->CCMR2 = 0x68;
												//开启oc3，pwm模式，向上计数时，timx_cnt<timx_ccr1，通道为有效电平，否则无效
												//开启timx_ccr3寄存器的预装载功能
												//cc3通道被配置为输出
  TIM1->CCER = 0x888;		//
	TIM1->PSC = 3;				//4分频，3+1
  TIM1->ARR = 512;			//计算pwm周期  T = ((64M/512)/4)/2 = 15.625k
	TIM1->RCR = 0;
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;
  TIM1->BDTR = 0x9020;	//开启pwm
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCER |= 0x555;	//0xddd
												//开启oc1低电平有效\oc1n高电平有效
												//开启oc2低电平有效\oc2n高电平有效
												//开启oc3低电平有效\oc3n高电平有效
  TIM1->EGR = 0x21;
												//当ccpc=1，允许更新ccxe\ccxne\ocxm位
												//禁止捕获比较事件
												//产生更新事件无动作
  TIM1->RCR = 1;				//pwm周期数目
  TIM1->CR1 |= 1;				//使能计数器
}
//-------------------------------------------------------------
void InitNvicFun()
{//优先级初始化
  NVIC_InitTypeDef nvic;
  nvic.NVIC_IRQChannel=25;//TIM1_UP_IRQChannel  
  nvic.NVIC_IRQChannelPreemptionPriority=1;
  nvic.NVIC_IRQChannelSubPriority=1;
  nvic.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&nvic);//根据 NVIC_InitStruct 中指定的参数初始化外设 NVIC 寄存器
}

//-------------------------------------------------------------
void InitADCFun()
{//初始化adc
  RCC->APB2ENR |= 0x8600u;//时钟配置
  RCC->APB2RSTR |= 0x600u;
  RCC->APB2RSTR &= 0xFFFFF9FF;
  RCC->CFGR &= 0xFFFF3FFF;//时钟配置
  RCC->CFGR |= 0x8000u;
	
  ADC1->CR1 = 0;
  ADC2->CR1 = 256;
  ADC3->CR1 = 256;//扫描模式
	
	ADC1->CR2 = 0;
	ADC2->CR2 = 0x8000;  //定时器T1的TRGO事件,右对齐，不使用DMA
	ADC3->CR2 = 0xC000;  //TIM3_CC4事件,右对齐
	//4个转换 ADC3_H13 ADC3_H0  ADC3_H10 ADC3_H12
	//PC3 PA0 PC0 PC2
	//w u相 电池检测
  ADC3->JSQR = 0x36280D;
	//4个转换 ADC2_H14 ADC2_H15 ADC2_H11 ADC2_H1
	//PC4 PC5 PC1 PA1
	//w u相 开关
  ADC2->JSQR = 0x30ADEE;
	
  ADC1->CR2 |= 1;//使能 触发模式
  ADC2->CR2 |= 1;//使能 触发模式 外部时间启动转换 T1的TRGO 事件
  ADC3->CR2 |= 1;//使能 触发模式 外部时间启动转换 T3的CC4 事件
  
	ADC1->CR2 |= 8;//使能
  while ( ADC1->CR2 & 8 )
    ;
  ADC1->CR2 |= 4u;
  while ( ADC1->CR2 & 4 )
    ;
  ADC2->CR2 |= 8u;
  while ( ADC2->CR2 & 8 )
    ;
  ADC2->CR2 |= 4u;
  while ( ADC2->CR2 & 4 )
    ;
  ADC3->CR2 |= 8u;
  while ( ADC3->CR2 & 8 )
    ;
  ADC3->CR2 |= 4u;
  while ( ADC3->CR2 & 4 )
		;
}

//-------------------------------------------------------------
void InitRccFun()
{//OK
  RCC_DeInit();//将外设 RCC 寄存器重设为缺省值
  RCC_HSICmd(ENABLE);														//使能内部高速晶振HSI
  while ( !RCC_GetFlagStatus(RCC_FLAG_HSIRDY) )	//检查指定的 RCC 标志位设置与否
    ;
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);//使能或者失能预取指缓存
  FLASH_SetLatency(FLASH_Latency_2);						//设置代码延时值
  RCC_HCLKConfig(0);														//设置 AHB 时钟
  RCC_PCLK2Config(0);														//设置高速 AHB 时钟（PCLK2）
  RCC_PCLK1Config(1024);												//设置低速 AHB 时钟（PCLK1）
	//这里使用的是8/2*16=64m的主频
  RCC_PLLConfig(0, RCC_PLLMul_16);							//设置 PLL 时钟源及倍频系数
  RCC_PLLCmd(ENABLE);														//使能或者失能 PLL
  while ( !RCC_GetFlagStatus(RCC_FLAG_PLLRDY) )	//检查指定的 RCC 标志位设置与否
    ;
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  while ( RCC_GetSYSCLKSource() != 8 );					//设置系统时钟PLL used as system clock
}

////-------------------------------------------------------------
//void InitUsartFun()
//{//波特率26315
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

//	//USART2->BRR = 0x3e8;//32000 = (64000000/2/32000)
//  //USART3->BRR = 0x3e8;

//  USART2->BRR = (64000000/2/26315);//波特率寄存器0x4c0
//  USART3->BRR = (64000000/2/26315);//波特率寄存器

//  /*USART2->CR1 = 0x300C;//uart模块使能、9位数据位、使能发送、接收
//  USART3->CR1 = 0x300C;*/
//	//匹配mm灵动微
//  USART2->CR1 = 0x200C;//uart模块使能、8位数据位、使能发送、接收
//  USART3->CR1 = 0x200C;
//}

//-------------------------------------------------------------
void InitFun()
{//初始化
  u16 i; 
	
	//延时
  i = 20000;
  while ( i-- );
  InitRccFun();
  InitPortFun();
  InitPWMSFun();//短线驱动电机初始化1------TIM1
//  InitPWMLFun();//长线驱动电机初始化2------TIM8
  InitADCFun();//
  InitNvicFun();
//  InitUsartFun();
}
