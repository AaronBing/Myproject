/***********************************************************
*	��ʼ��
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
	//���Ƶ��PA8 PB13 PA9 PB14 PA10 PB15  (TIM1  ------- L ��ߵ��)
	//���Ƶ��PC7 PB0  PB1 PC8  PC6  PA7   (TIM8  ------- R �ұߵ��)
	//PC2 	ĸ�ߵ�ѹ���  ���ad���
	//PA1  	��ⰴ���Ƿ���  (��������)  input
	//PA5   ������������֮��PA5ά�ֵ�Դ��ͨ  output
	
	//PC0 	���˷����--------ĸ�ߵ������------��
	//PC1 	���˷����--------ĸ�ߵ������------��
	//PA6 	TIM8_BKIN
	//PB12 	TIM1_BKIN
	//PA0 PC3  u v�������  �ɼ�������MOS�ܵ�����
	//PC4 PC5  u v������  	�ɼ�������MOS�ܵ����� (����һ�������)
	//PA4 	����
  GPIO_InitTypeDef type;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|
						RCC_APB2Periph_GPIOB|
						RCC_APB2Periph_GPIOA|
						RCC_APB2Periph_AFIO, ENABLE);//ʹ�ܻ���ʧ�� APB2 ����ʱ��
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//�ı�ָ���ܽŵ�ӳ�� JTAG-DP ��ֹ + SW-DP ʹ��
  
	/**************************/
	//USART PA3 PA2 
	//������ӿڼ�� PA12
	type.GPIO_Pin = 72;//PA3 PA6 
  type.GPIO_Speed = GPIO_Speed_50MHz;
  type.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &type);
	
  type.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;//PA0 PA1 ad����
  type.GPIO_Speed = GPIO_Speed_50MHz;
  type.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &type);
	
  type.GPIO_Pin = 1924;//PA2��7��8��9��10
  type.GPIO_Speed = GPIO_Speed_50MHz;
  type.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &type);
	
  type.GPIO_Pin = 24624;//PA4��5��13��14
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
	//û��
  type.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;//PD0\1
  type.GPIO_Speed = GPIO_Speed_2MHz;
  type.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOD, &type);
}

////-------------------------------------------------------------
//void InitPWMLFun()
//{//�μ���ʼ�����ߵ����pwm����
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);//ʹ�ܻ���ʧ�� APB2 ����ʱ��
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
{//��ʼ�����ߵ����pwm����
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//ʹ�ܻ���ʧ�� APB2 ����ʱ��
  TIM1->CR1 = 0xc4;			//timx_arr�Ĵ�����װ�뻺���������ϼ�������
												//���������¼�ʱ����������ֹͣ
												//�����ʱ������ж�
												//��ֹuev�¼�
												//��ֹ������
  TIM1->CR2 = 0x2A21;		//
												//�������״̬3��oc3n�������ois1nλ
												//�������״̬2��oc2n������� ois1nλ
												//���moe=0��������oc1n=0
												//���moe=0��ʵ����oc1n����������oc1=1
												//timx_ch1��������ti1����
												//�����¼���ѡ�봥��trgo
												//������񡢱ȽϿ���λ��Ԥװ�صġ�ֻ��ͨ������comλ��������
												//ccxne\ocxmλ��Ԥװ�صģ�����ֻ��������comλ�󱻸��£����������ͨ��������
  TIM1->DIER = 0x81;		//�жϿ���
												//��ֹdma
												//����ɲ���ж�
												//��ֹ�����ж�
												//��ֹcom�ж�
												//��ֹ����Ƚ�1��2��3��4�ж�
												//��������ж�
  TIM1->CCMR1 = 0x6868;
												//����oc2��pwmģʽ�����ϼ���ʱ��timx_cnt<timx_ccr1��ͨ��Ϊ��Ч��ƽ��������Ч
												//oc2pe����Ƚ�2Ԥװ��ʹ��
												//cc2ͨ��������Ϊ���
												//����oc1��pwmģʽ�����ϼ���ʱ��timx_cnt<timx_ccr1��ͨ��Ϊ��Ч��ƽ��������Ч
												//����timx_ccr1�Ĵ�����Ԥװ�ع���
												//cc1ͨ��������Ϊ���
  TIM1->CCMR2 = 0x68;
												//����oc3��pwmģʽ�����ϼ���ʱ��timx_cnt<timx_ccr1��ͨ��Ϊ��Ч��ƽ��������Ч
												//����timx_ccr3�Ĵ�����Ԥװ�ع���
												//cc3ͨ��������Ϊ���
  TIM1->CCER = 0x888;		//
	TIM1->PSC = 3;				//4��Ƶ��3+1
  TIM1->ARR = 512;			//����pwm����  T = ((64M/512)/4)/2 = 15.625k
	TIM1->RCR = 0;
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;
  TIM1->BDTR = 0x9020;	//����pwm
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCER |= 0x555;	//0xddd
												//����oc1�͵�ƽ��Ч\oc1n�ߵ�ƽ��Ч
												//����oc2�͵�ƽ��Ч\oc2n�ߵ�ƽ��Ч
												//����oc3�͵�ƽ��Ч\oc3n�ߵ�ƽ��Ч
  TIM1->EGR = 0x21;
												//��ccpc=1���������ccxe\ccxne\ocxmλ
												//��ֹ����Ƚ��¼�
												//���������¼��޶���
  TIM1->RCR = 1;				//pwm������Ŀ
  TIM1->CR1 |= 1;				//ʹ�ܼ�����
}
//-------------------------------------------------------------
void InitNvicFun()
{//���ȼ���ʼ��
  NVIC_InitTypeDef nvic;
  nvic.NVIC_IRQChannel=25;//TIM1_UP_IRQChannel  
  nvic.NVIC_IRQChannelPreemptionPriority=1;
  nvic.NVIC_IRQChannelSubPriority=1;
  nvic.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&nvic);//���� NVIC_InitStruct ��ָ���Ĳ�����ʼ������ NVIC �Ĵ���
}

//-------------------------------------------------------------
void InitADCFun()
{//��ʼ��adc
  RCC->APB2ENR |= 0x8600u;//ʱ������
  RCC->APB2RSTR |= 0x600u;
  RCC->APB2RSTR &= 0xFFFFF9FF;
  RCC->CFGR &= 0xFFFF3FFF;//ʱ������
  RCC->CFGR |= 0x8000u;
	
  ADC1->CR1 = 0;
  ADC2->CR1 = 256;
  ADC3->CR1 = 256;//ɨ��ģʽ
	
	ADC1->CR2 = 0;
	ADC2->CR2 = 0x8000;  //��ʱ��T1��TRGO�¼�,�Ҷ��룬��ʹ��DMA
	ADC3->CR2 = 0xC000;  //TIM3_CC4�¼�,�Ҷ���
	//4��ת�� ADC3_H13 ADC3_H0  ADC3_H10 ADC3_H12
	//PC3 PA0 PC0 PC2
	//w u�� ��ؼ��
  ADC3->JSQR = 0x36280D;
	//4��ת�� ADC2_H14 ADC2_H15 ADC2_H11 ADC2_H1
	//PC4 PC5 PC1 PA1
	//w u�� ����
  ADC2->JSQR = 0x30ADEE;
	
  ADC1->CR2 |= 1;//ʹ�� ����ģʽ
  ADC2->CR2 |= 1;//ʹ�� ����ģʽ �ⲿʱ������ת�� T1��TRGO �¼�
  ADC3->CR2 |= 1;//ʹ�� ����ģʽ �ⲿʱ������ת�� T3��CC4 �¼�
  
	ADC1->CR2 |= 8;//ʹ��
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
  RCC_DeInit();//������ RCC �Ĵ�������Ϊȱʡֵ
  RCC_HSICmd(ENABLE);														//ʹ���ڲ����پ���HSI
  while ( !RCC_GetFlagStatus(RCC_FLAG_HSIRDY) )	//���ָ���� RCC ��־λ�������
    ;
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);//ʹ�ܻ���ʧ��Ԥȡָ����
  FLASH_SetLatency(FLASH_Latency_2);						//���ô�����ʱֵ
  RCC_HCLKConfig(0);														//���� AHB ʱ��
  RCC_PCLK2Config(0);														//���ø��� AHB ʱ�ӣ�PCLK2��
  RCC_PCLK1Config(1024);												//���õ��� AHB ʱ�ӣ�PCLK1��
	//����ʹ�õ���8/2*16=64m����Ƶ
  RCC_PLLConfig(0, RCC_PLLMul_16);							//���� PLL ʱ��Դ����Ƶϵ��
  RCC_PLLCmd(ENABLE);														//ʹ�ܻ���ʧ�� PLL
  while ( !RCC_GetFlagStatus(RCC_FLAG_PLLRDY) )	//���ָ���� RCC ��־λ�������
    ;
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  while ( RCC_GetSYSCLKSource() != 8 );					//����ϵͳʱ��PLL used as system clock
}

////-------------------------------------------------------------
//void InitUsartFun()
//{//������26315
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

//	//USART2->BRR = 0x3e8;//32000 = (64000000/2/32000)
//  //USART3->BRR = 0x3e8;

//  USART2->BRR = (64000000/2/26315);//�����ʼĴ���0x4c0
//  USART3->BRR = (64000000/2/26315);//�����ʼĴ���

//  /*USART2->CR1 = 0x300C;//uartģ��ʹ�ܡ�9λ����λ��ʹ�ܷ��͡�����
//  USART3->CR1 = 0x300C;*/
//	//ƥ��mm�鶯΢
//  USART2->CR1 = 0x200C;//uartģ��ʹ�ܡ�8λ����λ��ʹ�ܷ��͡�����
//  USART3->CR1 = 0x200C;
//}

//-------------------------------------------------------------
void InitFun()
{//��ʼ��
  u16 i; 
	
	//��ʱ
  i = 20000;
  while ( i-- );
  InitRccFun();
  InitPortFun();
  InitPWMSFun();//�������������ʼ��1------TIM1
//  InitPWMLFun();//�������������ʼ��2------TIM8
  InitADCFun();//
  InitNvicFun();
//  InitUsartFun();
}
