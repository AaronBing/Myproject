/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * 
  *		��������ΪNew-Drive�ĵ綯������
  *
  *
  * 
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
uint8_t send_data[14];


uint8_t  State1 = 0;    
uint8_t  State2 = 0XC0;   
uint16_t Current = 1;  
uint16_t Velocity = 1000; 
uint16_t Check_Bit;

extern u8 MotorCurHall;
extern uint16_t RegularConvData_Tab[4];   
extern uint16_t TiaoSuPWM;

extern uint32_t	MsFlag;
extern uint8_t USART_RX_BUF[25];

/* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_DMA_Init(void);
//static void MX_ADC_Init(void);
//static void MX_DMA_Init(void);
//static void MX_TIM1_Init(void);
//static void MX_USART1_UART_Init(void);
//static void MX_NVIC_Init(void);
void TX_data(void);
void states(void);
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{	
	
	HardwareInit();
	
	board_self_test ();
	
	Ctl.State=MOTOR_INIT;
	
	while(1)
	{	
		//UI();			//�ⲿ����Ϊ����λ����ͨѶ
		MCL_Function();
		
		//TX_data();		ԭ���еģ�������
		//states();
	}
}



void SendData(uint8_t Data)
{
	 USART_SendData(USART1,Data); 
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
}

void TX_data(void)
{
	uint8_t i=0;	
	send_data[0]=2;			//0x02 ��ַ
	send_data[1]=14;		//14 ֡��
	send_data[2]=1;			//0x01�����
	send_data[3]=State1;  	/*BIT7	6kmѲ��״̬		1������6KMѲ��	0������
							  BIT6	HALL����		1������			0������
							  BIT5	ת�ѹ���			1������			0������
							  BIT4	����������		1������			0������								
							  BIT3	Ƿѹ����״̬		1��Ƿѹ      	0������									
							  BIT2	Ѳ��״̬			1������	Ѳ��	0������	
							  BIT1	ɲ�ѹ���			1��ɲ�ѹ���		0������	
							  BIT0	���ȱ��			1�����ȱ��		0������			*/

	send_data[4]=State2;	/*BIT7	���Ƿ���ˮƽ״̬		1��ˮƽ״̬			0������
							  BIT6	����������״̬			1������				0������
							  BIT5	�ϵ�ɲ��(ɲ��)			1���ϵ�ɲ������		0��û����
							  BIT4	ͨѶ����					1��ͨѶ����			0������								
							  BIT3	���״̬					1�������	      	0�������								
							  BIT2	����������״̬			1��������			0��������
							  BIT1		
							  BIT0	��λ				��ϳ�	0��1��2��3	*/

	
	send_data[5]=0x50;						//���е���		С����ǰ��λA
	send_data[6]=0x50;						//���е���		С�����		
	send_data[7]=0;							//��������ֵ		1-100%	
	send_data[8]=0x12;						//�ٶȷ���		��λms				((Velocity)>>8)&0xff;
	send_data[9]=0x12;						//�ٶȷ���						(Velocity)&0xff;
	send_data[10]=0x50;						//��ص���
	send_data[11]=2;						//ʣ�����
	send_data[12]=3;						//ʣ�����						
	Check_Bit=send_data[0]^send_data[1]^send_data[2]^send_data[3]^send_data[4]^send_data[5]^send_data[6]^send_data[7]\
	^send_data[8]^send_data[9]^send_data[10]^send_data[11]^send_data[12];	
	send_data[13]=Check_Bit;		
	if(MsFlag > 50)
	{					
		for(i=0;i<14;i++)
		{
			SendData(send_data[i]);
		}	
	}
}

void states(void)
{
	if((MotorCurHall == 7) || (MotorCurHall == 0)) 
		State1 |= 0x40;
	else  State1 &= 0xbf;	
	
	if( TiaoSuPWM == 0 )  
		State1 |= 0x20;
	else  State1 &= 0xdf;	
	
	if((RegularConvData_Tab[3] < 2314) || (RegularConvData_Tab[3] > 3400) )  //��ѹ��Ƿѹ����
		State1 |= 0x08;
	else State1 &= 0xf7;
	

}
//-----------------------------------------------------------





/*****************************END OF FILE***********************/
