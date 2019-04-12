/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * 
  *		本程序作为New-Drive的电动车程序
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
		//UI();			//这部分作为和上位机的通讯
		MCL_Function();
		
		//TX_data();		原先有的，先留着
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
	send_data[0]=2;			//0x02 地址
	send_data[1]=14;		//14 帧长
	send_data[2]=1;			//0x01命令号
	send_data[3]=State1;  	/*BIT7	6km巡航状态		1：正在6KM巡航	0：不在
							  BIT6	HALL故障		1：故障			0：正常
							  BIT5	转把故障			1：故障			0：正常
							  BIT4	控制器故障		1：故障			0：正常								
							  BIT3	欠压保护状态		1：欠压      	0：正常									
							  BIT2	巡航状态			1：正在	巡航	0：不在	
							  BIT1	刹把故障			1：刹把故障		0：正常	
							  BIT0	电机缺相			1：电机缺相		0：正常			*/

	send_data[4]=State2;	/*BIT7	车是否处于水平状态		1：水平状态			0：不在
							  BIT6	助力传感器状态			1：故障				0：正常
							  BIT5	断电刹把(刹车)			1：断电刹把启动		0：没启动
							  BIT4	通讯故障					1：通讯故障			0：正常								
							  BIT3	充电状态					1：充电中	      	0：不充电								
							  BIT2	控制器限速状态			1：不限速			0：限速中
							  BIT1		
							  BIT0	挡位				组合成	0，1，2，3	*/

	
	send_data[5]=0x50;						//运行电流		小数点前单位A
	send_data[6]=0x50;						//运行电流		小数点后		
	send_data[7]=0;							//电流比例值		1-100%	
	send_data[8]=0x12;						//速度反馈		单位ms				((Velocity)>>8)&0xff;
	send_data[9]=0x12;						//速度反馈						(Velocity)&0xff;
	send_data[10]=0x50;						//电池电量
	send_data[11]=2;						//剩余里程
	send_data[12]=3;						//剩余里程						
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
	
	if((RegularConvData_Tab[3] < 2314) || (RegularConvData_Tab[3] > 3400) )  //过压，欠压保护
		State1 |= 0x08;
	else State1 &= 0xf7;
	

}
//-----------------------------------------------------------





/*****************************END OF FILE***********************/
