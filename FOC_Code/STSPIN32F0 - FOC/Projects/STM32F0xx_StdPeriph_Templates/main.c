
#include "main.h"



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


void SendData(uint8_t Data)
{
	 USART_SendData(USART1,Data); 
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
}

void TX_data(void)
{
	uint8_t i=0;	
	send_data[0]=2;
	send_data[1]=14;
	send_data[2]=1;
	send_data[3]=State1;  
	send_data[4]=State2;
	send_data[5]=(Current>>8)&0xff;
	send_data[6]=(Current)&0xff;
	send_data[7]=0;
	send_data[8]=((Velocity)>>8)&0xff;
	send_data[9]=(Velocity)&0xff;
	send_data[10]=0;
	send_data[11]=2;
	send_data[12]=3;	
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
	if((RegularConvData_Tab[3] < 2314) || (RegularConvData_Tab[3] > 3400) )  
		State1 |= 0x08;
	else State1 &= 0xf7;
	

}
//-----------------------------------------------------------
int main(void)
{	
	device_init ();	
	board_self_test ();		
	while(1)
	{
		TX_data();
		states();
	}
}




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
