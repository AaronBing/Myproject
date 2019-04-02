
/************************ (C) COPYRIGHT 2019 New-Drive *************************
* File Name          : McState
* Author             : Aaron_Bing
* Version            : V0.0.1
* Date               : 04/02/2019
* Description        : ������Ʋ�
********************************************************************************/


/*******************************************************************************
* All Rights Reserved
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "McState.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
Status_TypeDef State;


/* Private function prototypes -----------------------------------------------*/

void MCL_Function(void);
//void MCL_Bkin_Isr(void);
//void MainISR(void);

static void MCL_Init(void); 
//static void MCL_Ready(void);
//static void MCL_Precharge(void);
//static void MCL_Alignment(void);
//static void MCL_OpenLoop(void);
//static void MCL_Normal(void);
//static void MCL_Stop(void);
//static void MCL_Failure(void);

//static void U_Task_Ptr(void);
//static void A_Task_Ptr(void);  
//static void B_Task_Ptr(void);
//static void C_Task_Ptr(void);
//static void D_Task_Ptr(void);

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : MCL_Function
* Description    : 
* Input          : None
* Output         : None
* Return         : 
*******************************************************************************/
void MCL_Function(void)
{
  switch(State)
  {
    case MOTOR_INIT:
      MCL_Init();
      break;
    case MOTOR_STOP:
      //MCL_Stop();
      break;
    case MOTOR_READY:
      //MCL_Ready();
      break; 
    case MOTOR_PRECHARGE:
      //MCL_Precharge();
      break;
    case MOTOR_ALIGNMENGT:
      //MCL_Alignment();
      break;
    case MOTOR_OPENLOOP:     
      //MCL_OpenLoop();
      break;
    case MOTOR_NORMAL:
      //MCL_Normal();
      break;
    case MOTOR_FAILURE:
      //MCL_Failure();
      break;
    default:
      break;
  }
}

/*******************************************************************************
* Function Name  : MCL_Init
* Description    : 
* Input          : None
* Output         : None
* Return         : 
*******************************************************************************/
void MCL_Init(void)
{
//  MCL_ModuleInit();

//  #if(FD6536PIN_EN) 
//  FD6536_EN;
//  #endif

//  #if (EFAL == FO_CMP)
//  SetBit(CMP_CR1, CMP3EN);  //���Ŵ򿪣��Ծٵ��ݳ�絼��Ӳ������
//  #elif (EFAL == FO_INT)
//  EX0 = 1;
//  #elif (EFAL == FO_CMPINT)
//  SetBit(CMP_CR1, CMP3EN);
//  EX0 = 1;
//  #endif

//  Ctl.State = MOTOR_STOP;

}

///*******************************************************************************
//* Function Name  : MCL_Stop
//* Description    : 
//��HALLͣ��ģʽ����6�����ؼ�⣬SAMR=0
//* Input          : None
//* Output         : None
//* Return         : 
//*******************************************************************************/
//void MCL_Stop(void)
//{
//  u16 tValue;                         
//  #if (POS_FB_MODE == SensorLess) 
//  if((CMP_SAMR != 0)||(TIM1_CR4 != 7))
//  {
//    CMP_SAMR = 0;
//    ANGLE_MASK(0);
//    SetBit(TIM1_CR2, T1BRS);
//    ClrBit(TIM1_CR3, T1TIS1);
//    SetBit(TIM1_CR3, T1TIS0);       //CMP
//    TIM1_CR4 = 0x07;                //ѡ��TIM1_DBRx 1 2 3 4 5 6 7
//  }
//  #endif

//  if((Ctl.gStopmodeC == FREE_DOWN)||(Ctl.gStopmodeC == SLOWING_DOWN))
//  {
//    DRV_CMR = PWMOUT_OFF;      //д DRV_CMP

//    ClrBit(TIM1_CR0, T1OPS0);
//    ClrBit(TIM1_CR0, T1OPS1);
//  }
//  else if(Ctl.gStopmodeC == BREAK_DOWN) 
//  {
////    tValue = (BREAK_VALUE)? BREAK_VALUE:Ctl.gBreakValueC;
////    Drv.PWM.DutyTar = tValue;                                
////    Drv.PWM.DutyCur = Drv.PWM.DutyTar;
////    Drv_PWM_Update(Drv.PWM.DutyCur); 

//    DRV_CMR = PWM_ULVLWL_ON;//PWM_ULVLWL_PWM;

//  }
//  if(Ctl.gStartC == FALSE) 
//  {
//    Ctl.State = MOTOR_STOP;
//  }
//  else
//  {
//    Ctl.State =  MOTOR_READY;
//    DRV_OE_ON;
//  }
//}

///*******************************************************************************
//* Function Name  : MCL_Ready
//* Description    : 
//* Input          : None
//* Output         : None
//* Return         : 
//*******************************************************************************/
//void MCL_Ready(void)
//{
//  #if (POS_FB_MODE == HallSensor)
//  MCL_ModuleDefault();
//  giEventCounter = 0;
//  Ctl.State =  MOTOR_PRECHARGE ;

//  #elif (POS_FB_MODE == SensorLess)
//  if(Drv.Stk.Calcnms == 0x7F)
//  { 
//    MCL_ModuleDefault();
//    
//    Drv.Stk.Calcnms = 0;
//    
//    if((CMP_SAMR != 0)||(TIM1_CR4 != 7))
//    {
//      CMP_SAMR = 0;
//      ANGLE_MASK(0);
//      ClrBit(TIM1_CR3, T1TIS1);       //TIM1_DBR7
//      SetBit(TIM1_CR3, T1TIS0);       //CMP
//      TIM1_CR4 = 0x07;                //ѡ�� TIM1_DBRx 1 2 3 4 5 6 7
//    }    
//    
//    giEventCounter = 0;
//    Drv.Stk.Calcnms = 0;
//    SetBit(TIM1_CR2, T1BRS);         
//  }
//  else
//  {
//    Drv_PosTrack();
//  }

//  #endif
//  if(Ctl.gStartC == FALSE) 
//  {
//    Ctl.State = MOTOR_STOP;
//  }
//     
//}

///*******************************************************************************
//* Function Name  : MCL_Precharge
//* Description    : 20170825��֤
//* Input          : None
//* Output         : None
//* Return         : 
//*******************************************************************************/
//void MCL_Precharge(void)
//{
//  #if (CHARGENMS != 0)
//  u8 temp = CHARGENMS;
//  if(temp >= 10)
//    temp = 10;
//  Drv.PWM.DutyCur = CHARGEDUTY;      //���ռ�ձ�
//  Drv_PWM_Update(Drv.PWM.DutyCur);   //ռ�ձȸ���
//  EA = 0;
//  DRV_DR = MDU_MULA_U16(MDUControl.DutyCur,Drv.PWM.DutyArr+2,15);
//  DRV_CMR = PWM_UL_PWM;    //PWM_UL_ON;
//  gDelayms(temp);
//  DRV_CMR = PWM_ULVL_PWM;  //PWM_VL_ON; //TPWM_UVL_PWM
//  gDelayms(temp);
//  DRV_CMR = PWM_ULVLWL_PWM;//PWM_WL_ON; //PWM_ULVLWL_ON
//  gDelayms(temp);
//  DRV_CMR = PWMOUT_OFF;
//  gDelayms(1);
//  EA = 1;
//  #endif

//  #if (POS_FB_MODE == HallSensor)
//    #if(SPEED_CLOSE_EN)
//    pid_spd.Out = 0;
//    #endif
//    Ctl.Tim.STAnms = 0;
//    Ctl.Spd.refTar = SPEED_REF_TAR;
//    Ctl.Spd.refCur = SPEED_REF_INIT; 
//    Drv.PWM.DutyTar = MOTOR_INIT_DUTY;          //���ռ�ձ�
//    Drv.PWM.DutyCur = Drv.PWM.DutyTar;
//    Drv_PWM_Update(Drv.PWM.DutyCur);           //ռ�ձȸ���

//    Hall_IRQHandler();                             //hall��ȡ  
//    TIM1_CR4 = Ctl.gStepCur;
//    Ctl.State = MOTOR_NORMAL;                          

//  #elif (POS_FB_MODE == SensorLess)
//  Ctl.Alig.NmsCount = 0;
//  Drv.PWM.DutyTar = Ctl.Alig.duty;
//  Drv.PWM.DutyCur = Drv.PWM.DutyTar; 
//  Drv_PWM_Update(Drv.PWM.DutyCur);  //ռ�ձȸ���
//  Ctl.State = MOTOR_ALIGNMENGT; 

//  #endif

//  if(Ctl.gStartC == FALSE)
//  {
//    Ctl.State = MOTOR_STOP;
//  }    
//}

///*******************************************************************************
//* Function Name  : MCL_Alignment
//* Description    : 
//* Input          : None
//* Output         : None
//* Return         : 
//*******************************************************************************/
//void MCL_Alignment(void)
//{
//#if (POS_FB_MODE == SensorLess )
//  if(Ctl.Alig.NmsCount < Ctl.Alig.timNms)     // 1000ms   2*Ctl.Alig.timNms
//  {
//    //AH->BL,CL    U->V/W         
//    DRV_CMR = PWM_UHVL_PWM;//PWM_WHVL_PWM;
//    Drv.PWM.DutyCur = Ctl.Alig.duty; 
//    Drv_PWM_Update(Ctl.Alig.duty);     //ռ�ձȸ���
//  }
//  else
//  {
//    DRV_CMR = PWMOUT_OFF;

//    Drv.PWM.DutyTar = Ctl.Ramp.DutySta;
//    Drv.PWM.DutyCur = Ctl.Ramp.DutySta; 

//    #if (MOTORROTORCALC ==2)        
//    Ctl.gStepCur = DRV_IRPD_F(5,20,2);    //12 24 10
//    #else
//    Ctl.gStepPre = 0;

//    if (Ctl.gDirectionC == CW) //U->W
//    {
//      Ctl.gStepCur = 1;          //1
//    }
//    else if (Ctl.gDirectionC == CCW)
//    {                        //W->V
//      Ctl.gStepCur = 1;        //2
//    }    
//    #endif
//    
//    CMP_SAMR = (PWM_MASK+PWM_DELAY)<<4;
//    CMP_SAMR += PWM_MASK;
//    //ClrBit(TIM1_IER, T1PDIE);     //λ�ü���ж�
//    SetBit(CMP_CR3, SAMSEL1);     // ʹ��ON ����
//    ClrBit(CMP_CR3, SAMSEL0);  
//    
//    TIM1_CR4 = 1;                   //ѡ��TIM1_DBRx  
//    ANGLE_MASK(60);                   //BCCR7 -> 60 ��

//    Drv.PWM.DutyArr = RAMP_PWMARR;
//    DRV_ARR = Drv.PWM.DutyArr;
//    DRV_COMR = PWM_FLOW;
//    //Ctl.Ramp.cpNmsCount = Ctl.Ramp.cpNms; 
//    Drv.PWM.DutyIncValue = Ctl.Ramp.Dutystep;   //loop INC DEC
//    Drv.PWM.DutyDecValue = Ctl.Ramp.Dutystep;
//    Drv.PWM.DutyTar = Ctl.Ramp.DutyEnd;
//    Drv.PWM.DutyCur = Ctl.Ramp.DutySta;
//    MDUControl.DutyCur = Drv.PWM.DutyCur;
//    DRV_DR = MDU_MULA_U16(MDUControl.DutyCur,Drv.PWM.DutyArr+2,15);
//    //NSS_ONOFF;
//    Ctl.State = MOTOR_OPENLOOP;
//  }
//  if(Ctl.gStartC == FALSE)
//  {
//    Ctl.State = MOTOR_STOP;
//  }    
//#endif  
//}

///*******************************************************************************
//* Function Name  : MCL_OpenLoop  RampUp
//* Description    : 
//* Input          : None
//* Output         : None
//* Return         : 
//*******************************************************************************/
//void MCL_OpenLoop(void)
//{  
//  Drv_SmartStartCalc();                 
//#if (POS_FB_MODE == SensorLess)
//  #if (WAIT_STEP == WAIT_RAMPUP)
//  Ctl.Ramp.cpNumB = 0;
//  #endif

//  if(Ctl.Ramp.cpNmsCount >= Ctl.Ramp.cpNms) 
//  { 
//    //�������ظ���
//    //NSS_ONOFF;
//    #if (RAMP_MODE != 3)         
//    if(Drv.PWM.DutyCur == Ctl.Ramp.DutyEnd)
//    {
//      Drv.PWM.DutyTar = Ctl.Ramp.DutySta;
//      Drv.PWM.DutyCur = Ctl.Ramp.DutySta; 
//    }
//    #else
//    Drv.PWM.DutyCur = Ctl.Ramp.DutySta;
//    Drv.PWM.DutyTar = Ctl.Ramp.DutyEnd;
//    #endif
//    
//    //Drv_DutyRampCale();
//    Drv_PWM_Update(Drv.PWM.DutyCur);
//   // DRV_DR = MDU_MULA_U16(MDUControl.DutyCur,Drv.PWM.DutyArr+2,15); 
//    //����ʱ�����
//    Ctl.Ramp.cpNmsCount = 0;
//    
//    if((Ctl.Ramp.cpNms - Ctl.Ramp.TimEnd) > Ctl.Ramp.TimStep)
//    {
//      Ctl.Ramp.cpNms = Ctl.Ramp.cpNms - Ctl.Ramp.TimStep;
//    }
//    else if((Ctl.Ramp.TimEnd - Ctl.Ramp.cpNms) > Ctl.Ramp.TimStep)
//    {
//      Ctl.Ramp.cpNms = Ctl.Ramp.cpNms + Ctl.Ramp.TimStep;
//    }
//      #if (RAMP_MODE != 3)
//    else if(Ctl.Ramp.cpNms == Ctl.Ramp.TimEnd)
//    {
//      Ctl.Ramp.cpNms = Ctl.Ramp.TimSta;
//    }
//      #endif
//    else 
//    {
//      Ctl.Ramp.cpNms = Ctl.Ramp.TimEnd;
//      if(Drv.PWM.DutyTar ==Ctl.Ramp.DutyEnd)
//      {
//        Ctl.Ramp.ComCnt++;                    //�������
//      }
//    }

//    //��ⷴ����
//    #if ((RAMP_MODE == 1)||(RAMP_MODE == 2))
//      SetBit(TIM1_IER, T1PDIE);             //λ�ü���ж�
//      SetBit(TIM1_IER, T1UPD); 
//      TIM1__RARR = Ctl.Ramp.MaskTime ;      //0.050ms   

//    
//    #elif (RAMP_MODE == 3)
//      Ctl.gStepCur = TIM1_CR4;
//      SetBit(TIM1_IER, T1UPD);
//      Drv.speed.EventPeriod = TIM1__BCCR;
//    #endif
//  }
//  
//  
//  #if (RAMP_MODE == 3)
//  if(((Ctl.Ramp.ComCnt >= Ctl.Ramp.ComNum)&&(Ctl.gStepCur == 1)))  
//  { 
//    DRV_CMR = PWMOUT_OFF;
//    Ctl.State = MOTOR_READY;
//  }
//  #endif
//          
//  if(Ctl.gStartC == FALSE)
//  {
//    Ctl.State = MOTOR_STOP;
//  }  
//#endif  

//}

///*******************************************************************************
//* Function Name  : MCL_Normal
//* Description    :
//* Input          : None
//* Output         : None
//* Return         : 
//*******************************************************************************/
//void MCL_Normal(void)
//{ 
//  if(Ctl.gStartC == FALSE)
//  {
//    DRV_CMR = PWMOUT_OFF;
//    Ctl.State = MOTOR_STOP;
//  }  
//  #if(OCRESTART_EN) 
//  if(Ctl.OC.u16Runms >= 5000)         //����5s
//  {
//    Ctl.OC.u16Runms = 5000;
//    Ctl.OC.u8ReNum = 0;               
//    Ctl.OC.u16nmsCount =0;
//  }
//  #endif  
//  #if(FO_EN==2)
//  if(Ctl.FO.ReCounter >= 1000)     //1S�����FO����
//  {
//    Ctl.FO.FoCounter = 0;           //��FO���ϼ�����
//    Ctl.FO.ReCounter = 1000;
//  }
//  #endif
//  #if(STALLRESTARTNUM) 
//  if((Ctl.Stall.u16NormalRunms >= STANMS+500)) //����5s         
//    {
//      Ctl.Bemf.RstartNmsConuter = 0;           //��������ʱ���ʱ��
//      Ctl.Stall.WaitReatartNms = 0;            //��ת�����ȴ�ʱ��
//      Ctl.Stall.u16NormalRunms = STANMS+500;   //
//      Ctl.Stall.u8Num = 0;                     //�����ת���� �������     
//    }
//  #endif

//}

///*******************************************************************************
//* Function Name  : MCL_Failure
//* Description    : 
//* Input          : None
//* Output         : None
//* Return         : 
//*******************************************************************************/
//void MCL_Failure(void)
//{
//  DRV_CMR = PWMOUT_OFF;
//  DRV_OE_OFF;             //
//  Ctl.gStartC = FALSE;
//  
////-------------------------------------------------------  
////���ϼ�¼    
//  #if(FAILLOGEN)
//  if(Ctl.E_message.ErrorF==0)
//  {
//    //������Ϣ��¼
//    Ctl.E_message.ErrorF   = 1 ;
//    Ctl.E_message.SysError = Ctl.SysError;
//    //Ctl.E_message.State  = Ctl.State ;
//    Ctl.E_message.FR       = Ui.flg.FR;
//    //Ctl.E_message.SpeedRef = pid_spd.Ref;
//    Ctl.E_message.SpeedFdb = Drv.speed.SpeedRpm;
//    Ctl.E_message.Duty     = Drv.PWM.DutyCurrent;
//    Ctl.E_message.ImeasBus = Drv.AdcMeas.ImeasBus;
//    Ctl.E_message.VdcMeas = Drv.AdcMeas.VdcAvgMeas;
//    Ctl.E_message.Ibus     = Drv.AdcMeas.Ibus;
//    Ctl.E_message.Vdc      = Drv.AdcMeas.Vdc;
////-------------------------------------------------------  
////������Ϣflash �洢��
//    #if(FAILSTOREEN)
//    if((Ctl.SysError != E_OV)&&(Ctl.SysError != E_UV))       
//        ;//Flash_Save_Data();
//    #endif     
//  }
//  #endif
////-------------------------------------------------------
////��������
//  #if(VBUSRECOVER_EN)
//  if((Ctl.SysError == E_OV)||(Ctl.SysError == E_UV))
//  {
//    if(Ctl.Tim.Nonms>=NONMS)
//    {
//      Mcl_MotorRestart();
//    }
//  }
//  #endif
////-------------------------------------------------------
////�����������  
//  #if (OCRESTARTNUM == 1)        
//  if(Ctl.SysError == E_OC) 
//  //if((Ctl.SysError == E_OC)||(Ctl.SysError == E_FAIL))  
//  {
//    if(Ctl.OC.ReFlag == 0)
//    {
//      if(Ctl.OC.u8ReNum < OCRESTARTNUM)
//      {
//        Ctl.OC.u8ReNum ++;
//        Ctl.OC.u16nmsCount = 0;  
//        Ctl.OC.ReFlag = 1;
//      }
//    }
//    if(Ctl.OC.ReFlag == 1)
//    {
//      if(Ctl.OC.u16nmsCount >= OCRESTARTTIM)
//      {
//        Ctl.OC.ReFlag = 0;
//        Mcl_MotorRestart();
//      }
//    }
//  }
//  #endif
////-------------------------------------------------------
////��ת����
//  #if(STALLRESTARTNUM)    
//  if((Ctl.SysError == E_STA)||(Ctl.SysError == E_STB3)||(Ctl.SysError == E_STB2)||((Ctl.SysError == E_STB1)))
//  {
//    if(Ctl.Stall.u8FL == 0)
//    {
//      if(Ctl.Stall.WaitReatartNms >= STALLRESTARTTIM)
//      {
//        Ctl.Stall.WaitReatartNms = 0;
//        Drv_StallRestart(); 
//      }
//    }
//  }
//  #endif
//    
////-------------------------------------------------------
////��������  
//  #if(OHRE_EN)           
//  if((Ctl.SysError == E_OH1)||(Ctl.SysError == E_OH2)||(Ctl.SysError == E_OH3))
//  {
//    if((Ctl.Tim.OH1REnms>=OH1NMS)&&((Ctl.Tim.OH2REnms>=OH2NMS)))
//    {
//      Mcl_MotorRestart();
//    }
//  }
//  #endif

//}

///*******************************************************************************
//* Function Name  : MCL_Bkin_Isr
//* Description    : 
//* Input          : break input
//* Output         : None
//* Return         : 
//*******************************************************************************/
//void MCL_Bkin_Isr(void)
//{
//  #if (FO_EN == 1)
//    DRV_OE_OFF;
//    Ctl.SysError = E_FAIL;
//    Ctl.State = MOTOR_FAILURE;    
//  #elif (FO_EN == 2)
//    Ctl.FO.FoCounter++;
//    Ctl.FO.ReCounter = 0;  //FO�źź����¼���
//   if(FO_NUM == 10000)
//   {
//     ;
//   }
//   else if(Ctl.FO.FoCounter >= FO_NUM)
//    {
//      DRV_OE_OFF;
//      Ctl.SysError = E_FAIL;
//      Ctl.State = MOTOR_FAILURE;  
//    }
//  #elif (FO_EN == 3)
//    ;
//  #endif
//      
//  #if(FAILLOGEN)
//  if(Ctl.E_message.State == 0)
//  {
//    Ctl.E_message.State = Ctl.State; 
//  }
//  #endif  
//}
///*******************************************************************************
//* Function Name  : MainISR
//* Description    : 100us�ж�ִ�У�����ͬ��PWM���� �ٶȱջ��������ջ�
//* Input          : 
//* Output         : 
//* Return         : 1
//*******************************************************************************/
//void MainISR(void)
//{
//  s16 xdata tDutyCur;
//  
//  Ctl.Tim.gIsrTicker++;

//  if(Ctl.Tim.gIsrTicker >= 10)
//  {
//    Ctl.Tim.gIsrTicker = 0;
//    A_Task_Ptr();                  
//  }
//  else if(Ctl.Tim.gIsrTicker == 1)
//  {
//    if(Ctl.gPowerOnF == 0X7f)
//    {
//      B_Task_Ptr();     
//    }
//  }
//  else if(Ctl.Tim.gIsrTicker == 2) 
//  {
//    if(Ctl.gPowerOnF == 0X7f)
//    {
//      C_Task_Ptr();       
//    }
//  }
//  else if(Ctl.Tim.gIsrTicker == 2) 
//  {
//    if(Ctl.gPowerOnF == 0X7f)
//    {
//      D_Task_Ptr();       
//    }
//  }
//  else if(Ctl.Tim.gIsrTicker == 4) 
//  {
//    U_Task_Ptr();   //�û�ʹ��1ms������
//  }
//  #if(SPEED_TICKPEREN)
//  else if(Ctl.Tim.gIsrTicker == 9)
//  #endif
//  {
//    if(Ctl.State != MOTOR_NORMAL)
//    {
//      //���룺 Ctl.Spd.refTar     ����� Ctl.Spd.refCur
//      if(Ctl.Spd.refTar >= _IQ(0.2))
//      Ctl.Spd.refTar = _IQ(0.2);
//    }
//    else
//    {
//    
//    }
//    Drv_SpeedRampCale();
//  }
//  
//  //if(Ctl.State == MOTOR_NORMAL)
//  {
//    #if(SPEED_CLOSE_EN)                        //�ٶȻ�
//    Ctl.Tim.counterSpeed++;
//    if(Ctl.Tim.counterSpeed > Ctl.Tim.numTicksPerSpeed)
//    {
//      Ctl.Tim.counterSpeed = 0;

//        pid_spd.Ref = Ctl.Spd.refCur;          //REF
//        pid_spd.Fdb = Drv.speed.Speed;         //FDB
//        #if (CURRENT_CLOSE_EN>1)
//        Ctl.Is.refTar = Pid_calc(&pid_spd);    //�ٶȻ����
//        pid_is.Ref = Ctl.Is.refTar;            //����������
//        #else
//        tDutyCur = Pid_calc(&pid_spd);
//        #endif
//    }
//    #elif (CURRENT_CLOSE_EN>1)                 //������
//    pid_is.Ref = Ctl.Is.refTar;                //����������
//    #else
//    tDutyCur = Ctl.Spd.refCur;
//    #endif

//    #if (CURRENT_CLOSE_EN>1)
//    Ctl.Tim.counterCurrent++;
//    if(Ctl.Tim.counterCurrent > Ctl.Tim.numTicksPerCurrent)
//    {
//      Ctl.Tim.counterCurrent = 0;

//        #if (CURRENT_CLOSE_EN == 2)
//        if(pid_is.Ref > CURRENT_INMAX)
//        {
//          pid_is.Ref = CURRENT_INMAX;
//        }
//        pid_is.Fdb = Drv.AdcMeas.ImeasBus;
//        #elif (CURRENT_CLOSE_EN == 3) 
//        Power.fdb = ((u32)Drv.AdcMeas.IBusAvgMeas*(u32)Drv.AdcMeas.VdcMeas)>>10;
//        pid_is.Fdb = Power.fdb;
//        pid_is.Ref = Power.ref;
//        #endif
//        Drv.PWM.DutyCur = Pid_calc(&pid_is); //���������
//      }
//    }
//    #endif
//    
//    Drv_DutyLimit(); //�����ѹ����

//    #if (POS_FB_MODE == HallSensor)  //��HALL���࣬���� DutyCur 
//      if(tDutyCur < 0)
//      {
//        Drv.PWM.DutyCur = -tDutyCur; 
//        if(Ctl.gDirectionC != CW)
//        {
//          ClrBit(TIM1_CR0, T1OPS1);        
//          ClrBit(TIM1_CR0, T1OPS0);           
//          giEventCounter = 0;
//          Ctl.gDirectionC = CW;   
//          ClrBit(DRV_CR, DDIR);
//        }
//      }
//      else if(tDutyCur > 0)
//      {
//        Drv.PWM.DutyCur = tDutyCur;
//        if(Ctl.gDirectionC != CCW)
//        {
//          ClrBit(TIM1_CR0, T1OPS1);       
//          ClrBit(TIM1_CR0, T1OPS0);                
//          giEventCounter = 0;
//          Ctl.gDirectionC = CCW;   
//          SetBit(DRV_CR, DDIR);
//        }      
//      }
//    #else
//      Drv.PWM.DutyCur = (tDutyCur<0)? -tDutyCur:tDutyCur;
//    #endif
//    Drv_PWM_Update(Drv.PWM.DutyCur);  
//  }
//}

///*******************************************************************************
//* Function Name  : A_Task_Ptr
//* Description    : ϵͳ���м�ʱ���ϵ硢��λ���϶���ת�ٸ��١�ɲ����
//* Input          : 
//* Output         : 
//* Return         : 1
//*******************************************************************************/
//void A_Task_Ptr(void)
//{  
//  if(Ctl.gPowerOnF == 0X7f)
//  {
//    #if(FAILLAMPEN)
//    gUserLEDmsCunter++;    //����ָʾled��˸
//    #endif
//    
//    #if (POS_FB_MODE == SensorLess)
//    Ctl.Alig.NmsCount++;      //��λ��ʱ��
//    Ctl.Ramp.cpNmsCount++;    //���ඨʱ��
//    if(Drv.Stk.Calcnms != 0x7F)
//    {  
//      Drv.Stk.Calcnms++;
//    }
//    else
//    {
//      Drv.Stk.Calcnms = 0x7F;
//    }
//    #endif
//    
//    
//    #if(STARSTARTNMS)
//    if((Ctl.gStartC == TRUE)&&(Ctl.State != MOTOR_NORMAL))
//    {
//      Ctl.Bemf.RstartNmsConuter++;
//    }
//    else if(Ui.flg.START == FALSE)
//    {
//      Ctl.Bemf.RstartNmsConuter = 0;
//    }
//    #endif

//  
//    #if(STALLRESTARTNUM)               //��ת����
//    if((Ctl.SysError == E_STA)||(Ctl.SysError == E_STB3)||(Ctl.SysError == E_STB2)||((Ctl.SysError == E_STB1)))
//    {
//      Ctl.Stall.WaitReatartNms++;
//    }
//    #endif

//    #if(OCRESTART_EN) 
//    Ctl.OC.u16nmsCount++;              //���ر��������ȴ�ʱ��
//    #endif
//  }
//  else
//  {
//    Ctl.Tim.PowerOnNms++;            //�ϵ綨ʱ��
//  }
//}
///*******************************************************************************
//* Function Name  : B_Task_Ptr
//* Description    : ������ʱ��������ʱ������Ƿѹ��ʱ������ת��ʱ�������¼�ʱ��������
//* Input          : 
//* Output         : 
//* Return         : 1
//*******************************************************************************/
//void B_Task_Ptr(void)  
//{
//  //_nop_(); 
////------------------------------------------
//  if(Ctl.State == MOTOR_NORMAL)
//  {
//    Ctl.Stall.u16NormalRunms++;
//    #if(OCRESTART_EN)
//    Ctl.OC.u16Runms++;
//    #endif

//    #if(STARSTARTNMS) 
//    Ctl.Tim.STAnms++;               //��ת��ʱ��    
//    if(Ctl.Tim.STAnms >= STANMS)
//    {
//      Ctl.SysError = E_STA; 
//    }   
//    #endif
//  }
//  else
//  {
//    giEventCounter = 0;
//    Ctl.Stall.u16NormalRunms = 0;
//  }
//  
//  #if(OVERLOAD_EN)
//  Ctl.OL.msFlag = 0x7F;              //���ر���
//  #endif
////------------------------------------------
////���¼��
//  #if(OH1_EN)
//  if(Drv.AdcMeas.Therm1 >= 1000)       // (100��)
//  {
//    Ctl.Tim.OH1nms++;  //
//    Ctl.Tim.OH1REnms = 0;
//    if(Ctl.Tim.OH1nms >= OH1NMS)
//    {
//      Ctl.SysError = E_OH1;
//      Ctl.Tim.OH1nms = OH1NMS;
//    }    
//  }
//  else if(Drv.AdcMeas.Therm1 <= 900)   //  (80~120��)
//  {
//    Ctl.Tim.OH1REnms++;
//    
//    Ctl.Tim.OH1nms--;
//    if(Ctl.Tim.OH1nms < 0)
//    {
//      Ctl.Tim.OH1nms = 0;
//    }
//  } 
//  else                                  //  (~80)
//  {
//    Ctl.Tim.OH1REnms++;
//   
//    Ctl.Tim.OH1nms--;
//    if(Ctl.Tim.OH1nms<0)
//    {
//      Ctl.Tim.OH1nms = 0;
//    }
//  }     
//  #endif
////------------------------------------------
////�������
//  #if (CBCCP_EN != 0)
//  Ctl.FO.ReCounter++; 
//  #endif   
//  #if(OVERCURRENT_EN)
//  if(Drv.AdcMeas.ImeasBus > MECASEOVERCURRENT)
//    {
//      Ctl.Tim.OCnms++;
//      if(Ctl.Tim.OCnms >= OCNMS)      //����
//      {
//        Ctl.SysError = E_OC;
//      }      
//    }
//  else
//    {
//      Ctl.Tim.OCnms--;
//      if(Ctl.Tim.OCnms<0)
//      {
//        Ctl.Tim.OCnms = 0; 
//      }
//    }    
//  #endif  

////------------------------------------------
////��ѹǷѹ���
//  #if((OVERVOLTAGE_EN)||(UNDERVOLTAGE_EN)||(VBUSRECOVER_EN))
//  if(Drv.AdcMeas.VdcAvgMeas != 0)             
//  {
//    if(Drv.AdcMeas.VdcAvgMeas > MECASEOVERVOLTAGE)
//    {
//      Ctl.Tim.OVnms++;
//      Ctl.Tim.UVnms = 0;
//      Ctl.Tim.Nonms = 0;
//      #if(OVERVOLTAGE_EN)
//      if(Ctl.Tim.OVnms >= OVNMS)      //��ѹ
//      {
//        Ctl.SysError = E_OV;
//      }
//      #endif      
//    }
//    else if (Drv.AdcMeas.VdcAvgMeas < MECASEUNDERVOLTAGE)
//    {
//      Ctl.Tim.OVnms = 0;
//      Ctl.Tim.UVnms++;
//      Ctl.Tim.Nonms = 0;
//      #if(UNDERVOLTAGE_EN)
//      if(Ctl.Tim.UVnms >= UVNMS)
//      {
//        Ctl.SysError = E_UV;
//      }
//      #endif      
//    }
//    else if ((Drv.AdcMeas.VdcAvgMeas < MECASEOVREERVOLTAGE)&&(Drv.AdcMeas.VdcAvgMeas > MECASEUNREDERVOLTAGE))
//    {
//      Ctl.Tim.OVnms = 0;
//      Ctl.Tim.UVnms = 0;
//      Ctl.Tim.Nonms++;
//    }      
//  }
//  #endif
////------------------------------------------
////��HALL ����ʱ�䵽 ����
//  #if(STARSTARTNMS)               
//  if(Ctl.Bemf.RstartNmsConuter > STANMS)   
//  {
//    Ctl.SysError = E_ERR1;
//  }
//  #endif
////------------------------------------------  
////����  
//  #if(OVERLOAD_EN)
//  if(Ctl.OL.Value != 0x7F)      
//  {
//    Ctl.SysError = E_OL;
//  }
//  #endif 
////------------------------------------------  
//  if (Ctl.SysError!=NONE)
//  {
//    DRV_CMR = PWMOUT_OFF;
//    DRV_OE_OFF;

//    #if(FAILSTOREEN)
//    if(Ctl.E_message.State == 0)
//    {
//     Ctl.E_message.State = Ctl.State;
//    }
//    #endif
//    Ctl.State = MOTOR_FAILURE;
//  }
//}
///*******************************************************************************
//* Function Name  : C_Task_Ptr
//* Description    : 
//* Input          : 
//* Output         : 
//* Return         : 1
//*******************************************************************************/
//void C_Task_Ptr(void)  
//{
//  Ctl.Tim.OnOffnms++;        //ONOFF��ʱ��

//  #if(LAMP_EN)               //�޲�����ʱ��ʱ��
//  Ctl.Tim.NoOperationDelaynms++;   
//  #endif 
//  
//  #if(ESC_PPMSREFEN) 
//  ESC_PPMCheckErr();
//  #endif
//}
///*******************************************************************************
//* Function Name  :  D_Task_Ptr
//* Description    : 
//* Input          : 
//* Output         : 
//* Return         : 
//*******************************************************************************/
//void D_Task_Ptr(void)
//{

//}
///*******************************************************************************
//* Function Name  : U_Task_Ptr
//* Description    : �����û���1ms��ʱ��
//* Input          : 
//* Output         : 
//* Return         : 
//*******************************************************************************/
//void U_Task_Ptr(void)
//{   
//  //����������������
//}


