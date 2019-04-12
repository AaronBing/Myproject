
/************************ (C) COPYRIGHT 2019 New-Drive *******************************
* File Name          : 
* Author             : AaronBing
* Version            : V0.0.1
* Date               : 04/02/2019
* Description        : 
********************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/

/* Includes ------------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/

typedef enum
{ 
  MOTOR_NONE       = 0,     //        0
  MOTOR_INIT       = 1,     //��ʼ��  1
  MOTOR_STOP       = 2,     //ֹͣ    2
  MOTOR_READY      = 3,     //׼��    3
  MOTOR_PRECHARGE  = 5,     //���    5
  MOTOR_ALIGNMENGT = 6,     //��λ    6
  MOTOR_OPENLOOP   = 7,     //����    7
  MOTOR_NORMAL     = 8,     //����    8
  MOTOR_FAILURE    = 9      //����    9
}Status_TypeDef;

typedef enum
{ 
  NONE      = 0,          //�޹���           0
  E_FAIL    = 1,          //ģ�鱣��         1
  E_OC      = 2,          //��������         2
  E_OL      = 3,          //���ر���         3
  E_OV      = 4,          //��ѹ����         4
  E_UV      = 5,          //Ƿѹ����         5
}Error_TypeDef;


typedef struct
{
	Status_TypeDef State;             //ϵͳ״̬
	Error_TypeDef	Error;
}MCL_TypeDef;

//typedef enum
//{ 
//  NONE      = 0,          //�޹���           0
//  E_FAIL    = 1,          //ģ�鱣��         1
//  E_OC      = 2,          //��������         2
//  E_OL      = 3,          //���ر���         3
//  E_OV      = 4,          //��ѹ����         4
//  E_UV      = 5,          //Ƿѹ����         5
//  E_STA     = 6,          //��תʧ�ٱ���     6 
//  E_STB1    = 7,          //����ʧ�ٹ���1    7 
//  E_STB2    = 8,          //����ʧ�ٹ���2    8 
//  E_STB3    = 9,          //����ʧ�ٹ���2    9 
//  E_HALL    = 10,         //HALL���߹���     10
//  E_QEP     = 11,         //����������       11
//  E_OH1     = 12,         //���ʹܹ��±���   12
//  E_OH2     = 13,         //������±���     13
//  E_OH3     = 14,         //��ع��±���     14
//  E_ESTA    = 15,         //��HALL����ʧ��   15
//  //����    
//  E_ERR1    = 21,         //δ�������2      21
//  E_ERR2    = 22,         //δ�������2      22
//  E_ERR3    = 23,         //δ�������3      23
//  E_ERR4    = 24,         //δ�������4      24
//  E_ERR5    = 25,         //δ�������5      25
//  E_ERR6    = 26,         //δ�������6      26
//  E_ERR7    = 27,         //δ�������7      27
//  E_ERR8    = 28,         //δ�������8      28
//  E_ERR9    = 29,         //δ�������9      29

//}Error_TypeDef;

//typedef struct 
//{ 
//  Error_TypeDef SysError;
//  Status_TypeDef State;
//  u8 ErrorF;
//  u8  FR;
//  u16 SpeedRef;
//  u16 SpeedFdb;
//  u16 ImeasBus;
//  u16 VdcMeas;    
//  u16 Therm;
//  u16 Duty;
//  s16 Ibus;
//  s16 Vdc;
//} E_MESSSAGE_TypeDef ;

//typedef struct
//{
//  u16 FoCounter;     //FO����
//  u16 ReCounter;     //FO 
//}FO_TypeDef;

////CTL ������������  ����
//typedef struct
//{
//  u8  FlagCurrentLimit;
//  u8  u8ReNum;                  //������������
//  u16 u16nmsCount;              //���������ȴ�ʱ��
//  u16 u16Runms;                 //���������ʶʱ��
//  u8  ReFlag;                   //������ʶ    
//}OC_TypeDef;

////CTL  
//typedef struct    
//{ 
//  u16  WaitReatartNms;   //�ȴ�Nms����
//  u16  u16NormalRunms;   //��ת����ʱ����ʱ�䣬���Խ����ת״̬��
//  u8   u8Num;            //������ת����
//  u8   u8FL;             //��ת״̬
//}Stall_TypeDef;  
////CTL
//  
//typedef struct 
//{  
//  u16 duty;
//  u16 timNms;
//  u16 NmsCount;
//}Alig_TypeDef;
////CTL  
//typedef struct 
//{ 
//  s8    cpmode;        //����ģʽ
//  s8    TimSta;        //�϶�ʱ��
//  s8    TimEnd;        //�϶�ʱ��
//  s8    TimStep;
//  u16   DutySta;
//  u16   DutyEnd;
//  u16   Dutystep;
//  s8    cpNmsCount;     //�����ʱ��
//  s8    cpNms;          //����ʱ��
//  u16   MaskTime;       //������������ʱ��
//  u8    ComCnt;         //б������϶�����
//  u8    ComNum;         //б����ɺ��жϴ���
//  u16   TimBufCur;      //TIM buf
//  u16   TimBufPer;      //TIM buf
//  u8    BemfTureCnt;
//  u8    BemfTureNum;    //������������
//  u8    BemfErrCnt;     //��������������
//  u8    BemfErrNum;
//  s8    FailCnt;        //����ʧ�ܼ�����
//}Ramp_TypeDef;

//typedef struct
//{
//  u8  CmpOut;         //60�ȵ�Ƕ�
//  u8  MaskAngle;      //���νǶ� 
//  u16 MaskTime;       //
//  u8  CTAngle;        //����Ƕ�
//  u8  TestOut;        //SPI�ӿڲ���
//  u16 RstartNmsConuter; //��HALL��������ʱ�������
//  s8  CompConter;
//  u8  Comflag;
//}BEMF_TypeDef; 
////CTL
//typedef struct
//{
//  u16 gIsrTicker;
//  u8  numTicksPerCurrent;  //������tick
//  u8  counterCurrent;      //������
//  u8  numTicksPerSpeed;    //�ٶȻ�tick
//  u8  counterSpeed ;       //�ٶȻ�
//  
//  u16 PowerOnNms;       //�ϵ�ȴ�ʱ��
//  u16 OnOffnms;         //ON/OFFʱ��
//  s8  OCnms;            //�����ж�ʱ��    
//  u16 UVnms;            //��ѹ�ж�ʱ��
//  u16 OVnms;            //Ƿѹ�ж�ʱ��
//  u16 Nonms;            //��ѹ�ָ��ж�ʱ��
//  u16 STAnms;           //��ת�ж�ʱ��
//  u16 STB1nms;          //ʧ���ж�ʱ��1
//  u16 STB2nms;          //ʧ���ж�ʱ��2

//  s16 OH1nms;           //�����ж�ʱ��1
//  s16 OH1REnms;         //���Ȼָ��ж�ʱ��1
//  s16 OH2nms;           //���Ȼָ��ж�ʱ��2
//  s16 OH2REnms;         //�����ж�ʱ��2
//  s16 OH3nms;           //���Ȼָ��ж�ʱ��3
//  s16 OH3REnms;         //�����ж�ʱ��3  
//}Tim_TypeDef;  
//  
////CTL

//typedef struct
//{ 
//  u8  RungDirection;        //���ʵʱ���з���
//  s8  SwitchCount;          //����������
//  s8  SpeedShift;           //��λ
//  s16 IncValue;             //����ֵ
//  s16 DecValue;             //����ֵ
//  s16 refTartemp;           //Ŀ��ֵ
//  s16 refTar;               //Ŀ��ֵ
//  s16 refCur;               //��ǰ����ֵ
//  u16 rpm;                  //�ٶȷ���
//  u16 rpmavg;               //�ٶȷ���ƽ��ֵ
//  s32 ComNum;               //���������
//  s32 MechCircleNum;        //��еȦ�� 
//  u8  RotorPosition ;       //ת��λ��
//  u8  RotorPositionqPre;    //ǰһʱ��ת��λ��
//}spd_TypeDef;

//typedef struct
//{
//  s16 refTar;               //Ŀ��ֵ
//  s16 refCur;               //��ǰ����ֵ
//}Is_TypeDef;

//typedef struct
//{
//  Status_TypeDef State;             //ϵͳ״̬
//  
//  u8  gStartC;                       //��ͣ 
//  s8  gDirectionC;                   //���з���
//  u8  gStopmodeC;                   //ͣ��ģʽ
//  u16 gBreakValueC;                  //ɲ��ǿ��
//  u8  gPowerOnF;                     //�ϵ��־  
//  u8  gStepCur;                     //��ǰ����
//  u8  gStepPre;                     //��һ������

//  s8  gDirectionS;                   //���з���
//  
//  Error_TypeDef  SysError;          //���ϴ���
//  FO_TypeDef FO;                    //FO     
//  OC_TypeDef OC;                    //��������
// 
//  spd_TypeDef    Spd;               //�ٶȼ���
//  Is_TypeDef     Is;
//  Tim_TypeDef    Tim;               //Tim
//  Stall_TypeDef  Stall;             //ת�ٸ���
//  Alig_TypeDef   Alig;              //��λ
//  Ramp_TypeDef   Ramp;              //�϶�
//  BEMF_TypeDef   Bemf;              //������

//} MCL_TypeDef;
//  
//extern MCL_TypeDef xdata Ctl;     //ȫ�ֱ�������

///* Exported constants --------------------------------------------------------*/
///* Exported macro ------------------------------------------------------------*/
///* Exported functions ------------------------------------------------------- */
//extern void MCL_Function(void);     //ȫ���ⲿ��������
//extern void MCL_Bkin_Isr(void);
//extern void MainISR(void);        //ȫ���ⲿ��������
//#endif /* __MAIN_H */

///******************* (C) COPYRIGHT 2014 FT *****END OF FILE****/

extern MCL_TypeDef Ctl;
extern void MCL_Function(void);

