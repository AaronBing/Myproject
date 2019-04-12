
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
  MOTOR_INIT       = 1,     //初始化  1
  MOTOR_STOP       = 2,     //停止    2
  MOTOR_READY      = 3,     //准备    3
  MOTOR_PRECHARGE  = 5,     //充电    5
  MOTOR_ALIGNMENGT = 6,     //定位    6
  MOTOR_OPENLOOP   = 7,     //启动    7
  MOTOR_NORMAL     = 8,     //正常    8
  MOTOR_FAILURE    = 9      //错误    9
}Status_TypeDef;

typedef enum
{ 
  NONE      = 0,          //无故障           0
  E_FAIL    = 1,          //模块保护         1
  E_OC      = 2,          //过流保护         2
  E_OL      = 3,          //过载保护         3
  E_OV      = 4,          //过压保护         4
  E_UV      = 5,          //欠压保护         5
}Error_TypeDef;


typedef struct
{
	Status_TypeDef State;             //系统状态
	Error_TypeDef	Error;
}MCL_TypeDef;

//typedef enum
//{ 
//  NONE      = 0,          //无故障           0
//  E_FAIL    = 1,          //模块保护         1
//  E_OC      = 2,          //过流保护         2
//  E_OL      = 3,          //过载保护         3
//  E_OV      = 4,          //过压保护         4
//  E_UV      = 5,          //欠压保护         5
//  E_STA     = 6,          //堵转失速保护     6 
//  E_STB1    = 7,          //过速失速故障1    7 
//  E_STB2    = 8,          //过速失速故障2    8 
//  E_STB3    = 9,          //过速失速故障2    9 
//  E_HALL    = 10,         //HALL断线故障     10
//  E_QEP     = 11,         //编码器断线       11
//  E_OH1     = 12,         //功率管过温保护   12
//  E_OH2     = 13,         //电机过温保护     13
//  E_OH3     = 14,         //电池过温保护     14
//  E_ESTA    = 15,         //无HALL启动失败   15
//  //……    
//  E_ERR1    = 21,         //未定义故障2      21
//  E_ERR2    = 22,         //未定义故障2      22
//  E_ERR3    = 23,         //未定义故障3      23
//  E_ERR4    = 24,         //未定义故障4      24
//  E_ERR5    = 25,         //未定义故障5      25
//  E_ERR6    = 26,         //未定义故障6      26
//  E_ERR7    = 27,         //未定义故障7      27
//  E_ERR8    = 28,         //未定义故障8      28
//  E_ERR9    = 29,         //未定义故障9      29

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
//  u16 FoCounter;     //FO次数
//  u16 ReCounter;     //FO 
//}FO_TypeDef;

////CTL 过流过载重启  过流
//typedef struct
//{
//  u8  FlagCurrentLimit;
//  u8  u8ReNum;                  //过流重启次数
//  u16 u16nmsCount;              //过流重启等待时间
//  u16 u16Runms;                 //解除过流标识时间
//  u8  ReFlag;                   //重启标识    
//}OC_TypeDef;

////CTL  
//typedef struct    
//{ 
//  u16  WaitReatartNms;   //等待Nms重启
//  u16  u16NormalRunms;   //堵转重启时运行时间，用以解除堵转状态。
//  u8   u8Num;            //连续堵转次数
//  u8   u8FL;             //堵转状态
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
//  s8    cpmode;        //启动模式
//  s8    TimSta;        //拖动时间
//  s8    TimEnd;        //拖动时间
//  s8    TimStep;
//  u16   DutySta;
//  u16   DutyEnd;
//  u16   Dutystep;
//  s8    cpNmsCount;     //换相计时器
//  s8    cpNms;          //换相时间
//  u16   MaskTime;       //启动屏蔽续流时间
//  u8    ComCnt;         //斜坡完成拖动次数
//  u8    ComNum;         //斜坡完成后判断次数
//  u16   TimBufCur;      //TIM buf
//  u16   TimBufPer;      //TIM buf
//  u8    BemfTureCnt;
//  u8    BemfTureNum;    //符合收敛条件
//  u8    BemfErrCnt;     //不符合收敛条件
//  u8    BemfErrNum;
//  s8    FailCnt;        //启动失败计数器
//}Ramp_TypeDef;

//typedef struct
//{
//  u8  CmpOut;         //60度电角度
//  u8  MaskAngle;      //屏蔽角度 
//  u16 MaskTime;       //
//  u8  CTAngle;        //换相角度
//  u8  TestOut;        //SPI接口测试
//  u16 RstartNmsConuter; //无HALL连续重启时间计数器
//  s8  CompConter;
//  u8  Comflag;
//}BEMF_TypeDef; 
////CTL
//typedef struct
//{
//  u16 gIsrTicker;
//  u8  numTicksPerCurrent;  //电流环tick
//  u8  counterCurrent;      //电流环
//  u8  numTicksPerSpeed;    //速度环tick
//  u8  counterSpeed ;       //速度环
//  
//  u16 PowerOnNms;       //上电等待时间
//  u16 OnOffnms;         //ON/OFF时间
//  s8  OCnms;            //过流判断时间    
//  u16 UVnms;            //过压判断时间
//  u16 OVnms;            //欠压判断时间
//  u16 Nonms;            //电压恢复判断时间
//  u16 STAnms;           //堵转判断时间
//  u16 STB1nms;          //失速判断时间1
//  u16 STB2nms;          //失速判断时间2

//  s16 OH1nms;           //过热判断时间1
//  s16 OH1REnms;         //过热恢复判断时间1
//  s16 OH2nms;           //过热恢复判断时间2
//  s16 OH2REnms;         //过热判断时间2
//  s16 OH3nms;           //过热恢复判断时间3
//  s16 OH3REnms;         //过热判断时间3  
//}Tim_TypeDef;  
//  
////CTL

//typedef struct
//{ 
//  u8  RungDirection;        //电机实时运行方向
//  s8  SwitchCount;          //启动计数器
//  s8  SpeedShift;           //档位
//  s16 IncValue;             //加速值
//  s16 DecValue;             //减速值
//  s16 refTartemp;           //目标值
//  s16 refTar;               //目标值
//  s16 refCur;               //当前给定值
//  u16 rpm;                  //速度反馈
//  u16 rpmavg;               //速度反馈平均值
//  s32 ComNum;               //换相计数器
//  s32 MechCircleNum;        //机械圈数 
//  u8  RotorPosition ;       //转子位置
//  u8  RotorPositionqPre;    //前一时刻转子位置
//}spd_TypeDef;

//typedef struct
//{
//  s16 refTar;               //目标值
//  s16 refCur;               //当前给定值
//}Is_TypeDef;

//typedef struct
//{
//  Status_TypeDef State;             //系统状态
//  
//  u8  gStartC;                       //启停 
//  s8  gDirectionC;                   //运行方向
//  u8  gStopmodeC;                   //停机模式
//  u16 gBreakValueC;                  //刹车强度
//  u8  gPowerOnF;                     //上电标志  
//  u8  gStepCur;                     //当前扇区
//  u8  gStepPre;                     //上一个扇区

//  s8  gDirectionS;                   //运行方向
//  
//  Error_TypeDef  SysError;          //故障代码
//  FO_TypeDef FO;                    //FO     
//  OC_TypeDef OC;                    //过流重启
// 
//  spd_TypeDef    Spd;               //速度计算
//  Is_TypeDef     Is;
//  Tim_TypeDef    Tim;               //Tim
//  Stall_TypeDef  Stall;             //转速跟踪
//  Alig_TypeDef   Alig;              //定位
//  Ramp_TypeDef   Ramp;              //拖动
//  BEMF_TypeDef   Bemf;              //反电势

//} MCL_TypeDef;
//  
//extern MCL_TypeDef xdata Ctl;     //全局变量声明

///* Exported constants --------------------------------------------------------*/
///* Exported macro ------------------------------------------------------------*/
///* Exported functions ------------------------------------------------------- */
//extern void MCL_Function(void);     //全局外部函数声明
//extern void MCL_Bkin_Isr(void);
//extern void MainISR(void);        //全局外部函数声明
//#endif /* __MAIN_H */

///******************* (C) COPYRIGHT 2014 FT *****END OF FILE****/

extern MCL_TypeDef Ctl;
extern void MCL_Function(void);

