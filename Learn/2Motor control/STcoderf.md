## Hall检测

### hall_speed_pos_fdbk.h

此文件包含电机控制SDK霍尔速度和位置反馈组件的所有定义和功能原型。

HALL顺序存储空间

```
#define HALL_SPEED_FIFO_SIZE  ((uint8_t)18)
```

hall安装角度

```
#define DEGREES_120 0u
#define DEGREES_60 1u
```

#### HALL_Handle_t  结构体定义

**SpeednPosFdbk_Handle_t _Super;**  速度反馈结构体

<!--软件设置-->

**uint8_t  SensorPlacement;**

在此定义传感器的机械位置，参考电气循环。
允许的值为：DEGREES_120或DEGREES_60。

**int16_t  PhaseShift;**

在s16degree中定义信号H1的低到高转换与A相引起的Bemf的最大值之间的电相移。

**uint16_t SpeedSamplingFreqHz;**

计算电机速度的频率（Hz）。
它必须等于调用函数SPD_CalcAvrgMecSpeed01Hz的频率。

**uint8_t  SpeedBufferSize;**

用于计算平均速度的缓冲区大小。
它必须小于18。

<!--硬件设置-->

**uint32_t TIMClockFreq;**

定时器时钟频率以Hz表示。

**TIM_TypeDef * TIMx;**

用于HALL传感器管理的定时器。

**GPIO_TypeDef * H1Port;**

HALL传感器H1通道GPIO输入端口（如果在重新映射后使用）。
它必须是GPIOx x = A，B，..

**uint32_t  H1Pin;**   

HALL传感器H1通道GPIO输出引脚（如果使用，则在重新映射后）。
它必须是GPIO_Pin_x x = 0,1，... * /

**GPIO_TypeDef * H2Port;**

**uint32_t  H2Pin;**   

**GPIO_TypeDef * H3Port;**

**uint32_t H3Pin;**     

<!--功能设置-->

 **bool SensorIsReliable;** 

标记表示霍尔传感器信号的配置错误。

 **volatile bool RatioDec;**   

标志以避免连续预分频器递减。

 **volatile bool RatioInc;**        

标志以避免连续预分频器增量。

 **volatile uint8_t FirstCapt;**   

用于丢弃速度测量的第一次捕获的标志

 **volatile uint8_t BufferFilled;**  

从一开始就指示缓冲区中存在的速度测量值的数量。
它将是max bSpeedBufferSize，它用于验证速度平均的开始。
如果bBufferFilled低于bSpeedBufferSize，则将瞬时测量的速度作为平均速度返回。

  **volatile uint8_t OVFCounter;**  

  如果预分频器太低，则计数溢出

  **int16_t SensorSpeed[HALL_SPEED_FIFO_SIZE];**

保持最后的速度捕获

  **uint8_t SpeedFIFOIdx;**

要存储在速度传感器缓冲区中的下一个元素的指针

  **int16_t  CurrentSpeed;** 

在HALL_IRQ_HANDLER中计算的最新速度

  **int32_t  ElSpeedSum;** 

速度累加器用于加速平均速度计算

  **int16_t PrevRotorFreq;** 

用于存储检测到HALL_MAX_PSEUDO_SPEED时使用的最后一个有效转子电气速度

  **int8_t Direction;** 

两次捕获之间转子的瞬时方向

  **int8_t  NewSpeedAcquisition;** 

指示新的速度信息已存储在缓冲区中。

  **int16_t AvrElSpeedDpp;**

它是平均转子电速在每个电流控制周期16度表示。

  **uint8_t HallState;**    

当前的HALL状态配置

  **int16_t DeltaAngle;**   

霍尔传感器信号边缘的三角形角度在当前电气转子同步角度之间。
这是s16degrees。 

  **int16_t MeasuredElAngle;**

这是在每个霍尔传感器信号边缘处测量的电角度。
它被认为是电动转子角度的最佳测量。

  **int16_t TargetElAngle;**

这是基于hMeasuredElAngle在速度控制频率下计算的电角度目标。

  **int16_t CompSpeed;**    

用于使当前电角度与目标电角度同步的速度补偿系数。

  **uint16_t HALLMaxRatio;** 

最大TIM预分频比定义了最低预期速度反馈。

  **uint16_t SatSpeed;**    

如果测量的速度高于最大实际值，则返回值。

  **uint32_t PseudoFreqConv;**

HALL传感器之间的时间间隔ΔT之间的转换因子捕获，以计时器计数表示，以及以dpp表示的电子转子速度。
转子速度（dpp）= wPseudoFreqConv / Delta T

它将是（（CKTIM / 6）/（SAMPLING_FREQ））* 65536。

  **uint32_t MaxPeriod;**  

当转子的速度在应用中是最小的时，两个传感器边缘之间的时间延迟：这允许例如区分太低的频率。
这段时间应以计时器计数表示，它将是：

wMaxPeriod =（（10 * CKTIM）/ 6）/ MinElFreq（0.1Hz）。

  **uint32_t MinPeriod;**

当转子的速度在应用中是最大的时，两个传感器边缘之间的时间延迟：这允许例如区分毛刺。
这段时间应以计时器计数表示，它将是：

wSpeedOverflow =（（10 * CKTIM）/ 6）/ MaxElFreq（0.1Hz）

  **uint16_t HallTimeout;**

两个霍尔传感器信号之间的最大延迟以断言零速度表示为毫秒。

  **uint16_t OvfFreq;**   

定时器溢出的频率（从0到0x10000）将是：hOvfFreq = CKTIM / 65536。

  **uint16_t PWMNbrPSamplingFreq;** 

每个速度控制周期内的当前控制周期数：

（hMeasurementFrequency / hSpeedSamplingFreqHz） -  1。



<!--函数定义-->

```
void * HALL_TIMx_UP_IRQHandler( void * pHandleVoid );
void * HALL_TIMx_CC_IRQHandler( void * pHandleVoid );
void HALL_Init( HALL_Handle_t * pHandle );
void HALL_Clear( HALL_Handle_t * pHandle );
int16_t HALL_CalcElAngle( HALL_Handle_t * pHandle );
bool HALL_CalcAvrgMecSpeed01Hz( HALL_Handle_t * pHandle, int16_t * hMecSpeed01Hz );
void HALL_SetMecAngle( HALL_Handle_t * pHandle, int16_t hMecAngle );
```



### hall_speed_pos_fdbk.c

基于霍尔传感器的速度和位置反馈实现

该组件用于控制配备霍尔效应传感器的电机的应用。

该组件使用两个霍尔效应传感器的输出来测量速度和电机转子的位置。

<!--私人定义-->

<!--降低阈值以请求减少时钟预分频器-->

```
#define LOW_RES_THRESHOLD（（uint16_t）0x5500u）
```

```
#define HALL_COUNTER_RESET（（uint16_t）0u）
#define S16_120_PHASE_SHIFT（int16_t）（65536/3）
#define S16_60_PHASE_SHIFT（int16_t）（65536/6）

#define STATE_0（uint8_t）0 
#define STATE_1（uint8_t）1 
#define STATE_2（uint8_t）2 
#define STATE_3（uint8_t）3 
#define STATE_4（uint8_t）4 
#define STATE_5（uint8_t）5 
#define STATE_6（uint8_t）6 
#define STATE_7（uint8_t）7 

#define NEGATIVE（int8_t）-1 
#define POSITIVE（int8_t） 1 
#define NEGATIVE_SWAP（int8_t）-2 
#define POSITIVE_SWAP（int8_t）2
```

<!--使用每PWM数字单位（此处2 * PI rad = 0xFFFF）：-->

```
#define HALL_MAX_PSEUDO_SPEED（（int16_t）0x7FFF）
```

```
#define CCER_CC1E_Set（（uint16_t）0x0001）
#define CCER_CC1E_Reset（（uint16_t）0xFFFE）
```

```
static void HALL_Init_Electrical_Angle（HALL_Handle_t * pHandle）; static int16_t HALL_CalcAvrgElSpeedDpp（HALL_Handle_t * pHandle）;
```

函数实现

#### HALL_Init()

**@brief**	它使用HALL传感器初始化速度位置传感器管理所需的硬件外设（TIMx，GPIO和NVIC）。

**@param**  pHandle：hall_speed_pos_fdbk组件的当前实例的处理程序

**@retval** none

```javascript
void HALL_Init（HALL_Handle_t * pHandle）
{
	TIM_TypeDef * TIMx = pHandle-> TIMx; 
	uint16_t hMinReliableElSpeed01Hz = 
		pHandle  - > _Super.hMinReliableMecSpeed01Hz * 
		pHandle  - > _ Super.bElToMecRatio; 
	uint16_t hMaxReliableElSpeed01Hz = 
		pHandle  - > _ Super.hMaxReliableMecSpeed01Hz * 
		pHandle  - > _ Super.bElToMecRatio; 
	
	uint8_t bSpeedBufferSize; 
	uint8_t bIndex; 
	
	/ *调整系数：最小可测速度是x时间小于最小可靠速度* / 		   			
    hMinReliableElSpeed01Hz / = 4u; 
	
	/ *调整系数：最大可测速度是x时间比最大可靠速度大* / 		 	    	
    hMaxReliableElSpeed01Hz * = 2u; 
	
	pHandle-> OvfFreq =（uint16_t）（pHandle-> TIMClockFreq / 65536u）; 
	
	/ * SW Init * / 
	if（hMinReliableElSpeed01Hz == 0u）{
	/ *设置固定为150 ms * / 
	pHandle-> HallTimeout = 150u; 
	} else {
	/ *相应地设置最小可靠速度* / 
	/ * 10000来自mS和01Hz * 6来自传感器每60度* / 
	pHandle-> HallTimeout = 10000u /（6u * hMinReliableElSpeed01Hz）; } 
	
	/ *计算预分频器到TimeOut的最接近值（以mS为单位）* / 
	pHandle-> HALLMaxRatio =（pHandle-> HallTimeout * pHandle-> OvfFreq）/ 1000; 
	
	/ *将MaxPeriod对齐为溢出的倍数。* / 
	pHandle-> MaxPeriod =（pHandle-> HALLMaxRatio）* 65536uL; 
	
	pHandle-> SatSpeed = hMaxReliableElSpeed01Hz; 
	
	pHandle-> PseudoFreqConv =（（pHandle-> TIMClockFreq / 6u）/（pHandle  - > _ Super.hMeasurementFrequency））* 65536u; 
	
	pHandle-> MinPeriod =（（10u * pHandle-> TIMClockFreq）/ 6u）/ hMaxReliableElSpeed01Hz; 
	
	pHandle-> PWMNbrPSamplingFreq =（pHandle  - > _ Super.hMeasurementFrequency / pHandle-> SpeedSamplingFreqHz） -  1u; 
	
	/ *复位速度可靠性* / 
	pHandle-> SensorIsReliable = true; 
	
	/ *强制TIMx预分频器立即访问（gen更新事件）* / 
	LL_TIM_SetPrescaler（TIMx，pHandle-> HALLMaxRatio）; 		   	      		  		LL_TIM_GenerateEvent_UPDATE（TIMx）; 

    / *清除TIMx的挂起标志* / 
    LL_TIM_WriteReg（TIMx，SR，0）; 
    / *选择的输入捕获和更新（溢出）事件生成中断* / 
    
    /*Update事件源仅计数器溢出/下溢 */ 
    LL_TIM_SetUpdateSource（TIMx，LL_TIM_UPDATESOURCE_COUNTER）; 
    
    LL_TIM_EnableIT_CC1（TIMx）; 
    LL_TIM_EnableIT_UPDATE（TIMx）; 
    LL_TIM_SetCounter（TIMx，HALL_COUNTER_RESET）; 
    
    LL_TIM_CC_EnableChannel（TIMx，LL_TIM_CHANNEL_CH1）; 
    LL_TIM_EnableCounter（TIMx）; 
    
    / *擦除速度缓冲区* / 
    bSpeedBufferSize = pHandle-> SpeedBufferSize; 
    
    for（bIndex = 0u; bIndex <bSpeedBufferSize; bIndex ++）{
        pHandle-> SensorSpeed [bIndex] = 0; 
    }
}
```



#### HALL_Clear()

@brief 	清除软件FIFO“推送”最后的速度信息,必须在启动电机初始化速度测量过程之前调用此功能。

@param  pHandle：hall_speed_pos_fdbk组件的当前实例的处理程序

@retval

```javascript
void HALL_Clear（HALL_Handle_t * pHandle）{
	TIM_TypeDef * TIMx = pHandle-> TIMx; 
	
    / *屏蔽中断以确保干净的初始化* / 
    LL_TIM_DisableIT_CC1（TIMx）; 
    
    pHandle-> RatioDec = false; 
    pHandle-> RatioInc = false; 
    
    / *复位速度可靠性* / 
    pHandle-> SensorIsReliable = true; 
    
    / *未实施加速度测量。* / 
    pHandle  - > _ Super.hMecAccel01HzP = 0; 
    
    pHandle-> FirstCapt = 0u; 
    pHandle-> BufferFilled = 0u; 
    pHandle-> OVFCounter = 0u; 
    
    pHandle-> CompSpeed = 0; 
    pHandle-> ElSpeedSum = 0; 
    
    pHandle-> Direction = POSITIVE; 
    
    / *初始化速度缓冲区索引* / 
    pHandle-> SpeedFIFOIdx = 0u; 
    
    / *清除新的速度采集标志* / 
    pHandle-> NewSpeedAcquisition = 0; 
    
    / *部分重新初始化定时器* / 
    LL_TIM_SetPrescaler（TIMx，pHandle-> HALLMaxRatio）; 
    
    LL_TIM_SetCounter（TIMx，HALL_COUNTER_RESET）; 
    
    LL_TIM_EnableCounter（TIMx）; 
    
    LL_TIM_EnableIT_CC1（TIMx）; 
    
    HALL_Init_Electrical_Angle（pHandle）; 
}
```



#### HALL_CalcElAngle()

@brief 更新转子电角度，将最后测量的瞬时电气速度表达为dpp。

@param	pHandle：hall_speed_pos_fdbk组件的当前实例的处理程序

@retval i nt16_t以s16degree格式测量的电角度。

```
int16_t HALL_CalcElAngle( HALL_Handle_t * pHandle )
{

  if ( pHandle->_Super.hElSpeedDpp != HALL_MAX_PSEUDO_SPEED )
  {
    pHandle->MeasuredElAngle += pHandle->_Super.hElSpeedDpp;
    pHandle->TargetElAngle += pHandle->_Super.hElSpeedDpp;
    pHandle->_Super.hElAngle += pHandle->_Super.hElSpeedDpp + pHandle->CompSpeed;
    pHandle->PrevRotorFreq = pHandle->_Super.hElSpeedDpp;
  }
  else
  {
    pHandle->_Super.hElAngle += pHandle->PrevRotorFreq;
  }

  return pHandle->_Super.hElAngle;
}
```



#### HALL_CalcAvrgMecSpeed01Hz()

@brief 	必须至少调用此方法，并且执行速度控制的周期相同。
*该方法计算并存储转子的瞬时速度（考虑到测量频率在dpp中表示*），以便将其提供给HALL_CalcElAngle函数和SPD_GetElAngle。
*然后根据IRQ填充的缓冲区计算转子平均el速度（以dpp表示），然后 - 作为结果 - 计算，存储和返回 - 通过参数* hMecSpeed01Hz  - 转子平均机械速度，表示 在01Hz。
*然后检查，存储并返回传感器的可靠性状态; 在此函数中，通过IRQ管理的内部变量，参考派生的*传感器（HALL）的特定参数来测量可靠性。

@param	 pHandle：hall_speed_pos_fdbk组件的当前实例的处理程序

@param  hMecSpeed01Hz指向int16_t的指针，用于返回转子平均机械速度（01Hz）

@retval  true =传感器信息可靠

​		 false =传感器信息不可靠

```javascript
bool HALL_CalcAvrgMecSpeed01Hz( HALL_Handle_t * pHandle, int16_t * hMecSpeed01Hz )
{
  TIM_TypeDef * TIMx = pHandle->TIMx;
  int16_t SpeedMeasAux;
  bool bReliability;

  /* 计算转子瞬时速度 */
  SpeedMeasAux = pHandle->CurrentSpeed;

  if ( pHandle->SensorIsReliable )
  {
    /* 在转子速度信息外推期间没有检测到错误 */
    if ( LL_TIM_GetPrescaler ( TIMx ) >= pHandle->HALLMaxRatio )
    {
      /* 在启动时或非常低的频率 */
      /* 仅基于当前预分频值 */
      pHandle->_Super.hElSpeedDpp = 0;
      *hMecSpeed01Hz = 0;
    }
    else
    {
      pHandle->_Super.hElSpeedDpp = SpeedMeasAux;
      if ( SpeedMeasAux == 0 )
      {
        /* 速度太低了 */
        *hMecSpeed01Hz = 0;
      }
      else
      {
        /* 检查速度是否快 */
        if ( SpeedMeasAux != HALL_MAX_PSEUDO_SPEED )
        {
#ifdef HALL_MTPA
          {
            pHandle->CompSpeed = 0;
          }
#else
          {
            pHandle->TargetElAngle = pHandle->MeasuredElAngle;
            pHandle->DeltaAngle = pHandle->MeasuredElAngle - pHandle->_Super.hElAngle;
            pHandle->CompSpeed = ( int16_t )
            ( ( int32_t )( pHandle->DeltaAngle ) /
              ( int32_t )( pHandle->PWMNbrPSamplingFreq ) );
          }
#endif

          *hMecSpeed01Hz = HALL_CalcAvrgElSpeedDpp( pHandle );

          /* 将el_dpp转换为Mec01Hz */
          *hMecSpeed01Hz = ( int16_t )( ( *hMecSpeed01Hz *
                                          ( int32_t )pHandle->_Super.hMeasurementFrequency * 10 ) /
                                        ( 65536 * ( int32_t )pHandle->_Super.bElToMecRatio ) );

        }
        else
        {
          *hMecSpeed01Hz = ( int16_t )pHandle->SatSpeed;
        }
      }
    }
    bReliability = SPD_IsMecSpeedReliable( &pHandle->_Super, hMecSpeed01Hz );
  }
  else
  {
    bReliability = false;
    pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
    /* 如果速度不可靠，则El和Mec速度设置为0 */
    pHandle->_Super.hElSpeedDpp = 0;
    *hMecSpeed01Hz = 0;
  }

  pHandle->_Super.hAvrMecSpeed01Hz = *hMecSpeed01Hz;

  return ( bReliability );
}
```



#### HALL_TIMx_CC_IRQHandler()

@brief  类HALL的私有方法示例，用于实现在发生TIMx捕获事件时要调用的MC IRQ函数

@param pHandle：hall_speed_pos_fdbk组件的当前实例的处理程序

```javascript
void * HALL_TIMx_CC_IRQHandler( void * pHandleVoid )
{
  HALL_Handle_t * pHandle = ( HALL_Handle_t * ) pHandleVoid;
  TIM_TypeDef * TIMx = pHandle->TIMx;
  uint8_t bPrevHallState;
  uint32_t wCaptBuf;
  uint16_t hPrscBuf;
  uint16_t hHighSpeedCapture;

  if ( pHandle->SensorIsReliable )
  {
    /* 捕获事件生成此中断 */
    bPrevHallState = pHandle->HallState;

    if ( pHandle->SensorPlacement == DEGREES_120 )
    {
      pHandle->HallState  = LL_GPIO_IsInputPinSet( pHandle->H3Port, pHandle->H3Pin ) << 2
                            | LL_GPIO_IsInputPinSet( pHandle->H2Port, pHandle->H2Pin ) << 1
                            | LL_GPIO_IsInputPinSet( pHandle->H1Port, pHandle->H1Pin );
    }
    else
    {
      pHandle->HallState  = ( LL_GPIO_IsInputPinSet( pHandle->H2Port, pHandle->H2Pin ) ^ 1 ) << 2
                            | LL_GPIO_IsInputPinSet( pHandle->H3Port, pHandle->H3Pin ) << 1
                            | LL_GPIO_IsInputPinSet( pHandle->H1Port, pHandle->H1Pin );
    }

    switch ( pHandle->HallState )
    {
      case STATE_5:
        if ( bPrevHallState == STATE_4 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = pHandle->PhaseShift;
        }
        else if ( bPrevHallState == STATE_1 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_60_PHASE_SHIFT );
        }
        else
        {
        }
        break;

      case STATE_1:
        if ( bPrevHallState == STATE_5 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = pHandle->PhaseShift + S16_60_PHASE_SHIFT;
        }
        else if ( bPrevHallState == STATE_3 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT );
        }
        else
        {
        }
        break;

      case STATE_3:
        if ( bPrevHallState == STATE_1 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT );
        }
        else if ( bPrevHallState == STATE_2 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT +
                                                  S16_60_PHASE_SHIFT );
        }
        else
        {
        }

        break;

      case STATE_2:
        if ( bPrevHallState == STATE_3 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT
                                                  + S16_60_PHASE_SHIFT );
        }
        else if ( bPrevHallState == STATE_6 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_120_PHASE_SHIFT );
        }
        else
        {
        }
        break;

      case STATE_6:
        if ( bPrevHallState == STATE_2 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_120_PHASE_SHIFT );
        }
        else if ( bPrevHallState == STATE_4 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT );
        }
        else
        {
        }
        break;

      case STATE_4:
        if ( bPrevHallState == STATE_6 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT );
        }
        else if ( bPrevHallState == STATE_5 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift );
        }
        else
        {
        }
        break;

      default:
        /* 坏霍尔传感器配置更新速度可靠性 */
        pHandle->SensorIsReliable = false;

        break;
    }


#ifdef HALL_MTPA
    {
      pHandle->_Super.hElAngle = pHandle->MeasuredElAngle;
    }
#endif

    /* 丢弃第一次捕获 */
    if ( pHandle->FirstCapt == 0u )
    {
      pHandle->FirstCapt++;
      LL_TIM_IC_GetCaptureCH1( TIMx );
    }
    else
    {
      /* 用于验证平均速度测量 */
      if ( pHandle->BufferFilled < pHandle->SpeedBufferSize )
      {
        pHandle->BufferFilled++;
      }

      /* 存储最新的速度采集 */
      hHighSpeedCapture = LL_TIM_IC_GetCaptureCH1( TIMx );
      wCaptBuf = ( uint32_t )hHighSpeedCapture;
      hPrscBuf =  LL_TIM_GetPrescaler ( TIMx );

      /* 将溢出次数添加到计数器 */
      wCaptBuf += ( uint32_t )pHandle->OVFCounter * 0x10000uL;

      if ( pHandle->OVFCounter != 0u )
      {
        /* 使用预分频器调整捕获 */
        uint16_t hAux;
        hAux = hPrscBuf + 1u;
        wCaptBuf *= hAux;

        if ( pHandle->RatioInc )
        {
          pHandle->RatioInc = false;  /* 以前的捕获导致溢出 */
          /* 不要更改预分频器（由于预加载/更新机制导致的延迟） */
        }
        else
        {
          if ( LL_TIM_GetPrescaler ( TIMx ) < pHandle->HALLMaxRatio ) 
              /* 避免OVF w /非常低的频率 */
          {
            LL_TIM_SetPrescaler ( TIMx, LL_TIM_GetPrescaler ( TIMx ) + 1 );
              /*  在速度降低期间避免OVF */
            pHandle->RatioInc = true;   /* 新的prsc值仅在下次捕获时更新 */
          }
        }
      }
      else
      {
        /* 如果在上次捕获时prsc预加载减少，则存储当前寄存器+ 1 */
        if ( pHandle->RatioDec ) /* 并且不要再减少它 */
        {
          /* 使用预分频器调整捕获 */
          uint16_t hAux;
          hAux = hPrscBuf + 2u;
          wCaptBuf *= hAux;

          pHandle->RatioDec = false;
        }
        else  /* 如果在之前的捕获中未对预分频器进行修改 */
        {
          /* 使用预分频器调整捕获 */
          uint16_t hAux = hPrscBuf + 1u;
          wCaptBuf *= hAux;

          if ( hHighSpeedCapture < LOW_RES_THRESHOLD ) /* 如果捕获范围正确 */
          {
            if ( LL_TIM_GetPrescaler ( TIMx ) > 0u ) /* 或预分频器不能进一步减少 */
            {
              LL_TIM_SetPrescaler ( TIMx, LL_TIM_GetPrescaler ( TIMx ) - 1 ); /* 通过降低prsc来提高准确性 */
              /* 避免在下一次捕获中再次递减。（注册预加载延迟） */
              pHandle->RatioDec = true;
            }
          }
        }
      }

#if 0
      /* 存入缓冲区 */
      /* 检测到空速，擦除缓冲区 */
      if ( wCaptBuf > pHandle->MaxPeriod )
      {
        uint8_t bIndex;
        for ( bIndex = 0u; bIndex < pHandle->SpeedBufferSize; bIndex++ )
        {
          pHandle->SensorSpeed[bIndex]  = 0;
        }
        pHandle->BufferFilled = 0 ;
        pHandle->SpeedFIFOSetIdx = 1;
        pHandle->SpeedFIFOGetIdx = 0;
        /* 表示新的速度收购 */
        pHandle->NewSpeedAcquisition = 1;
        pHandle->ElSpeedSum = 0;
      }
      /* 过滤到快速...可能是一个小故障  ? */
      /* HALL_MAX_PSEUDO_SPEED在缓冲区中是临时的，并且从不包含在平均计算中*/
      else
#endif
        if ( wCaptBuf < pHandle->MinPeriod )
        {
          pHandle->CurrentSpeed = HALL_MAX_PSEUDO_SPEED;
          pHandle->NewSpeedAcquisition = 0;
        }
        else
        {
          pHandle->ElSpeedSum -= pHandle->SensorSpeed[pHandle->SpeedFIFOIdx]; /* value we gonna removed from the accumulator */
          if ( wCaptBuf >= pHandle->MaxPeriod )
          {
            pHandle->SensorSpeed[pHandle->SpeedFIFOIdx]  = 0;
          }
          else
          {
            pHandle->SensorSpeed[pHandle->SpeedFIFOIdx] = ( int16_t ) ( pHandle->PseudoFreqConv / wCaptBuf );
            pHandle->SensorSpeed[pHandle->SpeedFIFOIdx] *= pHandle->Direction;
            pHandle->ElSpeedSum += pHandle->SensorSpeed[pHandle->SpeedFIFOIdx];
          }
          /* Update pointers to speed buffer */
          pHandle->CurrentSpeed = pHandle->SensorSpeed[pHandle->SpeedFIFOIdx];
          pHandle->SpeedFIFOIdx++;
          if ( pHandle->SpeedFIFOIdx == pHandle->SpeedBufferSize )
          {
            pHandle->SpeedFIFOIdx = 0u;
          }
          /* Indicate new speed acquisitions */
          pHandle->NewSpeedAcquisition = 1;
        }
      /* Reset the number of overflow occurred */
      pHandle->OVFCounter = 0u;
    }
  }
  return MC_NULL;
}
```



#### HALL_TIMx_UP_IRQHandler()

@brief   Example of private method of the class HALL to implement an MC IRQ function

*         to be called when TIMx update event occurs

@param    pHandle: handler of the current instance of the hall_speed_pos_fdbk component

```javascript
void * HALL_TIMx_UP_IRQHandler( void * pHandleVoid )
{
  HALL_Handle_t * pHandle = ( HALL_Handle_t * ) pHandleVoid;
  TIM_TypeDef * TIMx = pHandle->TIMx;

  if ( pHandle->SensorIsReliable )
  {
    uint16_t hMaxTimerOverflow;
    /* an update event occured for this interrupt request generation */
    pHandle->OVFCounter++;

    hMaxTimerOverflow = ( uint16_t )( ( ( uint32_t )pHandle->HallTimeout * pHandle->OvfFreq )
                                      / ( ( LL_TIM_GetPrescaler ( TIMx ) + 1 ) * 1000u ) );
    if ( pHandle->OVFCounter >= hMaxTimerOverflow )
    {
      /* Set rotor speed to zero */
      pHandle->_Super.hElSpeedDpp = 0;

      /* Reset the electrical angle according the hall sensor configuration */
      HALL_Init_Electrical_Angle( pHandle );

      /* Reset the overflow counter */
      pHandle->OVFCounter = 0u;


#if 1
      /* Reset the SensorSpeed buffer*/
      uint8_t bIndex;
      for ( bIndex = 0u; bIndex < pHandle->SpeedBufferSize; bIndex++ )
      {
        pHandle->SensorSpeed[bIndex]  = 0;
      }
      pHandle->BufferFilled = 0 ;
      pHandle->CurrentSpeed = 0;
      pHandle->SpeedFIFOIdx = 1;
      pHandle->ElSpeedSum = 0;
#else
      pHandle->ElSpeedSum -= pHandle->SensorSpeed[pHandle->SpeedFIFOIdx];
      pHandle->SensorSpeed[pHandle->SpeedFIFOSetIdx]  = 0;
      pHandle->CurrentSpeed = 0;
      pHandle->SpeedFIFOIdx++;
      if ( pHandle->SpeedFIFOIdx == pHandle->SpeedBufferSize )
      {
        pHandle->SpeedFIFOIdx = 0u;
      }
#endif
    }
  }
  return MC_NULL;
}
```



#### HALL_CalcAvrgElSpeedDpp()

@brief   Compute and returns the average rotor electrical speed express in dpp

@param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component

@retval   int16_t the average rotor electrical speed express in dpp

```javascript
static int16_t HALL_CalcAvrgElSpeedDpp( HALL_Handle_t * pHandle )
{

  if ( pHandle->NewSpeedAcquisition == 1 )
  {

    if ( pHandle->BufferFilled < pHandle->SpeedBufferSize )
    {
      pHandle->AvrElSpeedDpp = ( int16_t )  pHandle->CurrentSpeed;
    }
    else
    {
      pHandle->AvrElSpeedDpp = ( int16_t )( pHandle->ElSpeedSum / ( int32_t )(
                                              pHandle->SpeedBufferSize ) ); /* Average value */
    }

    /* Clear new speed acquisitions flag */
    pHandle->NewSpeedAcquisition = 0;
  }

  return pHandle->AvrElSpeedDpp;
}
```



#### HALL_Init_Electrical_Angle()

@brief    Read the logic level of the three Hall sensor and individuates in this

*         way the position of the rotor (+/- 30ï¿½). Electrical angle is then
*         initialized.

@param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component

```javascript
static void HALL_Init_Electrical_Angle( HALL_Handle_t * pHandle )
{

  if ( pHandle->SensorPlacement == DEGREES_120 )
  {
    pHandle->HallState  = LL_GPIO_IsInputPinSet( pHandle->H3Port, pHandle->H3Pin ) << 2
                          | LL_GPIO_IsInputPinSet( pHandle->H2Port, pHandle->H2Pin ) << 1
                          | LL_GPIO_IsInputPinSet( pHandle->H1Port, pHandle->H1Pin );
  }
  else
  {
    pHandle->HallState  = ( LL_GPIO_IsInputPinSet( pHandle->H2Port, pHandle->H2Pin ) ^ 1 ) << 2
                          | LL_GPIO_IsInputPinSet( pHandle->H3Port, pHandle->H3Pin ) << 1
                          | LL_GPIO_IsInputPinSet( pHandle->H1Port, pHandle->H1Pin );
  }

  switch ( pHandle->HallState )
  {
    case STATE_5:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_1:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_60_PHASE_SHIFT +
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_3:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT +
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_2:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift - S16_120_PHASE_SHIFT -
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_6:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT -
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_4:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT / 2 );
      break;
    default:
      /* Bad hall sensor configutarion so update the speed reliability */
      pHandle->SensorIsReliable = false;
      break;
  }

  /* Initialize the measured angle */
  pHandle->MeasuredElAngle = pHandle->_Super.hElAngle;

}
```









