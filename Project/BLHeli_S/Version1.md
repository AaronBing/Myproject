# BLHeli_s 程序整理

此文档作为BLHeli_s的学习文档，记录了疑问点和一些程序思路



## 1、简介

主时钟是内部24MHz振荡器（或48MHz，下面的时间减半）;

虽然代码中使用了24/48，但确切的时钟频率为24.5MHz或49.0MHz

---

定时器0（41.67ns计数）总是向上计数并用于; -  RC脉冲测量 

定时器1（41.67ns计数）总是向上计数并用于; -  DShot帧同步检测

定时器2（500ns计数）总是向上计数并用于; -  RC脉冲超时计数和换向时间

定时器3（500ns计数）总是向上计数并用于; - 换班超时; 

PCA0（41.67ns计数）总是计数并用于; - 硬件PWM生成

---

**中断处理**

进入中断程序时，C8051不会禁止中断。
还有一些中断标志需要由软件清除;

代码禁用某些中断例程中的中断;

- 在蜂鸣声期间禁用中断，以避免来自中断的可听干扰;

---

​	**电机控制：** 

- 无刷电机控制，每个电气360度有6种状态;

- 0deg的提前时间在一次换向后为零交叉30deg，在下一次之前为30deg

- 该实施中的时间推进名义上设定为15度;

- 电机pwm始终为互补

  **电机序列从零交叉开始：**

- 定时器等待：Wt_Comm 15deg;等待从过零点到实际换向的时间

- 定时器等待：Wt_Advance 15deg;是时候等待时间提前了。名义上的换向点是在此之后

- 定时器等待：Wt_Zc_Scan 7.5deg;在寻找过零点之前等待的时间

- 扫描过零点    22.5deg;标称，有一些电机变化

电机启动：

​	在正常的bemf换向运行开始之前，有一个启动阶段和一个初始运行阶段。



```asm
$NOMOD51
;必须根据所选环境修改软件的第一行：
; ESCNO EQU "ESC"
; MCU_48MHZ EQU "N"
; FETON_DELAY EQU "N"

至少8KB的flash
最小512B的SRAM
```



选择MCU类型（或取消选择以与外部批处理编译文件一起使用）

```asm
MCU_48MHZ EQU 0
```



选择死区时间（或取消选择与外部批处理编译文件一起使用）

```asm
FETON_DELAY EQU 15;每步20.4ns
```



编程初始值

```asm
DEFAULT_PGM_STARTUP_PWR EQU 9 ; 1=0.031 2=0.047 3=0.063 4=0.094 ;5=0.125 6=0.188 7=0.25 8=0.38 9=0.50 10=0.75 11=1.00 12=1.25 13=1.50

DEFAULT_PGM_COMM_TIMING EQU 3 ; 1=Low 2=MediumLow 3=Medium
							;4=MediumHigh 5=High
DEFAULT_PGM_DEMAG_COMP EQU 2 ; 1=Disabled 2=Low 3=High
DEFAULT_PGM_DIRECTION EQU 1 ; 1=Normal 2=Reversed 3=Bidir 4=Bidir rev
DEFAULT_PGM_BEEP_STRENGTH EQU 40 ; Beep strength
DEFAULT_PGM_BEACON_STRENGTH EQU 80 ; Beacon strength
DEFAULT_PGM_BEACON_DELAY EQU 4 ; 1=1m 2=2m 3=5m
								;4=10m 5=Infinite
```



COMMON    

```asm
DEFAULT_PGM_ENABLE_TX_PROGRAM EQU 1 ; 1=Enabled 0=Disabled
DEFAULT_PGM_MIN_THROTTLE EQU 37 ; 4*37+1000=1148
DEFAULT_PGM_MAX_THROTTLE EQU 208 ; 4*208+1000=1832
DEFAULT_PGM_CENTER_THROTTLE EQU 122 ; 4*122+1000=1488 (用于双向模式)
DEFAULT_PGM_ENABLE_TEMP_PROT EQU 7 ; 0=Disabled 1=80C 2=90C 3=100C 4=110C 5=120C 6=130C 7=140C
DEFAULT_PGM_ENABLE_POWER_PROT EQU 1 ; 1=Enabled 0=Disabled
DEFAULT_PGM_BRAKE_ON_STOP EQU 0 ; 1=Enabled 0=Disabled
DEFAULT_PGM_LED_CONTROL EQU 0 ; 用于LED控制的字节。每个LED 2bits, 0=Off, 1=On
```



临时寄存器定义

```asm
Temp1 EQU R0
Temp2 EQU R1
Temp3 EQU R2
Temp4 EQU R3
Temp5 EQU R4
Temp6 EQU R5
Temp7 EQU R6
Temp8 EQU R7
```

  

寄存器定义

```asm
DSEG AT 20h	;变量段

Bit_Access：DS 1;必须在这个地址。位可访问地址变量（非中断例程）
Bit_Access_Int：DS 1;位可访问地址变量（用于中断）

Rcp_Outside_Range_Cnt：DS 1; RC脉冲外部范围计数器（递增）
Rcp_Timeout_Cntd：DS 1 ; RC脉冲超时计数器（递减）

Flags0：DS 1;州旗。在init_start时复位
T3_PENDING EQU 0;定时器3挂起标志
DEMAG_DETECTED EQU 1;检测到过多的demag时间时设置
COMP_TIMED_OUT EQU 2;比较器读数超时时设置
```



```asm
标志2：DS 1;州旗。在init_start
RCP_UPDATED EQU 0;时不复位;新的RC脉冲长度值可用RCP_ONESHOT125 EQU 1; RC脉冲输入为OneShot125（125-250us）RCP_ONESHOT42 EQU 2; RC脉冲输入为OneShot42（41.67-83us）RCP_MULTISHOT EQU 3; RC脉冲输入为Multishot（5-25us）RCP_DSHOT EQU 4; RC脉冲输入是数字镜头RCP_DIR_REV EQU 5;双向模式下的RC脉冲方向RCP_FULL_RANGE EQU 6;设置完全输入信号范围（10002000us）时，忽略存储的校准值
```









