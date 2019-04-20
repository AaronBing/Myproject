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



枚举支持的ESC列表

```asm
A_ EQU 1 ; X X RC X MC MB MA CC X X Cc Cp Bc Bp Ac Ap
B_ EQU 2 ; X X RC X MC MB MA CC X X Ap Ac Bp Bc Cp Cc
C_ EQU 3 ; Ac Ap MC MB MA CC X RC X X X X Cc Cp Bc Bp
D_ EQU 4 ; X X RC X CC MA MC MB X X Cc Cp Bc Bp Ac Ap Com fets inverted
E_ EQU 5 ; L1 L0 RC X MC MB MA CC X L2 Cc Cp Bc Bp Ac Ap A with LEDs
F_ EQU 6 ; X X RC X MA MB MC CC X X Cc Cp Bc Bp Ac Ap
G_ EQU 7 ; X X RC X CC MA MC MB X X Cc Cp Bc Bp Ac Ap Like D, but
noninverted com fets
H_ EQU 8 ; RC X X X MA MB CC MC X Ap Bp Cp X Ac Bc Cc
I_ EQU 9 ; X X RC X MC MB MA CC X X Ac Bc Cc Ap Bp Cp
J_ EQU 10 ; L2 L1 L0 RC CC MB MC MA X X Cc Bc Ac Cp Bp Ap LEDs
K_ EQU 11 ; X X MC X MB CC MA RC X X Ap Bp Cp Cc Bc Ac Com fets inverted
L_ EQU 12 ; X X RC X CC MA MB MC X X Ac Bc Cc Ap Bp Cp
M_ EQU 13 ; MA MC CC MB RC L0 X X X Cc Bc Ac Cp Bp Ap X LED
N_ EQU 14 ; X X RC X MC MB MA CC X X Cp Cc Bp Bc Ap Ac
O_ EQU 15 ; X X RC X CC MA MC MB X X Cc Cp Bc Bp Ac Ap Like D, but low side
pwm
P_ EQU 16 ; X X RC MA CC MB MC X X Cc Bc Ac Cp Bp Ap X
Q_
EQU 17 ; Cp Bp Ap L1 L0 X RC X X MA MB MC CC Cc Bc Ac LEDs
R_ EQU 18 ; X X RC X MC MB MA CC X X Ac Bc Cc Ap Bp Cp
S_ EQU 19 ; X X RC X CC MA MC MB X X Cc Cp Bc Bp Ac Ap Like O, but
com fets inverted
T_ EQU 20 ; RC X MA X MB CC MC X X X Cp Bp Ap Ac Bc Cc
U_ EQU 21 ; MA MC CC MB RC L0 L1 L2 X Cc Bc Ac Cp Bp Ap X Like M, but with 3 LEDs
V_ EQU 22 ; Cc X RC X MC CC MB MA X Ap Ac Bp X X Bc Cp
W_ EQU 23 ; RC MC MB X CC MA X X X Ap Bp Cp X X X X Tristate
gate driver
```



选择要使用的端口映射（或取消选择全部用于外部批处理编译文件）

```asm
;ESCNO EQU A_
;ESCNO EQU B_
;ESCNO EQU C_
;ESCNO EQU D_
;ESCNO EQU E_
;ESCNO EQU F_
;ESCNO EQU G_
;ESCNO EQU H_
;ESCNO EQU I_
;ESCNO EQU J_
;ESCNO EQU K_
;ESCNO EQU L_
;ESCNO EQU M_
;ESCNO EQU N_
;ESCNO EQU O_
;ESCNO EQU P_
;ESCNO EQU Q_
;ESCNO EQU R_
;ESCNO EQU S_
;ESCNO EQU T_
;ESCNO EQU U_
;ESCNO EQU V_
;ESCNO EQU W_
```



选择MCU类型（或取消选择以与外部批处理编译文件一起使用）

```asm
MCU_48MHZ EQU 0
```



选择死区时间（或取消选择与外部批处理编译文件一起使用）

```asm
FETON_DELAY EQU 15;每步20.4ns
```



ESC选择陈述

----此处省略



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





















