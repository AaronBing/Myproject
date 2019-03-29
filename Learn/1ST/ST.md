# 三相永磁同步电动机

结构

| 组建 | 结构                   |
| ---- | ---------------------- |
| 转子 | 外转子/内转子          |
|      | 表面贴装磁石内嵌式磁石 |
| 绕组 | 集中绕组，分布绕组     |

定律

| 名称     | 功能               | 公式 |
| -------- | ------------------ | ---- |
| 库伦定律 | 静止点电荷相互作用 |      |
|          |                    |      |
|          |                    |      |



#  1、STM32Cube

## stm32固件包介绍

Drivers

Middlewares

Projects

Utilities



### Drivers文件夹

|            名称             | 描述                                                         |
| :-------------------------: | :----------------------------------------------------------- |
|         BSP 文件夹          | 也叫板级支持包，此支持包提供的是直接与硬件打交道的 API，例如触摸屏， LCD， SRAM 以及 EEPROM 等板载硬件资源等驱动。 BSP 文件夹下面有多种 ST 官方 Discovery 开发板, Nucleo 开发板以及 EVAL 板的 硬件驱动 API 文件，每一种开发板对应一个文件夹。 |
|        CMSIS 文件夹         | 顾名思义就是符合 CMSIS 标准的软件抽象层组件相关文件。文件夹内部文件比较多。主要包括 DSP 库(DSP_LIB 文件夹)， Cortex-M 内核及其设备文件 （Include 文件夹）， 微控制器专用头文件/启动代码/ 专用系统文件等(Device 文件夹)。 |
| STM32F7xx_HAL_Driver 文件夹 | 这个文件夹非常重要，它包含了所有的 STM32F7xx 系列 HAL 库头文件和源文件，也就是所有底层硬件抽象层 API 声明和定义。 它的作用是屏蔽了复杂的 硬件寄存器操作，统一了外设的接口函数。 该文件 夹包含 Src 和 Inc 两个子文件夹，其中 Src 子文件夹 存放的是.c 源文件， Inc 子文件夹存放的是与之对应 的.h 头文件。每个.c 源文件对应一个.h 头文件。源文件名称基本遵循 stm32f7xx_hal_ppp.c 定义格式， 头文件名称基本遵循 stm32f7xx_hal_ppp.h 定义格 式。比如 gpio 相关的 API 的声明和定义在文件 stm32f7xx_hal_gpio.h 和 stm32f7xx_hal_gpio.c 中。 |

###  

### Middlewares

 该文件夹下面有 ST 和 Third_Party 2 个子文件夹。 ST 文件夹下面存放的是 STM32 相关的 一些文件，包括 STemWin 和 USB 库等。 Third_Party 文件夹是第三方中间件，这些中间价都是 非常成熟的开源解决方案。     

 

###  Project

该文件夹存放的是一些可以直接编译的实例工程。每个文件夹对应一个 ST 官方的 Demo 板。     



###  Utilities 

 该文件夹下面是一些其他组件，在项目中使用得不多。    

 



# Keil的Debug功能

汇编窗口：通过该按钮，就可以查看汇编代码，这对分析程序很有用。 

堆栈局部变量窗口：通过该按钮， 显示 Call Stack+Locals 窗口，显示当前函数的局部变量 及其值，方便查看。 观察窗口： MDK5 提供 2 个

观察窗口（下拉选择），该按钮按下，会弹出一个显示变量的 窗口， 输入你所想要观察的变量/表达式，即可查看其值， 是很常用的一个调试窗口。 

内存查看窗口： MDK5 提供 4 个内存查看窗口（下拉选择）， 该按钮按下，会弹出一个内 存查看窗口，可以在里面输入你要查看的内存地址，然后观察这一片内存的变化情况。是很常 用的一个调试窗口 

串口打印窗口： MDK5 提供 4 个串口打印窗口（下拉选择）， 该按钮按下，会弹出一个类 似串口调试助手界面的窗口，用来显示从串口打印出来的内容。 

逻辑分析窗口： 该图标下面有 3 个选项（下拉选择），我们一般用第一个，也就是逻辑分析 窗口(Logic Analyzer)，点击即可调出该窗口， 通过 SETUP 按钮新建一些 IO 口，就可以观察这 些 IO 口的电平变化情况，以多种形式显示出来，比较直观。 

系统查看窗口： 该按钮可以提供各种外设寄存器的查看窗口（通过下拉选择），选择对应外 设，即可调出该外设的相关寄存器表，并显示这些寄存器的值，方便查看设置的是否正确。    











# 2、HAL库笔记



## 学习方法

1、掌握时钟树图

2、Datasheet，编程手册

3、多思考，多动手







## 关键文件介绍

**1、HAL 库关键文件**    

| 文件                      | 描述                                                         |
| ------------------------- | ------------------------------------------------------------ |
| stm32f7xx_hal_ppp.c/.h    | 基本外设的操作 API,ppp 代表任意外设。 其 中 stm32f7xx_hal_cortex.c/.h 比较特殊，它是 一些 Cortex 内核通用函数声明和定义，例如 中断优先级 NVIC 配置，系统软复位以及 Systick 配置等。 |
| stm32f7xx_hal_ppp_ex.c/.h | 拓展外设特性的 API。                                         |
| sm32f7xx_hal.c            | 包含 HAL 通用 API（比如 HAL_Init,HAL_DeInit,HAL_Delay 等）。 |
| stm32f7xx_hal.h           | HAL 的头文件，它应被客户代码所包含。                         |
| stm32f7xx_hal_conf.h      | HAL 的配置文件， 主要用来选择使能何种外 设以及一些时钟相关参数设置。 其本身应该 被客户代码所包含。 |
| stm32f7xx_hal_def.h       | 包含 HAL 的通用数据类型定义和宏定义                          |
| stm32f7xx_II_ppp.c/.h     | 在一些复杂外设中实现底层功能，它们在 stm32f7xx_hal_ppp.c 中被调用 |

 **2、stm32f7xx_it.c/stm32f7xx_it.h** 

 tm32f7xx_it.h 中主要是一些中断服务函数的申明。 stm32f7xx_it.h 中是这些中断服务函数定义，而这些函数定义除了 Systick 中断服务函数 SysTick_Handler 外基本都是空函数，没有任何控制逻辑。  

  

**3、stm32f7xx.h 头文件**    

头文件 stm32f7xx.h 内容看似非常少，却非常重要，它是所有 stm32f7 系列的顶层头文件。 使用 STM32F7 任何型号的芯片，都需要包含这个头文件。同时，因为 stm32f7 系列芯片型 号非常多， ST 为每种芯片型号定义了一个特有的片上外设访问层头文件，比如 STM32F767 系列， ST 定义了一个头文件 stm32f767xx.h，然后 stm32f7xx.h 顶层头文件会根据工程芯片 型号，来选择包含对应芯片的片上外设访问层头文件。    



**4、stm32f767xx.h 头文件**

根据前面的讲解， stm32f767xx.h 是 stm32f767 系列芯片通用的片上外设访问层头文件，只 要我们进行STM32F767 开发，就必然要使用到该文件。打开该文件我们可以看到里面主要 是一些结构体和宏定义标识符。这个文件的主要作用是寄存器定义声明以及封装内存操作。 



**5、system_stm32f7xx.c/system_stm32f7xx.h 文件**    

头文件 system_stm32f7xx.h和源文件 system_stm32f7xx.c主要是声明和定义了系统初始化函 数 SystemInit 以及系统时钟更新函数 SystemCoreClockUpdate。

SystemInit 函数的作用是进行 时钟系统的一些初始化操作以及中断向量表偏移地址设置，但它并没有设置具体的时钟值， 这是与标准库的最大区别，在使用标准库的时候， SystemInit 函数会帮我们配置好系统时钟 配置相关的各个寄存器。 在启动文件 startup_stm32f767xx.s 中会设置系统复位后，直接调 用 SystemInit 函数进行系统初始化。 SystemCoreClockUpdate 函数是在系统时钟配置进行修 改后，调用这个函数来更新全局变量 SystemCoreClock 的值，变量 SystemCoreClock 是一个 全局变量，开放这个变量可以方便我们在用户代码中直接使用这个变量来进行一些时钟运 算。    



**6、stm32f7xx_hal_msp.c 文件**

MSP，全称为 MCU support package,函数名字中带有 MspInit 的函数，它们的作用是进行 MCU 级别硬件初始化设置，并且它们通常会被上一层的初始化函数所调用， 这样做的目的是为了把 MCU 相关的硬件初始化剥夺出来，方便用户代码在不同型号的 MCU 上移植。 

stm32f7xx_hal_msp.c 文件定义了两个函数 HAL_MspInit 和 HAL_MspDeInit。这两个 函数分别被文件 stm32f7xx_hal.c 中的 HAL_Init 和 HAL_DeInit 所调用。 HAL_MspInit 函数的 主要作用是进行 MCU 相关的硬件初始化操作。例如我们要初始化某些硬件，我们可以硬件相关的初始化配置写在 HAL_MspDeinit 函数中。这样的话，在系统启动后调用了 HAL_Init 之 后 ， 会 自 动 调 用 硬 件 初 始 化 函 数 。 实 际 上 ， 我 们 在 工 程 模 板 中 直 接 删 掉 stm32f7xx_hal_msp.c 文件也不会对程序运行产生任何影响。  



**7、startup_stm32f767xx.s 启动文件** 

STM32 系列所有芯片工程都会有一个.s 启动文件。对于不同型号的 stm32 芯片启动文件也 是不一样的。我们的开发板是 STM32F767 系列，所以我们需要使用与之对应的启动文件 startup_stm32f767xx.s。启动文件的作用主要是进行堆栈的初始化，中断向量表以及中断函 数定义等。启动文件有一个很重要的作用就是系统复位后引导进入 main 函数。打开启动文件 startup_stm32f767xx.s，可以看到下面几行代码：    

​							**HAL 库工程文件包含关系**    

![1553590070926](C:\Users\ADMINI~1\AppData\Local\Temp\1553590070926.png)

从上面工程文件包含关系图可以看出，顶层头文件 stm32f7xx.h 直接或间接包含了其他所 有工程必要头文件，所以在我们的用户代码中，我们只需要包含顶层头文件 stm32f7xx.h 即可。    



## __weak 修饰符讲解 

weak 顾名思义是“弱”的意思，所以如果函数名称前面加上__weak 修饰符，我们一般称 这个函数为“弱函数”。加上了__weak 修饰符的函数， 用户可以在用户文件中重新定义一个同 名函数，最终编译器编译的时候，会选择用户定义的函数，如果用户没有重新定义这个函数， 那么编译器就会执行__weak 声明的函数，并且编译器不会报错 。  



## 回调函数执行过程解读

驱动方式的初始化流程就是：HAL_USART_Init()—>HAL_USART_MspInit() ，先初始化与 MCU 无 关 的 串 口 协 议 ， 再 初 始 化 与 MCU 相关的串口 引 脚 。 在 STM32 的 HAL 驱 动 中 HAL_PPP_MspInit()作为回调， 被 HAL_PPP_Init()函数所调用。当我们需要移植程序到 STM32F1 平台的时候，我们只需要修改 HAL_PPP_MspInit 函数内容而不需要修改 HAL_PPP_Init 入口参 数内容。    



## 程序执行流程图 

![1553591441420](C:\Users\ADMINI~1\AppData\Local\Temp\1553591441420.png)



## STM32基础

### **MDK下C语言**

#### 1、位操作

1) 不改变其他位的值的状况下，对某几个位进行设值。    

```c
GPIOA->ODR &=0XFF0F; //将第 4-7 位清 0
```

然后再与需要设置的值进行|或运算    

```c
GPIOA->ODR |=0X0040; //设置相应位的值，不改变其他位的值
```

2) 移位操作提高代码的可读性。 

移位操作在单片机开发中也非常重要， 我们来看看下面一行代码

```c
GPIOA->ODR| = 1 << 5; 
```

这个操作就是将 ODR 寄存器的第 5 位设置为 1，

这是为了提高代码的可读性以及可重用性。这行代码可以很直观 明了的知道，是将第 5 位设置为 1，其他位的值不变。

```c
GPIOA->ODR =0x0020; 
```

这样的代码可读性非常差同时也不好重用。    

3) ~取反操作使用技巧    

例如 GPIOA->ODR 寄存器的每一位都用来设置一个 IO 口的输出状态，某个时刻我们 希望去设置某一位的值为 0，同时其他位都为 1，简单的作法是直接给寄存器设置一个值：    

```c
GPIOA->ODR =0xFFF7；
```

这样的作法设置第 3 位为 0，但是这样的写法可读性很差。看看如果我们使用取反操作怎么实现：    

```c
GPIOA->ODR= (uint16_t)~(1<<3);
```

看这行代码应该很容易明白， 我们设置的是 ODR 寄存器的第 3 位为 0，其他位为 1，可读性 非常强。

#### 2、define 宏定义   

define 是 C 语言中的预处理命令，它用于宏定义，可以提高源代码的可读性，为编程提供 方便。    

```c
#define 标识符 字符串
```

 **条件编译**

```c
#ifdef 标识符
	程序段 1
#else
	程序段 2
#endif
```

#### 3、extern变量申明

C 语言中 extern 可以置于变量或者函数前，以表示变量或者函数的定义在别的文件中，提示 编译器遇到此变量和函数时在其他模块中寻找其定义。这里面要注意，对于 extern 申明变量可以多次，但定义只有一次。    

#### 4、typedef类型别名

typedef 用于为现有类型创建一个新的名字，或称为类型别名，用来简化变量的定义。 typedef 在 MDK 用得最多的就是定义结构体的类型别名和枚举类型了。    

```c
struct _GPIO
{
__IO uint32_t MODER;
__IO uint32_t OTYPER;
…
};
```

定义了一个结构体 GPIO，这样我们定义变量的方式为：    

```c
struct _GPIO GPIOA;//定义结构体变量 GPIOA
```

但是这样很繁琐， MDK 中有很多这样的结构体变量需要定义。这里我们可以为结体定义一个别 名 GPIO_TypeDef，这样我们就可以在其他地方通过别名 GPIO_TypeDef 来定义结构体变量了。 方法如下：    

```c
typedef struct
{
__IO uint32_t MODER;
__IO uint32_t OTYPER;
…
} GPIO_TypeDef;
```

Typedef 为结构体定义一个别名 GPIO_TypeDef，这样我们可以通过 GPIO_TypeDef 来定义结构体 变量：    

```c
GPIO_TypeDef _GPIOA,_GPIOB;
```

#### 5、结构体

声明结构体类型：    

```c
Struct 结构体名{
	成员列表;
}变量名列表；
```

例如：

```c
Struct G_TYPE {
	uint32_t Pin;
	uint32_t Mode;
	uint32_t Speed;
}GPIOA， GPIOB;
```

在结构体申明的时候可以定义变量，也可以申明之后定义，方法是：    

```c
Struct 结构体名字 结构体变量列表 ;
```

结构体成员变量的引用方法是：    

```c
结构体变量名字.成员名
```

结构体指针变量定义也是一样的，跟其他变量没有啥区别。 

例如： struct G_TYPE *GPIOC； //定义结构体指针变量 GPIOC; 

结构体指针成员变量引用方法是通过“->”符号实现，比如要访问 GPIOC 结构体指针指向的结 构体的成员变量 Speed,方法是：    

```c
GPIOC-> Speed;
```

在以后的开发过程中，如果你的变量定义过多， 如果某几个变量是用来描述某一个对象，你可以考虑将这些变量定义在结构体中，这样也许可 以提高你的代码的可读性。    



### STM32F7 总线架构    





### STM32F7 时钟系统    

#### 1、STM32F7 时钟树概述    

![1553606623076](C:\Users\ADMINI~1\AppData\Local\Temp\1553606623076.png)

在 STM32F7 中，有 5 个最重要的时钟源，为 HSI、 HSE、 LSI、 LSE、 PLL。 

PLL 实 际是分为三个时钟源，分别为主 PLL 和 I2S 部分专用 PLLI2S 和 SAI 部分专用 PLLSAI。

从时 钟频率来分可以分为高速时钟源和低速时钟源，在这 5 个中 HSI， HSE 以及 PLL 是高速时钟， LSI 和 LSE 是低速时钟。

从来源可分为外部时钟源和内部时钟源，外部时钟源就是从外部通过接晶振的方式获取时钟源，其中 HSE 和 LSE 是外部时钟源，其他的是内部时钟源。

①、 LSI 是低速内部时钟， RC 振荡器，频率为 32kHz 左右。 LSI 主要可以作为 IWDG 独立看门 狗时钟， LPTimer 低功耗定时器时钟以及 RTC 时钟。    

②、 LSE 是低速外部时钟，接频率为 32.768kHz 的石英晶体。 这个主要是 RTC 的时钟源。

③、HSE 是高速外部时钟，可接石英/陶瓷谐振器，或者接外部时钟源，频率范围为 4MHz~26MHz。 阿波罗 STM32F7 开发板接的是 25MHz 外部晶振。 HSE 可以直接做为系统时钟或者 PLL 输入 时钟，同时它经过 2~31 分频后也可以作为 RTC 时钟。 

④、 HSI 是高速内部时钟， RC 振荡器， 频率为 16MHz。 可以直接作为系统时钟或者用作 PLL 输入，同时它经过 488 分频之后也可以作为 HDMI-CEC 时钟。 

⑤、 PLL 为锁相环倍频输出。 STM32F7 有三个 PLL:

​	 1） 主 PLL(PLL)由 HSE 或者 HSI 提供时钟信号，并具有两个不同的输出时钟。 第一个输出 PLLP 用于生成高速的系统时钟（最高 216MHz） 第二个输出 PLLQ 为 48M 时钟， 用于 USB OTG FS 时钟，随机数发生器的时钟和 SDMMC 时钟。

​	 2） 第一个专用 PLL(PLLI2S)用于生成精确时钟， 在 I2S、 SAI 和 SPDIFRX 上实现高品质音频性 能。 其中， N 是用于 PLLI2S vco 的倍频系数，其取值范围是： 50~432； R 是 I2S 时钟的 分频系数，其取值范围是： 2~7； Q 是 SAI 时钟分频系数，其取值范围是： 2~15； P 没 用到。

​	 3） 第二个专用 PLL(PLLSAI)用于为 SAI 接口生成时钟，生成 LCD-TFT 时钟以及可供 USB OTG FS、 SDMMC 和 RNG 选择的 48MHz 时钟。 其中， N 是用于 PLLSAI vco 的倍频系数，其取值 范围是： 50~432； Q 是 SAI 时钟分频系数，其取值范围是： 2~15； R 是 LTDC 时钟的分 频系数，其取值范围是： 2~7； P 没用到。    

这里我们着重看看主 PLL 时钟第一个高速时钟输出 PLLP 的计算方法，其他 PLL 时钟计算方法 类似。图 4.3.1.2 是主 PLL 的时钟图。    

![1553607662706](C:\Users\ADMINI~1\AppData\Local\Temp\1553607662706.png)

从上图可以看出。主 PLL 时钟的时钟源要先经过一个分频系数为 M 的分频器，然后经过 倍频系数为 N 的倍频器出来之后还需要经过一个分频系数为 P（第一个输出 PLLP）或者 Q（第 二个输出 PLLQ）的分频器分频之后，最后才生成最终的主 PLL 时钟。    

---

例如我们的外部晶振选择 25MHz。同时我们设置相应的分频器 M=25，倍频器倍频系数 N=432， 分频器分频系数 P=2，那么主 PLL 生成的第一个输出高速时钟 PLLP 为：    

```
PLL=25MHz * N/ (M*P)=25MHz* 432/(25*2) = 216MHz
```

---

```
A. 这是低功耗定时器 LPTimer 时钟，从图中可以看出， LPTimer 有四个时钟源可以 选择，分别为 LSI、 HSI、 LSE 和 PCLKx，默认情况下 LPTimer 选用 PCLKx 作为 时钟源。 

B. 这里是 USART 时钟源。从图中可以看出， USART 时钟源可选为 LSE、 HSI、 SYSCLK 以及 PCLKx，默认情况下 USART 选用 PCLKx 作为时钟源。 

C. 这里是硬件 I2C 时钟源，从图上可以看出， I2C 可选时钟源为 HSI、 SYSCLK 以 及 PCLKx。 默认情况下 I2C 选用 PCLKx 作为时钟源。 

D. 这是 STM32F7 独立看门狗 IWDG 时钟，来源为 LSI。 

E. 这里是 RTC 时钟源，可选 LSI、 LSE 和 HSE 的 2~31 分频。 

F. 这是 SDMMC 时钟源，来源为系统时钟 SYSCLK 或者 PLL48CLK，其中 PLL48CLK 来源为 PLLQ 或者 PLLSAIP。 

G. 这是 STM32F7 输出时钟 MCO1 和 MCO2。
MCO1 是向芯片的 PA8 引脚输出时钟。 它有四个时钟来源分别为： HSI,LSE,HSE 和 PLL 时钟， MCO1 时钟源经过 1~5 分 频后向 PA8 引脚输出时钟。
MCO2 是向芯片的 PC9 输出时钟，它同样有四个时钟 来源分别为： HSE,PLL， SYSCLK 以及 PLLI2S 时钟， MCO2 时钟源同样经过 1~5 分频后向 PC9 引脚输出时钟。    

H. 这是系统时钟 SYSCLK 时钟源，可选 HSI、 HSE 和 PLLCLK。 
HSI 是内部 16MHz时钟精度不够， 
HSE 是外部晶振产生时钟频率较低，
大部分情况下系统都会选择PLLCLK 作为系统时钟。

I. 这是以太网 PTP 时钟，来源为系统时钟 SYSCLK。

J. 这是 AHB 总线预分频器，分频系数为 2的N次方(N=0~9)。系统时钟 SYSCLK 经过 AHB预分频器之后产生 AHB 总线时钟 HCLK。

K. 这是 APBx 预分频器（分频系数可选 1,2,4,8,16），HCLK（AHB 总线时钟）经过APBx预分频器之后，产生PCLKx。这里大家还要注意，APBx定时器时钟是PCLKx
经过倍频后得来，倍频系数为 1 或者 2，如果 APBx 预分频系数等于 1，那么这里
的倍频系数为 1，否则倍频系数为 2。

L~N. 这是 PLL 时钟。
L 为主 PLL 时钟， M 为专用 PLL 时钟 PLLI2S， N 为专用 PLL时钟 PLLSAI。主 PLL 主要用来产生 PLL 时钟作为系统时钟，同时 PLL48CLK 时钟也可以选择 PLLQ 或者 PLLSAIP。 
PLLI2S 主要用来为 I2S、 SAI 和 SPDIFRX产生精确时钟。 
PLLSAIP 则为 SAI 接口生成时钟，生成 LCD-TFT 时钟以及可供USB OTG FS、 SDMMC 和 RNG 选择的 48MHz 时钟 PLL48CLK。

O. 这是 SPDIFRX 时钟，由 PLLI2SP 提供。

P． 这是 LCD-TFT 时钟，由 PLLSAIP 提供

Q． 这是 STM32F7 内部以太网 MAC 时钟的来源。对于 MII 接口来说，必须向外部
PHY 芯片提供 25Mhz 的时钟，这个时钟，可以由 PHY 芯片外接晶振，或者使用
STM32F7 的 MCO 输出来提供。然后， PHY 芯片再给 STM32F7 提供
ETH_MII_TX_CLK 和 ETH_MII_RX_CLK 时钟。对于 RMII 接口来说，外部必须
提供 50Mhz 的时钟驱动 PHY 和 STM32F7 的 ETH_RMII_REF_CLK，这个 50Mhz
时钟可以来自 PHY、有源晶振或者 STM32F7 的 MCO。我们的开发板使用的是 RMII
接口，使用 PHY 芯片提供 50Mhz 时钟驱动 STM32F7 的 ETH_RMII_REF_CLK。

R. 这里是指外部 PHY 提供的 USB OTG HS（60MHZ）时钟。
```





#### 2、STM32F7 时钟初始化配置   

SystemInit 主要做了如下三个方面工作： 

1） FPU 设置 

2） 复位 RCC 时钟配置为默认复位值（默认开启 HSI）    

3） 中断向量表地址配置    

HAL 库的 SystemInit 函数除了打开 HSI 之外，没有任何时钟相关配置，所以使用 HAL 库我们必须编 写自己的时钟配置函数。    

```
/时钟设置函数
// VCO 频率 Fvco=Fs*(plln/pllm);
//系统时钟频率 Fsys=Fvco/pllp=Fs*(plln/(pllm*pllp));
// USB,SDIO,RNG 等的时钟频率 Fusb=Fvco/pllq=Fs*(plln/(pllm*pllq));

//Fs:PLL 输入时钟频率,可以是 HSI,HSE 等.
//plln:主 PLL 倍频系数(PLL 倍频),取值范围:64~432.
//pllm:主 PLL 和音频 PLL 分频系数(PLL 之前的分频),取值范围:2~63.
//pllp:系统时钟的主 PLL 分频系数(PLL 之后的分频),取值范围:2,4,6,8.(仅限这 4 个值!)
//pllq:USB/SDIO/随机数产生器等的主 PLL 分频系数(PLL 之后的分频),取值范围:2~15.

//外部晶振为 25M 的时候,推荐值:plln=432,pllm=25,pllp=2,pllq=9.
//得到:Fvco=25*(432/25)=432Mhz
// Fsys=432/2=316Mhz
// Fusb=432/9=48Mhz
//返回值:0,成功;1,失败

```







#### 3、STM32F7 时钟使能和配置    

在配置好时钟系统之后，如果我们要使用某些外设， 例如 GPIO， ADC 等，我们还要使能这些外设时钟。 如果在使用外设之前 没有使能外设时钟，这个外设是不可能正常运行的。     



#### 4、IO引脚复用器和映射

STM32F7 系列微控制器 IO 引脚通过一个复用器连接到内置外设或模块。该复用器一次只允 许一个外设的复用功能（AF） 连接到对应的 IO 口。这样可以确保共用同一个 IO 引脚的外设之 间不会发生冲突。    

每个 IO 引脚都有一个复用器，该复用器采用 16 路复用功能输入（AF0 到 AF15），可通过 GPIOx_AFRL(针对引脚 0-7)和 GPIOx_AFRH（针对引脚 8-15）寄存器对这些输入进行配置，每 四位控制一路复用：    

 1） 完成复位后，所有 IO 都会连接到系统的复用功能 0（AF0）。

 2） 外设的复用功能映射到 AF1 到 AF13。

 3） Cortex-M7 EVENTOUT 映射到 AF15。    





#### 5、STM32 NVIC中断优先级管理





#### 6、HAL 库中寄存器地址名称映射分析    





### STM32CubeMX

使用 STM32CubeMX 配置工程的一般步骤为： 

1) 工程初步建立和保存

2) RCC 设置 

3) 时钟系统（时钟树） 配置 

4) GPIO 功能引脚配置 

5) 生成工程源码 

6) 编写用户代码    

#### 1、工程初步建立和保存

图中黄色的引脚主要是一些电源和 GND 引脚。 

如果某个引脚被使用，那么会显示为绿色。    

#### 2、RCC 设置



















## 实际应用



### 跑马灯实验

IO简介

STM32F7 每组通用 I/O 端口包括 4 个 32 位配置寄存器（MODER、 OTYPER、 OSPEEDR 和 PUPDR）、 2 个 32 位数据寄存器（IDR 和 ODR）、 1 个 32 位置位/复位寄存器 (BSRR)、 1 个 32 位锁定寄存器 (LCKR) 和 2 个 32 位复用功能选择寄存器（AFRH 和 AFRL）等。    

8种常用模式

**输入模式**

    -输入浮空（GPIO_Mode_IN_FLOATING）

    -输入上拉(GPIO_Mode_IPU)

    -输入下拉(GPIO_Mode_IPD)

    -模拟输入(GPIO_Mode_AIN)

**输出模式**

    -开漏输出(GPIO_Mode_Out_OD)

    -开漏复用功能(GPIO_Mode_AF_OD)

    -推挽式输出(GPIO_Mode_Out_PP)

    -推挽式复用功能(GPIO_Mode_AF_PP)



IO 配置常 用的 8 个寄存器： MODER、 OTYPER、 OSPEEDR、 PUPDR、 ODR、 IDR 、 AFRH 和 AFRL。 

 **MODER** 寄存器，该寄存器是 GPIO 端口模式控制寄存器，用于控制 GPIOx （STM32F7 最多有 9 组 IO，分别用大写字母表示，即 x=A/B/C/D/E/F/G/H/I，下同）的工作模 式，该寄存器各位描述如图所示

![1553777585002](C:\Users\ADMINI~1\AppData\Local\Temp\1553777585002.png)

---

**OTYPER** 寄存器，该寄存器用于控制 GPIOx 的输出类型，该寄存器仅用于输出模式，在输入模式（MODER[1:0]=00/11 时）下不起作用。该寄存器 低 16 位有效，每一个位控制一个 IO 口。设置为 0 是推挽输出，设置为 1 是开漏输出。 复位后， 该寄存器值均为 0，也就是在输出模式下 IO 口默认为推挽输出    

![1553777691250](C:\Users\ADMINI~1\AppData\Local\Temp\1553777691250.png)

---

**OSPEEDR** 寄存器，该寄存器用于控制 GPIOx 的输出速度，  该寄存器也仅用于输出模式，在输入模式（MODER[1:0]=00/11 时）下不起作用。该寄存 器每 2 个位控制一个 IO 口，复位后，该寄存器值一般为 0      

![1553777800712](C:\Users\ADMINI~1\AppData\Local\Temp\1553777800712.png)

---

PUPDR 寄存器，该寄存器用于控制 GPIOx 的上拉/下拉，该寄存器每 2 个位控制一个 IO 口，用于设置上下拉，这里提醒大家， STM32F1 是通过 ODR 寄存器控制上下拉的，而 STM32F7 则由单独的寄存器 PUPDR 控制上下拉，使用起来更加灵活。 复位后，该寄存器值一般为 0。    

![1553777912937](C:\Users\ADMINI~1\AppData\Local\Temp\1553777912937.png)

---

GPIO 相 关 的 函 数 和 定 义 分 布 在 HAL 库 文 件 stm32f7xx_hal_gpio.c 和 头 文 件 stm32f7xx_hal_gpio.h 文件中。 在 HAL 库中， 操作四个配置寄存器初始化 GPIO 是通过 HAL_GPIO_Init 函数完成：    

```c
void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init);
```

该函数有两个参数，第一个参数是用来指定需要初始化的 GPIO 对应的 GPIO 组，取值范 围为 GPIOA~GPIOK。第二个参数为初始化参数结构体指针，结构体类型为 GPIO_InitTypeDef。 下面我们看看这个结构体的定义。首先我们打开我们光盘的跑马灯实验，然后找到 HALLIB 组 下面的 stm32f7xx_hal_gpio.c 文件，定位到 HAL_GPIO_Init 函数体处，双击入口参数类型 GPIO_InitTypeDef 后右键选择“Go to definition of …” 可以查看结构体的定义如下：    

```c
typedef struct
{
	uint32_t Pin; //指定 IO 口
	uint32_t Mode; //模式设置
	uint32_t Pull; //上下拉设置
	uint32_t Speed; //速度设置
	uint32_t Alternate;//复用映射配置
}GPIO_InitTypeDef;
```



---

**ODR** 寄存器，该寄存器用于控制 GPIOx 的输出电平，   

![1553778212808](C:\Users\ADMINI~1\AppData\Local\Temp\1553778212808.png) 

该寄存器用于设置某个 IO 输出低电平(ODRy=0)还是高电平(ODRy=1)，该寄存器也仅在输 出模式下有效，在输入模式（MODER[1:0]=00/11 时）下不起作用。 该寄存器在 HAL 库中使用 不多，操作这个寄存器的库函数主要是 HAL_GPIO_TogglePin 函数：    该函数是通过操作 ODR 寄存器，达到取反 IO 口输出电平的功能。    

```c
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
```





