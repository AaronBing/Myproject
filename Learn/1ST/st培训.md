## 1、GUI Advanced 

#### 1.1简介

选型问题什么屏幕，接口，分辨率是多少

##### 图形加速

MJPEG图片硬解码、Chorm-ART\

##### 接口

​	8080/8800 IF

​	LCD TFT

​	MIPI-DSI (4线)

##### 分辨率

​	CGA/QVGA



#### 1.2加速器的优点

Chrom-ART

Chrom-grc

​	硬件加速，省20%的RAM

HW JPEG

​	图片叠加，动态说明（打印机上的说明）

RAM体积大，不用那个外部增加



#### 1.3软件

前期推介客户用Cubemx，StemWin

后面优化使用高端TouchGFX,支持C++，只通过界面，不用编程来

​	有各种医疗商用的标准图库

​	有字库

​	动画和流程要求比较高的也有代码编译功能



对资源的要求RAM 11-35k，Flash 21-221k



#### 1.4Demo 版







## 2、STM32G0

64MHZ

93%GPIO,  	48PIN 一下只有一路电源

ram/flash比例	1：4，36kram：128k，64kflash

flash16k-128k

最小到8pin-100多pin 

加密特性

稳定性更高，

电压1.7-3.6



#### 成本上

节省外部晶振

去耦电容

pcb更小，节省io

支持typeC 



#### 更低的功耗

运行功耗更低   小于100uA



#### 效率更高

adc 12bit   2.5mhz

更大的ram

定时器高达128mhz

温度有-40-125度



typeC pd软件

  Cube MX uspd/ucpd



#### 目前版本

只有07，08系列

g031，可以申请样品（工程样片）





evt

​	4.5kv 打mcu 0 失效率 

​	8kv	打整版  0 失效率

emi

​	level降到2

ESD没有改进





#### G0和f0选择

​	g0没有usb，

​	不支持触屏

​	pin脚不会兼容，只有一路电路



G0和GDE230对比

​	ram更大，flash更大，便于后期升级





下午

G0参数

1、三条产品线



2、区别

GPIO口



内核

​	效率和安全性更高



36kSRAM

​	使能eccjiaoyan



SYSCOF

​	

​	使用boost升压器

RCC时钟

​	nrst可以作为gpio

​	复位保存器

​	PLL3个输出

​	

复位保持器 

​	复位时候 一直把电压往下拉，知道0.3VDD为止，再释放



时钟

​	加了个LSCO,在  stop0，stop



低功耗

​	底到33nA，可IO口唤醒

​	870nA，保留ram36kb

​	运行98.3ua









## 4、GFX

触摸手势，交互比较号，快速响应

​	GUI需求;	字体，触摸效果，2d3d，动画，场景切换，分辨率，刷新率

touchGFX

​	只能用于stm32，比较高级的gui界面

​	尽量少占用cup，做出更好的效果，





























































