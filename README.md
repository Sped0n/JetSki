# JetSki
Jetson nano智能小车

## 简介

本项目是基于学校内机器人比赛的项目，要求是利用小车实现多个简单功能的结合。

我本人也是第一次接触这方面相关的知识，很多代码写的也很生涩，很感谢网上各路大佬的代码和帮助，希望可以帮助到一些希望利用树莓派或者Jetson nano实现一些简单项目的同学。

目前实现的功能有：

* 自动避障：基于红外传感器和超声波传感器；
* 视觉循迹：基于`openCV`，使小车可沿单黑线行驶；
* 识别红绿灯以及物块：基于`openCV`形状识别和HSV颜色识别，使小车可以识别比赛中要求的红绿灯和黄色物块；
* 自动绕障：可在循线时自动避障，基于双红外传感器。

计划实现且之后会尝试实现的功能：

* 实时目标检测：基于自己训练的`yoloV4-tiny`权重，实现实时目标识别；
* 跨板通讯：与Arduino/STM32进行跨板通讯，Jetson只负责图像处理部分，以提高系统反应速度。

购买的是[慧静电子](http://www.hlmcu.com/)的小车，商家提供了一些基于c语言基本教程，资料蛮多的，不过后面代码都是Python就没怎么用到，硬件的话还算齐全，红外线超声那些，这个价位小车该有的基本都有。

原来计划使用树莓派3B来做完这台小车的，但后面搞完视觉代码发现树莓派3B的性能跑`openCV`还是有点吃力，尤其对于高速小车而言，识别速度至关重要，就中途转用Jetson nano了，所幸Jetson的GPIO和树莓派的**基本一致**，迁移工作没有多花很多精力，但还是有些意想不到的坑，这个后面会说。

本来是计划用C的，但无奈Python真的太香了，也方便后面做目标识别，再加上C也忘得差不多了，再看和重学差不多，那索性直接梭哈Python。

接下来会对小车硬件，功能实现和原理进行逐一介绍。本文的结构如下：

* 硬件平台
* 准备工作
* 硬件调试
* 原理介绍与功能实现

若想成功实现本项目的功能，请：

* 首先确保完成**准备工作**
* 之后进行**硬件调试**
* 之后阅读过**原理介绍**的基础上
* 根据**功能实现**的代码来运行相应的程序、实现功能

## 硬件平台

* Jetson nano（2GB）
* 驱动板（L298N）
* USB摄像头**x2**
* 超声传感器**x1**
* 红外避障传感器**x2**
* 小车车体 + tt电机**x4**
* PCA9285 *PWM拓展版* **x1**
* 杜邦线**若干**

## 准备工作

* 给Jetson nano刷上系统
* 初始化Jetson nano
* 可选搭建环境（[参考链接](https://blog.sped0nwen.com/post/jestsonopencv/)）
  * 安装Qt5
  * 自行编译带CUDA加速的openCV

## 硬件调试

首先要确定Jetson、驱动板、传感器之间的连线是正确的。

### Jetson GPIO

![image-31](https://img.sped0nwen.com/2022/01/12/743a6bb2ee11e.png)

在Jetson的GPIO中，有四种模式，它们分别是BOARD、BCM、CVM和TEGRA_SOC，接下来我就来简单介绍一下他们之间的不同。

##### BOARD和BCM

他们都来源于RPi.GPIO library，因此Jetson Nano的标识对照和树莓派一致，可参考树莓派对照表。物理管脚BOARD编码和Jetson Nano上的管脚是正好**一一对应**的，是物理数值关系，手册中写为GPIO Header。但BCM中就不是物理数值关系对应了，**BCM是根据博通SoC的GPIO规则命名的**。BOARD编码中的12号管脚，在BCM 中的编码就是18号管脚，他们的功能都是GPIO.1(*通用输入输出管脚1*)。**在Jetson Nano正面写的是Board编码，反面写的是相应的BCM编码**，可以留心查看。

##### CVM和TEGRA_SOC

CVM和TEGRA_SOC均**使用字符串（strings ）而不是数字**作为各个GPIO管脚的命名。其中CVM是根据 CVM/CVB connector中信号名称命名，TEGRA_SOC是根据Tegra SoC中信号名称来命名的。其中TEGRA_SOC的使用可以看看[这篇帖子](https://stackoverflow.com/questions/61039191/how-to-setup-gpio-pins-in-gpio-tegra-soc-mode-vs-gpio-bcm-mode-using-jetson-nano)。

##### 和树莓派GPIO的不同

很不幸，Jetson的GPIO**只有两个管脚支持GPIO**，根本不够用，如果是PWM重度需求者，需要外接一个**I2C转PWM**信号的板子，也就是我们前面提到的PCA9285。

> 在使用PCA9685时，还要注意**GPIO会被设置成TEGRA_SOC模式**，而不是树莓派上常用的BCM，使用GPIO时要注意.

### 电机

经典黄色tt电机，利用PWM调速，频率为800Hz，每侧车轮的前进和后退由L298N的H桥电路控制，转向通过左右轮差速实现。

### 超声传感器

##### 基本指标

性能：

- 探测距离： 2cm-400cm
- 没有温度修正功能
- 感应角度：不大于15度
- 高精度：可达0.3cm

接口定义：

- VCC, TRIG(触发端), ECHO(接收端), GND

##### 基本原理

控制口发一个10us以上的高电平，就可以在接收口等待高电平输出。一有输出就可以开定时器计时，当此口变为低电平时就可以读定时器的值，此时就为此次测距的时间，方可根据声速算出距离。

1. 采用IO口TRIG触发测距，给至少10us的高电平信号;
2. 模块自动发送8个40khz的方波，自动检测是否有信号返回；
3. 有信号返回，通过IO口ECHO输出一个高电平，**高电平持续的时间就是超声波从发射到返回的时间**。测试距离=高电平时间*声速/2。

##### 优化

1. 对测距值进行移动平均

```python
dist_current = self.DistMeasure()
dist_mov_ave = (0.2 * dist_current) + (0.8 * dist_mov_ave)
```

2. 防止传感器错过回波而停止运行

```python
while GPIO.input(self.GPIO_ECHO) == 0:
    pass
start_time = time.time()
```

在上述程序中，如果sensor错过了回波，那么ECHO永远也不会变为高电平，程序就会一直陷在while循环里，无法继续运行。所以需要对ECHO上升沿检测程序进行修正：

```python
ii = 0
while GPIO.input(self.GPIO_ECHO) == 0:
    ii = ii + 1
    if ii > 10000: 
        print('Ultrasound error: the sensor missed the echo')
        return 0
    pass
start_time = time.time()
```

经实验，在正常测距的情况下，`ii`不可能需要累加到10000。所以当`ii>10000`时，可以认为传感器已经错过了回波，则`break`，本次测距无效。同时，如果本次测距无效，则`dist_mov_ave`与上一时刻的`dist_mov_ave`保持一致。

### 红外避障传感器

红外避障传感器传回高电平表示前方有障碍物，传回低电平表示前方无障碍物。探测距离的阈值需要自己使用工具进行调节，函数编写较为简单，这里不再赘述。

### USB摄像头

可以直接在openCV中调用，但性能不如CSI摄像头。

### PCA9685（I2C转PWM）

##### 硬件接线

* SCL — 接5号管脚(*GEN2_I2C_SCL*)

* SDA — 接3号管脚(*GEN2_I2C_SDA*)

* GND接GND

* VCC随便找个5V管脚接上就行

  > PCA9685还需要外接供电，在拓展底板上找到一个VCC(5V)和GND，VCC接PCA9685上SCL、SDA那一排的V+，负极接GND，直接接到控制电机那一排的GND上就行。

##### 模块安装

```shell
sudo apt-get install i2c-tools
sudo pip3 install adafruit-blinka
sudo apt-get install python-smbus
sudo pip3 install adafruit-circuitpython-motor
sudo pip3 install adafruit-circuitpython-lis3dh
sudo pip3 install adafruit-circuitpython-PCA9685
sudo pip3 install adafruit-circuitpython-motorkit
```

安装完并接好线以后在终端输入

```shell
sudo i2cdetect -y -r -a 1
```

应该可以发现和未接线时有区别，有区别则为成功。

![33220b98344b4da67e558473e5e34edf](https://img.sped0nwen.com/2022/01/15/6a42f2c347d76.png)

##### 模块使用

先给出[官方文档](https://circuitpython.readthedocs.io/projects/motor/en/latest/index.html)

这里有一份测试代码，看一遍大概就可以知道这些模块是怎么使用的了。

```python
import time

from board import SCL, SDA
import busio

# 导入PCA9685模块
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

i2c = busio.I2C(SCL, SDA)

# 为电机的默认地址创建一个简单的pca类
pca = PCA9685(i2c, address=0x60)
pca.frequency = 100

# 马达1是管脚9高电平代表前进，管脚10高电平代表后退，其中管脚8保持高电平。
# 马达2是管脚11高电平代表前进，管脚12高电平代表后退，其中管脚13保持高电平。
# 马达3是管脚3高电平代表前进，管脚4高电平代表后退，其中管脚2保持高电平。
# 马达4是管脚5高电平代表前进，管脚6高电平代表后退，其中管脚7保持高电平。
# 管脚计数从0开始，类似数组，其并非从1开始。
# 这些马达使用的管脚也可以按照你自己的需要自由更改，详见https://circuitpython.readthedocs.io/projects/motor/en/latest/api.html#

# 直流电动机在运行时产生不稳定的电流噪声，在极端情况下会重置微处理器，不过可以用通过添加电容来防止这种情况的出现。这个演示中使用了上述定义的4号电机，因为它在没有电容的情况下也能正常工作。
pca.channels[7].duty_cycle = 0xFFFF
motor4 = motor.DCMotor(pca.channels[5], pca.channels[6])
motor4.decay_mode = (motor.SLOW_DECAY)  
# 将电机设置成主动刹车模式（速度缓慢衰减）来提高马达的性能。

print("缓慢向前")
motor4.throttle = 0.5 # 这个throttle油门值可以介于-1到1之间，0就是停
print("throttle:", motor4.throttle)
time.sleep(1)

print("向前")
motor4.throttle = 1
print("throttle:", motor4.throttle)
time.sleep(1)

print("向后")
motor4.throttle = -1
print("throttle:", motor4.throttle)
time.sleep(1)

print("缓慢向后")
motor4.throttle = -0.5
print("throttle:", motor4.throttle)
time.sleep(1)

print("停车")
motor4.throttle = 0
print("throttle:", motor4.throttle)
time.sleep(1)

print("原地旋转")
motor4.throttle = None
print("throttle:", motor4.throttle)

pca.deinit() # 清除各管脚上的电平信号
```

## 原理介绍与功能实现

### 纯自动避障

主要基于超声波与红外，超声波负责检测前方障碍物，两侧的红外传感器则负责两侧的障碍物检测，经过验证在一些棱角较少的场地避障效果良好，因为超声波的散射性太强，所以比赛时只使用了红外传感器。

![IMG_2728](https://img.sped0nwen.com/2022/01/15/0d34846519d4c.GIF)

<center><font color=gray>直角拐弯+避障</font></center>

### 视觉单黑线车道循迹

基于openCV，使小车沿车道线行驶。环境要求为白色的地板，黑色的单车道线。

> 也可以按自己的需求魔改，只要保证在灰度图下对比度足够就行，白车道线这种也是可以的。

![IMG_2727](https://img.sped0nwen.com/2022/01/15/b6e8ff088c106.GIF)

<center><font color=gray>曲线行驶</font></center>

其识别的流程大概如下：

* 车道线检测
  * 对图像进行**高斯模糊处理**，减少噪声；
  * 将图像转化为灰度图；
  * **大津法**二值化；
  * 通过**白色区域膨胀让黑色区域缩小**或者让**腐蚀白色区域让黑色区域增大**来进一步减少噪声信号；
  * 选取一行像素作为进行循迹，相当于**一行里每一个像素都是一个红外循迹传感器**；
  * 找到**黑色像素的个数和位置索引**，计算出黑色像素目前的中心点位置；
  * 通过将黑色像素的中心点与小车自身的中心点作比较计算出**小车的偏移量**，得出运动决策。

* 运动决策
  * 如果能检测到黑色像素
    * 偏移量在设置好**较小的死区**内时，为保证速度直接直行，不做微调，最大**双电机**满输出；
    * 偏移量在微调区内时，执行微调，此时最大**单侧**电机满输出；
    * 偏移量较大时，提高差速比，此时**单侧电机85%输出**。
  * 如果不能检测到黑色像素 / 黑色像素过多
    * 低速向后移动一小段距离，相当于我们平时的“撤回”。

### 红绿蓝灯识别

基于openCV，使小车可以识别**圆形**的LED灯并辨别颜色。

其识别的流程大概如下：

* 圆检测

  * 对图像进行高斯模糊；
  * 调用openCV自带的霍夫圆函数识别图像中的**特定大小范围**内的**每个圆**；
  * 记录下每个圆的位置和半径。

* 颜色检测

  * 利用**HSV**创建红绿蓝三色的颜色遮罩；
  * 给记录好的每个圆利用圆心和半径框个**正方形的区域**；
  * 检查这些区域在三种颜色遮罩中**有效像素占比**；
  * 选出占比最大的颜色，如果其占比同时大于设定阈值，则记录为检测到一次信号灯，准备校验；
  * 如果在**一秒内连续检测到五次信号灯**，校验通过，确认为信号灯。

  ![IMG_6034](https://img.sped0nwen.com/2022/01/15/0d21cc6d682ac.JPG)
  
  <center><font color=gray>视觉代码效果</font></center>
  
  ![IMG_2729](https://img.sped0nwen.com/2022/01/15/1ae084b4e0c75.GIF)
  
  <center><font color=gray>蓝灯识别</font></center>
  
  ![IMG_2730](https://img.sped0nwen.com/2022/01/15/5df95db8f4a36.GIF)
  
  <center><font color=gray>红绿灯识别</font></center>
  
  > 对于LED的颜色识别，有两种方法，两种方法都需在特定硬件条件下才能达成较高的精度。
  >
  > 1. 需要摄像头可以锁定白平衡和曝光
  >
  >    这种一般为**一摄像头多用**的情况，需要保证摄像头在场地里有一个正常的曝光值，此时在摄像头里LED灯必然是过曝的，不能通过检测圆自身的颜色，因为在摄像头中LED的**中心是一片白的**，只能靠**检测LED周围的光环颜色**，如果需要用到上述提到的算法，需要将阈值保持在**大小和范围**都较小的一个区间内，如果摄像头无法锁定白平衡和曝光颜色就很容易飘，导致识别率不高。
  >
  > 2. 需要相机可以将曝光锁定在一个较低的水平
  >
  >    这种为**专用摄像头**的情况，用起来比较方便，在**低曝光**的情况下，摄像头里就只有信号灯，信号灯的颜色区别也肉眼可见，缺点就是这个摄像头就**只能用来识别LED灯**，比较浪费。

### 黄色方块识别

基于openCV，使小车可以识别终点停车的黄色方块。

![IMG_2731](https://img.sped0nwen.com/2022/01/15/1822464e518fa.GIF)

<center><font color=gray>识别黄色方块并停车</font></center>

其识别的流程大概如下：

* 颜色检测
  * 对图像进行**高斯模糊**处理；
  * 将**BGR**转换为**HSV**；
  * 对图像进行腐蚀，减少噪声；
  * 利用遮罩将黄色其他部分去除，化为**二值化图像**。
* 形状检测
  * 利用openCV自带函数(*cv2.findcontours*)找出**黄色区域的边界**；
  * 找出图像中的矩形，并得到矩形四点的坐标；
  * 利用坐标计算得出矩形的长与宽，以此作为依据判断矩形是否接近正方形，如果近似则开始校验；
  * 如果**一秒内连续检测到五次正方形**，校验通过，确认为黄色方块。

### 比赛代码

[比赛的赛例文件](https://1drv.ms/b/s!AmO_vhOIGsOXgY4kLpIRLms4nS1n7g)

> 如果文件链接失效可以在github的本项目中找到。

比赛的代码就是将上述的功能进行结合，使用到了python的多线程功能，不过性能比较一般，所以之后会将电控代码迁移到arduino上，两板间利用UART通信。

