# 版本更新记录

## 目录

1. [ V1.0 ](##V1.0)
2. 

## V1.0

time: 2025/4/8

1. ​	初始版本

## V1.1

Date: 2025/4/13

C：

1. 添加了串口控制的电机测试模块，可通过电脑输入参数对电机进行测试。（完成测试）

​		数据帧格式为：p,v,a,kp,kd,\r\n
2. 将位置控制改为位置增量控制，即在初始值+增量的方式进行控制。（完成测试）
3. 添加了freertos，并添加电机驱动线程，可以控制电机驱动频率为1khz（完成测试）

测试：

1. 优宝特电机具有扭矩和位置的调节功能，当输入具有扭矩值时，其代表的意义是在目标位置下的扭矩大小。即当设置为1°，10Nm时，电机先会转到一定角度，当你用力掰回到1°时，扭矩为10Nm，并非为以10Nm的扭矩转动、
2. D3测试情况：按每1k次输出一个数据。

```
[2025-04-14 03:59:22.010]RX：1
[2025-04-14 03:59:23.010]RX：1
[2025-04-14 03:59:24.009]RX：1
```



  ## V1.2

Date：2025/4/24

C：

1. 添加了FREERTOS操作系统，完成进程的创建和进程的定时。（完成测试）
2. 添加导纳控制。在由电脑输入的相关信息后，电机可沿着设定曲线转动。（完成测试）



