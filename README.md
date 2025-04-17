# 项目简介

本仓库是基于BMCU的[星辰370版本](https://github.com/Xing-C/BMCU370x)修改而来，根据个人理解整理代码、添加注释，以作学习之用。

# 项目计划

在此仓库中计划完成以下事项。
 - [ ] 通读代码，检查可能存在的错误。
 - [ ] 统一化函数、变量的命名风格，增加代码自身的可读性。
 - [ ] 增加注释，详细阐述代码功能。
 - [ ] 剥离业务代码和底层驱动，方便以后向其他平台和芯片上移植。、
 - [ ] 剥离针对Arduino和Platform的环境依赖，使用自有代码实现所有业务功能，方便在其他任何环境下编译和运行。

# 分支

目前本项目仓库中的各分支和功能如下：

|分支名称|功能|
|-|-|
|main|主分支，用于托管已经经过实验的成熟代码（但不保证没有Bug）|
|develope|开发分支，用于编写和实验一些功能和特性上的修改|
|BMCU370x|用于托管和同步星辰原版BMCU370x的代码实装|

# 已知问题
以下列出固件当前存在的问题，如果您发现有其他人已经修改了这些问题，麻烦告知一下，谢谢。如果您发现了其他新的问题，也请告知一下，也许我会在将来完全了解代码架构后尝试进行修复。
 - [ ] 当前正在使用的耗材通道耗尽时，再添加新的耗材，打印机将一直处于缺料状态，无法继续打印。
 - [ ] 如果上一次打印没有正常完成，而是手动取消或因其他原因异常终止，下次开始打印时，可能无法正确检查正在使用的耗材通道，导致动作错误。
 > 打印机连接AMS/BMCU时，打印结束后会切断当前耗材并退料，这种情况下一次开始打印是没有问题的。

 # 修改内容
 以下内容已经完成修改（[*]）或正在修改（[ ]）

 - [x] RS485串口的底层驱动剥离。
 - [x] USART3的格式化日志输出，使用DMA，对系统影响做到最小。
 - [x] 使用环形缓冲区接收数据，并将数据格式分析与解包处理与串口接收中断分离。
 - [ ] 剥离基于Arduino的CRC计算模块并自行编写CRC计算模组。
 - [x] 构建完整的、支持环绕计算的32位/64位时间戳计算函数，将来以此为基础完成对64位数学运算的剥离。


# 本项目相关链接
 - BMCU项目Wiki：https://bmcu.wanzii.cn/
 - BMCU原始项目：https://gitee.com/at_4061N/BMCU。
 - BMCU370x项目WiKi：https://bmcu.wanzii.cn/doc/build/bmcu370x.html
 - BMCU370x项目源码：https://github.com/Xing-C/BMCU370x
 - BMCU硬件设计资料：https://oshwhub.com/xingcc1/bmcu-370x
 - AMCU原始版本设计资料：https://github.com/Bambu-Research-Group/Bambu-Bus

# 声明

 - **注意:本项目遵循GPL2.0开源协议。**
 - **本项目禁止商业用途。**

# 致谢

 - BMCU开发者，4061N-程序员、括号等。
 - BMCU交流群(829433420)。
 - 测试和数据提供者风雪、二月喵等。
 - 我不知道的为项目提供改进和修改建议的网友……
 
