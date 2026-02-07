~~温馨提示，cubemx重新生成之后的.ld文件有问题， 待解决，~~

# 此为全向轮步兵达秒开发板程序

## 概述

- English doc [readme](.doc/en/readme.md)
- 以步兵为例进行讲解，如果你在其他兵种的代码里面发现了此readme，也可以参考

## 软件环境

- Toolchain:arm-none-eabi
- package version:STM32Cube STM32H7 Series 1.12.1
- STM32CubeMX version: 6.13.0

> 没用过gcc?请看这篇文章[VSCode+Ozone使用方法](.doc/ch/VSCode+Ozone使用方法.md)

## 快速开始

---

### 硬件接口

### 文件说明

25-EE-code/
├── Algorithm/          # 算法模块（包含AHRS、滤波、PID等核心算法）
├── build/              # 编译输出目录（包含.o/.d文件及最终固件）
├── Core/               # 核心系统文件（启动文件、链接脚本等）
├── debug/              # 调试相关配置
├── Drivers/            # 硬件驱动层（HAL库、外设驱动）
├── MDK-ARM/            # Keil工程文件
├── Middlewares/        # 中间件（FreeRTOS、USB协议栈等）
├── USB_DEVICE/         # USB设备配置
└── User/               # 用户代码（底盘/云台/射击等业务逻辑）
**其中较为重要的为User文件夹，功能实现的代码均位于此**

### 代码结构
