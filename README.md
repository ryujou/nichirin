# Nichirin V2

<p align="center">
  <img src="assets/peacock.jpg" alt="Peacock hardware photo" width="560">
</p>

<p align="center">
  <a href="https://space.bilibili.com/3546647883680530"><img src="https://img.shields.io/badge/Author-ryujou-2B90D9?style=flat-square" alt="author"></a>
  <img src="https://img.shields.io/badge/MCU-STM32G030xx-0F6FC6?style=flat-square" alt="mcu">
  <img src="https://img.shields.io/badge/Language-C11-3F6EA5?style=flat-square" alt="language"></a>
  <img src="https://img.shields.io/badge/Build-CMake-064F8C?style=flat-square" alt="cmake">
  <img src="https://img.shields.io/badge/Status-Active-1EAE98?style=flat-square" alt="status">
</p>



> STM32G030xx 项目，基于 STM32CubeMX 生成，使用 CMake 构建，12 路 LED I2C 恒流驱动与旋转编码器交互。

## 目录

- [项目简介](#项目简介)
- [功能列表](#功能列表)
- [驱动与接口说明](#驱动与接口说明)
- [编码器使用说明](#编码器使用说明)
- [LED 驱动芯片说明](#led-驱动芯片说明)
- [环境与依赖](#环境与依赖)
- [快速开始](#快速开始)
- [模式与参数](#模式与参数)
- [目录结构](#目录结构)
- [重新生成 CubeMX 代码](#重新生成-cubemx-代码)
- [常见问题](#常见问题)

## 项目简介

Nichirin V2 是基于 STM32G030 的 12 路 LED 灯效控制工程，使用 TLC59116 I2C 恒流驱动，输入为旋转编码器 A/B + 按下键。支持多模式灯效、参数调节与 Flash 掉电记忆。

## 功能列表

- TLC59116 12 路恒流 PWM 调光 (8-bit)
- 9 种灯效模式（流水/呼吸/爆闪/常亮/全灯呼吸）
- 旋转编码器输入：短按切模式，长按进出设置
- 参数 0..255 无级调节
- 配置写入 Flash 轮转日志，减少擦写

## 驱动与接口说明

- **LED 驱动**: TLC59116 I2C，地址 A0=GND，默认 7-bit 地址 `0x60`
- **I2C 引脚**: SCL=PB3，SDA=PB7（需在 CubeMX 中确认）
- **编码器**: A=PA0，B=PA1，K=PA2，上拉输入
- **节拍**: SysTick 1ms 采样编码器，主循环 10ms 更新灯效

## 编码器使用说明

### 硬件连接

- A 相接 PA0，B 相接 PA1，按下键 K 接 PA2
- 三路均配置为上拉输入，编码器公共端接 GND

### 软件处理流程

- SysTick 1ms 调用 `Encoder_1msTick()` 做采样与去抖
- A/B 采用四相状态机译码，累计边沿到一个“步进”才输出增量
- K 键采用积分去抖，支持短按与长按事件

### 事件规则

- 短按：切换模式
- 长按：进入/退出设置
- 设置模式下旋转：调整当前模式参数（0..255）

### 调整旋转方向

若旋转方向与实际相反：
1. 交换硬件 A/B 接线，或
2. 在 `Core/Src/drivers/encoder.c` 中交换读取顺序

## LED 驱动芯片说明

### TLC59116 基本连接

- LED 阳极接 5V，TLC59116 负责下拉恒流
- TLC59116 VCC=3.3V，与 MCU 共地
- A0 接地时默认 7-bit 地址为 `0x60`

### 软件驱动方式

- 初始化时配置 MODE1/MODE2，启用自动地址递增
- LEDOUT0..2 设为 PWM 控制模式，OUT0..OUT11 有效
- 使用一次 I2C 写入 PWM0..PWM11，避免逐通道写带来的延迟

### 刷新频率

- 效果节拍 10ms，约 100Hz 刷新
- 每次生成 12 路 PWM 值，立即写入 TLC59116

## 环境与依赖

| 组件 | 说明 | 备注 |
| --- | --- | --- |
| CMake | 3.22+ | 配置生成 |
| Ninja | 构建器 | 推荐 |
| ARM GNU Toolchain | `arm-none-eabi-gcc` | 需在 PATH 中 |
| STM32CubeMX | 可选 | 重新生成代码 |

## 快速开始

### 1. 配置与编译（Debug）

```sh
cmake --preset Debug
cmake --build --preset Debug
```

### 2. Release 构建

```sh
cmake --preset Release
cmake --build --preset Release
```

### 3. 清理构建产物

```sh
cmake --build --preset Debug --target clean
```

### 4. 构建产物

- `build/Debug/nichirin_V2.elf`
- `build/Debug/nichirin_V2.map`

## 模式与参数

| 模式 | 灯效描述 | 参数含义 |
| --- | --- | --- |
| 1 | 正向单灯流水 | 流水速度 |
| 2 | 反向单灯流水 | 流水速度 |
| 3 | 对称分组流水 | 流水速度 |
| 4 | 呼吸流动（软亮环绕） | 流水速度 |
| 5 | 反向呼吸流动 | 流水速度 |
| 6 | 分组呼吸 | 流水速度 |
| 7 | 全亮/全灭爆闪 | 爆闪周期 |
| 8 | 常亮 | 亮度 |
| 9 | 全灯呼吸 | 呼吸速度 |

### 参数范围

参数范围为 0~255，数值越大通常变化越快/更亮。

说明：
- 模式 1~6 使用“流水速度”参数
- 模式 7 使用“爆闪周期”参数
- 模式 8 使用“常亮亮度”参数
- 模式 9 使用“呼吸速度”参数

按键为低电平有效（上拉输入）：
- 短按：切换灯效模式
- 长按：进入/退出设置模式
- 设置模式下旋转：调节当前模式参数

设置模式下会以全灯亮度显示当前参数值，退出设置后写入 Flash。

## 目录结构

```
nichirin_V2/
├── Core/                    # 应用代码 (main, app, drivers)
├── Drivers/                 # CMSIS 与 HAL 驱动
├── cmake/                   # 工具链与 CubeMX CMake 集成
├── assets/                  # 图片等资源
├── startup_stm32g030xx.s    # 启动文件
├── STM32G030XX_FLASH.ld     # 链接脚本
└── nichirin_V2.ioc          # CubeMX 工程文件
```

## 重新生成 CubeMX 代码

1. 使用 STM32CubeMX 打开 `nichirin_V2.ioc`
2. 重新生成代码（保持 CMake/Makefile 工程配置）
3. 按照 [快速开始](#快速开始) 重新构建

## 常见问题

**Q: 编译失败提示找不到工具链？**  
A: 确保已安装 ARM GNU Toolchain，并把 `arm-none-eabi-gcc` 加入 PATH。

**Q: I2C 引脚不对？**  
A: 在 CubeMX 中确认 I2C1 SCL=PB3、SDA=PB7，与硬件一致。

**Q: 旋转方向反了？**  
A: 交换 A/B 两相，或在 `Core/Src/drivers/encoder.c` 中调整。
