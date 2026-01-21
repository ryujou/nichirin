# Nichirin V2

<p align="center">
  <img src="assets/peacock.jpg" alt="Peacock hardware photo" width="560">
</p>

<p align="center">
  <a href="https://space.bilibili.com/3546647883680530"><img src="https://img.shields.io/badge/Author-ryujou-2B90D9?style=flat-square" alt="author"></a>
  <img src="https://img.shields.io/badge/MCU-STM32G030xx-0F6FC6?style=flat-square" alt="mcu">
  <img src="https://img.shields.io/badge/Language-C11-3F6EA5?style=flat-square" alt="language">
  <img src="https://img.shields.io/badge/Build-CMake-064F8C?style=flat-square" alt="cmake">
  <img src="https://img.shields.io/badge/Status-Active-1EAE98?style=flat-square" alt="status">
</p>

> STM32G030xx 项目，基于 STM32CubeMX 生成，使用 CMake 构建，12 路 LED I2C 恒流驱动与旋转编码器交互。

## 目录

- [项目简介](#项目简介)
- [主要特性](#主要特性)
- [新功能详细说明](#新功能详细说明)
- [硬件与引脚](#硬件与引脚)
- [系统节拍与架构](#系统节拍与架构)
- [交互说明（编码器）](#交互说明编码器)
- [快速使用说明](#快速使用说明)
- [快速开始](#快速开始)
- [使用 AI](#使用-ai)
- [目录结构](#目录结构)
- [重新生成 CubeMX 代码](#重新生成-cubemx-代码)
- [常见问题](#常见问题)

## 项目简介

Nichirin V2 是基于 STM32G030 的 12 路 LED 灯效控制工程，使用 TLC59116 I2C 恒流驱动，输入为旋转编码器 A/B + 按下键。工程采用 SysTick 1ms/10ms 驱动状态机，无 RTOS。支持多模式灯效、参数调节、掉电记忆与独立看门狗保护。

## 主要特性

### 灯效与亮度

- TLC59116 12 路恒流 PWM 调光（8-bit）
- 9 种灯效模式（流水/呼吸/爆闪/常亮/全灯呼吸）
- 亮度映射层（gamma 2.2 / 2.6 / lowboost）

### 交互与手感

- 编码器多击：单击 / 长按
- 旋转加速度：慢速 1 步、中速 4 步、快速 16 步
- 软阻尼：靠近边界更细腻

### 工程结构

- SysTick 1ms/10ms 驱动状态机，无 RTOS
- CMake 构建，CubeMX 生成代码

## 新功能详细说明

### 亮度映射层（Perceptual Consistency）

在写入 TLC PWM 之前统一做亮度映射：

- 输入：逻辑亮度 0..255
- 输出：实际 PWM 0..255
- 目标：低亮更可分辨、整体更线性

支持的曲线类型：

- Gamma 2.2：常规感知曲线
- Gamma 2.6：低亮更暗、对比更强
- 低亮增强（lowboost）：前 1..20 档更敏感

配置项：

- gamma_profile：0=2.2，1=2.6，2=lowboost
- adv_gamma：在 lowboost 模式下作为强度（0..255）

实现位置：Core/Src/bright_map.c

### 参数软阻尼（Soft Damping）

为改善旋钮手感，在接近边界时降低变化速度：

- 靠近 0 或 255 时需要更多 detent 才能变化 1
- 先应用加速度，再进入阻尼
- 可对不同参数设置不同阻尼强度

实现位置：Core/Src/ui_damping.c
关键宏：DAMP_ZONE_* / DAMP_GAIN_*

## 硬件与引脚

- **MCU**: STM32G030F6Px
- **I2C1**: SCL=PB3，SDA=PB7（与 TLC59116 相连）
- **编码器**: A=PA0，B=PA1，K=PA2，上拉输入
- **LED 驱动**: TLC59116，A0=GND，7-bit 地址 `0x60`

## 系统节拍与架构

- **1ms Tick**：`HAL_SYSTICK_Callback()` 内部调用 `Encoder_1msTick()`
- **10ms Tick**：主循环中每 10ms 调用 `Effect_Tick()`
- **无 RTOS**：所有逻辑由 SysTick 驱动的状态机完成
- **主循环简洁**：仅做初始化和周期调用

## 交互说明（编码器）

### 按键事件

| 事件 | 操作 | 说明 |
| --- | --- | --- |
| 单击 | 切换模式 / 切换设置项 | 运行态切模式，设置态切参数项 |
| 双击 | 快捷功能 1 | 运行态切到常用模式（MODE_MIN）/返回上次模式 |
| 长按 | 进入/退出设置 | 退出时保存配置 |

### 旋转加速度

- 慢速：每 detent ±1
- 中速：每 detent ±4（默认阈值 120ms）
- 快速：每 detent ±16（默认阈值 60ms）

调整阈值与倍率：`Core/Src/drivers/encoder.c`

```c
#define ENCODER_ACCEL_MID_MS 120U
#define ENCODER_ACCEL_FAST_MS 60U
#define ENCODER_ACCEL_MID_STEP 4
#define ENCODER_ACCEL_FAST_STEP 16
```

## 快速使用说明

1. 上电默认进入运行态
2. 单击切换模式
3. 长按进入设置态，旋转修改当前模式参数；再长按退出

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

设置模式下会以全灯亮度显示当前参数值

## 快速开始

### Debug 构建

```sh
cmake --preset Debug
cmake --build --preset Debug
```

### Release 构建

```sh
cmake --preset Release
cmake --build --preset Release
```

### 清理构建产物

```sh
cmake --build --preset Debug --target clean
```

### 构建产物

- `build/Debug/nichirin_V2.elf`
- `build/Debug/nichirin_V2.map`

## AI 辅助

本项目的部分迭代使用了 AI 辅助（Codex）进行代码重构与文档整理。AI 仅作为辅助工具，最终实现与行为以源码为准。若需复现改动，可在本仓库基础上提供同样的需求描述与约束，并结合实际硬件验证。

## 目录结构

```
nichirin_V2/
|-- Core/
|   |-- Inc/
|   |   |-- drivers/
|   |   |-- storage/
|   |   |   |-- flash_cfg.h
|   |   |   `-- cfg_store.h
|   |   `-- app/
|   `-- Src/
|       |-- app/
|       |-- drivers/
|       |-- storage/
|       |   |-- flash_cfg.c
|       |   `-- cfg_store.c
|       `-- main.c
|-- Drivers/
|-- cmake/
|-- assets/
|-- startup_stm32g030xx.s
|-- STM32G030XX_FLASH.ld
`-- nichirin_V2.ioc
```
## 重新生成 CubeMX 代码

1. 使用 STM32CubeMX 打开 `nichirin_V2.ioc`
2. 确保 I2C1 配置与工程一致
3. 重新生成代码（保留 USER CODE 区间）
4. 按照 [快速开始](#快速开始) 重新构建


## 常见问题

**Q: 旋转方向反了？**  
A: 交换 A/B 两相，或在 `Core/Src/drivers/encoder.c` 中交换读取顺序。

**Q: 编码器按键无响应？**  
A: 确认 K 脚上拉并低电平触发，去抖时间可在 `ENCODER_DEBOUNCE_MAX` 调整。

**Q: 掉电后配置丢失？**  
A: 请确认 `CFG_SLOT0_ADDR/CFG_SLOT1_ADDR` 与链接脚本不冲突，并确实有擦写权限。
