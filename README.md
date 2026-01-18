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
- [参数分层说明](#参数分层说明)
- [配置存储系统（A/B 双槽原子提交）](#配置存储系统ab-双槽原子提交)
- [系统自检与安全模式](#系统自检与安全模式)
- [配置版本与迁移](#配置版本与迁移)
- [看门狗说明](#看门狗说明)
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

- 编码器多击：单击 / 双击 / 三击 / 长按 / 超长按
- 旋转加速度：慢速 1 步、中速 4 步、快速 16 步
- 参数分层：基础 / 高级两层
- 软阻尼：靠近边界更细腻
- 童锁 / 软锁定（超长按 5 秒锁定 / 解锁）

### 可靠性与存储

- A/B 双槽原子配置保存，断电安全
- 配置版本识别与自动迁移
- 上电自检与安全模式
- IWDG 独立看门狗健康喂狗

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
| 三击 | 切换参数层 | BASIC <-> ADV | 
| 长按 | 进入/退出设置 | 退出时保存配置 |
| 超长按（5 秒） | 锁定/解锁 | 仅在运行态触发 |

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

### 童锁（软锁定）

- 运行态下长按 5 秒进入锁定
- 锁定后旋转无效，按键事件被忽略
- 再长按 5 秒解锁
- 锁定状态会保存到配置中

## 参数分层说明

### BASIC（基础层）

- **亮度**：全局亮度（0..255）
- **速度**：当前模式速度（0..255，映射到对应效果）

### ADV（高级层）

- **Gamma Profile**：曲线类型（2.2 / 2.6 / lowboost）
- **Lowboost 强度**：仅在 lowboost 模式有效（0..255）
- **呼吸曲线**：三角波混合曲线（0..255）
- **爆闪周期**：50ms~1000ms（步进 10ms）
- **相位偏移**：用于流水/分组显示相位偏移

高级参数保存到配置中，掉电记忆。

## 配置存储系统（A/B 双槽原子提交）

新增 `cfg_store` 模块替代原日志式写入，保证断电安全。

### 槽结构（每槽一页）

- A 槽 / B 槽各占 1 页（默认 2KB）
- 每槽格式：
  - magic
  - version
  - len
  - seq
  - crc32（覆盖 payload）
  - state（ERASED / WRITING / VALID）
  - payload（Config）

### 原子提交流程

1. 选择 **非当前槽** 作为目标槽
2. 擦除目标槽
3. 写入 header（state = WRITING）
4. 写入 payload
5. 写入 crc32
6. 最后一步写入 state = VALID（单次 8 字节写入，原子完成）

旧槽直到新槽 VALID 完成才算切换成功。

### 上电加载策略

- 扫描 A/B 槽
- 仅接受：magic/version/len 正确 + state=VALID + CRC 通过
- 两槽有效时选 seq 更大的
- 都无效则加载默认并写入一次

### Flash 地址

默认使用 STM32G030F6 最后两页：

```c
#define CFG_SLOT0_ADDR 0x08007000UL
#define CFG_SLOT1_ADDR 0x08007800UL
#define CFG_PAGE_SIZE  0x800U
```

若你的芯片或链接脚本不同，请在 `Core/Src/storage/cfg_store.c` 调整。

## 系统自检与安全模式

上电自检尽量在 100ms 内完成，失败进入安全模式：

自检项：

- 配置 CRC（Cfg_Load 失败视为异常）
- I2C SDA/SCL 空闲电平检查
- TLC59116 最小 ACK 访问
- 编码器 A/B/K 粗检（全 0 持续多次则告警）

安全模式行为：

- 强制简单灯效（呼吸）且降低 I2C 更新频率
- 禁止写 Flash（避免异常状态下反复擦写）
- 仍保持喂狗与灯效刷新

退出方式：

- 三击或超长按触发重新自检

相关实现：Core/Src/boot_selftest.c

## 配置版本与迁移

配置结构新增版本信息字段：

- cfg_version
- created_by_fw
- last_migrated_by_fw

启动时逻辑：

- 读取 A/B 槽后校验
- 若为旧版本，执行逐版本迁移并补默认值
- 迁移完成后更新版本字段并保存一次
- 若版本跨度过大或不兼容，则回退默认并进入安全模式提示

固件版本常量：Core/Inc/fw_version.h

## 看门狗说明

- 使用 IWDG（LSI）约 4 秒超时
- 健康喂狗策略：只有关键任务在窗口内都正常上报才刷新
- 关键任务示例：编码器 1ms、灯效 10ms、TLC I2C 刷新

相关实现：`Core/Src/watchdog.c`

## 快速使用说明

1. 上电默认进入运行态，旋转调整当前模式参数
2. 单击切换模式；双击切换常用模式（MODE_MIN）或返回上次模式
3. 长按进入设置态，旋转修改当前设置项；再长按退出并保存
4. 三击切换参数层（BASIC <-> ADV），高级层包含 Gamma/Lowboost 等
5. 超长按（5 秒）锁定/解锁，锁定时旋转无效
6. 若进入安全模式，可三击或超长按触发重新自检

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
2. 确保 I2C1/LSI/IWDG 配置与工程一致
3. 重新生成代码（保留 USER CODE 区间）
4. 按照 [快速开始](#快速开始) 重新构建

提示：IWDG 初始化由 `WDG_Init()` 统一设置超时，CubeMX 默认值会被覆盖。

## 常见问题

**Q: 旋转方向反了？**  
A: 交换 A/B 两相，或在 `Core/Src/drivers/encoder.c` 中交换读取顺序。

**Q: 编码器按键无响应？**  
A: 确认 K 脚上拉并低电平触发，去抖时间可在 `ENCODER_DEBOUNCE_MAX` 调整。

**Q: 掉电后配置丢失？**  
A: 请确认 `CFG_SLOT0_ADDR/CFG_SLOT1_ADDR` 与链接脚本不冲突，并确实有擦写权限。
