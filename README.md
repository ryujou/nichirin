# Nichirin V3

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

> STM32G0 + WS2812 项目，基于 STM32CubeMX 生成，使用 CMake 构建。12 组灯效 + 旋转编码器 UI + UART 频谱输入 + Flash 掉电记忆。

## 目录

- [项目简介](#项目简介)
- [功能列表](#功能列表)
- [硬件与接口说明](#硬件与接口说明)
- [编码器使用说明](#编码器使用说明)
- [WS2812 刷新机制](#ws2812-刷新机制)
- [模式与参数](#模式与参数)
- [掉电记忆说明](#掉电记忆说明)
- [环境与依赖](#环境与依赖)
- [快速开始](#快速开始)
- [目录结构](#目录结构)
- [重新生成 CubeMX 代码](#重新生成-cubemx-代码)
- [常见问题](#常见问题)

## 项目简介

Nichirin V3 是基于 STM32G030 的 WS2812 灯效控制工程，12 组灯珠（每组 5 颗），支持多模式灯效、参数调节、UART 频谱输入，以及 Flash 掉电记忆。控制方式为旋转编码器 + 按键，UI 提供颜色与模式参数两个页面。

## 功能列表

- WS2812（TIM1 + DMA）全帧刷新，5ms 帧周期（约 200Hz 调度）
- 12 组灯效同步（模式与参数全组一致）
- 旋转编码器 UI（短按切模式、长按进出设置、双击切条目）
- 颜色参数 HSV（hue/sat/val）与模式参数实时调整
- UART 频谱输入（音频频谱模式）
- Flash 日志式双页存储，掉电记忆 + 版本/CRC 校验

## 硬件与接口说明

- **MCU**: STM32G030xx
- **WS2812**: TIM1 + DMA 驱动
- **编码器**: A/B + 按键（上拉输入）
- **UART**: 频谱数据输入

> 引脚定义以 CubeMX 生成的 `Core/Inc/main.h` 为准。

## 编码器使用说明

### 事件规则

- **短按（CLICK）**：切换模式（1 → 2 → 3 → 4 → 6 循环）
- **长按（LONGPRESS）**：进入/退出设置
- **双击（DOUBLE_CLICK）**：在设置页面切换条目
- **旋转（ROTATE）**：调整当前条目参数

### UI 页面

- **Color 页**：Hue / Sat / Val
- **ModeParam 页**：根据当前模式切换参数（速度/周期/亮度/增益等）

## WS2812 刷新机制

- 主循环以 1ms tick 驱动调度
- 每 5ms 标记一帧
- 仅在 DMA 空闲时发送新帧
- 若 DMA 正忙，帧会 pending，DMA 完成后立即补发

## 模式与参数

| 模式 | 名称 | 描述 | 参数 |
| --- | --- | --- | --- |
| 1 | 流水 | 组间流水点亮 | flow_speed |
| 2 | 爆闪 | 周期闪烁 | strobe_period |
| 3 | 常亮 | 固定亮度 | steady_bright |
| 4 | 呼吸 | 呼吸亮度变化 | breath_speed |
| 6 | 频谱 | 音频频谱响应 | spectrum_gain |

### 参数范围

- **flow_speed / strobe_period / steady_bright / breath_speed / spectrum_gain**：0~255
- **Hue**：0~359
- **Sat / Val**：0~255

说明：
- 速度/周期类参数数值越大，变化越慢
- 亮度类参数数值越大，越亮

## 掉电记忆说明

工程使用日志式双页 Flash 存储（2KB/page）保存配置，包含：

- 当前模式（1/2/3/4/6）
- HSV 颜色参数（hue/sat/val）
- 各模式公共参数（flow_speed / strobe_period / steady_bright / breath_speed / spectrum_gain）
- 格式标记 + 版本号 + CRC32

### 保存策略

- 参数或模式变化后标记 dirty
- 500ms 内无变化自动保存
- 退出设置（长按）时强制保存一次
- 保存在主循环执行，DMA 忙时会延后

### 断电安全

- 使用 commit 标记 + CRC 校验
- 断电时只会读到旧记录或新记录，不会读到半写记录

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

- `build/Debug/nichirin_V3.elf`
- `build/Debug/nichirin_V3.map`

## 目录结构

```
nichirin_V3/
├── Core/                    # 应用代码 (main, app, drivers, storage)
├── Drivers/                 # CMSIS 与 HAL 驱动
├── cmake/                   # 工具链与 CubeMX CMake 集成
├── assets/                  # 图片等资源
├── startup_stm32g030xx.s    # 启动文件
├── STM32G030XX_FLASH.ld     # 链接脚本
└── nichirin_V3.ioc          # CubeMX 工程文件
```

## 重新生成 CubeMX 代码

1. 使用 STM32CubeMX 打开 `nichirin_V3.ioc`
2. 重新生成代码（保持 CMake/Makefile 工程配置）
3. 按照 [快速开始](#快速开始) 重新构建

## 常见问题

**Q: 编译失败提示找不到工具链？**  
A: 确保已安装 ARM GNU Toolchain，并把 `arm-none-eabi-gcc` 加入 PATH。

**Q: 旋转方向反了？**  
A: 交换 A/B 两相，或在 `Core/Src/drivers/encoder.c` 中调整读取顺序。

**Q: 上电后参数未恢复？**  
A: 确保已触发保存（旋转后 500ms 或退出设置），并检查 UART/WS2812 刷新是否阻塞主循环。
