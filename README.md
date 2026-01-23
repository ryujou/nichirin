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
- [上位机脚本（PC 端）](#上位机脚本pc-端)
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

## 上位机脚本（PC 端）

PC 端入口位于 [PC_Host/Script/nichirin_pc.py](PC_Host/Script/nichirin_pc.py)。功能已拆分为多个模块，便于维护：

- [PC_Host/Script/gui_app.py](PC_Host/Script/gui_app.py)：GUI 主界面与交互逻辑
- [PC_Host/Script/audio_threads.py](PC_Host/Script/audio_threads.py)：麦克风采集、文件预分析、文件频谱线程
- [PC_Host/Script/dsp.py](PC_Host/Script/dsp.py)：频谱分箱与自适应处理
- [PC_Host/Script/serial_sender.py](PC_Host/Script/serial_sender.py)：串口发送线程
- [PC_Host/Script/pc_common.py](PC_Host/Script/pc_common.py)：协议与常量

整体功能：从麦克风或本地媒体文件提取 12 段频谱，通过 UART 实时发送到板端。支持音频/视频播放、拖拽文件、预分析锁定风格、全局 AGC，以及播放进度条。

### 依赖与环境

- Python 3.9+
- 依赖包：PyQt6、sounddevice、pyserial、numpy
- 需要安装 FFmpeg 并加入 PATH（用于文件预分析解码）

### 安装依赖

```sh
pip install PyQt6 sounddevice pyserial numpy
```

### 使用步骤

1. 连接板卡并确认串口号。
2. 运行脚本：

```sh
python PC_Host/Script/nichirin_pc.py
```

3. 在界面中选择串口与波特率，设置发送频率（默认 400Hz）。
4. 选择输入源：麦克风或文件。
5. 文件模式：打开音频/视频，等待预分析完成后点击播放。
6. 点击“开始（频谱+发送）”开始串口发送。

### 说明

- 文件模式会先整首预分析，锁定风格与每段参考曲线，避免播放中漂移。
- 仅需显示视频可保持静音，频谱由解码音频生成。
- 发送协议为 16 字节帧：地址 + 功能码 + 12 段数据 + CRC16(Modbus)。

### Web
Web version is in `PC_Host/Web/`. Open `PC_Host/Web/index.html` in a browser.
- Browser mic/file spectrum analysis
- Web Serial UART sending


## 目录结构

```
nichirin_V3/
??? .git/                    # Git metadata
??? .vscode/                 # VS Code config
??? .settings/               # IDE/tool config
??? .clangd                  # clangd config
??? .mxproject               # CubeMX project meta
??? CMakeLists.txt           # Root build script
??? CMakePresets.json        # Build presets
??? README.md                # Project doc
??? nichirin_V3.ioc          # CubeMX project file
??? STM32G030XX_FLASH.ld     # Linker script
??? startup_stm32g030xx.s    # Startup file
??? cmake/                   # Toolchain + CubeMX CMake
?   ??? stm32cubemx/          # Generated CMake fragments
??? Core/                    # Application code
?   ??? Inc/                  # Headers
?   ?   ??? drivers/
?   ?   ??? storage/
?   ??? Src/                  # Sources
?       ??? drivers/
?       ??? storage/
?       ??? utils/
??? Drivers/                 # CMSIS + HAL
?   ??? CMSIS/
?   ??? STM32G0xx_HAL_Driver/
??? PC_Host/                 # Host tools
?   ??? Script/               # PC script version
?   ?   ??? nichirin_pc.py
?   ?   ??? gui_app.py
?   ?   ??? audio_threads.py
?   ?   ??? dsp.py
?   ?   ??? serial_sender.py
?   ?   ??? pc_common.py
?   ??? Web/                  # Web version
?       ??? index.html
?       ??? style.css
?       ??? app.js
?       ??? README.md
??? build/                   # Build output
    ??? Debug/
    ??? Release/
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
