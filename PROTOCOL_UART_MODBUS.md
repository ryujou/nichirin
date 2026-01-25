# Nichirin V3 UART 协议（Modbus RTU 风格）

日期：2026-01-25

本文档定义 Nichirin V3 的 UART 协议，采用 Modbus RTU 风格帧结构（ADDR + FUNC + DATA + CRC16）。
包含配置寄存器访问与频谱寄存器写入。

## 1. 传输层

- UART：115200 8N1
- 帧结构：Modbus RTU 风格
  - [ADDR][FUNC][DATA...][CRC16_L][CRC16_H]
  - CRC16：Modbus RTU CRC16（poly=0xA001，init=0xFFFF），小端（低字节在前）

## 2. 功能码

- 0x03：读保持寄存器
- 0x06：写单个保持寄存器
- 0x10：写多个保持寄存器

说明：频谱数据通过保持寄存器写入，不再使用 0x20 频谱帧。

## 3. 频谱寄存器（Modbus 写入）

频谱数据 12 段（BAND0..BAND11）通过寄存器写入：

地址范围：
- 0x0100..0x010B：BAND0..BAND11

说明：
- 每个寄存器 16-bit，仅使用低 8 位（0..255）。
- 推荐使用 0x10 一次写 12 个寄存器：
  - START = 0x0100
  - COUNT = 12
  - BYTE_COUNT = 24
- 写入后设备更新“最近一帧频谱”，用于模式 5 显示。

## 4. 保持寄存器

### 4.1 寄存器映射（16 位寄存器）

地址    名称              范围     说明
0x0000  MODE              1..5     当前模式
0x0001  HUE               0..359   HSV 色相
0x0002  SAT               0..255   HSV 饱和度
0x0003  VAL               0..255   HSV 明度
0x0004  PARAM             0..255   当前模式参数

说明：
- PARAM（0x0004）为“当前模式参数”统一入口。
- 模式 1~4 使用 PARAM；模式 5 使用 spectrum_gain。
- 模式 5（频谱）显示频谱；HUE/SAT/VAL 写入仅保存，不影响显示。

### 4.2 读保持寄存器（FUNC = 0x03）

请求：
  [ADDR][0x03][START_H][START_L][COUNT_H][COUNT_L][CRC16]

响应：
  [ADDR][0x03][BYTE_COUNT][DATA...][CRC16]

- DATA 为 COUNT 个寄存器，每个 2 字节，按 Modbus 规范为大端。
- 上位机读取“当前模式+颜色+参数”建议读取：
  - START = 0x0000
  - COUNT = 5（MODE, HUE, SAT, VAL, PARAM）
- 模式 5：PARAM 为 spectrum_gain。

### 4.3 写单个寄存器（FUNC = 0x06）

请求：
  [ADDR][0x06][REG_H][REG_L][VAL_H][VAL_L][CRC16]

响应：
  若成功，原样回显请求帧。

### 4.4 写多个寄存器（FUNC = 0x10）

请求：
  [ADDR][0x10][START_H][START_L][COUNT_H][COUNT_L][BYTE_COUNT][DATA...][CRC16]

响应：
  [ADDR][0x10][START_H][START_L][COUNT_H][COUNT_L][CRC16]

## 5. 示例

### 5.1 设置：模式=1，H=120，S=200，V=255，flow_speed=80

写多个寄存器（0x10）：
- START = 0x0000
- COUNT = 5（MODE, HUE, SAT, VAL, PARAM）
- DATA：
  MODE = 0x0001
  HUE  = 0x0078
  SAT  = 0x00C8
  VAL  = 0x00FF
  PARAM= 0x0050

CRC 之前的帧：
  01 10 00 00 00 05 0A  00 01 00 78 00 C8 00 FF 00 50

### 5.2 写入频谱数据（12 段）

写多个寄存器（0x10）：
- START = 0x0100
- COUNT = 12
- BYTE_COUNT = 24
- DATA：BAND0..BAND11（每个 0..255，放在 16-bit 低字节）

### 5.3 读取当前模式与配置

读保持寄存器（0x03）：
- START = 0x0000
- COUNT = 5（MODE, HUE, SAT, VAL, PARAM）

响应 DATA 顺序（每项 16 位）：
MODE, HUE, SAT, VAL, PARAM

**示例（含 CRC）**

请求（读 0x0000..0x0004）：
```
01 03 00 00 00 05 85 C9
```

示例响应（MODE=1, HUE=120, SAT=200, VAL=255, PARAM=80）：
```
01 03 0A 00 01 00 78 00 C8 00 FF 00 50 00 FD
```

## 6. 模式参数含义

模式 1：PARAM = flow_speed  
模式 2：PARAM = strobe_period  
模式 3：PARAM = steady_bright  
模式 4：PARAM = breath_speed  
模式 5：PARAM = spectrum_gain

## 7. CRC16（Modbus RTU）

- 多项式：0xA001
- 初始值：0xFFFF
- 帧内小端：低字节在前

与以下实现一致：
- Core/Src/drivers/uart.c
- PC_Host/Script/pc_common.py
