# Nichirin V3 Web

浏览器版上位机（无需改动 Script）。

## 使用步骤
1. 用浏览器打开 `Web/index.html`。
2. 串口：点击“连接”，选择设备后再点“开始”。
3. 输入源：选择“麦克风”或“文件”。
   - 文件模式会先预分析，完成后再播放/开始发送。
4. 点击“开始（频谱 + 发送）”。

## 说明
- Web Serial 仅在支持的浏览器与 https/localhost 环境可用。
- 若文件预分析失败（例如部分视频容器），将自动切换为在线自适应。
- 频谱数据通过 Modbus RTU 写寄存器（0x0100..0x010B），不再使用 0x20 频谱帧。
- 频谱模式编号为 5（模式 1~4 为普通模式）。

## 协议细节（摘要）

### Modbus RTU
- 地址：0x01
- CRC16：poly=0xA001，init=0xFFFF，小端
- 功能码：0x03（读保持寄存器）、0x06（写单寄存器）、0x10（写多寄存器）

### 配置寄存器（16-bit，大端）
```
0x0000 MODE   (1..5)
0x0001 HUE    (0..359)
0x0002 SAT    (0..255)
0x0003 VAL    (0..255)
0x0004 PARAM  (模式1~4有效；模式5为 spectrum_gain)
```

### 频谱寄存器（16-bit，大端，低8位有效）
```
0x0100..0x010B  BAND0..BAND11  (0..255)
```

推荐写入频谱：
- 功能码：0x10
- START=0x0100, COUNT=12, BYTE_COUNT=24

### 模式参数含义
- 模式1：PARAM = flow_speed
- 模式2：PARAM = strobe_period
- 模式3：PARAM = steady_bright
- 模式4：PARAM = breath_speed
- 模式5：PARAM = spectrum_gain（频谱；HUE/SAT/VAL 写入仅保存，不影响显示）

完整协议详见 `PROTOCOL_UART_MODBUS.md`。
