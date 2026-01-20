#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PC 端上位机公共常量与协议工具。

此文件专门放置与 GUI/音频处理无关的“协议与常量”，便于多模块复用。
"""

from __future__ import annotations

import struct

# ---------------- UART protocol constants ----------------
# 设备地址与功能码
ADDR = 0x01
FUNC = 0x20

# 频谱段数与帧长度（ADDR + FUNC + 12*U8 + CRC16）
BANDS = 12
FRAME_LEN = 1 + 1 + BANDS + 2  # 16 bytes

# 支持的媒体扩展名
AUDIO_EXT = {".mp3", ".wav", ".flac", ".aac", ".m4a", ".ogg", ".opus"}
VIDEO_EXT = {".mp4", ".mkv", ".avi", ".mov", ".webm", ".wmv"}


def crc16_modbus(data: bytes) -> int:
    """
    计算 Modbus RTU CRC16（poly=0xA001, init=0xFFFF）。

    Args:
        data: 需要计算 CRC 的字节序列

    Returns:
        16 位 CRC 值（小端拼接时需低字节在前）
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(bands_u8: list[int]) -> bytes:
    """
    构建 UART 数据帧。

    帧格式：
    [ADDR=0x01][FUNC=0x20][12*U8 bands][CRC16_L][CRC16_H]

    Args:
        bands_u8: 12 段频谱强度（0..255）

    Returns:
        完整帧 bytes
    """
    if len(bands_u8) != BANDS:
        bands_u8 = (bands_u8 + [0] * BANDS)[:BANDS]
    payload = bytes([ADDR, FUNC] + [int(b) & 0xFF for b in bands_u8])
    crc = crc16_modbus(payload)
    return payload + struct.pack("<H", crc)
