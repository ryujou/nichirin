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
FUNC_READ_HOLDING = 0x03
FUNC_WRITE_SINGLE = 0x06
FUNC_WRITE_MULTI = 0x10

# 频谱段数
BANDS = 12

# Modbus 寄存器映射
REG_MODE = 0x0000
REG_HUE = 0x0001
REG_SAT = 0x0002
REG_VAL = 0x0003
REG_PARAM = 0x0004
REG_BAND0 = 0x0100
REG_BAND_COUNT = 12

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


def build_read_holding(start: int, count: int, addr: int = ADDR) -> bytes:
    """构建 0x03 读保持寄存器请求帧。"""
    payload = struct.pack(">BBHH", addr & 0xFF, FUNC_READ_HOLDING, start & 0xFFFF, count & 0xFFFF)
    crc = crc16_modbus(payload)
    return payload + struct.pack("<H", crc)


def build_write_single(reg: int, value: int, addr: int = ADDR) -> bytes:
    """构建 0x06 写单寄存器请求帧。"""
    payload = struct.pack(">BBHH", addr & 0xFF, FUNC_WRITE_SINGLE, reg & 0xFFFF, value & 0xFFFF)
    crc = crc16_modbus(payload)
    return payload + struct.pack("<H", crc)


def build_write_multi(start: int, values: list[int], addr: int = ADDR) -> bytes:
    """构建 0x10 写多个寄存器请求帧。"""
    count = len(values)
    byte_count = count * 2
    header = struct.pack(
        ">BBHHB",
        addr & 0xFF,
        FUNC_WRITE_MULTI,
        start & 0xFFFF,
        count & 0xFFFF,
        byte_count & 0xFF,
    )
    data = b"".join(struct.pack(">H", v & 0xFFFF) for v in values)
    payload = header + data
    crc = crc16_modbus(payload)
    return payload + struct.pack("<H", crc)


def build_write_bands(bands_u8: list[int], addr: int = ADDR) -> bytes:
    """构建写频谱寄存器（0x10, 0x0100..0x010B）。"""
    if len(bands_u8) != REG_BAND_COUNT:
        bands_u8 = (bands_u8 + [0] * REG_BAND_COUNT)[:REG_BAND_COUNT]
    values = [int(b) & 0xFF for b in bands_u8]
    return build_write_multi(REG_BAND0, values, addr=addr)


def verify_crc(frame: bytes) -> bool:
    """校验 Modbus CRC16。"""
    if len(frame) < 4:
        return False
    crc = crc16_modbus(frame[:-2])
    return frame[-2:] == struct.pack("<H", crc)


def parse_read_holding_response(frame: bytes) -> list[int]:
    """解析 0x03 读保持寄存器响应帧。"""
    if len(frame) < 5:
        raise ValueError("响应长度不足")
    if not verify_crc(frame):
        raise ValueError("CRC 校验失败")
    if frame[1] & 0x80:
        raise ValueError(f"异常响应码: 0x{frame[2]:02X}")
    if frame[1] != FUNC_READ_HOLDING:
        raise ValueError("功能码不匹配")
    byte_count = frame[2]
    if len(frame) != (3 + byte_count + 2):
        raise ValueError("响应长度与字节数不匹配")
    if byte_count % 2 != 0:
        raise ValueError("字节数不是 2 的倍数")
    values = []
    for i in range(0, byte_count, 2):
        values.append((frame[3 + i] << 8) | frame[4 + i])
    return values
