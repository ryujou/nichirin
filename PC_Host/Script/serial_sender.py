#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
串口发送线程。

负责按照指定频率读取“频谱提供者”的最新数据并发送 UART 帧。
"""

from __future__ import annotations

import time
import struct
from dataclasses import dataclass

import serial
from PyQt6.QtCore import QThread, pyqtSignal

from pc_common import FRAME_LEN, build_frame


@dataclass
class SenderConfig:
    """串口发送配置。"""

    port: str
    baud: int
    hz: float
    print_frames: bool
    print_every_n: int


class SenderThread(QThread):
    """串口发送线程。"""

    status = pyqtSignal(str)
    stats = pyqtSignal(int, float, int)
    log = pyqtSignal(str)

    def __init__(self, cfg: SenderConfig, provider):
        super().__init__()
        self.cfg = cfg
        self.provider = provider
        self._stop = False

    def request_stop(self):
        """请求停止线程。"""
        self._stop = True

    def run(self):
        """线程主循环：按固定频率发送频谱帧。"""
        self._stop = False
        period = 1.0 / max(1e-3, float(self.cfg.hz))
        err = 0

        try:
            ser = serial.Serial(self.cfg.port, self.cfg.baud, timeout=0, write_timeout=0.2)
        except Exception as e:
            self.status.emit(f"串口打开失败：{e}")
            return

        try:
            time.sleep(0.10)
            self.status.emit(f"串口发送：{self.cfg.port} @ {self.cfg.baud}, {self.cfg.hz:.1f}Hz")

            t0 = time.perf_counter()
            next_time = t0
            sent = 0
            last_report = t0
            every_n = max(1, int(self.cfg.print_every_n))

            while not self._stop:
                now = time.perf_counter()
                if now < next_time:
                    time.sleep(min(0.001, next_time - now))
                    continue

                if now - next_time > 3.0 * period:
                    next_time = now

                bands_u8 = self.provider.get_latest()
                frame = build_frame(bands_u8)

                try:
                    ser.write(frame)
                except Exception as e:
                    err += 1
                    if err % 10 == 1:
                        self.status.emit(f"写串口异常（累计{err}）：{e}")

                sent += 1
                if self.cfg.print_frames and (sent % every_n == 0):
                    crc = struct.unpack_from("<H", frame, FRAME_LEN - 2)[0]
                    self.log.emit(f"[TX {sent:6d}] bands={bands_u8} crc=0x{crc:04X}")

                if now - last_report >= 1.0:
                    self.stats.emit(sent, now - t0, err)
                    last_report = now

                next_time += period

        except Exception as e:
            self.status.emit(f"发送异常：{e}")
        finally:
            try:
                ser.close()
            except Exception:
                pass
            self.status.emit("串口已停止")
