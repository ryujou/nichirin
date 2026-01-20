#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PyQt6 GUI: Send 12-band spectrum frames over UART (115200) at up to 200Hz.

Frame (17 bytes):
  [ADDR=0x01][FUNC=0x20][12*U8 bands][CRC16_L][CRC16_H]
CRC16: Modbus RTU (poly 0xA001, init 0xFFFF), little-endian appended.
"""

import math
import random
import struct
import sys
import time
from dataclasses import dataclass

from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QComboBox, QSpinBox,
    QDoubleSpinBox, QTextEdit, QHBoxLayout, QVBoxLayout, QGroupBox,
    QMessageBox, QCheckBox
)

import serial
from serial.tools import list_ports


ADDR = 0x01
FUNC = 0x20
BANDS = 12
FRAME_LEN = 1 + 1 + BANDS + 2


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def clamp_u8(x: float) -> int:
    if x < 0:
        return 0
    if x > 255:
        return 255
    return int(x)


def build_frame(bands: list[int]) -> tuple[bytes, int]:
    if len(bands) != BANDS:
        raise ValueError("bands must have length 12")
    payload = bytes([ADDR, FUNC] + [b & 0xFF for b in bands])
    crc = crc16_modbus(payload)
    frame = payload + struct.pack("<H", crc)
    return frame, crc


def gen_bands_sine(t: float, base=110, amp=120, speed=0.35) -> list[int]:
    out = []
    for i in range(BANDS):
        v = base + amp * math.sin(2 * math.pi * (speed * t) + i * 0.55)
        out.append(clamp_u8(v))
    return out


def gen_bands_bar(t: float, peak=255, floor=0, hold_width=1, scan_speed=6.0) -> list[int]:
    idx = int((t * scan_speed) % (2 * (BANDS - 1)))
    if idx >= BANDS:
        idx = 2 * (BANDS - 1) - idx
    out = [floor] * BANDS
    for k in range(hold_width):
        j = max(0, min(BANDS - 1, idx + k))
        out[j] = peak
    return out


def gen_bands_random(prev: list[int], jitter=60, pull=0.25) -> list[int]:
    out = []
    for i in range(BANDS):
        target = 128 + random.randint(-jitter, jitter)
        v = int(prev[i] + (target - prev[i]) * pull)
        out.append(max(0, min(255, v)))
    return out


@dataclass
class SenderConfig:
    port: str
    baud: int
    hz: float
    mode: str
    print_frames: bool
    print_every_n: int


class SenderThread(QThread):
    log = pyqtSignal(str)
    status = pyqtSignal(str)
    stats = pyqtSignal(int, float)  # sent_count, elapsed_s

    def __init__(self, cfg: SenderConfig):
        super().__init__()
        self.cfg = cfg
        self._stop = False

    def request_stop(self):
        self._stop = True

    def run(self):
        self._stop = False
        period = 1.0 / max(1e-3, float(self.cfg.hz))
        prev = [128] * BANDS

        try:
            ser = serial.Serial(self.cfg.port, self.cfg.baud, timeout=0)
        except Exception as e:
            self.status.emit(f"打开串口失败：{e}")
            return

        try:
            time.sleep(0.15)  # let USB-UART settle
            self.status.emit(f"发送中：{self.cfg.port} @ {self.cfg.baud}, {self.cfg.hz:.1f}Hz, mode={self.cfg.mode}")

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

                t = now - t0
                m = self.cfg.mode

                if m == "sine":
                    bands = gen_bands_sine(t)
                elif m == "bar":
                    bands = gen_bands_bar(t)
                else:
                    prev = gen_bands_random(prev)
                    bands = prev

                frame, crc = build_frame(bands)
                if len(frame) != FRAME_LEN:
                    self.status.emit(f"内部错误：frame长度={len(frame)}")
                    break

                ser.write(frame)
                sent += 1

                # Optional: print what we send (throttled)
                if self.cfg.print_frames and (sent % every_n == 0):
                    # Show bands + CRC in hex
                    self.log.emit(
                        f"[TX {sent:6d}] bands={bands} crc=0x{crc:04X}"
                    )

                # stats about once per second
                if now - last_report >= 1.0:
                    self.stats.emit(sent, t)
                    last_report = now

                next_time += period

        except Exception as e:
            self.status.emit(f"发送异常：{e}")
        finally:
            try:
                ser.close()
            except Exception:
                pass
            self.status.emit("已停止")


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("WS2812 12段频谱灯 串口发送器 (RTU风格帧)")

        self.sender: SenderThread | None = None

        # Controls
        self.port_box = QComboBox()
        self.refresh_btn = QPushButton("刷新串口列表")

        self.baud_box = QComboBox()
        self.baud_box.addItems(["115200", "230400", "460800"])
        self.baud_box.setCurrentText("115200")

        self.mode_box = QComboBox()
        self.mode_box.addItems(["sine", "bar", "random"])

        self.hz_box = QDoubleSpinBox()
        self.hz_box.setRange(1.0, 500.0)
        self.hz_box.setDecimals(1)
        self.hz_box.setSingleStep(10.0)
        self.hz_box.setValue(200.0)

        # NEW: print checkbox + throttle
        self.print_box = QCheckBox("打印发送内容")
        self.print_every = QSpinBox()
        self.print_every.setRange(1, 1000000)
        self.print_every.setValue(20)
        self.print_every.setSuffix(" 帧/次")
        self.print_every.setToolTip("例如 20：200Hz 时每0.1秒打印一次；1=每帧打印(日志会很快刷屏)")

        self.start_btn = QPushButton("开始发送")
        self.stop_btn = QPushButton("停止")
        self.stop_btn.setEnabled(False)

        self.status_label = QLabel("未开始")
        self.status_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)

        # Layout
        top = QGroupBox("串口设置")
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("串口："))
        row1.addWidget(self.port_box, 2)
        row1.addWidget(self.refresh_btn)
        row1.addSpacing(12)
        row1.addWidget(QLabel("波特率："))
        row1.addWidget(self.baud_box)
        top.setLayout(row1)

        mid = QGroupBox("发送设置")
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("模式："))
        row2.addWidget(self.mode_box)
        row2.addSpacing(12)
        row2.addWidget(QLabel("频率(Hz)："))
        row2.addWidget(self.hz_box)
        row2.addSpacing(12)
        row2.addWidget(self.print_box)
        row2.addWidget(QLabel("每"))
        row2.addWidget(self.print_every)
        row2.addStretch(1)
        row2.addWidget(self.start_btn)
        row2.addWidget(self.stop_btn)
        mid.setLayout(row2)

        bottom = QGroupBox("状态 / 日志")
        vb = QVBoxLayout()
        vb.addWidget(self.status_label)
        vb.addWidget(self.log_view)
        bottom.setLayout(vb)

        root = QVBoxLayout()
        root.addWidget(top)
        root.addWidget(mid)
        root.addWidget(bottom)
        self.setLayout(root)

        # Signals
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.start_btn.clicked.connect(self.start_sending)
        self.stop_btn.clicked.connect(self.stop_sending)

        self.refresh_ports()

    def log(self, s: str):
        self.log_view.append(s)

    def set_status(self, s: str):
        self.status_label.setText(s)
        self.log(s)

    def refresh_ports(self):
        self.port_box.clear()
        ports = list_ports.comports()
        for p in ports:
            text = f"{p.device}  ({p.description})"
            self.port_box.addItem(text, p.device)
        if self.port_box.count() == 0:
            self.port_box.addItem("未发现串口", "")

    def start_sending(self):
        if self.sender is not None:
            return

        port = self.port_box.currentData()
        if not port:
            QMessageBox.warning(self, "提示", "请选择有效串口")
            return

        cfg = SenderConfig(
            port=str(port),
            baud=int(self.baud_box.currentText()),
            hz=float(self.hz_box.value()),
            mode=str(self.mode_box.currentText()),
            print_frames=bool(self.print_box.isChecked()),
            print_every_n=int(self.print_every.value()),
        )

        self.sender = SenderThread(cfg)
        self.sender.status.connect(self.set_status)
        self.sender.stats.connect(self.on_stats)
        self.sender.log.connect(self.log)
        self.sender.start()

        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.refresh_btn.setEnabled(False)
        self.port_box.setEnabled(False)
        self.baud_box.setEnabled(False)
        self.mode_box.setEnabled(False)
        self.hz_box.setEnabled(False)
        self.print_box.setEnabled(False)
        self.print_every.setEnabled(False)

    def stop_sending(self):
        if self.sender is None:
            return
        self.sender.request_stop()
        self.sender.wait(1500)
        self.sender = None

        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.refresh_btn.setEnabled(True)
        self.port_box.setEnabled(True)
        self.baud_box.setEnabled(True)
        self.mode_box.setEnabled(True)
        self.hz_box.setEnabled(True)
        self.print_box.setEnabled(True)
        self.print_every.setEnabled(True)

    def on_stats(self, sent: int, elapsed_s: float):
        self.status_label.setText(f"发送中… 已发送 {sent} 帧，用时 {elapsed_s:.1f}s")

    def closeEvent(self, event):
        try:
            self.stop_sending()
        finally:
            event.accept()


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.resize(920, 560)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
