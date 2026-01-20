#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Nichirin V3 PC 上位机入口。

该文件作为启动入口，具体功能已拆分到多个模块：
- pc_common.py     协议与常量
- dsp.py           频谱与自适应处理
- audio_threads.py 音频采集与文件解码线程
- serial_sender.py 串口发送线程
- gui_app.py       主界面
"""

from __future__ import annotations

import sys
from PyQt6.QtWidgets import QApplication

from gui_app import MainWindow


def main():
    """程序入口。"""
    app = QApplication(sys.argv)
    w = MainWindow()
    w.resize(1220, 900)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
