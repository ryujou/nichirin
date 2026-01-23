#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GUI 主界面。

将 UI、媒体播放器与线程控制集中在此文件，便于维护。
"""

from __future__ import annotations

import os
from typing import Optional

import numpy as np
import sounddevice as sd
from serial.tools import list_ports

from PyQt6.QtCore import Qt, QUrl
from PyQt6.QtWidgets import (
    QWidget, QLabel, QPushButton, QComboBox,
    QDoubleSpinBox, QSpinBox, QTextEdit, QHBoxLayout, QVBoxLayout, QGroupBox,
    QMessageBox, QFileDialog, QCheckBox, QSlider
)
from PyQt6.QtMultimedia import QMediaPlayer, QAudioOutput
from PyQt6.QtMultimediaWidgets import QVideoWidget

from pc_common import AUDIO_EXT, VIDEO_EXT
from dsp import DspConfig
from audio_threads import MicAudioThread, FilePreAnalyzeThread, FileAudioThread
from serial_sender import SenderConfig, SenderThread


class MainWindow(QWidget):
    """主窗口：媒体播放 + 频谱提取 + 串口发送。"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("12段频谱灯 上位机")
        self.setAcceptDrops(True)

        # 媒体播放器：支持音频/视频
        self.video = QVideoWidget()
        self.player = QMediaPlayer(self)
        self.audio_out = QAudioOutput(self)
        self.player.setAudioOutput(self.audio_out)
        self.player.setVideoOutput(self.video)
        self.audio_out.setVolume(1.0)

        self.file_path: Optional[str] = None

        # 分析缓存（当前文件）
        self.file_genre_fixed: Optional[float] = None
        self.file_bandref_fixed: Optional[np.ndarray] = None
        self.pre_thread: Optional[FilePreAnalyzeThread] = None

        # 媒体控制
        self.open_btn = QPushButton("打开音频/视频…")
        self.play_pause_btn = QPushButton("播放")
        self.stop_media_btn = QPushButton("停止")
        self.play_pause_btn.setEnabled(False)
        self.stop_media_btn.setEnabled(False)

        # 播放进度条
        self.progress = QSlider(Qt.Orientation.Horizontal)
        self.progress.setRange(0, 0)
        self.progress.setEnabled(False)
        self.progress_time = QLabel("00:00 / 00:00")
        self._seeking = False

        # 串口 UI
        self.port_box = QComboBox()
        self.refresh_port_btn = QPushButton("刷新串口")
        self.baud_box = QComboBox()
        self.baud_box.addItems(["115200", "230400", "460800"])
        self.baud_box.setCurrentText("115200")

        self.hz_box = QDoubleSpinBox()
        self.hz_box.setRange(1.0, 1000.0)
        self.hz_box.setDecimals(1)
        self.hz_box.setSingleStep(50.0)
        self.hz_box.setValue(400.0)  # default 400Hz

        self.print_box = QCheckBox("打印发送")
        self.print_every = QSpinBox()
        self.print_every.setRange(1, 1000000)
        self.print_every.setValue(200)
        self.print_every.setSuffix(" 帧/次")

        # 输入源选择
        self.source_box = QComboBox()
        self.source_box.addItems([
            "麦克风 (Mic)",
            "文件 (File: 先预分析整首，再播放+频谱)",
        ])

        # 麦克风设备
        self.audio_dev_box = QComboBox()
        self.refresh_audio_btn = QPushButton("刷新麦克风")

        # DSP 参数
        self.sr_box = QComboBox()
        self.sr_box.addItems(["48000", "44100"])
        self.sr_box.setCurrentText("48000")

        self.block_box = QComboBox()
        self.block_box.addItems(["256", "512", "1024"])
        self.block_box.setCurrentText("512")

        self.floor_db = QDoubleSpinBox()
        self.floor_db.setRange(0.0, 120.0)
        self.floor_db.setValue(28.0)
        self.floor_db.setSuffix(" dB")

        # 运行控制
        self.start_btn = QPushButton("开始（频谱+发送）")
        self.stop_btn = QPushButton("停止发送")
        self.stop_btn.setEnabled(False)

        # 状态与日志
        self.status_label = QLabel("未开始")
        self.status_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.genre_label = QLabel("风格：0.00  (-1=ACG  +1=低音DJ)")
        self.genre_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)

        # 线程
        self.provider_thread = None
        self.sender_thread: Optional[SenderThread] = None

        # ---------------- Layout ----------------
        media_box = QGroupBox("文件播放（拖拽音频/视频到窗口也行；打开后先预分析整首，完成后才能播放）")
        mrow = QHBoxLayout()
        mrow.addWidget(self.open_btn)
        mrow.addWidget(self.play_pause_btn)
        mrow.addWidget(self.stop_media_btn)
        mrow.addStretch(1)

        prow = QHBoxLayout()
        prow.addWidget(self.progress, 1)
        prow.addWidget(self.progress_time)

        mv = QVBoxLayout()
        mv.addLayout(mrow)
        mv.addLayout(prow)
        media_box.setLayout(mv)

        serial_box = QGroupBox("串口发送")
        srow = QHBoxLayout()
        srow.addWidget(QLabel("串口："))
        srow.addWidget(self.port_box, 2)
        srow.addWidget(self.refresh_port_btn)
        srow.addSpacing(12)
        srow.addWidget(QLabel("波特率："))
        srow.addWidget(self.baud_box)
        srow.addSpacing(12)
        srow.addWidget(QLabel("Hz："))
        srow.addWidget(self.hz_box)
        srow.addSpacing(12)
        srow.addWidget(self.print_box)
        srow.addWidget(QLabel("每"))
        srow.addWidget(self.print_every)
        serial_box.setLayout(srow)

        audio_box = QGroupBox("输入源 / 参数")
        arow1 = QHBoxLayout()
        arow1.addWidget(QLabel("输入源："))
        arow1.addWidget(self.source_box, 2)
        arow1.addWidget(QLabel("麦克风："))
        arow1.addWidget(self.audio_dev_box, 3)
        arow1.addWidget(self.refresh_audio_btn)

        arow2 = QHBoxLayout()
        arow2.addWidget(QLabel("采样率："))
        arow2.addWidget(self.sr_box)
        arow2.addSpacing(12)
        arow2.addWidget(QLabel("块："))
        arow2.addWidget(self.block_box)
        arow2.addSpacing(12)
        arow2.addWidget(QLabel("Floor："))
        arow2.addWidget(self.floor_db)
        arow2.addStretch(1)

        av = QVBoxLayout()
        av.addLayout(arow1)
        av.addLayout(arow2)
        audio_box.setLayout(av)

        run_box = QGroupBox("运行")
        rrow = QHBoxLayout()
        rrow.addWidget(self.start_btn)
        rrow.addWidget(self.stop_btn)
        rrow.addStretch(1)
        rrow.addWidget(self.status_label)
        run_box.setLayout(rrow)

        stat_box = QGroupBox("自适应状态（文件模式：预分析锁定，不再漂移）")
        sv = QVBoxLayout()
        sv.addWidget(self.genre_label)
        stat_box.setLayout(sv)

        log_box = QGroupBox("日志")
        lv = QVBoxLayout()
        lv.addWidget(self.log_view)
        log_box.setLayout(lv)

        root = QVBoxLayout()
        root.addWidget(media_box)
        root.addWidget(self.video, 4)
        root.addWidget(serial_box)
        root.addWidget(audio_box)
        root.addWidget(run_box)
        root.addWidget(stat_box)
        root.addWidget(log_box, 2)
        self.setLayout(root)

        # ---------------- Signals ----------------
        self.refresh_port_btn.clicked.connect(self.refresh_ports)
        self.refresh_audio_btn.clicked.connect(self.refresh_mic_devices)
        self.source_box.currentIndexChanged.connect(self.on_source_changed)

        self.open_btn.clicked.connect(self.open_file)
        self.play_pause_btn.clicked.connect(self.toggle_play_pause)
        self.stop_media_btn.clicked.connect(self.stop_media)

        self.start_btn.clicked.connect(self.start_all)
        self.stop_btn.clicked.connect(self.stop_all)

        # 媒体播放器状态
        self.player.playbackStateChanged.connect(self.on_player_state)
        self.player.durationChanged.connect(self.on_duration_changed)
        self.player.positionChanged.connect(self.on_position_changed)
        self.progress.sliderPressed.connect(self.on_slider_pressed)
        self.progress.sliderReleased.connect(self.on_slider_released)
        self.progress.sliderMoved.connect(self.on_slider_moved)

        # 初始刷新
        self.refresh_ports()
        self.refresh_mic_devices()
        self.on_source_changed()

    # ---- drag & drop ----
    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.acceptProposedAction()

    def dropEvent(self, event):
        urls = event.mimeData().urls()
        if not urls:
            return
        path = urls[0].toLocalFile()
        if path:
            self.load_media(path)

    # ---- helpers ----
    def log(self, s: str):
        self.log_view.append(s)

    def set_status(self, s: str):
        self.status_label.setText(s)
        self.log(s)

    # ---- serial ports ----
    def refresh_ports(self):
        """刷新串口列表，并按“常用优先”排序。"""
        self.port_box.clear()
        ports = list_ports.comports()

        def _port_priority(p):
            desc = (p.description or "").lower()
            name = (p.device or "").lower()
            hwid = (getattr(p, "hwid", "") or "").lower()

            # 优先常见 USB-UART 芯片
            chip_keywords = (
                "ch340", "ch341", "ch9102", "cp210", "cp2102", "cp210x",
                "ft232", "ftdi", "pl2303",
            )
            if any(k in desc or k in name or k in hwid for k in chip_keywords):
                return (0, name)

            # 再优先蓝牙串口
            bt_keywords = (
                "bluetooth", "rfcomm", "serial over bluetooth",
            )
            if any(k in desc for k in bt_keywords):
                return (1, name)

            # 将空闲的 COM1 等排后
            if name == "com1" or "communications port" in desc:
                return (3, name)

            return (2, name)

        ports = sorted(ports, key=_port_priority)
        for p in ports:
            self.port_box.addItem(f"{p.device} ({p.description})", p.device)
        if self.port_box.count() == 0:
            self.port_box.addItem("未发现串口", "")

    # ---- mic devices ----
    def refresh_mic_devices(self):
        """枚举可用麦克风设备。"""
        self.audio_dev_box.clear()
        try:
            devs = sd.query_devices()
        except Exception as e:
            self.audio_dev_box.addItem("枚举失败", -1)
            self.log(f"枚举麦克风失败：{e}")
            return

        for idx, d in enumerate(devs):
            if int(d.get("max_input_channels", 0)) <= 0:
                continue
            self.audio_dev_box.addItem(f"[{idx}] {d.get('name','')}", idx)

        if self.audio_dev_box.count() == 0:
            self.audio_dev_box.addItem("未找到输入设备", -1)

        try:
            default_in = sd.default.device[0]
            if default_in is not None:
                for i in range(self.audio_dev_box.count()):
                    if self.audio_dev_box.itemData(i) == default_in:
                        self.audio_dev_box.setCurrentIndex(i)
                        break
        except Exception:
            pass

    def on_source_changed(self):
        """切换输入源时调整 UI 可用性。"""
        is_file = (self.source_box.currentIndex() == 1)

        self.audio_dev_box.setEnabled(not is_file)
        self.refresh_audio_btn.setEnabled(not is_file)

        # 文件播放按钮依赖预分析完成
        has_file = self.file_path is not None
        analyzed = (self.file_genre_fixed is not None and self.file_bandref_fixed is not None)
        self.play_pause_btn.setEnabled(is_file and has_file and analyzed)
        self.stop_media_btn.setEnabled(is_file and has_file)

    # ---- configs ----
    def _make_dsp_cfg(self) -> DspConfig:
        """读取 DSP 参数。"""
        sr = int(self.sr_box.currentText())
        block = int(self.block_box.currentText())
        return DspConfig(
            sr=sr,
            block=block,
            nfft=2048,
            fmin=30.0,
            fmax=16000.0,
            noise_floor_db=float(self.floor_db.value()),
        )

    def _make_sender_cfg(self) -> SenderConfig:
        """读取串口发送配置。"""
        port = self.port_box.currentData()
        if not port:
            raise RuntimeError("请选择有效串口")
        return SenderConfig(
            port=str(port),
            baud=int(self.baud_box.currentText()),
            hz=float(self.hz_box.value()),
            print_frames=bool(self.print_box.isChecked()),
            print_every_n=int(self.print_every.value()),
        )

    # ---- media ----
    def open_file(self):
        """通过对话框选择媒体文件。"""
        fn, _ = QFileDialog.getOpenFileName(
            self, "选择音频或视频文件", "",
            "Media Files (*.mp3 *.wav *.flac *.aac *.m4a *.ogg *.opus *.mp4 *.mkv *.avi *.mov *.webm *.wmv);;All Files (*.*)"
        )
        if fn:
            self.load_media(fn)

    def _cancel_preanalysis(self):
        """取消正在进行的预分析。"""
        if self.pre_thread is not None:
            try:
                self.pre_thread.request_stop()
                self.pre_thread.wait(1500)
            except Exception:
                pass
            self.pre_thread = None

    def load_media(self, path: str):
        """加载媒体文件并触发预分析。"""
        ext = os.path.splitext(path)[1].lower()
        if ext not in AUDIO_EXT and ext not in VIDEO_EXT:
            QMessageBox.warning(self, "提示", f"不支持的文件类型：{ext}")
            return

        # 停止任何正在运行的发送/采集
        if self.sender_thread is not None or self.provider_thread is not None:
            self.stop_all()

        self._cancel_preanalysis()

        self.file_path = path
        self.file_genre_fixed = None
        self.file_bandref_fixed = None

        self.progress.setEnabled(True)
        self.progress.setRange(0, 0)
        self.progress.setValue(0)
        self.progress_time.setText("00:00 / 00:00")

        self.player.setSource(QUrl.fromLocalFile(path))
        self.player.pause()
        self.play_pause_btn.setText("播放")
        self.stop_media_btn.setEnabled(True)

        # 切换到文件模式
        self.source_box.setCurrentIndex(1)
        self.on_source_changed()

        self.set_status(f"已加载：{os.path.basename(path)}（将先预分析整首，完成后才能播放）")
        self.genre_label.setText("风格：分析中…")

        # 启动全量预分析
        dsp = self._make_dsp_cfg()
        self.pre_thread = FilePreAnalyzeThread(path, dsp)
        self.pre_thread.status.connect(self.set_status)
        self.pre_thread.done.connect(self.on_preanalysis_done)
        self.pre_thread.start()

        # 预分析前禁用播放
        self.play_pause_btn.setEnabled(False)

    def on_preanalysis_done(self, genre_fixed: float, band_ref_fixed: object):
        """预分析完成后回调。"""
        self.file_genre_fixed = float(genre_fixed)
        self.file_bandref_fixed = np.array(band_ref_fixed, dtype=np.float32)
        self.genre_label.setText(f"风格：{self.file_genre_fixed:+.2f}  (-1=ACG  +1=低音DJ)")
        self.set_status("预分析完成：现在可以点击“播放”开始（风格已锁定）")
        self.on_source_changed()

    def toggle_play_pause(self):
        """播放/暂停切换。"""
        st = self.player.playbackState()
        if st == QMediaPlayer.PlaybackState.PlayingState:
            self.player.pause()
        else:
            self.player.play()

    def stop_media(self):
        """停止媒体播放。"""
        self.player.stop()
        self.play_pause_btn.setText("播放")
        if isinstance(self.provider_thread, FileAudioThread):
            self.provider_thread.set_paused(True)
        self.progress.setValue(0)

    def on_player_state(self, state):
        """同步播放状态到频谱线程。"""
        if state == QMediaPlayer.PlaybackState.PlayingState:
            self.play_pause_btn.setText("暂停")
            if isinstance(self.provider_thread, FileAudioThread):
                self.provider_thread.set_paused(False)
        elif state == QMediaPlayer.PlaybackState.PausedState:
            self.play_pause_btn.setText("播放")
            if isinstance(self.provider_thread, FileAudioThread):
                self.provider_thread.set_paused(True)
        else:
            self.play_pause_btn.setText("播放")
            if isinstance(self.provider_thread, FileAudioThread):
                self.provider_thread.set_paused(True)

    # ---- media progress ----
    @staticmethod
    def _fmt_ms(ms: int) -> str:
        """将毫秒格式化为 mm:ss。"""
        s = max(0, int(ms // 1000))
        m = s // 60
        s = s % 60
        return f"{m:02d}:{s:02d}"

    def on_duration_changed(self, duration_ms: int):
        """播放器时长更新时更新进度条。"""
        self.progress.setRange(0, max(0, int(duration_ms)))
        pos = int(self.player.position())
        self.progress_time.setText(f"{self._fmt_ms(pos)} / {self._fmt_ms(duration_ms)}")

    def on_position_changed(self, position_ms: int):
        """播放位置变化时更新进度条与时间。"""
        if not self._seeking:
            self.progress.setValue(int(position_ms))
        dur = int(self.player.duration())
        self.progress_time.setText(f"{self._fmt_ms(position_ms)} / {self._fmt_ms(dur)}")

    def on_slider_pressed(self):
        """用户开始拖拽进度条。"""
        self._seeking = True

    def on_slider_released(self):
        """用户结束拖拽进度条，跳转播放位置。"""
        self._seeking = False
        self.player.setPosition(int(self.progress.value()))

    def on_slider_moved(self, value: int):
        """拖拽时即时显示目标时间。"""
        dur = int(self.player.duration())
        self.progress_time.setText(f"{self._fmt_ms(value)} / {self._fmt_ms(dur)}")

    # ---- start/stop ----
    def start_all(self):
        """启动频谱采集与串口发送。"""
        if self.provider_thread is not None or self.sender_thread is not None:
            return

        dsp = self._make_dsp_cfg()
        try:
            scfg = self._make_sender_cfg()
        except Exception as e:
            QMessageBox.warning(self, "启动失败", str(e))
            return

        if self.source_box.currentIndex() == 0:
            dev = int(self.audio_dev_box.currentData())
            if dev < 0:
                QMessageBox.warning(self, "启动失败", "请选择有效麦克风设备")
                return
            self.provider_thread = MicAudioThread(dev, dsp)
            self.provider_thread.status.connect(self.set_status)
            self.provider_thread.debug.connect(self.on_genre)
            self.provider_thread.start()

        else:
            if not self.file_path:
                QMessageBox.warning(self, "启动失败", "文件模式需要先加载文件")
                return
            if self.file_genre_fixed is None or self.file_bandref_fixed is None:
                QMessageBox.warning(self, "启动失败", "文件尚未预分析完成，无法开始")
                return

            self.provider_thread = FileAudioThread(
                self.file_path, dsp,
                genre_fixed=self.file_genre_fixed,
                band_ref_fixed=self.file_bandref_fixed
            )
            self.provider_thread.status.connect(self.set_status)
            self.provider_thread.debug.connect(self.on_genre)
            # paused unless player is playing
            self.provider_thread.set_paused(
                self.player.playbackState() != QMediaPlayer.PlaybackState.PlayingState
            )
            self.provider_thread.start()

        self.sender_thread = SenderThread(scfg, self.provider_thread)
        self.sender_thread.status.connect(self.set_status)
        self.sender_thread.log.connect(self.log)
        self.sender_thread.stats.connect(self.on_stats)
        self.sender_thread.start()

        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

        # 运行中锁定设置
        for w in (
            self.refresh_port_btn, self.port_box, self.baud_box, self.hz_box,
            self.source_box,
            self.audio_dev_box, self.refresh_audio_btn,
            self.sr_box, self.block_box, self.floor_db,
            self.print_box, self.print_every,
            self.open_btn
        ):
            w.setEnabled(False)

        # 文件模式允许播放控制
        self.on_source_changed()

    def stop_all(self):
        """停止发送与采集线程。"""
        if self.sender_thread is not None:
            self.sender_thread.request_stop()
            self.sender_thread.wait(1500)
            self.sender_thread = None
        if self.provider_thread is not None:
            self.provider_thread.request_stop()
            self.provider_thread.wait(1500)
            self.provider_thread = None

        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

        for w in (
            self.refresh_port_btn, self.port_box, self.baud_box, self.hz_box,
            self.source_box,
            self.audio_dev_box, self.refresh_audio_btn,
            self.sr_box, self.block_box, self.floor_db,
            self.print_box, self.print_every,
            self.open_btn
        ):
            w.setEnabled(True)

        self.on_source_changed()
        self.set_status("已停止发送")

    def on_stats(self, sent: int, elapsed_s: float, err: int):
        """串口统计显示。"""
        self.status_label.setText(f"运行中：{sent} 帧 / {elapsed_s:.1f}s / 写异常 {err}")

    def on_genre(self, g: float):
        """风格显示。"""
        self.genre_label.setText(f"风格：{g:+.2f}  (-1=ACG  +1=低音DJ)")

    def closeEvent(self, event):
        """窗口关闭时安全释放线程。"""
        try:
            self.stop_all()
            self._cancel_preanalysis()
        finally:
            event.accept()
