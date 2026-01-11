#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import struct
import subprocess
from dataclasses import dataclass

import numpy as np
import serial
from serial.tools import list_ports

try:
    import cv2
except ImportError:
    cv2 = None

from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton,
    QComboBox, QSpinBox, QDoubleSpinBox, QCheckBox, QTextEdit, QFileDialog,
    QHBoxLayout, QVBoxLayout, QGridLayout, QMessageBox
)

MAGIC = b"\xA5\x5A"
TYPE_FRAME_START = 0x01
TYPE_FRAME_DATA = 0x02
TYPE_STOP = 0x03
TYPE_PING = 0x04
TYPE_PONG = 0x05
PIX_FMT_RGB565 = 0x00

WIDTH = 80
HEIGHT = 160
FRAME_BYTES = WIDTH * HEIGHT * 2

NFFT = 2048
HOP = 1024
SAMPLE_RATE = 48000
EDGES_HZ = [60, 93, 145, 226, 351, 546, 849, 1320, 2052, 3191, 4962, 7717, 12000]


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_packet(pkt_type: int, seq: int, payload: bytes) -> bytes:
    header = struct.pack("<BHH", pkt_type, seq & 0xFFFF, len(payload) & 0xFFFF)
    crc = crc16_ccitt(header + payload)
    return MAGIC + header + payload + struct.pack("<H", crc)


def rgb_to_rgb565(rgb: np.ndarray, swap_endian: bool) -> bytes:
    r = (rgb[:, :, 0] >> 3).astype(np.uint16)
    g = (rgb[:, :, 1] >> 2).astype(np.uint16)
    b = (rgb[:, :, 2] >> 3).astype(np.uint16)
    rgb565 = (r << 11) | (g << 5) | b
    if swap_endian:
        rgb565 = rgb565.byteswap()
    return rgb565.tobytes()


def bgr_frame_to_rgb565(frame_bgr: np.ndarray, swap_endian: bool) -> bytes:
    frame = cv2.resize(frame_bgr, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return rgb_to_rgb565(rgb, swap_endian)


def iter_frames_opencv(path: str):
    if cv2 is None:
        return None
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        cap.release()
        return None

    def gen():
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                yield frame  # BGR
        finally:
            cap.release()

    return gen()


def iter_frames_ffmpeg(path: str):
    cmd = [
        "ffmpeg",
        "-i", path,
        "-f", "rawvideo",
        "-pix_fmt", "rgb24",
        "-vf", f"scale={WIDTH}:{HEIGHT}",
        "-"
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    frame_size = WIDTH * HEIGHT * 3

    def gen():
        try:
            while True:
                raw = proc.stdout.read(frame_size)
                if raw is None or len(raw) < frame_size:
                    break
                rgb = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT, WIDTH, 3))
                yield rgb  # RGB
        finally:
            try:
                proc.terminate()
            except Exception:
                pass

    return gen()


class AudioReader:
    def __init__(self, path: str):
        cmd = [
            "ffmpeg",
            "-i", path,
            "-vn",
            "-ac", "1",
            "-ar", str(SAMPLE_RATE),
            "-f", "s16le",
            "-"
        ]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def read_samples(self, count: int) -> np.ndarray:
        if count <= 0:
            return np.zeros(0, dtype=np.float32)
        bytes_needed = count * 2
        raw = self.proc.stdout.read(bytes_needed)
        if raw is None:
            raw = b""
        if len(raw) < bytes_needed:
            raw = raw + (b"\x00" * (bytes_needed - len(raw)))
        samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
        return samples

    def close(self):
        try:
            self.proc.terminate()
        except Exception:
            pass


class SpectrumAnalyzer:
    def __init__(self):
        self.window = np.hanning(NFFT).astype(np.float32)
        freqs = np.fft.rfftfreq(NFFT, d=1.0 / SAMPLE_RATE)
        self.bin_ranges = []
        for i in range(12):
            lo = np.searchsorted(freqs, EDGES_HZ[i], side="left")
            hi = np.searchsorted(freqs, EDGES_HZ[i + 1], side="left")
            self.bin_ranges.append((lo, hi))
        self.peak_ema = 0.15
        self.smooth = np.zeros(12, dtype=np.float32)
        self.peaks = np.zeros(12, dtype=np.float32)

    def process(self, samples: np.ndarray):
        if samples.size == 0:
            power_avg = np.zeros(NFFT // 2 + 1, dtype=np.float32)
            count = 1
        else:
            power_sum = np.zeros(NFFT // 2 + 1, dtype=np.float32)
            count = 0
            for start in range(0, samples.size, HOP):
                seg = samples[start:start + NFFT]
                if seg.size < NFFT:
                    seg = np.pad(seg, (0, NFFT - seg.size))
                seg = seg.astype(np.float32) * self.window
                spec = np.fft.rfft(seg)
                power = (np.abs(spec) ** 2) / (NFFT ** 2)
                power_sum += power.astype(np.float32)
                count += 1
            if count == 0:
                count = 1
            power_avg = power_sum / float(count)

        band_power = np.zeros(12, dtype=np.float32)
        for i, (lo, hi) in enumerate(self.bin_ranges):
            if hi > lo:
                band_power[i] = float(np.mean(power_avg[lo:hi]))

        db = 10.0 * np.log10(band_power + 1e-12)
        db = np.clip(db, -60.0, 0.0)
        x = (db + 60.0) / 60.0

        peak = float(np.max(x))
        self.peak_ema = 0.90 * self.peak_ema + 0.10 * peak
        gain = 1.0 / max(self.peak_ema, 0.15)
        x2 = np.clip(x * gain, 0.0, 1.0)

        self.smooth = 0.65 * self.smooth + 0.35 * x2
        self.peaks = np.maximum(self.peaks * 0.92, self.smooth)

        bands_val = np.round(self.smooth * 255.0).astype(np.uint8)
        bands_peak = np.round(self.peaks * 255.0).astype(np.uint8)
        return bands_val, bands_peak, self.peak_ema


@dataclass
class StreamConfig:
    port: str
    video_path: str
    fps_max: float
    chunk: int
    swap_endian: bool
    baud: int = 921600


@dataclass
class StreamStats:
    fps_current: float
    bytes_per_s: float
    seq: int
    peak_ema: float


class PacketParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        if data:
            self.buf.extend(data)

    def iter_packets(self):
        i = 0
        while i + 7 <= len(self.buf):
            if self.buf[i] != 0xA5 or self.buf[i + 1] != 0x5A:
                i += 1
                continue
            if i + 7 > len(self.buf):
                break
            pkt_type = self.buf[i + 2]
            seq = self.buf[i + 3] | (self.buf[i + 4] << 8)
            length = self.buf[i + 5] | (self.buf[i + 6] << 8)
            total = 2 + 1 + 2 + 2 + length + 2
            if i + total > len(self.buf):
                break
            payload = bytes(self.buf[i + 7:i + 7 + length])
            crc_recv = self.buf[i + 7 + length] | (self.buf[i + 8 + length] << 8)
            crc_calc = crc16_ccitt(bytes(self.buf[i + 2:i + 7 + length]))
            if crc_calc == crc_recv:
                yield pkt_type, seq, payload
                i += total
            else:
                i += 1
        if i > 0:
            del self.buf[:i]


class StreamWorker(QThread):
    log = pyqtSignal(str)
    started_ok = pyqtSignal()
    stopped = pyqtSignal()
    failed = pyqtSignal(str)
    stats = pyqtSignal(StreamStats)

    def __init__(self, cfg: StreamConfig, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self._stop_req = False

    def request_stop(self):
        self._stop_req = True

    def _log(self, s: str):
        self.log.emit(s)

    def run(self):
        cfg = self.cfg
        seq = 0
        ser = None

        frame_iter = iter_frames_opencv(cfg.video_path)
        use_bgr = True
        if frame_iter is None:
            use_bgr = False
            try:
                frame_iter = iter_frames_ffmpeg(cfg.video_path)
                self._log("OpenCV failed, using ffmpeg for video.")
            except FileNotFoundError:
                self.failed.emit("OpenCV failed and ffmpeg not found in PATH.")
                return

        try:
            audio = AudioReader(cfg.video_path)
        except FileNotFoundError:
            self.failed.emit("ffmpeg not found in PATH (required for audio).")
            return

        try:
            ser = serial.Serial(cfg.port, cfg.baud, timeout=0)
        except Exception as e:
            audio.close()
            self.failed.emit(f"Failed to open serial: {e}")
            return

        parser = PacketParser()
        analyzer = SpectrumAnalyzer()

        fps_current = float(cfg.fps_max)
        frame_interval = 1.0 / max(fps_current, 1.0)
        next_deadline = time.perf_counter()
        last_ping = time.perf_counter()
        slow_count = 0
        fast_count = 0
        bytes_accum = 0
        stats_start = time.perf_counter()

        self.started_ok.emit()
        self._log(f"Serial open: {cfg.port}, fps_max={cfg.fps_max}, chunk={cfg.chunk}, swap_endian={cfg.swap_endian}")
        self._log(f"Streaming video -> {WIDTH}x{HEIGHT} RGB565, audio FFT bands.")

        try:
            for frame in frame_iter:
                if self._stop_req:
                    self._log("Stop requested, sending STOP...")
                    break

                frame_begin = time.perf_counter()

                frame_samples = int(round(SAMPLE_RATE / max(fps_current, 1.0)))
                samples = audio.read_samples(frame_samples)
                bands_val, bands_peak, peak_ema = analyzer.process(samples)

                if use_bgr:
                    frame_bytes = bgr_frame_to_rgb565(frame, cfg.swap_endian)
                else:
                    frame_bytes = rgb_to_rgb565(frame, cfg.swap_endian)

                payload_start = (
                    struct.pack("<BBBH", WIDTH, HEIGHT, PIX_FMT_RGB565, FRAME_BYTES)
                    + bytes(bands_val)
                    + bytes(bands_peak)
                    + struct.pack("<B", 0)
                )
                pkt = build_packet(TYPE_FRAME_START, seq, payload_start)
                ser.write(pkt)
                bytes_accum += len(pkt)

                for off in range(0, len(frame_bytes), cfg.chunk):
                    if self._stop_req:
                        break
                    chunk = frame_bytes[off:off + cfg.chunk]
                    pkt = build_packet(TYPE_FRAME_DATA, seq, chunk)
                    ser.write(pkt)
                    bytes_accum += len(pkt)

                if time.perf_counter() - last_ping >= 1.0:
                    ping_pkt = build_packet(TYPE_PING, seq, b"")
                    ser.write(ping_pkt)
                    bytes_accum += len(ping_pkt)
                    last_ping = time.perf_counter()

                rx_len = ser.in_waiting
                if rx_len:
                    data = ser.read(rx_len)
                    parser.feed(data)
                    for pkt_type, _, payload in parser.iter_packets():
                        if pkt_type == TYPE_PONG and len(payload) >= 6:
                            _ = payload  # placeholder for future stats

                seq = (seq + 1) & 0xFFFF

                tx_time = time.perf_counter() - frame_begin
                if tx_time > 0.95 * frame_interval:
                    slow_count += 1
                    fast_count = 0
                elif tx_time < 0.60 * frame_interval:
                    fast_count += 1
                    slow_count = 0
                else:
                    slow_count = 0
                    fast_count = 0

                if slow_count >= 10:
                    fps_current = max(6.0, fps_current * 0.85)
                    frame_interval = 1.0 / fps_current
                    slow_count = 0
                    next_deadline = time.perf_counter() + frame_interval
                elif fast_count >= 60:
                    fps_current = min(cfg.fps_max, fps_current * 1.05)
                    frame_interval = 1.0 / fps_current
                    fast_count = 0
                    next_deadline = time.perf_counter() + frame_interval

                now = time.perf_counter()
                if now - stats_start >= 1.0:
                    bytes_per_s = bytes_accum / (now - stats_start)
                    bytes_accum = 0
                    stats_start = now
                    self.stats.emit(StreamStats(
                        fps_current=fps_current,
                        bytes_per_s=bytes_per_s,
                        seq=seq,
                        peak_ema=peak_ema,
                    ))

                next_deadline += frame_interval
                sleep_time = next_deadline - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_deadline = time.perf_counter()

        except Exception as e:
            self._log(f"Stream error: {e}")

        try:
            ser.write(build_packet(TYPE_STOP, seq, b""))
        except Exception:
            pass

        try:
            ser.close()
        except Exception:
            pass
        audio.close()

        self._log("Streaming stopped.")
        self.stopped.emit()


class DragDropLineEdit(QLineEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAcceptDrops(True)

    def dragEnterEvent(self, e):
        if e.mimeData().hasUrls():
            e.acceptProposedAction()
        else:
            super().dragEnterEvent(e)

    def dropEvent(self, e):
        urls = e.mimeData().urls()
        if not urls:
            return
        path = urls[0].toLocalFile()
        if path:
            self.setText(path)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 Video+Spectrum Stream (USB CDC)")
        self.worker = None

        root = QWidget()
        self.setCentralWidget(root)

        self.video_edit = DragDropLineEdit()
        self.video_edit.setPlaceholderText("Drop a video file or browse...")
        btn_browse = QPushButton("Browse")
        btn_browse.clicked.connect(self.on_browse)

        self.port_combo = QComboBox()
        btn_refresh = QPushButton("Refresh Ports")
        btn_refresh.clicked.connect(self.refresh_ports)

        self.fps_spin = QDoubleSpinBox()
        self.fps_spin.setRange(6.0, 60.0)
        self.fps_spin.setDecimals(1)
        self.fps_spin.setValue(12.0)

        self.chunk_spin = QSpinBox()
        self.chunk_spin.setRange(256, 4096)
        self.chunk_spin.setSingleStep(256)
        self.chunk_spin.setValue(1024)

        self.swap_chk = QCheckBox("Swap RGB565 bytes (MSB first)")
        self.swap_chk.setChecked(True)

        self.btn_start = QPushButton("Start")
        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setEnabled(False)
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)

        self.label_fps = QLabel("FPS: --")
        self.label_bps = QLabel("Bytes/s: --")
        self.label_seq = QLabel("Seq: --")
        self.label_peak = QLabel("Peak EMA: --")

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)

        grid = QGridLayout()
        row = 0
        grid.addWidget(QLabel("Video"), row, 0)
        grid.addWidget(self.video_edit, row, 1)
        grid.addWidget(btn_browse, row, 2)
        row += 1

        grid.addWidget(QLabel("Port"), row, 0)
        grid.addWidget(self.port_combo, row, 1)
        grid.addWidget(btn_refresh, row, 2)
        row += 1

        grid.addWidget(QLabel("FPS Max"), row, 0)
        grid.addWidget(self.fps_spin, row, 1)
        row += 1

        grid.addWidget(QLabel("Chunk (bytes)"), row, 0)
        grid.addWidget(self.chunk_spin, row, 1)
        row += 1

        grid.addWidget(self.swap_chk, row, 0, 1, 3)
        row += 1

        stats_grid = QGridLayout()
        stats_grid.addWidget(self.label_fps, 0, 0)
        stats_grid.addWidget(self.label_bps, 0, 1)
        stats_grid.addWidget(self.label_seq, 1, 0)
        stats_grid.addWidget(self.label_peak, 1, 1)

        btn_row = QHBoxLayout()
        btn_row.addWidget(self.btn_start)
        btn_row.addWidget(self.btn_stop)

        vbox = QVBoxLayout()
        vbox.addLayout(grid)
        vbox.addLayout(btn_row)
        vbox.addLayout(stats_grid)
        vbox.addWidget(QLabel("Log"))
        vbox.addWidget(self.log_view)

        root.setLayout(vbox)

        self.refresh_ports()

    def append_log(self, s: str):
        self.log_view.append(s)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = list(list_ports.comports())
        for p in ports:
            self.port_combo.addItem(f"{p.device}  |  {p.description}", p.device)
        if not ports:
            self.port_combo.addItem("(no ports)", "")

    def on_browse(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select Video", "", "Video Files (*.*)")
        if path:
            self.video_edit.setText(path)

    def _get_selected_port(self) -> str:
        data = self.port_combo.currentData()
        return data if isinstance(data, str) else ""

    def on_start(self):
        video_path = self.video_edit.text().strip()
        port = self._get_selected_port().strip()

        if not video_path or not os.path.isfile(video_path):
            QMessageBox.warning(self, "Error", "Please select a valid video file.")
            return
        if not port:
            QMessageBox.warning(self, "Error", "Please select a valid COM port.")
            return

        cfg = StreamConfig(
            port=port,
            video_path=video_path,
            fps_max=float(self.fps_spin.value()),
            chunk=int(self.chunk_spin.value()),
            swap_endian=bool(self.swap_chk.isChecked()),
        )

        self.worker = StreamWorker(cfg)
        self.worker.log.connect(self.append_log)
        self.worker.stats.connect(self.on_stats)
        self.worker.started_ok.connect(self.on_worker_started)
        self.worker.stopped.connect(self.on_worker_stopped)
        self.worker.failed.connect(self.on_worker_failed)

        self.append_log("Starting stream...")
        self.worker.start()

    def on_stop(self):
        if self.worker is not None:
            self.append_log("Stop requested...")
            self.worker.request_stop()
            self.btn_stop.setEnabled(False)

    def on_stats(self, stats: StreamStats):
        self.label_fps.setText(f"FPS: {stats.fps_current:.2f}")
        self.label_bps.setText(f"Bytes/s: {stats.bytes_per_s:.0f}")
        self.label_seq.setText(f"Seq: {stats.seq}")
        self.label_peak.setText(f"Peak EMA: {stats.peak_ema:.3f}")

    def on_worker_started(self):
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.append_log("Streaming started.")

    def on_worker_stopped(self):
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.worker = None
        self.append_log("Worker exited.")

    def on_worker_failed(self, msg: str):
        QMessageBox.critical(self, "Stream Error", msg)
        self.append_log(f"Failed: {msg}")
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.worker = None

    def closeEvent(self, e):
        if self.worker is not None:
            self.worker.request_stop()
            self.worker.wait(1500)
        super().closeEvent(e)


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.resize(820, 520)
    w.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
