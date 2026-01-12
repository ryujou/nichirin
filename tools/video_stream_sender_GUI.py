#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import struct
import subprocess
import threading
import socket
from dataclasses import dataclass

import numpy as np
import serial
from serial.tools import list_ports

try:
    import cv2
except ImportError:
    cv2 = None

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QIODevice
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QPen
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton,
    QComboBox, QSpinBox, QDoubleSpinBox, QCheckBox, QTextEdit, QFileDialog,
    QGroupBox,
    QHBoxLayout, QVBoxLayout, QGridLayout, QMessageBox
)

try:
    from PyQt5.QtMultimedia import QAudioOutput, QAudioFormat
except ImportError:
    QAudioOutput = None
    QAudioFormat = None
MAGIC = b"\xA5\x5A"
TYPE_FRAME_START = 0x01
TYPE_FRAME_DATA = 0x02
TYPE_STOP = 0x03
TYPE_PING = 0x04
TYPE_PONG = 0x05
TYPE_FRAME_SMALL = 0x10
PIX_FMT_RGB565 = 0x00
SIM_HOST = "127.0.0.1"
SIM_PORT = 18080

WIDTH = 80
HEIGHT = 160
FRAME_W = 20
FRAME_H = 40
USB_TX_CHUNK = 512
USB_TX_GAP_S = 0.001

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
    return MAGIC + header + payload


def rgb_to_rgb565(rgb: np.ndarray, swap_endian: bool) -> bytes:
    r = (rgb[:, :, 0] >> 3).astype(np.uint16)
    g = (rgb[:, :, 1] >> 2).astype(np.uint16)
    b = (rgb[:, :, 2] >> 3).astype(np.uint16)
    rgb565 = (r << 11) | (g << 5) | b
    if swap_endian:
        rgb565 = rgb565.byteswap()
    return rgb565.tobytes()


def rgb_to_qimage(frame_rgb: np.ndarray) -> QImage:
    if frame_rgb is None:
        return QImage()
    if not frame_rgb.flags["C_CONTIGUOUS"]:
        frame_rgb = np.ascontiguousarray(frame_rgb)
    h, w, _ = frame_rgb.shape
    bytes_per_line = 3 * w
    qimg = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
    return qimg.copy()


def resize_rgb_nearest(frame_rgb: np.ndarray, dst_w: int, dst_h: int) -> np.ndarray:
    src_h, src_w = frame_rgb.shape[:2]
    if src_h == dst_h and src_w == dst_w:
        return frame_rgb
    x_idx = (np.arange(dst_w) * src_w / dst_w).astype(np.int32)
    y_idx = (np.arange(dst_h) * src_h / dst_h).astype(np.int32)
    return frame_rgb[y_idx[:, None], x_idx[None, :]]


def resize_rgb(frame_rgb: np.ndarray, dst_w: int, dst_h: int) -> np.ndarray:
    if cv2 is not None:
        return cv2.resize(frame_rgb, (dst_w, dst_h), interpolation=cv2.INTER_AREA)
    return resize_rgb_nearest(frame_rgb, dst_w, dst_h)


def rgb565_to_qimage(frame_bytes: bytes, w: int, h: int, swap_endian: bool) -> QImage:
    if not frame_bytes or w <= 0 or h <= 0:
        return QImage()
    arr = np.frombuffer(frame_bytes, dtype=np.uint16)
    if swap_endian:
        arr = arr.byteswap()
    if arr.size < w * h:
        return QImage()
    arr = arr[:w * h].reshape((h, w))
    r = ((arr >> 11) & 0x1F).astype(np.uint8) << 3
    g = ((arr >> 5) & 0x3F).astype(np.uint8) << 2
    b = (arr & 0x1F).astype(np.uint8) << 3
    rgb = np.dstack((r, g, b))
    return rgb_to_qimage(rgb)


def prepare_bgr_frame(
    frame_bgr: np.ndarray,
    swap_endian: bool,
    target_w: int,
    target_h: int,
    rotate_cw: bool,
):
    if rotate_cw:
        frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_90_CLOCKWISE)
    frame = cv2.resize(frame_bgr, (target_w, target_h), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return rgb_to_rgb565(rgb, swap_endian), rgb


def prepare_rgb_frame(
    frame_rgb: np.ndarray,
    swap_endian: bool,
    target_w: int,
    target_h: int,
    rotate_cw: bool,
):
    if rotate_cw:
        frame_rgb = np.rot90(frame_rgb, -1)
    resized = resize_rgb(frame_rgb, target_w, target_h)
    return rgb_to_rgb565(resized, swap_endian), resized


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


def iter_frames_ffmpeg(path: str, width: int, height: int, scale: bool, rotate_cw: bool):
    vf_parts = []
    if rotate_cw:
        vf_parts.append("transpose=1")
    if scale:
        vf_parts.append(f"scale={width}:{height}")
    vf = ",".join(vf_parts) if vf_parts else None
    cmd = [
        "ffmpeg",
        "-i", path,
        "-f", "rawvideo",
        "-pix_fmt", "rgb24",
    ]
    if vf:
        cmd += ["-vf", vf]
    cmd += [
        "-"
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if proc.stderr is not None:
        threading.Thread(target=_drain_stream, args=(proc.stderr,), daemon=True).start()
    frame_size = width * height * 3

    def gen():
        try:
            while True:
                raw = proc.stdout.read(frame_size)
                if raw is None or len(raw) < frame_size:
                    break
                rgb = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 3))
                yield rgb  # RGB
        finally:
            try:
                proc.terminate()
            except Exception:
                pass

    return gen()


def detect_video_size(path: str):
    if not path or not os.path.isfile(path):
        return None
    if cv2 is not None:
        cap = cv2.VideoCapture(path)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if w > 0 and h > 0:
                cap.release()
                return w, h
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None and frame.size:
                h, w = frame.shape[:2]
                return int(w), int(h)
        else:
            cap.release()

    cmd = [
        "ffprobe",
        "-v", "error",
        "-select_streams", "v:0",
        "-show_entries", "stream=width,height",
        "-of", "csv=p=0:s=x",
        path,
    ]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    except FileNotFoundError:
        return None

    text = (proc.stdout or "").strip()
    if "x" in text:
        parts = text.split("x")
        if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
            return int(parts[0]), int(parts[1])
    return None



def detect_video_fps(path: str):
    if not path or not os.path.isfile(path):
        return None
    if cv2 is not None:
        cap = cv2.VideoCapture(path)
        if cap.isOpened():
            fps = float(cap.get(cv2.CAP_PROP_FPS))
            cap.release()
            if fps > 1.0:
                return fps
        else:
            cap.release()

    cmd = [
        "ffprobe",
        "-v", "error",
        "-select_streams", "v:0",
        "-show_entries", "stream=r_frame_rate,avg_frame_rate",
        "-of", "default=nw=1:nk=1",
        path,
    ]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    except FileNotFoundError:
        return None
    for line in (proc.stdout or "").splitlines():
        text = line.strip()
        if not text:
            continue
        if "/" in text:
            num, den = text.split("/", 1)
            try:
                n = float(num)
                d = float(den)
                if d > 0.0:
                    fps = n / d
                    if fps > 1.0:
                        return fps
            except ValueError:
                continue
        else:
            try:
                fps = float(text)
                if fps > 1.0:
                    return fps
            except ValueError:
                continue
    return None



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
        if self.proc.stderr is not None:
            threading.Thread(target=_drain_stream, args=(self.proc.stderr,), daemon=True).start()

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


def _drain_stream(stream):
    try:
        while True:
            chunk = stream.read(4096)
            if not chunk:
                break
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

        bands_val = np.round(self.smooth * 254.0).astype(np.uint8)
        bands_peak = np.round(self.peaks * 254.0).astype(np.uint8)
        return bands_val, bands_peak, self.peak_ema


@dataclass
class StreamConfig:
    port: str
    video_path: str
    fps_max: float
    chunk: int
    swap_endian: bool
    rotate_cw: bool
    frame_width: int
    frame_height: int
    log_tx: bool
    use_udp: bool
    udp_host: str
    udp_port: int
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
            total = 2 + 1 + 2 + 2 + length
            if i + total > len(self.buf):
                break
            payload = bytes(self.buf[i + 7:i + 7 + length])
            yield pkt_type, seq, payload
            i += total
        if i > 0:
            del self.buf[:i]


class StreamWorker(QThread):
    log = pyqtSignal(str)
    started_ok = pyqtSignal()
    stopped = pyqtSignal()
    failed = pyqtSignal(str)
    stats = pyqtSignal(StreamStats)
    frame_ready = pyqtSignal(QImage, object, object)
    audio_ready = pyqtSignal(bytes)

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
        udp_sock = None
        frame_w = int(cfg.frame_width)
        frame_h = int(cfg.frame_height)
        frame_bytes_size = frame_w * frame_h * 2
        log_tx = bool(cfg.log_tx)
        use_udp = bool(cfg.use_udp)
        rotate_cw = bool(cfg.rotate_cw)

        frame_iter = iter_frames_opencv(cfg.video_path)
        use_bgr = True
        if frame_iter is None:
            use_bgr = False
            try:
                src_size = detect_video_size(cfg.video_path)
                if src_size:
                    src_w, src_h = src_size
                    frame_iter = iter_frames_ffmpeg(cfg.video_path, src_w, src_h, False, rotate_cw)
                else:
                    frame_iter = iter_frames_ffmpeg(cfg.video_path, frame_w, frame_h, True, rotate_cw)
                self._log("OpenCV 不可用，改用 ffmpeg 读取视频。")
            except FileNotFoundError:
                self.failed.emit("OpenCV 不可用且未找到 ffmpeg（请检查 PATH）。")
                return

        try:
            audio = AudioReader(cfg.video_path)
        except FileNotFoundError:
            self.failed.emit("未找到 ffmpeg（音频频谱需要）。")
            return

        if not use_udp:
            try:
                ser = serial.Serial(cfg.port, cfg.baud, timeout=0)
            except Exception as e:
                audio.close()
                self.failed.emit(f"打开串口失败: {e}")
                return
        if use_udp:
            try:
                udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            except Exception as e:
                audio.close()
                self.failed.emit(f"打开 UDP 发送失败: {e}")
                return

        parser = PacketParser() if (not use_udp) else None
        analyzer = SpectrumAnalyzer()

        src_fps = detect_video_fps(cfg.video_path)
        if src_fps is None or src_fps <= 1.0:
            fps_current = float(cfg.fps_max)
        else:
            fps_current = float(src_fps)
        frame_interval = 1.0 / max(fps_current, 1.0)
        next_deadline = time.perf_counter()
        last_ping = time.perf_counter()
        bytes_accum = 0
        stats_start = time.perf_counter()

        self.started_ok.emit()
        if use_udp:
            self._log(f"UDP 发送到 {cfg.udp_host}:{cfg.udp_port}")
            self._log(f"开始发送: {frame_w}x{frame_h} RGB565 + 音频频谱。")
        else:
            self._log(f"串口已打开: {cfg.port}, fps_max={cfg.fps_max}, chunk={cfg.chunk}, swap_endian={cfg.swap_endian}")
            self._log(f"开始传输: {frame_w}x{frame_h} RGB565 + 音频频谱。")
        if rotate_cw:
            self._log("竖屏视频：逆时针旋转 90° 转横屏，再缩放。")
        else:
            self._log("横屏视频：直接缩放。")

        try:
            while not self._stop_req:
                for frame in frame_iter:
                    if self._stop_req:
                        self._log("Stop requested, sending STOP...")
                        break

                    frame_begin = time.perf_counter()

                    frame_samples = int(round(SAMPLE_RATE / max(fps_current, 1.0)))
                    samples = audio.read_samples(frame_samples)
                    bands_val, bands_peak, peak_ema = analyzer.process(samples)
                    if samples.size:
                        pcm = np.clip(samples, -1.0, 1.0)
                        pcm = (pcm * 32767.0).astype(np.int16)
                        self.audio_ready.emit(pcm.tobytes())

                    if use_bgr:
                        if frame is None or frame.size == 0:
                            continue
                        preview_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        if rotate_cw:
                            preview_rgb = np.rot90(preview_rgb, -1)
                        frame_bytes, _ = prepare_bgr_frame(
                            frame,
                            cfg.swap_endian,
                            frame_w,
                            frame_h,
                            rotate_cw,
                        )
                    else:
                        preview_rgb = frame
                        if rotate_cw:
                            preview_rgb = np.rot90(preview_rgb, -1)
                        frame_bytes, _ = prepare_rgb_frame(
                            frame,
                            cfg.swap_endian,
                            frame_w,
                            frame_h,
                            rotate_cw,
                        )

                    payload = (
                        struct.pack("<BBB", frame_w, frame_h, PIX_FMT_RGB565)
                        + bytes(bands_val)
                        + frame_bytes
                    )
                    pkt = build_packet(TYPE_FRAME_SMALL, seq, payload)
                    if use_udp:
                        udp_sock.sendto(pkt, (cfg.udp_host, cfg.udp_port))
                    else:
                        for off in range(0, len(pkt), USB_TX_CHUNK):
                            ser.write(pkt[off:off + USB_TX_CHUNK])
                            time.sleep(USB_TX_GAP_S)
                    if log_tx:
                        self._log(f"TX FRAME_SMALL {len(pkt)}B: {pkt[:48].hex()}")
                    bytes_accum += len(pkt)

                    if self._stop_req:
                        self._log("Stop requested, sending STOP...")
                        break

                    preview = rgb_to_qimage(preview_rgb)
                    self.frame_ready.emit(preview, bands_val, bands_peak)

                    if not use_udp and time.perf_counter() - last_ping >= 1.0:
                        last_ping = time.perf_counter()

                    seq = (seq + 1) & 0xFFFF

                    tx_time = time.perf_counter() - frame_begin

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

                if self._stop_req:
                    break
                self._log("Looping video playback...")
                if audio is not None:
                    audio.close()
                audio = AudioReader(cfg.video_path)
                if use_bgr:
                    frame_iter = iter_frames_opencv(cfg.video_path)
                    if frame_iter is None:
                        use_bgr = False
                if not use_bgr:
                    src_size = detect_video_size(cfg.video_path)
                    if src_size:
                        src_w, src_h = src_size
                        frame_iter = iter_frames_ffmpeg(cfg.video_path, src_w, src_h, False, rotate_cw)
                    else:
                        frame_iter = iter_frames_ffmpeg(cfg.video_path, frame_w, frame_h, True, rotate_cw)
                next_deadline = time.perf_counter()

        except Exception as e:
            self._log(f"传输错误: {e}")

        if ser is not None:
            try:
                stop_pkt = build_packet(TYPE_STOP, seq, b"")
                ser.write(stop_pkt)
                if log_tx:
                    self._log(f"TX STOP {len(stop_pkt)}B: {stop_pkt[:48].hex()}")
            except Exception:
                pass

            try:
                ser.close()
            except Exception:
                pass
        if udp_sock is not None:
            try:
                stop_pkt = build_packet(TYPE_STOP, seq, b"")
                udp_sock.sendto(stop_pkt, (cfg.udp_host, cfg.udp_port))
                if log_tx:
                    self._log(f"TX STOP {len(stop_pkt)}B: {stop_pkt[:48].hex()}")
            except Exception:
                pass
            try:
                udp_sock.close()
            except Exception:
                pass
        audio.close()

        self._log("传输已停止。")
        self.stopped.emit()


class LedStripWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.levels = np.zeros(12, dtype=np.uint8)
        self.peaks = np.zeros(12, dtype=np.uint8)
        self.setMinimumHeight(80)

    def set_levels(self, levels, peaks=None):
        if levels is None:
            return
        self.levels = np.array(levels, dtype=np.uint8).flatten()
        if peaks is not None:
            self.peaks = np.array(peaks, dtype=np.uint8).flatten()
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(14, 14, 14))
        n = 12
        w = self.width()
        h = self.height()
        gap = 8
        led_size = int((w - gap * (n + 1)) / n)
        led_size = max(6, min(led_size, h - 16))
        y = int((h - led_size) / 2)

        edge_pen = QPen(QColor(30, 30, 30))
        edge_pen.setWidth(1)
        painter.setPen(edge_pen)

        for i in range(n):
            x = gap + i * (led_size + gap)
            level = int(self.levels[i]) if i < self.levels.size else 0
            level = max(0, min(level, 254))
            color = QColor(0, level, 0)
            painter.setBrush(color)
            painter.drawEllipse(x, y, led_size, led_size)


class AudioBufferDevice(QIODevice):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._buffer = bytearray()
        self._lock = threading.Lock()
        self._max_bytes = SAMPLE_RATE * 2 * 2

    def start(self):
        self.open(QIODevice.ReadOnly)

    def stop(self):
        self.close()
        self.clear()

    def clear(self):
        with self._lock:
            self._buffer.clear()

    def write_data(self, data: bytes):
        if not data:
            return
        with self._lock:
            self._buffer.extend(data)
            if len(self._buffer) > self._max_bytes:
                extra = len(self._buffer) - self._max_bytes
                del self._buffer[:extra]

    def readData(self, maxlen):
        with self._lock:
            if not self._buffer:
                return bytes(maxlen)
            n = min(maxlen, len(self._buffer))
            data = self._buffer[:n]
            del self._buffer[:n]
        if n < maxlen:
            data += b"\x00" * (maxlen - n)
        return bytes(data)

    def bytesAvailable(self):
        with self._lock:
            return len(self._buffer) + super().bytesAvailable()


class AudioPlayer:
    def __init__(self, parent=None):
        self.available = QAudioOutput is not None and QAudioFormat is not None
        self._device = None
        self._output = None
        if not self.available:
            return
        fmt = QAudioFormat()
        fmt.setSampleRate(SAMPLE_RATE)
        fmt.setChannelCount(1)
        fmt.setSampleSize(16)
        fmt.setCodec("audio/pcm")
        fmt.setByteOrder(QAudioFormat.LittleEndian)
        fmt.setSampleType(QAudioFormat.SignedInt)
        self._device = AudioBufferDevice(parent)
        self._output = QAudioOutput(fmt, parent)
        self._output.setVolume(1.0)

    def start(self):
        if not self.available:
            return
        if self._device.isOpen():
            return
        self._device.start()
        self._output.start(self._device)

    def stop(self):
        if not self.available:
            return
        self._output.stop()
        self._device.stop()

    def write(self, data: bytes):
        if not self.available:
            return
        self._device.write_data(data)


class SimulatorReceiver(QThread):
    log = pyqtSignal(str)
    frame_ready = pyqtSignal(int, int, bytes, object, object)

    def __init__(self, host: str, port: int, parent=None):
        super().__init__(parent)
        self.host = host
        self.port = port
        self._stop_req = False

    def request_stop(self):
        self._stop_req = True

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((self.host, self.port))

        except Exception as e:
            self.log.emit(f"模拟器绑定失败: {e}")
            sock.close()
            return
        sock.settimeout(0.2)
        parser = PacketParser()
        cur_seq = None
        expected = 0
        buf = bytearray()
        bands_val = np.zeros(12, dtype=np.uint8)
        bands_peak = np.zeros(12, dtype=np.uint8)
        frame_w = 0
        frame_h = 0

        self.log.emit(f"模拟器监听 {self.host}:{self.port}")
        try:
            while not self._stop_req:
                try:
                    data, _ = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                if not data:
                    continue
                parser.feed(data)
                for pkt_type, seq, payload in parser.iter_packets():
                    if pkt_type == TYPE_FRAME_START and len(payload) >= 4 + 12 + 12 + 1:
                        frame_w = payload[0]
                        frame_h = payload[1]
                        fmt = payload[2]
                        expected = payload[3] | (payload[4] << 8)
                        if fmt != PIX_FMT_RGB565 or expected <= 0:
                            cur_seq = None
                            buf.clear()
                            continue
                        bands_val = np.frombuffer(payload[5:17], dtype=np.uint8).copy()
                        bands_peak = np.frombuffer(payload[17:29], dtype=np.uint8).copy()
                        cur_seq = seq
                        buf.clear()
                    elif pkt_type == TYPE_FRAME_DATA and cur_seq is not None and seq == cur_seq:
                        buf.extend(payload)
                        if expected and len(buf) >= expected:
                            frame_bytes = bytes(buf[:expected])
                            self.frame_ready.emit(frame_w, frame_h, frame_bytes, bands_val, bands_peak)
                            buf.clear()
        finally:
            sock.close()


class SimulatorWindow(QMainWindow):
    def __init__(self, host: str, port: int, parent=None):
        super().__init__(parent)
        self.setWindowTitle("LCD/LED 模拟器")
        self.resize(480, 640)
        self.receiver = SimulatorReceiver(host, port)
        self.receiver.log.connect(self.append_log)
        self.receiver.frame_ready.connect(self.on_frame_ready)

        root = QWidget()
        self.setCentralWidget(root)

        self.lcd_label = QLabel("等待数据")
        self.lcd_label.setAlignment(Qt.AlignCenter)
        self.lcd_label.setFixedSize(WIDTH * 2, HEIGHT * 2)
        self.lcd_label.setStyleSheet("background: #000; border: 1px solid #333;")
        self.led_widget = LedStripWidget()
        self.swap_chk = QCheckBox("RGB565 字节交换")
        self.swap_chk.setChecked(True)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)

        vbox = QVBoxLayout()
        vbox.addWidget(QLabel("LCD 模拟器"))
        vbox.addWidget(self.lcd_label)
        vbox.addWidget(self.swap_chk)
        vbox.addWidget(QLabel("LED 频谱"))
        vbox.addWidget(self.led_widget)
        vbox.addWidget(QLabel("日志"))
        vbox.addWidget(self.log_view)
        root.setLayout(vbox)

    def append_log(self, s: str):
        self.log_view.append(s)

    def start(self):
        if not self.receiver.isRunning():
            self.receiver.start()

    def closeEvent(self, e):
        if self.receiver.isRunning():
            self.receiver.request_stop()
            self.receiver.wait(800)
        super().closeEvent(e)

    def on_frame_ready(self, w: int, h: int, frame_bytes: bytes, bands_val, bands_peak):
        qimg = rgb565_to_qimage(frame_bytes, w, h, self.swap_chk.isChecked())
        pixmap = QPixmap.fromImage(qimg)
        if not pixmap.isNull():
            if w > h:
                self.lcd_label.setFixedSize(HEIGHT * 2, WIDTH * 2)
            else:
                self.lcd_label.setFixedSize(WIDTH * 2, HEIGHT * 2)
            target = self.lcd_label.size()
            self.lcd_label.setPixmap(
                pixmap.scaled(target, Qt.KeepAspectRatio, Qt.FastTransformation)
            )
        self.led_widget.set_levels(bands_val, bands_peak)


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
        self.setWindowTitle("STM32 视频+频谱 传输工具 (USB CDC)")
        self.worker = None

        root = QWidget()
        self.setCentralWidget(root)

        self.video_edit = DragDropLineEdit()
        self.video_edit.setPlaceholderText("拖拽视频文件或点击浏览...")
        btn_browse = QPushButton("浏览")
        btn_browse.clicked.connect(self.on_browse)

        self.port_combo = QComboBox()
        self.btn_refresh = QPushButton("刷新串口")
        self.btn_refresh.clicked.connect(self.refresh_ports)

        self.fps_spin = QDoubleSpinBox()
        self.fps_spin.setRange(6.0, 60.0)
        self.fps_spin.setDecimals(1)
        self.fps_spin.setValue(60.0)

        self.chunk_spin = QSpinBox()
        self.chunk_spin.setRange(256, 4096)
        self.chunk_spin.setSingleStep(256)
        self.chunk_spin.setValue(1024)

        self.swap_chk = QCheckBox("RGB565 字节交换 (MSB 优先)")
        self.swap_chk.setChecked(True)

        self.log_tx_chk = QCheckBox("显示发送数据（截断）")
        self.btn_sim = QPushButton("打开模拟器窗口")
        self.btn_sim.clicked.connect(self.on_open_simulator)

        self.btn_reset = QPushButton("重置")
        self.btn_reset.clicked.connect(self.on_reset)

        self.btn_start = QPushButton("开始")
        self.btn_stop = QPushButton("停止")
        self.btn_stop.setEnabled(False)
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)

        self.label_fps = QLabel("帧率: --")
        self.label_bps = QLabel("字节/秒: --")
        self.label_seq = QLabel("序号: --")
        self.label_peak = QLabel("峰值 EMA: --")

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)

        self.preview_label = QLabel("等待播放")
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setFixedSize(WIDTH * 3, HEIGHT * 3)
        self.preview_label.setStyleSheet(
            "background: #111; border: 1px solid #333; border-radius: 6px;"
        )

        self.lcd_label = None
        self.led_widget = None
        self.audio_player = AudioPlayer(self)

        grid = QGridLayout()
        row = 0
        grid.addWidget(QLabel("视频文件"), row, 0)
        grid.addWidget(self.video_edit, row, 1)
        grid.addWidget(btn_browse, row, 2)
        row += 1

        grid.addWidget(QLabel("串口"), row, 0)
        grid.addWidget(self.port_combo, row, 1)
        grid.addWidget(self.btn_refresh, row, 2)
        row += 1

        grid.addWidget(QLabel("最大 FPS"), row, 0)
        grid.addWidget(self.fps_spin, row, 1)
        row += 1

        grid.addWidget(QLabel("分包大小 (字节)"), row, 0)
        grid.addWidget(self.chunk_spin, row, 1)
        row += 1

        grid.addWidget(self.swap_chk, row, 0, 1, 3)
        row += 1

        grid.addWidget(self.log_tx_chk, row, 0, 1, 3)
        row += 1

        grid.addWidget(self.btn_sim, row, 0, 1, 3)
        row += 1

        grid.addWidget(self.btn_reset, row, 0, 1, 3)
        row += 1

        stats_grid = QGridLayout()
        stats_grid.addWidget(self.label_fps, 0, 0)
        stats_grid.addWidget(self.label_bps, 0, 1)
        stats_grid.addWidget(self.label_seq, 1, 0)
        stats_grid.addWidget(self.label_peak, 1, 1)

        btn_row = QHBoxLayout()
        btn_row.addWidget(self.btn_start)
        btn_row.addWidget(self.btn_stop)

        left_col = QVBoxLayout()
        left_col.addLayout(grid)
        left_col.addLayout(btn_row)
        left_col.addLayout(stats_grid)
        left_col.addWidget(QLabel("日志"))
        left_col.addWidget(self.log_view)

        preview_group = QGroupBox("同步播放")
        preview_group.setStyleSheet(
            "QGroupBox{font-weight:600; margin-top:6px;}"
            "QGroupBox::title{subcontrol-origin: margin; left:10px; padding:0 6px;}"
        )
        preview_layout = QVBoxLayout()
        preview_layout.setContentsMargins(16, 16, 16, 16)
        preview_layout.addWidget(self.preview_label, alignment=Qt.AlignCenter)
        preview_group.setLayout(preview_layout)

        right_col = QVBoxLayout()
        right_col.setContentsMargins(12, 8, 12, 8)
        right_col.addWidget(preview_group)
        right_col.addStretch(1)

        main_layout = QHBoxLayout()
        main_layout.addLayout(left_col, 2)
        main_layout.addLayout(right_col, 1)

        root.setLayout(main_layout)

        self.refresh_ports()
        self._update_orientation_ui()
        self._auto_timer = QTimer(self)
        self._auto_timer.setSingleShot(True)
        self.video_edit.textChanged.connect(self.on_video_path_changed)
        self.sim_window = None

    def append_log(self, s: str):
        self.log_view.append(s)

    def _update_orientation_ui(self):
        self.preview_label.setFixedSize(HEIGHT * 3, WIDTH * 3)
        if self.worker is None:
            self._clear_previews("等待播放")

    def on_video_path_changed(self, text: str):
        if self.worker is not None:
            return
        if not text or not os.path.isfile(text):
            return
        self._reset_ui(keep_video=True)

    def _clear_previews(self, text: str):
        self.preview_label.setPixmap(QPixmap())
        self.preview_label.setText(text)

    def _reset_ui(self, keep_video: bool = False):
        if self.worker is not None:
            self.worker.request_stop()
            self.worker.wait(1000)
            self.worker = None
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.port_combo.setEnabled(True)
        self.btn_refresh.setEnabled(True)
        self.log_tx_chk.setEnabled(True)
        if not keep_video:
            self.video_edit.setText("")
        self.fps_spin.setValue(60.0)
        self.chunk_spin.setValue(1024)
        self.swap_chk.setChecked(True)
        self.log_tx_chk.setChecked(False)
        self.label_fps.setText("帧率: --")
        self.label_bps.setText("字节/秒: --")
        self.label_seq.setText("序号: --")
        self.label_peak.setText("峰值 EMA: --")
        self.log_view.clear()
        self._update_orientation_ui()
        self._clear_previews("等待播放")

    def on_open_simulator(self):
        if self.sim_window is None:
            self.sim_window = SimulatorWindow(SIM_HOST, SIM_PORT, self)
        self.sim_window.show()
        self.sim_window.raise_()
        self.sim_window.start()

    def on_reset(self):
        self._reset_ui(keep_video=False)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = list(list_ports.comports())
        for p in ports:
            self.port_combo.addItem(f"{p.device}  |  {p.description}", p.device)
        self.port_combo.addItem("模拟器 (本地 UDP)", "__SIM__")
        if not ports:
            self.port_combo.addItem("(无串口)", "")

    def on_browse(self):
        path, _ = QFileDialog.getOpenFileName(self, "选择视频文件", "", "视频文件 (*.*)")
        if path:
            self.video_edit.setText(path)

    def _get_selected_port(self) -> str:
        data = self.port_combo.currentData()
        return data if isinstance(data, str) else ""

    def on_start(self):
        video_path = self.video_edit.text().strip()
        port = self._get_selected_port().strip()

        if not video_path or not os.path.isfile(video_path):
            QMessageBox.warning(self, "错误", "请选择有效的视频文件。")
            return
        size = detect_video_size(video_path)
        portrait = False
        if size:
            w, h = size
            portrait = h > w
            orient = "竖屏" if portrait else "横屏"
            self.append_log(f"自动识别视频方向: {w}x{h} -> {orient}")
        else:
            self.append_log("无法自动识别视频方向，默认按横屏处理。")
        if not port:
            QMessageBox.warning(self, "错误", "请选择有效的串口。")
            return
        frame_w = FRAME_W
        frame_h = FRAME_H
        use_udp = (port == "__SIM__")
        if use_udp:
            self.on_open_simulator()

        cfg = StreamConfig(
            port=port,
            video_path=video_path,
            fps_max=float(self.fps_spin.value()),
            chunk=int(self.chunk_spin.value()),
            swap_endian=bool(self.swap_chk.isChecked()),
            rotate_cw=not portrait,
            frame_width=frame_w,
            frame_height=frame_h,
            log_tx=bool(self.log_tx_chk.isChecked()),
            use_udp=use_udp,
            udp_host=SIM_HOST,
            udp_port=SIM_PORT,
        )

        self.worker = StreamWorker(cfg)
        self.worker.log.connect(self.append_log)
        self.worker.stats.connect(self.on_stats)
        self.worker.frame_ready.connect(self.on_frame_ready)
        self.worker.audio_ready.connect(self.on_audio_ready)
        self.worker.started_ok.connect(self.on_worker_started)
        self.worker.stopped.connect(self.on_worker_stopped)
        self.worker.failed.connect(self.on_worker_failed)

        self.append_log("开始传输...")
        self.worker.start()

    def on_stop(self):
        if self.worker is not None:
            self.append_log("已请求停止...")
            self.worker.request_stop()
            self.btn_stop.setEnabled(False)

    def on_stats(self, stats: StreamStats):
        self.label_fps.setText(f"帧率: {stats.fps_current:.2f}")
        self.label_bps.setText(f"字节/秒: {stats.bytes_per_s:.0f}")
        self.label_seq.setText(f"序号: {stats.seq}")
        self.label_peak.setText(f"峰值 EMA: {stats.peak_ema:.3f}")

    def on_frame_ready(self, qimg: QImage, bands_val, bands_peak):
        pixmap = QPixmap.fromImage(qimg)
        if not pixmap.isNull():
            preview_size = self.preview_label.size()
            if not preview_size.isEmpty():
                self.preview_label.setPixmap(
                    pixmap.scaled(preview_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                )
        _ = (bands_val, bands_peak)

    def on_audio_ready(self, data: bytes):
        if self.audio_player is not None and self.audio_player.available:
            self.audio_player.write(data)

    def on_worker_started(self):
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.port_combo.setEnabled(False)
        self.btn_refresh.setEnabled(False)
        self.log_tx_chk.setEnabled(False)
        if self.audio_player is not None and self.audio_player.available:
            self.audio_player.start()
        else:
            self.append_log("QtMultimedia 不可用，无法播放声音。")
        self.append_log("传输已开始。")
        self._clear_previews("播放中...")

    def on_worker_stopped(self):
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.log_tx_chk.setEnabled(True)
        self.worker = None
        if self.audio_player is not None and self.audio_player.available:
            self.audio_player.stop()
        self.append_log("传输线程已退出。")
        self._clear_previews("已停止")

    def on_worker_failed(self, msg: str):
        QMessageBox.critical(self, "传输错误", msg)
        self.append_log(f"失败: {msg}")
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.log_tx_chk.setEnabled(True)
        self.worker = None
        if self.audio_player is not None and self.audio_player.available:
            self.audio_player.stop()
        self._clear_previews("错误")

    def closeEvent(self, e):
        if self.worker is not None:
            self.worker.request_stop()
            self.worker.wait(1500)
        if self.sim_window is not None:
            self.sim_window.close()
        super().closeEvent(e)


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.resize(1120, 680)
    w.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
