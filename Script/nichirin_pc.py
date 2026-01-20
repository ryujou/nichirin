#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
12-band spectrum UART sender with:
- Mic spectrum (sounddevice)
- File playback (QMediaPlayer) + File spectrum (ffmpeg decode)
- File: FULL pre-analysis of whole track BEFORE enabling Play
  -> locks genre + per-band EQ reference to avoid drifting mid-song
  -> runtime keeps only GLOBAL AGC (volume tracking)
- Drag & drop media files
- Manual play (open does NOT auto-play)
- Default send rate: 400Hz

UART frame (16 bytes):
  [ADDR=0x01][FUNC=0x20][12*U8 bands][CRC16_L][CRC16_H]
CRC16: Modbus RTU (poly 0xA001, init 0xFFFF), little-endian appended.
"""

import os
import sys
import time
import math
import struct
import subprocess
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import serial
from serial.tools import list_ports
import sounddevice as sd

from PyQt6.QtCore import Qt, QThread, pyqtSignal, QUrl
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QComboBox,
    QDoubleSpinBox, QSpinBox, QTextEdit, QHBoxLayout, QVBoxLayout, QGroupBox,
    QMessageBox, QFileDialog, QCheckBox, QSlider
)
from PyQt6.QtMultimedia import QMediaPlayer, QAudioOutput
from PyQt6.QtMultimediaWidgets import QVideoWidget


# ---------------- UART protocol ----------------
ADDR = 0x01
FUNC = 0x20
BANDS = 12
FRAME_LEN = 1 + 1 + BANDS + 2  # 16 bytes

AUDIO_EXT = {".mp3", ".wav", ".flac", ".aac", ".m4a", ".ogg", ".opus"}
VIDEO_EXT = {".mp4", ".mkv", ".avi", ".mov", ".webm", ".wmv"}


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


def build_frame(bands_u8: list[int]) -> bytes:
    if len(bands_u8) != BANDS:
        bands_u8 = (bands_u8 + [0] * BANDS)[:BANDS]
    payload = bytes([ADDR, FUNC] + [int(b) & 0xFF for b in bands_u8])
    crc = crc16_modbus(payload)
    return payload + struct.pack("<H", crc)


# ---------------- Spectrum binning ----------------
def make_log_bins(sr: int, nfft: int, bands: int, fmin: float, fmax: float):
    fmax = min(float(fmax), sr * 0.49)
    fmin = max(10.0, float(fmin))
    edges = np.logspace(np.log10(fmin), np.log10(fmax), bands + 1)
    bin_hz = sr / nfft
    out = []
    for i in range(bands):
        lo = int(edges[i] / bin_hz)
        hi = int(edges[i + 1] / bin_hz)
        lo = max(1, lo)
        hi = max(lo + 1, hi)
        out.append((lo, hi))
    return out


# ---------------- Adaptive processor (lockable genre+EQ, runtime global AGC only) ----------------
class AdaptiveSpectrumProcessor:
    """
    Input: band_db (shape=(12,), 0..80 dB above noise floor)
    Output: u8 list length 12 (0..255)

    Two modes:
    - training (freeze=False): updates genre_ema + band_ema baselines
    - frozen  (freeze=True): uses fixed genre + fixed band_ref, only global AGC tracks volume
    """
    def __init__(self, bands=12):
        self.bands = bands

        # runtime gain control
        self.gain = 1.0
        self.loud_ema = 0.15
        self.last_t = time.perf_counter()

        # training stats
        self.genre_ema = 0.0  # -1..+1 (ACG..BassDJ)
        self.band_ema = np.ones(bands, dtype=np.float32) * 0.25

        # freeze
        self.freeze = False
        self.genre_fixed = 0.0
        self.band_ref_fixed = np.ones(bands, dtype=np.float32) * 0.25

    @staticmethod
    def _spectral_flatness(x, eps=1e-6):
        x = np.clip(x, eps, None)
        gm = math.exp(float(np.mean(np.log(x))))
        am = float(np.mean(x))
        return gm / (am + eps)

    def _genre_score(self, x01):
        x = np.clip(x01, 1e-6, 1.0)
        low = float(np.mean(x[0:3]))
        mid = float(np.mean(x[3:8]))
        high = float(np.mean(x[8:12]))
        allv = float(np.mean(x))

        bass_ratio = low / max(1e-6, (low + mid + high))
        idx = np.arange(self.bands, dtype=np.float32)
        centroid = float(np.sum(idx * x) / np.sum(x))
        flat = self._spectral_flatness(x)

        bassdj = 0.0
        bassdj += (bass_ratio - 0.33) * 2.4
        bassdj += (0.45 - centroid / (self.bands - 1)) * 1.8
        bassdj += (allv - 0.35) * 1.2
        bassdj += (0.35 - flat) * 0.8

        acg = 0.0
        acg += (0.28 - allv) * 2.0
        acg += (0.33 - bass_ratio) * 1.2
        acg += ((centroid / (self.bands - 1)) - 0.5) * 0.8
        acg += (flat - 0.35) * 0.6

        score = bassdj - acg
        return max(-1.0, min(1.0, score))

    def update_training(self, band_db: np.ndarray, dt: float):
        """Update genre_ema and band_ema for offline pre-analysis."""
        dt = max(1e-3, min(0.25, float(dt)))
        x01 = np.clip(np.array(band_db, dtype=np.float32) / 45.0, 0.0, 1.0)

        score = self._genre_score(x01)
        a_genre = 1.0 - math.exp(-dt / 1.0)
        self.genre_ema += (score - self.genre_ema) * a_genre

        g = self.genre_ema
        tau_band = 2.0 - 0.8 * max(0.0, g) + 0.6 * max(0.0, -g)
        a_band = 1.0 - math.exp(-dt / max(0.2, tau_band))
        self.band_ema += (x01 - self.band_ema) * a_band

    def freeze_from_training(self):
        self.genre_fixed = float(self.genre_ema)
        self.band_ref_fixed = np.clip(self.band_ema.copy(), 0.04, 0.8)
        self.freeze = True

        # reset runtime AGC (optional but recommended so it starts stable)
        self.gain = 1.0
        self.loud_ema = 0.15
        self.last_t = time.perf_counter()

    def debug_genre(self) -> float:
        return float(self.genre_fixed if self.freeze else self.genre_ema)

    def process(self, band_db: np.ndarray, dt: Optional[float] = None) -> list[int]:
        now = time.perf_counter()
        if dt is None:
            dt = now - self.last_t
        self.last_t = now
        dt = max(1e-3, min(0.1, float(dt)))

        x01 = np.clip(np.array(band_db, dtype=np.float32) / 45.0, 0.0, 1.0)

        if self.freeze:
            g = float(self.genre_fixed)
            band_ref = self.band_ref_fixed
        else:
            # online adaptive (not used for file playback after pre-analysis)
            score = self._genre_score(x01)
            a_genre = 1.0 - math.exp(-dt / 1.0)
            self.genre_ema += (score - self.genre_ema) * a_genre
            g = self.genre_ema

            tau_band = 2.0 - 0.8 * max(0.0, g) + 0.6 * max(0.0, -g)
            a_band = 1.0 - math.exp(-dt / max(0.2, tau_band))
            self.band_ema += (x01 - self.band_ema) * a_band
            band_ref = np.clip(self.band_ema, 0.04, 0.8)

        # per-band normalization (Auto-EQ baseline)
        y = x01 / np.clip(band_ref, 0.04, 0.8)

        # style tilt based on fixed g
        idx = np.linspace(0.0, 1.0, self.bands, dtype=np.float32)
        tilt = np.ones(self.bands, dtype=np.float32)
        if g > 0.25:
            tilt *= (0.85 + 0.35 * idx)  # DJ: low down, high up
        elif g < -0.25:
            tilt *= (1.15 - 0.25 * idx)  # ACG: low up, high slightly down
        y = y * tilt

        # -------- GLOBAL AGC (volume tracking) --------
        loud = float(np.mean(np.clip(y, 0.0, 3.0)))
        a_loud = 1.0 - math.exp(-dt / 0.5)
        self.loud_ema += (loud - self.loud_ema) * a_loud
        loud_ref = max(0.08, self.loud_ema)

        # target depends on style but fixed for whole track
        target = 0.95 + (-g) * 0.25  # ACG higher, DJ lower
        target = max(0.65, min(1.25, target))

        gain = target / loud_ref
        gain = max(0.35, min(3.5, gain))

        tau_g = 0.08 if gain > self.gain else 0.25
        a_g = 1.0 - math.exp(-dt / tau_g)
        self.gain += (gain - self.gain) * a_g

        y = y * self.gain
        # ---------------------------------------------

        # compression by style (fixed)
        if g < -0.25:
            y = np.sqrt(np.clip(y / 1.6, 0.0, 1.0))
        elif g > 0.25:
            y = np.log1p(2.2 * np.clip(y, 0.0, 3.0)) / np.log1p(2.2 * 1.6)
        else:
            y = np.sqrt(np.clip(y / 1.8, 0.0, 1.0))

        y = np.clip(y, 0.0, 1.0)
        u8 = (y * 255.0 + 0.5).astype(np.int32)
        return np.clip(u8, 0, 255).tolist()


# ---------------- DSP config ----------------
@dataclass
class DspConfig:
    sr: int
    block: int
    nfft: int
    fmin: float
    fmax: float
    noise_floor_db: float


# ---------------- Mic provider ----------------
class MicAudioThread(QThread):
    status = pyqtSignal(str)
    debug = pyqtSignal(float)

    def __init__(self, device_index: int, cfg: DspConfig):
        super().__init__()
        self.dev = int(device_index)
        self.cfg = cfg
        self._stop = False
        self._latest = [0] * BANDS
        self.proc = AdaptiveSpectrumProcessor(BANDS)  # online adaptive is fine for mic

    def request_stop(self):
        self._stop = True

    def get_latest(self) -> list[int]:
        return self._latest

    def run(self):
        self._stop = False
        sr = self.cfg.sr
        nfft = self.cfg.nfft

        bins = make_log_bins(sr, nfft, BANDS, self.cfg.fmin, self.cfg.fmax)
        window = np.hanning(nfft).astype(np.float32)
        ring = np.zeros(nfft, dtype=np.float32)
        ring_pos = 0

        floor_db = float(self.cfg.noise_floor_db)

        def cb(indata, frames, time_info, status):
            nonlocal ring, ring_pos
            if self._stop:
                raise sd.CallbackStop()

            x = indata.astype(np.float32)
            if x.ndim > 1 and x.shape[1] > 1:
                x = np.mean(x, axis=1)
            else:
                x = x.reshape(-1)

            n = x.shape[0]
            if n >= nfft:
                ring[:] = x[-nfft:]
                ring_pos = 0
            else:
                end = ring_pos + n
                if end < nfft:
                    ring[ring_pos:end] = x
                    ring_pos = end
                else:
                    first = nfft - ring_pos
                    ring[ring_pos:] = x[:first]
                    ring[:(n - first)] = x[first:]
                    ring_pos = (n - first)

            buf = ring.copy() if ring_pos == 0 else np.concatenate((ring[ring_pos:], ring[:ring_pos]))
            buf = buf * window

            spec = np.fft.rfft(buf, n=nfft)
            mag2 = (spec.real * spec.real + spec.imag * spec.imag) + 1e-12
            db = 10.0 * np.log10(mag2)
            db = db - floor_db
            db = np.clip(db, 0.0, 80.0)

            band = np.zeros(BANDS, dtype=np.float32)
            for i, (lo, hi) in enumerate(bins):
                hi = min(hi, db.shape[0])
                lo = min(lo, hi - 1)
                band[i] = float(np.max(db[lo:hi])) if hi > lo else float(db[lo])

            self._latest = self.proc.process(band, dt=frames / sr)

        try:
            self.status.emit(f"麦克风频谱启动：dev={self.dev} sr={sr} block={self.cfg.block}")
            with sd.InputStream(
                device=self.dev, channels=1,
                samplerate=sr, blocksize=self.cfg.block,
                dtype="float32", callback=cb
            ):
                last_emit = time.perf_counter()
                while not self._stop:
                    time.sleep(0.05)
                    if time.perf_counter() - last_emit > 0.4:
                        self.debug.emit(self.proc.debug_genre())
                        last_emit = time.perf_counter()
        except Exception as e:
            self.status.emit(f"麦克风频谱失败：{e}")
        finally:
            self.status.emit("麦克风频谱已停止")

class FilePreAnalyzeThread(QThread):
    status = pyqtSignal(str)
    done = pyqtSignal(float, object)  # genre_fixed, band_ref_fixed (np.ndarray)

    # --- silence gate (time-domain RMS, robust) ---
    RMS_SILENCE_TH = 0.004  # adjust 0.003~0.008 if needed

    def __init__(self, file_path: str, cfg: DspConfig):
        super().__init__()
        self.file_path = file_path
        self.cfg = cfg
        self._stop = False

    def request_stop(self):
        self._stop = True

    def _start_ffmpeg(self) -> subprocess.Popen:
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel", "error",
            "-i", self.file_path,
            "-vn",
            "-ac", "1",
            "-ar", str(self.cfg.sr),
            "-f", "s16le",
            "pipe:1",
        ]
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)

    def _read_exact(self, proc: subprocess.Popen, nbytes: int) -> Optional[bytes]:
        if proc.stdout is None:
            return None
        out = bytearray()
        while len(out) < nbytes and not self._stop:
            chunk = proc.stdout.read(nbytes - len(out))
            if chunk:
                out.extend(chunk)
                continue
            if proc.poll() is not None:
                return None
            time.sleep(0.001)
        if len(out) < nbytes:
            return None
        return bytes(out)

    @staticmethod
    def _trimmed_mean(x: np.ndarray, trim: float = 0.10) -> float:
        """Trim both tails and mean. trim=0.10 => drop 10% low + 10% high."""
        if x.size == 0:
            return 0.0
        x = np.sort(x)
        n = x.size
        k = int(n * trim)
        if n - 2 * k <= 1:
            return float(np.mean(x))
        return float(np.mean(x[k:n - k]))

    def run(self):
        sr = self.cfg.sr
        nfft = self.cfg.nfft
        bins = make_log_bins(sr, nfft, BANDS, self.cfg.fmin, self.cfg.fmax)
        window = np.hanning(nfft).astype(np.float32)
        floor_db = float(self.cfg.noise_floor_db)

        # FULL scan: use block as configured (you can increase block in GUI for faster scan)
        block_frames = int(self.cfg.block)
        bytes_per_sample = 2
        read_bytes = block_frames * bytes_per_sample

        # rolling ring buffer for FFT
        ring = np.zeros(nfft, dtype=np.float32)
        ring_pos = 0

        proc: Optional[subprocess.Popen] = None
        analyzer = AdaptiveSpectrumProcessor(BANDS)

        scores = []
        x01_samples = []
        silent_skipped = 0
        blocks = 0
        frames_total = 0

        t0 = time.perf_counter()
        last_msg = time.perf_counter()

        try:
            self.status.emit("预分析：开始（全曲全量扫描，完成后才能播放）")
            proc = self._start_ffmpeg()
            if proc.stdout is None:
                raise RuntimeError("ffmpeg stdout 无法打开")

            while not self._stop:
                raw = self._read_exact(proc, read_bytes)
                if raw is None:
                    break

                x = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
                n = x.shape[0]
                frames_total += n
                blocks += 1

                # ring push
                if n >= nfft:
                    ring[:] = x[-nfft:]
                    ring_pos = 0
                else:
                    end = ring_pos + n
                    if end < nfft:
                        ring[ring_pos:end] = x
                        ring_pos = end
                    else:
                        first = nfft - ring_pos
                        ring[ring_pos:] = x[:first]
                        ring[:(n - first)] = x[first:]
                        ring_pos = (n - first)

                # reconstruct buffer
                buf = ring.copy() if ring_pos == 0 else np.concatenate((ring[ring_pos:], ring[:ring_pos]))

                # ---- RMS silence gate (use time-domain buf) ----
                rms = float(np.sqrt(np.mean(buf * buf) + 1e-12))
                if rms < self.RMS_SILENCE_TH:
                    silent_skipped += 1
                    continue

                # FFT
                bufw = buf * window
                spec = np.fft.rfft(bufw, n=nfft)
                mag2 = (spec.real * spec.real + spec.imag * spec.imag) + 1e-12
                db = 10.0 * np.log10(mag2)
                db = db - floor_db
                db = np.clip(db, 0.0, 80.0)

                band = np.zeros(BANDS, dtype=np.float32)
                for i, (lo, hi) in enumerate(bins):
                    hi = min(hi, db.shape[0])
                    lo = min(lo, hi - 1)
                    band[i] = float(np.max(db[lo:hi])) if hi > lo else float(db[lo])

                x01 = np.clip(band / 45.0, 0.0, 1.0)

                score = analyzer._genre_score(x01)
                scores.append(score)
                x01_samples.append(x01)

                # training update (optional)
                analyzer.update_training(band, dt=block_frames / sr)

                now = time.perf_counter()
                if now - last_msg > 0.6 and len(scores) > 0:
                    sec = frames_total / sr
                    g_preview = self._trimmed_mean(np.array(scores, dtype=np.float32), trim=0.10)
                    self.status.emit(
                        f"预分析中… 已扫 {sec:.1f}s  预估风格 {g_preview:+.2f}  "
                        f"(有效{len(scores)} 跳过静音{silent_skipped})"
                    )
                    last_msg = now

            if self._stop:
                self.status.emit("预分析：已取消")
                return

            if len(scores) < 40:
                # fallback if too few effective blocks
                analyzer.freeze_from_training()
                g_fixed = float(analyzer.genre_fixed)
                ref = analyzer.band_ref_fixed.copy()
            else:
                s = np.array(scores, dtype=np.float32)
                g_fixed = self._trimmed_mean(s, trim=0.10)

                xs = np.stack(x01_samples, axis=0)  # (N,12)
                ref = np.percentile(xs, 60, axis=0).astype(np.float32)
                ref = np.clip(ref, 0.04, 0.8)

            elapsed = time.perf_counter() - t0
            self.status.emit(
                f"预分析完成：耗时 {elapsed:.2f}s  风格锁定 {g_fixed:+.2f}  "
                f"(有效{len(scores)} 跳过静音{silent_skipped})"
            )
            self.done.emit(float(g_fixed), ref)

        except FileNotFoundError:
            self.status.emit("预分析失败：未找到 ffmpeg。请安装并加入 PATH。")
        except Exception as e:
            self.status.emit(f"预分析失败：{e}")
        finally:
            try:
                if proc is not None:
                    proc.kill()
            except Exception:
                pass


# ---------------- File provider (ffmpeg decode audio, locked style) ----------------
class FileAudioThread(QThread):
    status = pyqtSignal(str)
    debug = pyqtSignal(float)

    def __init__(self, file_path: str, cfg: DspConfig, genre_fixed: float, band_ref_fixed: np.ndarray):
        super().__init__()
        self.file_path = file_path
        self.cfg = cfg
        self._stop = False
        self._paused = True
        self._latest = [0] * BANDS
        self._proc: Optional[subprocess.Popen] = None

        self.proc = AdaptiveSpectrumProcessor(BANDS)
        self.proc.genre_ema = float(genre_fixed)
        self.proc.band_ema = np.array(band_ref_fixed, dtype=np.float32)
        self.proc.freeze_from_training()  # lock style+ref, runtime only global AGC

        # timing
        self._t0 = None
        self._pause_t0 = None
        self._paused_total = 0.0
        self._samples_total = 0

    def request_stop(self):
        self._stop = True

    def set_paused(self, paused: bool):
        paused = bool(paused)
        if paused == self._paused:
            return
        self._paused = paused
        now = time.perf_counter()
        if paused:
            self._pause_t0 = now
        else:
            if self._pause_t0 is not None:
                self._paused_total += (now - self._pause_t0)
            self._pause_t0 = None

    def get_latest(self) -> list[int]:
        return self._latest

    def _start_ffmpeg(self) -> subprocess.Popen:
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel", "error",
            "-i", self.file_path,
            "-vn",
            "-ac", "1",
            "-ar", str(self.cfg.sr),
            "-f", "s16le",
            "pipe:1",
        ]
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)

    def _read_exact(self, nbytes: int) -> Optional[bytes]:
        if self._proc is None or self._proc.stdout is None:
            return None
        out = bytearray()
        while len(out) < nbytes and not self._stop:
            chunk = self._proc.stdout.read(nbytes - len(out))
            if chunk:
                out.extend(chunk)
                continue
            if self._proc.poll() is not None:
                return None
            time.sleep(0.001)
        if len(out) < nbytes:
            return None
        return bytes(out)

    def run(self):
        self._stop = False
        sr = self.cfg.sr
        nfft = self.cfg.nfft
        bins = make_log_bins(sr, nfft, BANDS, self.cfg.fmin, self.cfg.fmax)
        window = np.hanning(nfft).astype(np.float32)
        ring = np.zeros(nfft, dtype=np.float32)
        ring_pos = 0

        floor_db = float(self.cfg.noise_floor_db)

        bytes_per_sample = 2
        block_frames = int(self.cfg.block)
        read_bytes = block_frames * bytes_per_sample

        self._t0 = time.perf_counter()
        self._pause_t0 = None
        self._paused_total = 0.0
        self._samples_total = 0

        try:
            self.status.emit(f"文件频谱启动：{os.path.basename(self.file_path)}（风格已锁定 {self.proc.genre_fixed:+.2f}）")
            self._proc = self._start_ffmpeg()
            if self._proc.stdout is None:
                raise RuntimeError("ffmpeg stdout 无法打开")

            last_emit = time.perf_counter()

            while not self._stop:
                if self._paused:
                    time.sleep(0.01)
                    continue

                raw = self._read_exact(read_bytes)
                if raw is None:
                    self._latest = [0] * BANDS
                    break

                x = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0

                n = x.shape[0]
                if n >= nfft:
                    ring[:] = x[-nfft:]
                    ring_pos = 0
                else:
                    end = ring_pos + n
                    if end < nfft:
                        ring[ring_pos:end] = x
                        ring_pos = end
                    else:
                        first = nfft - ring_pos
                        ring[ring_pos:] = x[:first]
                        ring[:(n - first)] = x[first:]
                        ring_pos = (n - first)

                buf = ring.copy() if ring_pos == 0 else np.concatenate((ring[ring_pos:], ring[:ring_pos]))
                buf = buf * window

                spec = np.fft.rfft(buf, n=nfft)
                mag2 = (spec.real * spec.real + spec.imag * spec.imag) + 1e-12
                db = 10.0 * np.log10(mag2)
                db = db - floor_db
                db = np.clip(db, 0.0, 80.0)

                band = np.zeros(BANDS, dtype=np.float32)
                for i, (lo, hi) in enumerate(bins):
                    hi = min(hi, db.shape[0])
                    lo = min(lo, hi - 1)
                    band[i] = float(np.max(db[lo:hi])) if hi > lo else float(db[lo])

                self._latest = self.proc.process(band, dt=block_frames / sr)

                # audio-clock pacing
                self._samples_total += block_frames
                expected = self._samples_total / sr
                now = time.perf_counter()
                real = (now - self._t0) - self._paused_total
                sleep_s = expected - real
                if sleep_s > 0:
                    time.sleep(min(0.02, sleep_s))

                if now - last_emit > 0.4:
                    self.debug.emit(self.proc.debug_genre())
                    last_emit = now

        except FileNotFoundError:
            self.status.emit("文件频谱失败：未找到 ffmpeg。请安装 ffmpeg 并加入 PATH。")
        except Exception as e:
            self.status.emit(f"文件频谱失败：{e}")
        finally:
            try:
                if self._proc is not None:
                    self._proc.kill()
            except Exception:
                pass
            self._proc = None
            self.status.emit("文件频谱已停止")


# ---------------- UART sender thread ----------------
@dataclass
class SenderConfig:
    port: str
    baud: int
    hz: float
    print_frames: bool
    print_every_n: int


class SenderThread(QThread):
    status = pyqtSignal(str)
    stats = pyqtSignal(int, float, int)
    log = pyqtSignal(str)

    def __init__(self, cfg: SenderConfig, provider):
        super().__init__()
        self.cfg = cfg
        self.provider = provider
        self._stop = False

    def request_stop(self):
        self._stop = True

    def run(self):
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


# ---------------- GUI ----------------
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("12段频谱灯 上位机")
        self.setAcceptDrops(True)

        # Media player
        self.video = QVideoWidget()
        self.player = QMediaPlayer(self)
        self.audio_out = QAudioOutput(self)
        self.player.setAudioOutput(self.audio_out)
        self.player.setVideoOutput(self.video)
        self.audio_out.setVolume(1.0)

        self.file_path: Optional[str] = None

        # analysis cache for current file
        self.file_genre_fixed: Optional[float] = None
        self.file_bandref_fixed: Optional[np.ndarray] = None
        self.pre_thread: Optional[FilePreAnalyzeThread] = None

        # Media controls (manual play)
        self.open_btn = QPushButton("打开音频/视频…")
        self.play_pause_btn = QPushButton("播放")
        self.stop_media_btn = QPushButton("停止")
        self.play_pause_btn.setEnabled(False)
        self.stop_media_btn.setEnabled(False)

        # Media progress
        self.progress = QSlider(Qt.Orientation.Horizontal)
        self.progress.setRange(0, 0)
        self.progress.setEnabled(False)
        self.progress_time = QLabel("00:00 / 00:00")
        self._seeking = False

        # Serial UI
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

        # Source select
        self.source_box = QComboBox()
        self.source_box.addItems([
            "麦克风 (Mic)",
            "文件 (File)",
        ])

        # Mic devices
        self.audio_dev_box = QComboBox()
        self.refresh_audio_btn = QPushButton("刷新麦克风")

        # DSP params
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

        # Run controls
        self.start_btn = QPushButton("开始")
        self.stop_btn = QPushButton("停止发送")
        self.stop_btn.setEnabled(False)

        # Status/log
        self.status_label = QLabel("未开始")
        self.status_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.genre_label = QLabel("风格：0.00")
        self.genre_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)

        # Threads
        self.provider_thread = None
        self.sender_thread: Optional[SenderThread] = None

        # Layout
        media_box = QGroupBox("文件播放")
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

        stat_box = QGroupBox("自适应状态")
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

        # Signals
        self.refresh_port_btn.clicked.connect(self.refresh_ports)
        self.refresh_audio_btn.clicked.connect(self.refresh_mic_devices)
        self.source_box.currentIndexChanged.connect(self.on_source_changed)

        self.open_btn.clicked.connect(self.open_file)
        self.play_pause_btn.clicked.connect(self.toggle_play_pause)
        self.stop_media_btn.clicked.connect(self.stop_media)

        self.start_btn.clicked.connect(self.start_all)
        self.stop_btn.clicked.connect(self.stop_all)

        self.player.playbackStateChanged.connect(self.on_player_state)
        self.player.durationChanged.connect(self.on_duration_changed)
        self.player.positionChanged.connect(self.on_position_changed)
        self.progress.sliderPressed.connect(self.on_slider_pressed)
        self.progress.sliderReleased.connect(self.on_slider_released)
        self.progress.sliderMoved.connect(self.on_slider_moved)

        # Initial
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
        self.port_box.clear()
        ports = list_ports.comports()

        def _port_priority(p):
            desc = (p.description or "").lower()
            name = (p.device or "").lower()
            hwid = (getattr(p, "hwid", "") or "").lower()

            # prefer common USB-UART chips
            chip_keywords = (
                "ch340", "ch341", "ch9102", "cp210", "cp2102", "cp210x",
                "ft232", "ftdi", "pl2303",
            )
            if any(k in desc or k in name or k in hwid for k in chip_keywords):
                return (0, name)

            # prefer bluetooth serial ports
            bt_keywords = (
                "bluetooth", "rfcomm", "serial over bluetooth",
            )
            if any(k in desc for k in bt_keywords):
                return (1, name)

            # de-prioritize empty/legacy com1 style
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
        is_file = (self.source_box.currentIndex() == 1)

        self.audio_dev_box.setEnabled(not is_file)
        self.refresh_audio_btn.setEnabled(not is_file)

        # file play controls depend on analysis completion
        has_file = self.file_path is not None
        analyzed = (self.file_genre_fixed is not None and self.file_bandref_fixed is not None)
        self.play_pause_btn.setEnabled(is_file and has_file and analyzed)
        self.stop_media_btn.setEnabled(is_file and has_file)

    # ---- configs ----
    def _make_dsp_cfg(self) -> DspConfig:
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
        fn, _ = QFileDialog.getOpenFileName(
            self, "选择音频或视频文件", "",
            "Media Files (*.mp3 *.wav *.flac *.aac *.m4a *.ogg *.opus *.mp4 *.mkv *.avi *.mov *.webm *.wmv);;All Files (*.*)"
        )
        if fn:
            self.load_media(fn)

    def _cancel_preanalysis(self):
        if self.pre_thread is not None:
            try:
                self.pre_thread.request_stop()
                self.pre_thread.wait(1500)
            except Exception:
                pass
            self.pre_thread = None

    def load_media(self, path: str):
        ext = os.path.splitext(path)[1].lower()
        if ext not in AUDIO_EXT and ext not in VIDEO_EXT:
            QMessageBox.warning(self, "提示", f"不支持的文件类型：{ext}")
            return

        # stop any running stuff (but do not stop UART if user wants; simpler: stop all)
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

        # switch to file mode
        self.source_box.setCurrentIndex(1)
        self.on_source_changed()

        self.set_status(f"已加载：{os.path.basename(path)}（将先预分析整首，完成后才能播放）")
        self.genre_label.setText("风格：分析中…")

        # start full pre-analysis
        dsp = self._make_dsp_cfg()
        self.pre_thread = FilePreAnalyzeThread(path, dsp)
        self.pre_thread.status.connect(self.set_status)
        self.pre_thread.done.connect(self.on_preanalysis_done)
        self.pre_thread.start()

        # disable play until analysis done
        self.play_pause_btn.setEnabled(False)

    def on_preanalysis_done(self, genre_fixed: float, band_ref_fixed: object):
        # store
        self.file_genre_fixed = float(genre_fixed)
        self.file_bandref_fixed = np.array(band_ref_fixed, dtype=np.float32)
        self.genre_label.setText(f"风格：{self.file_genre_fixed:+.2f}")
        self.set_status("预分析完成：现在可以点击“播放”开始")
        self.on_source_changed()

    def toggle_play_pause(self):
        st = self.player.playbackState()
        if st == QMediaPlayer.PlaybackState.PlayingState:
            self.player.pause()
        else:
            self.player.play()

    def stop_media(self):
        self.player.stop()
        self.play_pause_btn.setText("播放")
        if isinstance(self.provider_thread, FileAudioThread):
            self.provider_thread.set_paused(True)
        self.progress.setValue(0)

    def on_player_state(self, state):
        # if we are in file mode and provider exists, sync pause state
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
        s = max(0, int(ms // 1000))
        m = s // 60
        s = s % 60
        return f"{m:02d}:{s:02d}"

    def on_duration_changed(self, duration_ms: int):
        self.progress.setRange(0, max(0, int(duration_ms)))
        pos = int(self.player.position())
        self.progress_time.setText(f"{self._fmt_ms(pos)} / {self._fmt_ms(duration_ms)}")

    def on_position_changed(self, position_ms: int):
        if not self._seeking:
            self.progress.setValue(int(position_ms))
        dur = int(self.player.duration())
        self.progress_time.setText(f"{self._fmt_ms(position_ms)} / {self._fmt_ms(dur)}")

    def on_slider_pressed(self):
        self._seeking = True

    def on_slider_released(self):
        self._seeking = False
        self.player.setPosition(int(self.progress.value()))

    def on_slider_moved(self, value: int):
        dur = int(self.player.duration())
        self.progress_time.setText(f"{self._fmt_ms(value)} / {self._fmt_ms(dur)}")

    # ---- start/stop ----
    def start_all(self):
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
            self.provider_thread.set_paused(self.player.playbackState() != QMediaPlayer.PlaybackState.PlayingState)
            self.provider_thread.start()

        self.sender_thread = SenderThread(scfg, self.provider_thread)
        self.sender_thread.status.connect(self.set_status)
        self.sender_thread.log.connect(self.log)
        self.sender_thread.stats.connect(self.on_stats)
        self.sender_thread.start()

        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

        # lock settings while running
        for w in (
            self.refresh_port_btn, self.port_box, self.baud_box, self.hz_box,
            self.source_box,
            self.audio_dev_box, self.refresh_audio_btn,
            self.sr_box, self.block_box, self.floor_db,
            self.print_box, self.print_every,
            self.open_btn
        ):
            w.setEnabled(False)

        # allow play/pause/stop in file mode
        self.on_source_changed()

    def stop_all(self):
        # stop sender/provider
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
        self.status_label.setText(f"运行中：{sent} 帧 / {elapsed_s:.1f}s / 写异常 {err}")

    def on_genre(self, g: float):
        # mic shows online g; file shows fixed g
        self.genre_label.setText(f"风格：{g:+.2f}  (-1=ACG  +1=低音DJ)")

    def closeEvent(self, event):
        try:
            self.stop_all()
            self._cancel_preanalysis()
        finally:
            event.accept()


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.resize(1220, 900)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
