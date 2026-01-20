#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
音频输入/文件解码线程。

包含：
- 麦克风实时频谱采集
- 文件预分析（整首扫描，锁定风格与参考曲线）
- 文件播放期频谱提取（带暂停/继续）
"""

from __future__ import annotations

import os
import time
import subprocess
from typing import Optional

import numpy as np
import sounddevice as sd
from PyQt6.QtCore import QThread, pyqtSignal

from dsp import DspConfig, AdaptiveSpectrumProcessor, make_log_bins
from pc_common import BANDS


class MicAudioThread(QThread):
    """麦克风实时频谱采集线程。"""

    status = pyqtSignal(str)
    debug = pyqtSignal(float)

    def __init__(self, device_index: int, cfg: DspConfig):
        super().__init__()
        self.dev = int(device_index)
        self.cfg = cfg
        self._stop = False
        self._latest = [0] * BANDS
        self.proc = AdaptiveSpectrumProcessor(BANDS)  # 麦克风使用在线自适应

    def request_stop(self):
        """请求停止线程。"""
        self._stop = True

    def get_latest(self) -> list[int]:
        """获取最新频谱（12 段 0..255）。"""
        return self._latest

    def run(self):
        """线程主循环：开启音频输入流并计算频谱。"""
        self._stop = False
        sr = self.cfg.sr
        nfft = self.cfg.nfft

        bins = make_log_bins(sr, nfft, BANDS, self.cfg.fmin, self.cfg.fmax)
        window = np.hanning(nfft).astype(np.float32)
        ring = np.zeros(nfft, dtype=np.float32)
        ring_pos = 0

        floor_db = float(self.cfg.noise_floor_db)

        def cb(indata, frames, time_info, status):
            """sounddevice 回调：从麦克风缓冲区生成频谱。"""
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
    """文件预分析线程：整首扫描并锁定风格与参考曲线。"""

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
        """请求停止预分析。"""
        self._stop = True

    def _start_ffmpeg(self) -> subprocess.Popen:
        """启动 ffmpeg 解码输出为 s16le PCM。"""
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
        """从 ffmpeg stdout 精确读取 nbytes。"""
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
        """双侧裁剪均值：trim=0.10 => 去掉两端 10%。"""
        if x.size == 0:
            return 0.0
        x = np.sort(x)
        n = x.size
        k = int(n * trim)
        if n - 2 * k <= 1:
            return float(np.mean(x))
        return float(np.mean(x[k:n - k]))

    def run(self):
        """线程主流程：全曲扫描后给出风格与参考曲线。"""
        sr = self.cfg.sr
        nfft = self.cfg.nfft
        bins = make_log_bins(sr, nfft, BANDS, self.cfg.fmin, self.cfg.fmax)
        window = np.hanning(nfft).astype(np.float32)
        floor_db = float(self.cfg.noise_floor_db)

        block_frames = int(self.cfg.block)
        bytes_per_sample = 2
        read_bytes = block_frames * bytes_per_sample

        ring = np.zeros(nfft, dtype=np.float32)
        ring_pos = 0

        proc: Optional[subprocess.Popen] = None
        analyzer = AdaptiveSpectrumProcessor(BANDS)

        scores = []
        x01_samples = []
        silent_skipped = 0
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

                # ---- RMS silence gate (time-domain buf) ----
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


class FileAudioThread(QThread):
    """文件播放期频谱提取线程（与播放器同步暂停/继续）。"""

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
        """请求停止线程。"""
        self._stop = True

    def set_paused(self, paused: bool):
        """设置暂停状态（与播放器同步）。"""
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
        """获取最新频谱（12 段 0..255）。"""
        return self._latest

    def _start_ffmpeg(self) -> subprocess.Popen:
        """启动 ffmpeg 解码输出为 s16le PCM。"""
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
        """从 ffmpeg stdout 精确读取 nbytes。"""
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
        """线程主循环：解码、FFT、发送最新频谱。"""
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
            self.status.emit(
                f"文件频谱启动：{os.path.basename(self.file_path)}（风格已锁定 {self.proc.genre_fixed:+.2f}）"
            )
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

                # audio-clock pacing: 按音频采样时钟节拍运行
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
