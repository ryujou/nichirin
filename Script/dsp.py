#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
数字信号处理（DSP）相关模块。

包含：
- 频谱分箱（对数频段）
- 自适应风格与全局 AGC 处理器
- DSP 配置数据结构
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from pc_common import BANDS


def make_log_bins(sr: int, nfft: int, bands: int, fmin: float, fmax: float):
    """
    构造对数频率分箱（适合人耳频谱显示）。

    Args:
        sr: 采样率
        nfft: FFT 点数
        bands: 频段数
        fmin: 最低频率
        fmax: 最高频率

    Returns:
        列表[(lo, hi), ...]，每段对应 FFT bin 范围
    """
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


class AdaptiveSpectrumProcessor:
    """
    频谱自适应处理器。

    输入：band_db（shape=(12,), 0..80 dB above noise floor）
    输出：u8 list (0..255)

    两种模式：
    - training (freeze=False)：更新风格与每段参考
    - frozen  (freeze=True)：锁定风格与每段参考，仅保留“全局 AGC”跟踪音量
    """

    def __init__(self, bands: int = BANDS):
        self.bands = bands

        # runtime gain control（全局 AGC）
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
        """谱平坦度：衡量频谱是否接近“白噪声”。"""
        x = np.clip(x, eps, None)
        gm = math.exp(float(np.mean(np.log(x))))
        am = float(np.mean(x))
        return gm / (am + eps)

    def _genre_score(self, x01):
        """
        风格打分：
        - +1 更偏低音/电音
        - -1 更偏 ACG/中高频
        """
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
        """训练期更新风格与每段参考。"""
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
        """将训练结果冻结为固定风格与参考曲线。"""
        self.genre_fixed = float(self.genre_ema)
        self.band_ref_fixed = np.clip(self.band_ema.copy(), 0.04, 0.8)
        self.freeze = True

        # reset runtime AGC
        self.gain = 1.0
        self.loud_ema = 0.15
        self.last_t = time.perf_counter()

    def debug_genre(self) -> float:
        """获取当前风格值（训练中为 EMA，冻结后为固定值）。"""
        return float(self.genre_fixed if self.freeze else self.genre_ema)

    def process(self, band_db: np.ndarray, dt: Optional[float] = None) -> list[int]:
        """
        将 12 段频谱 dB 映射为 0..255。

        Args:
            band_db: 12 段频谱 dB
            dt: 处理间隔（秒）

        Returns:
            12 段 0..255
        """
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
            # 在线自适应（麦克风模式用）
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

        # style tilt based on g
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


@dataclass
class DspConfig:
    """DSP 参数配置。"""

    sr: int
    block: int
    nfft: int
    fmin: float
    fmax: float
    noise_floor_db: float
