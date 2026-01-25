const BANDS = 12;
const FRAME_LEN = 1 + 1 + BANDS + 2;
const AUDIO_EXT = [".mp3", ".wav", ".flac", ".aac", ".m4a", ".ogg", ".opus"]; 
const VIDEO_EXT = [".mp4", ".mkv", ".avi", ".mov", ".webm", ".wmv"]; 

const ui = {
  openBtn: document.getElementById("openBtn"),
  playPauseBtn: document.getElementById("playPauseBtn"),
  stopMediaBtn: document.getElementById("stopMediaBtn"),
  progress: document.getElementById("progress"),
  progressTime: document.getElementById("progressTime"),
  media: document.getElementById("media"),
  dropHint: document.getElementById("dropHint"),
  filePicker: document.getElementById("filePicker"),
  portSelect: document.getElementById("portSelect"),
  refreshPortsBtn: document.getElementById("refreshPortsBtn"),
  connectBtn: document.getElementById("connectBtn"),
  disconnectBtn: document.getElementById("disconnectBtn"),
  serialToggle: document.getElementById("serialToggle"),
  serialSettingsRow: document.getElementById("serialSettingsRow"),
  baudSelect: document.getElementById("baudSelect"),
  hzInput: document.getElementById("hzInput"),
  printFrames: document.getElementById("printFrames"),
  printEvery: document.getElementById("printEvery"),
  serialStatus: document.getElementById("serialStatus"),
  serialStats: document.getElementById("serialStats"),
  sourceSelect: document.getElementById("sourceSelect"),
  micSelect: document.getElementById("micSelect"),
  refreshMicsBtn: document.getElementById("refreshMicsBtn"),
  srSelect: document.getElementById("srSelect"),
  blockSelect: document.getElementById("blockSelect"),
  floorInput: document.getElementById("floorInput"),
  floorValue: document.getElementById("floorValue"),
  dspPanel: document.getElementById("dspPanel"),
  srRow: document.getElementById("srRow"),
  floorRow: document.getElementById("floorRow"),
  startBtn: document.getElementById("startBtn"),
  stopBtn: document.getElementById("stopBtn"),
  modeSelect: document.getElementById("modeSelect"),
  configModeRow: document.getElementById("configModeRow"),
  configParamRow: document.getElementById("configParamRow"),
  configColorRow: document.getElementById("configColorRow"),
  paramSlider: document.getElementById("paramSlider"),
  paramValue: document.getElementById("paramValue"),
  paramHint: document.getElementById("paramHint"),
  colorPicker: document.getElementById("colorPicker"),
  colorSwatch: document.getElementById("colorSwatch"),
  colorHex: document.getElementById("colorHex"),
  readConfigBtn: document.getElementById("readConfigBtn"),
  sendConfigBtn: document.getElementById("sendConfigBtn"),
  statusText: document.getElementById("statusText"),
  genreText: document.getElementById("genreText"),
  logPanel: document.getElementById("logPanel"),
  logToggle: document.getElementById("logToggle"),
  logView: document.getElementById("logView"),
};

let fileState = {
  file: null,
  url: null,
  preAnalyzed: false,
  genreFixed: 0.0,
  bandRefFixed: null,
  analyzing: false,
  analysisToken: 0,
};

let audioState = {
  ctx: null,
  analyser: null,
  sourceNode: null,
  mediaSourceNode: null,
  stream: null,
  running: false,
  processor: null,
  latestBands: new Array(BANDS).fill(0),
  lastProcessTs: 0,
  updateTimer: null,
};

let serialState = {
  port: null,
  writer: null,
  sending: false,
  sent: 0,
  err: 0,
  t0: 0,
  printEvery: 200,
  knownPorts: [],
  connecting: false,
};

let configState = {
  mode: 1,
  hue: 0,
  sat: 255,
  val: 255,
  param: 128,
};

function log(msg) {
  if (ui.logPanel && ui.logPanel.dataset.hidden === "1") {
    return;
  }
  const stamp = new Date().toLocaleTimeString();
  ui.logView.value += `[${stamp}] ${msg}\n`;
  ui.logView.scrollTop = ui.logView.scrollHeight;
}

function setStatus(msg) {
  ui.statusText.textContent = msg;
  log(msg);
}

function setGenreText(value) {
  ui.genreText.textContent = value.toFixed(2);
}

function fmtMs(ms) {
  const s = Math.max(0, Math.floor(ms / 1000));
  const m = Math.floor(s / 60);
  const ss = s % 60;
  return `${String(m).padStart(2, "0")}:${String(ss).padStart(2, "0")}`;
}

function crc16Modbus(bytes) {
  let crc = 0xffff;
  for (let b of bytes) {
    crc ^= b;
    for (let i = 0; i < 8; i++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xa001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc & 0xffff;
}

function buildFrame(bands) {
  const payload = new Uint8Array(2 + BANDS);
  payload[0] = 0x01;
  payload[1] = 0x20;
  for (let i = 0; i < BANDS; i++) {
    payload[2 + i] = bands[i] & 0xff;
  }
  const crc = crc16Modbus(payload);
  const frame = new Uint8Array(FRAME_LEN);
  frame.set(payload, 0);
  frame[FRAME_LEN - 2] = crc & 0xff;
  frame[FRAME_LEN - 1] = (crc >> 8) & 0xff;
  return frame;
}

function buildWriteSingleRegister(reg, value) {
  const payload = new Uint8Array(6);
  payload[0] = 0x01;
  payload[1] = 0x06;
  payload[2] = (reg >> 8) & 0xff;
  payload[3] = reg & 0xff;
  payload[4] = (value >> 8) & 0xff;
  payload[5] = value & 0xff;
  const crc = crc16Modbus(payload);
  const frame = new Uint8Array(8);
  frame.set(payload, 0);
  frame[6] = crc & 0xff;
  frame[7] = (crc >> 8) & 0xff;
  return frame;
}

function buildWriteMultipleRegisters(startReg, values) {
  const count = values.length;
  const byteCount = count * 2;
  const payload = new Uint8Array(7 + byteCount);
  payload[0] = 0x01;
  payload[1] = 0x10;
  payload[2] = (startReg >> 8) & 0xff;
  payload[3] = startReg & 0xff;
  payload[4] = (count >> 8) & 0xff;
  payload[5] = count & 0xff;
  payload[6] = byteCount & 0xff;
  for (let i = 0; i < count; i++) {
    const v = values[i] & 0xffff;
    payload[7 + i * 2] = (v >> 8) & 0xff;
    payload[8 + i * 2] = v & 0xff;
  }
  const crc = crc16Modbus(payload);
  const frame = new Uint8Array(payload.length + 2);
  frame.set(payload, 0);
  frame[frame.length - 2] = crc & 0xff;
  frame[frame.length - 1] = (crc >> 8) & 0xff;
  return frame;
}

function buildReadHoldingRegisters(startReg, count) {
  const payload = new Uint8Array(6);
  payload[0] = 0x01;
  payload[1] = 0x03;
  payload[2] = (startReg >> 8) & 0xff;
  payload[3] = startReg & 0xff;
  payload[4] = (count >> 8) & 0xff;
  payload[5] = count & 0xff;
  const crc = crc16Modbus(payload);
  const frame = new Uint8Array(8);
  frame.set(payload, 0);
  frame[6] = crc & 0xff;
  frame[7] = (crc >> 8) & 0xff;
  return frame;
}

async function sendOnce(frame, label) {
  if (!serialState.port || !serialState.port.writable) {
    setStatus("请先连接串口");
    throw new Error("serial not ready");
  }
  const writer = serialState.port.writable.getWriter();
  try {
    await writer.write(frame);
    if (label) {
      log(`TX ${label}`);
    }
  } finally {
    await writer.releaseLock();
  }
}

async function sendModeSelect(mode) {
  const regMode = 0x0000;
  const frame = buildWriteSingleRegister(regMode, mode & 0xffff);
  await sendOnce(frame, `mode=${mode}`);
}

async function readExactBytes(reader, len, timeoutMs = 300) {
  const out = new Uint8Array(len);
  let offset = 0;
  const t0 = performance.now();
  while (offset < len) {
    const elapsed = performance.now() - t0;
    if (elapsed > timeoutMs) {
      throw new Error("读取超时");
    }
    const { value, done } = await reader.read();
    if (done) break;
    if (value && value.length) {
      const n = Math.min(value.length, len - offset);
      out.set(value.subarray(0, n), offset);
      offset += n;
    }
  }
  if (offset < len) {
    throw new Error("读取长度不足");
  }
  return out;
}

function verifyCrc(frame) {
  const data = frame.subarray(0, frame.length - 2);
  const crc = crc16Modbus(data);
  const rx = frame[frame.length - 2] | (frame[frame.length - 1] << 8);
  return crc === rx;
}

async function readConfigFromDevice() {
  if (serialState.sending) {
    setStatus("请先停止频谱发送，再读取配置");
    return;
  }
  if (!serialState.port || !serialState.port.writable || !serialState.port.readable) {
    setStatus("请先连接串口");
    return;
  }
  const startReg = 0x0000;
  const count = 5;
  const req = buildReadHoldingRegisters(startReg, count);
  const reader = serialState.port.readable.getReader();
  try {
    await sendOnce(req, "read cfg");
    const head = await readExactBytes(reader, 3, 200);
    const byteCount = head[2];
    if (byteCount !== 8 && byteCount !== 10) {
      throw new Error("响应长度不匹配");
    }
    const tail = await readExactBytes(reader, byteCount + 2, 300);
    const frame = new Uint8Array(3 + tail.length);
    frame.set(head, 0);
    frame.set(tail, 3);
    if (frame[0] !== 0x01 || frame[1] !== 0x03) {
      throw new Error("响应功能码不匹配");
    }
    if (!verifyCrc(frame)) {
      throw new Error("CRC 校验失败");
    }
    const values = [];
    const regs = byteCount / 2;
    for (let i = 0; i < regs; i++) {
      const hi = frame[3 + i * 2];
      const lo = frame[4 + i * 2];
      values.push((hi << 8) | lo);
    }
    configState.mode = values[0];
    configState.hue = clamp(values[1], 0, 359);
    configState.sat = clamp(values[2], 0, 255);
    configState.val = clamp(values[3], 0, 255);
    if (values.length >= 5 && configState.mode !== 6) {
      configState.param = clamp(values[4], 0, 255);
    }
    ui.modeSelect.value = String(configState.mode);
    updateColorUIFromHSV();
    updateParamUI();
    updateConfigVisibility();
    setStatus("读取配置成功");
  } catch (err) {
    setStatus(`读取配置失败: ${err}`);
  } finally {
    try { await reader.releaseLock(); } catch (_) {}
  }
}

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function rgbToHsv(r, g, b) {
  const rn = r / 255;
  const gn = g / 255;
  const bn = b / 255;
  const max = Math.max(rn, gn, bn);
  const min = Math.min(rn, gn, bn);
  const d = max - min;
  let h = 0;
  if (d > 1e-6) {
    if (max === rn) {
      h = ((gn - bn) / d) % 6;
    } else if (max === gn) {
      h = (bn - rn) / d + 2;
    } else {
      h = (rn - gn) / d + 4;
    }
    h *= 60;
    if (h < 0) h += 360;
  }
  const s = max === 0 ? 0 : d / max;
  const v = max;
  return {
    h: Math.round(h) % 360,
    s: Math.round(s * 255),
    v: Math.round(v * 255),
  };
}

function hsvToRgb(h, s, v) {
  const sn = clamp(s, 0, 255) / 255;
  const vn = clamp(v, 0, 255) / 255;
  const hh = ((h % 360) + 360) % 360;
  const c = vn * sn;
  const x = c * (1 - Math.abs(((hh / 60) % 2) - 1));
  const m = vn - c;
  let r1 = 0, g1 = 0, b1 = 0;
  if (hh < 60) { r1 = c; g1 = x; b1 = 0; }
  else if (hh < 120) { r1 = x; g1 = c; b1 = 0; }
  else if (hh < 180) { r1 = 0; g1 = c; b1 = x; }
  else if (hh < 240) { r1 = 0; g1 = x; b1 = c; }
  else if (hh < 300) { r1 = x; g1 = 0; b1 = c; }
  else { r1 = c; g1 = 0; b1 = x; }
  return {
    r: Math.round((r1 + m) * 255),
    g: Math.round((g1 + m) * 255),
    b: Math.round((b1 + m) * 255),
  };
}

function parseHexColor(text) {
  if (!text) return null;
  let hex = text.trim();
  if (hex.startsWith("#")) hex = hex.slice(1);
  if (hex.length !== 6) return null;
  const num = parseInt(hex, 16);
  if (Number.isNaN(num)) return null;
  return {
    r: (num >> 16) & 0xff,
    g: (num >> 8) & 0xff,
    b: num & 0xff,
  };
}

function toHex2(v) {
  return v.toString(16).padStart(2, "0");
}

function updateColorUIFromHSV() {
  const rgb = hsvToRgb(configState.hue, configState.sat, configState.val);
  const hex = `#${toHex2(rgb.r)}${toHex2(rgb.g)}${toHex2(rgb.b)}`;
  ui.colorPicker.value = hex;
  ui.colorHex.value = hex;
  if (ui.colorSwatch) {
    ui.colorSwatch.style.background = hex;
  }
}

function updateParamUI() {
  ui.paramSlider.value = String(configState.param);
  ui.paramValue.value = String(configState.param);
  let hint = `${configState.param}`;
  const mode = configState.mode;
  if (mode === 1) {
    const ms = 40 + (configState.param * (320 - 40)) / 255;
    hint = `${ms.toFixed(0)} ms`;
  } else if (mode === 2) {
    const ms = 80 + (configState.param * (2000 - 80)) / 255;
    hint = `${ms.toFixed(0)} ms`;
  } else if (mode === 3) {
    const pct = (configState.param / 255) * 100;
    hint = `${pct.toFixed(0)} %`;
  } else if (mode === 4) {
    const step = 1 + ((255 - configState.param) * 4) / 255;
    hint = `步进 ${step.toFixed(1)}`;
  } else if (mode === 6) {
    const pct = (configState.param / 255) * 100;
    hint = `${pct.toFixed(0)} %`;
  }
  ui.paramHint.textContent = hint;
}

function updateConfigVisibility() {
  const isSpectrum = configState.mode === 6;
  if (ui.configParamRow) {
    ui.configParamRow.style.display = isSpectrum ? "none" : "";
  }
  if (ui.configColorRow) {
    ui.configColorRow.style.display = isSpectrum ? "none" : "";
  }
  if (ui.dspPanel) {
    ui.dspPanel.style.display = isSpectrum ? "" : "none";
  }
  if (ui.srRow) {
    ui.srRow.style.display = "none";
  }
}

function buildWriteSingleRegister(reg, value) {
  const payload = new Uint8Array(6);
  payload[0] = 0x01;
  payload[1] = 0x06;
  payload[2] = (reg >> 8) & 0xff;
  payload[3] = reg & 0xff;
  payload[4] = (value >> 8) & 0xff;
  payload[5] = value & 0xff;
  const crc = crc16Modbus(payload);
  const frame = new Uint8Array(8);
  frame.set(payload, 0);
  frame[6] = crc & 0xff;
  frame[7] = (crc >> 8) & 0xff;
  return frame;
}

async function sendOnce(frame, label) {
  if (!serialState.port || !serialState.port.writable) {
    setStatus("请先连接串口");
    throw new Error("serial not ready");
  }
  const writer = serialState.port.writable.getWriter();
  try {
    await writer.write(frame);
    if (label) {
      log(`TX ${label}`);
    }
  } finally {
    await writer.releaseLock();
  }
}

async function sendModeSelect(mode) {
  const regMode = 0x0000;
  const frame = buildWriteSingleRegister(regMode, mode & 0xffff);
  await sendOnce(frame, `mode=${mode}`);
}

function makeLogBins(sr, nfft, bands, fmin, fmax) {
  const maxF = Math.min(fmax, sr * 0.49);
  const minF = Math.max(10.0, fmin);
  const edges = [];
  for (let i = 0; i <= bands; i++) {
    const t = i / bands;
    edges.push(minF * Math.pow(maxF / minF, t));
  }
  const binHz = sr / nfft;
  const out = [];
  for (let i = 0; i < bands; i++) {
    let lo = Math.floor(edges[i] / binHz);
    let hi = Math.floor(edges[i + 1] / binHz);
    lo = Math.max(1, lo);
    hi = Math.max(lo + 1, hi);
    out.push([lo, hi]);
  }
  return out;
}

class AdaptiveSpectrumProcessor {
  constructor(bands = BANDS) {
    this.bands = bands;
    this.gain = 1.0;
    this.loudEma = 0.15;
    this.lastT = performance.now() / 1000;
    this.genreEma = 0.0;
    this.bandEma = new Float32Array(bands).fill(0.25);
    this.freeze = false;
    this.genreFixed = 0.0;
    this.bandRefFixed = new Float32Array(bands).fill(0.25);
  }

  _spectralFlatness(x) {
    let gm = 0.0;
    let am = 0.0;
    const eps = 1e-6;
    for (let i = 0; i < x.length; i++) {
      const v = Math.max(eps, x[i]);
      gm += Math.log(v);
      am += v;
    }
    gm = Math.exp(gm / x.length);
    am = am / x.length;
    return gm / (am + eps);
  }

  _genreScore(x01) {
    const x = x01;
    const low = (x[0] + x[1] + x[2]) / 3.0;
    const mid = (x[3] + x[4] + x[5] + x[6] + x[7]) / 5.0;
    const high = (x[8] + x[9] + x[10] + x[11]) / 4.0;
    const allv = x.reduce((a, b) => a + b, 0) / x.length;
    const bassRatio = low / Math.max(1e-6, low + mid + high);
    let centroid = 0.0;
    let sum = 0.0;
    for (let i = 0; i < x.length; i++) {
      centroid += i * x[i];
      sum += x[i];
    }
    centroid = centroid / Math.max(1e-6, sum);
    const flat = this._spectralFlatness(x);

    let bassdj = 0.0;
    bassdj += (bassRatio - 0.33) * 2.4;
    bassdj += (0.45 - centroid / (this.bands - 1)) * 1.8;
    bassdj += (allv - 0.35) * 1.2;
    bassdj += (0.35 - flat) * 0.8;

    let acg = 0.0;
    acg += (0.28 - allv) * 2.0;
    acg += (0.33 - bassRatio) * 1.2;
    acg += (centroid / (this.bands - 1) - 0.5) * 0.8;
    acg += (flat - 0.35) * 0.6;

    const score = bassdj - acg;
    return Math.max(-1.0, Math.min(1.0, score));
  }

  updateTraining(bandDb, dt) {
    const safeDt = Math.max(1e-3, Math.min(0.25, dt));
    const x01 = bandDb.map((v) => Math.max(0.0, Math.min(1.0, v / 45.0)));
    const score = this._genreScore(x01);
    const aGenre = 1.0 - Math.exp(-safeDt / 1.0);
    this.genreEma += (score - this.genreEma) * aGenre;

    const g = this.genreEma;
    const tauBand = 2.0 - 0.8 * Math.max(0.0, g) + 0.6 * Math.max(0.0, -g);
    const aBand = 1.0 - Math.exp(-safeDt / Math.max(0.2, tauBand));
    for (let i = 0; i < this.bands; i++) {
      this.bandEma[i] += (x01[i] - this.bandEma[i]) * aBand;
    }
  }

  freezeFromTraining() {
    this.genreFixed = this.genreEma;
    for (let i = 0; i < this.bands; i++) {
      this.bandRefFixed[i] = Math.max(0.04, Math.min(0.8, this.bandEma[i]));
    }
    this.freeze = true;
    this.gain = 1.0;
    this.loudEma = 0.15;
    this.lastT = performance.now() / 1000;
  }

  debugGenre() {
    return this.freeze ? this.genreFixed : this.genreEma;
  }

  process(bandDb, dt = null) {
    const now = performance.now() / 1000;
    let usedDt = dt === null ? now - this.lastT : dt;
    this.lastT = now;
    usedDt = Math.max(1e-3, Math.min(0.1, usedDt));

    const x01 = bandDb.map((v) => Math.max(0.0, Math.min(1.0, v / 45.0)));
    let g;
    let bandRef;
    if (this.freeze) {
      g = this.genreFixed;
      bandRef = this.bandRefFixed;
    } else {
      const score = this._genreScore(x01);
      const aGenre = 1.0 - Math.exp(-usedDt / 1.0);
      this.genreEma += (score - this.genreEma) * aGenre;
      g = this.genreEma;
      const tauBand = 2.0 - 0.8 * Math.max(0.0, g) + 0.6 * Math.max(0.0, -g);
      const aBand = 1.0 - Math.exp(-usedDt / Math.max(0.2, tauBand));
      for (let i = 0; i < this.bands; i++) {
        this.bandEma[i] += (x01[i] - this.bandEma[i]) * aBand;
      }
      bandRef = this.bandEma;
    }

    const y = new Float32Array(this.bands);
    for (let i = 0; i < this.bands; i++) {
      const ref = Math.max(0.04, Math.min(0.8, bandRef[i]));
      y[i] = x01[i] / ref;
    }

    const tilt = new Float32Array(this.bands).fill(1.0);
    for (let i = 0; i < this.bands; i++) {
      const idx = i / (this.bands - 1);
      if (g > 0.25) {
        tilt[i] *= 0.85 + 0.35 * idx;
      } else if (g < -0.25) {
        tilt[i] *= 1.15 - 0.25 * idx;
      }
    }
    for (let i = 0; i < this.bands; i++) {
      y[i] *= tilt[i];
    }

    let loud = 0.0;
    for (let i = 0; i < this.bands; i++) {
      loud += Math.min(3.0, Math.max(0.0, y[i]));
    }
    loud /= this.bands;
    const aLoud = 1.0 - Math.exp(-usedDt / 0.5);
    this.loudEma += (loud - this.loudEma) * aLoud;
    const loudRef = Math.max(0.08, this.loudEma);

    let target = 0.95 + (-g) * 0.25;
    target = Math.max(0.65, Math.min(1.25, target));
    let gain = target / loudRef;
    gain = Math.max(0.35, Math.min(3.5, gain));
    const tauG = gain > this.gain ? 0.08 : 0.25;
    const aG = 1.0 - Math.exp(-usedDt / tauG);
    this.gain += (gain - this.gain) * aG;
    for (let i = 0; i < this.bands; i++) {
      y[i] *= this.gain;
    }

    for (let i = 0; i < this.bands; i++) {
      if (g < -0.25) {
        y[i] = Math.sqrt(Math.min(1.0, Math.max(0.0, y[i] / 1.6)));
      } else if (g > 0.25) {
        const val = Math.min(3.0, Math.max(0.0, y[i]));
        y[i] = Math.log1p(2.2 * val) / Math.log1p(2.2 * 1.6);
      } else {
        y[i] = Math.sqrt(Math.min(1.0, Math.max(0.0, y[i] / 1.8)));
      }
      y[i] = Math.max(0.0, Math.min(1.0, y[i]));
    }

    const out = new Array(this.bands);
    for (let i = 0; i < this.bands; i++) {
      out[i] = Math.max(0, Math.min(255, Math.round(y[i] * 255.0)));
    }
    return out;
  }
}

function createFFT(n) {
  const bitrev = new Uint32Array(n);
  const bits = Math.log2(n);
  for (let i = 0; i < n; i++) {
    let x = i;
    let y = 0;
    for (let j = 0; j < bits; j++) {
      y = (y << 1) | (x & 1);
      x >>= 1;
    }
    bitrev[i] = y;
  }
  const window = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    window[i] = 0.5 - 0.5 * Math.cos((2 * Math.PI * i) / (n - 1));
  }
  return { bitrev, window };
}

function fftInPlace(re, im, bitrev) {
  const n = re.length;
  for (let i = 0; i < n; i++) {
    const j = bitrev[i];
    if (j > i) {
      const tr = re[i];
      const ti = im[i];
      re[i] = re[j];
      im[i] = im[j];
      re[j] = tr;
      im[j] = ti;
    }
  }

  for (let len = 2; len <= n; len <<= 1) {
    const ang = (-2 * Math.PI) / len;
    const wlenRe = Math.cos(ang);
    const wlenIm = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let wRe = 1.0;
      let wIm = 0.0;
      for (let j = 0; j < len / 2; j++) {
        const uRe = re[i + j];
        const uIm = im[i + j];
        const vRe = re[i + j + len / 2] * wRe - im[i + j + len / 2] * wIm;
        const vIm = re[i + j + len / 2] * wIm + im[i + j + len / 2] * wRe;
        re[i + j] = uRe + vRe;
        im[i + j] = uIm + vIm;
        re[i + j + len / 2] = uRe - vRe;
        im[i + j + len / 2] = uIm - vIm;
        const nextRe = wRe * wlenRe - wIm * wlenIm;
        const nextIm = wRe * wlenIm + wIm * wlenRe;
        wRe = nextRe;
        wIm = nextIm;
      }
    }
  }
}

function rfftMag2(time, workRe, workIm, window, bitrev) {
  const n = time.length;
  for (let i = 0; i < n; i++) {
    workRe[i] = time[i] * window[i];
    workIm[i] = 0.0;
  }
  fftInPlace(workRe, workIm, bitrev);
  const half = n / 2;
  const out = new Float32Array(half + 1);
  for (let i = 0; i <= half; i++) {
    const re = workRe[i];
    const im = workIm[i];
    out[i] = re * re + im * im + 1e-12;
  }
  return out;
}

function trimmedMean(values, trim = 0.1) {
  if (!values.length) return 0.0;
  const arr = values.slice().sort((a, b) => a - b);
  const k = Math.floor(arr.length * trim);
  if (arr.length - 2 * k <= 1) {
    return arr.reduce((a, b) => a + b, 0) / arr.length;
  }
  let sum = 0.0;
  for (let i = k; i < arr.length - k; i++) sum += arr[i];
  return sum / (arr.length - 2 * k);
}

function percentile(values, p) {
  if (!values.length) return 0.0;
  const arr = values.slice().sort((a, b) => a - b);
  const idx = Math.floor((p / 100) * (arr.length - 1));
  return arr[idx];
}

async function refreshPorts(allowRequest = false) {
  ui.portSelect.innerHTML = "";
  serialState.knownPorts = [];
  if (!isSecureContext) {
    ui.portSelect.innerHTML = "<option>需要 https / localhost 才能使用</option>";
    return;
  }
  if (!navigator.serial) {
    ui.portSelect.innerHTML = "<option>浏览器不支持 Web Serial</option>";
    return;
  }
  let requestedPort = null;
  let ports = await navigator.serial.getPorts();
  if (allowRequest) {
    try {
      log("刷新触发授权串口");
      requestedPort = await navigator.serial.requestPort();
      ports = await navigator.serial.getPorts();
    } catch (err) {
      log(`授权串口失败/取消: ${err}`);
    }
  }
  if (requestedPort && !ports.includes(requestedPort)) {
    ports = ports.length ? [...ports, requestedPort] : [requestedPort];
  }
  serialState.knownPorts = ports;
  if (!ports.length) {
    ui.portSelect.innerHTML = "<option>未发现已授权串口（点刷新授权）</option>";
    return;
  }
  ports.forEach((port, idx) => {
    const option = document.createElement("option");
    option.value = idx;
    option.textContent = `已授权串口 ${idx + 1}`;
    ui.portSelect.appendChild(option);
  });
  ui.portSelect.dataset.ports = JSON.stringify(ports.map((_, i) => i));
}

async function connectSerial() {
  if (serialState.connecting) {
    log("串口正在连接，请稍候");
    return;
  }
  if (!isSecureContext) {
    setStatus("需要 https / localhost 才能使用 Web Serial");
    return;
  }
  if (!navigator.serial) {
    setStatus("浏览器不支持 Web Serial");
    return;
  }
  if (serialState.port) {
    setStatus("串口已连接，请先断开");
    return;
  }
  try {
    serialState.connecting = true;
    ui.connectBtn.disabled = true;
    ui.refreshPortsBtn.disabled = true;
    const baud = parseInt(ui.baudSelect.value, 10);
    let port = null;
    const selected = parseInt(ui.portSelect.value, 10);
    if (!Number.isNaN(selected) && serialState.knownPorts[selected]) {
      port = serialState.knownPorts[selected];
    } else {
      port = await navigator.serial.requestPort();
    }
    await port.open({ baudRate: baud });
    serialState.port = port;
    ui.serialStatus.textContent = "已连接";
    setStatus(`串口已连接 @ ${baud}`);
    await refreshPorts(false);
  } catch (err) {
    setStatus(`串口连接失败: ${err}`);
  } finally {
    serialState.connecting = false;
    ui.connectBtn.disabled = false;
    ui.refreshPortsBtn.disabled = false;
  }
}

async function disconnectSerial() {
  await stopSending();
  if (serialState.port) {
    try {
      await serialState.port.close();
    } catch (err) {
      log(`串口关闭失败: ${err}`);
    }
  }
  serialState.port = null;
  ui.serialStatus.textContent = "未连接";
  ui.connectBtn.disabled = false;
  ui.refreshPortsBtn.disabled = false;
  await refreshPorts(false);
}

async function startSending() {
  if (!serialState.port || !serialState.port.writable) {
    setStatus("请先连接串口");
    return;
  }
  if (serialState.sending) return;
  serialState.writer = serialState.port.writable.getWriter();
  serialState.sending = true;
  serialState.sent = 0;
  serialState.err = 0;
  serialState.t0 = performance.now();
  serialState.printEvery = Math.max(1, parseInt(ui.printEvery.value, 10));
  const hz = Math.max(1, parseFloat(ui.hzInput.value));
  const period = 1000 / hz;

  const loop = async () => {
    while (serialState.sending) {
      const start = performance.now();
      const frame = buildFrame(audioState.latestBands);
      try {
        await serialState.writer.write(frame);
      } catch (err) {
        serialState.err += 1;
        if (serialState.err % 10 === 1) {
          setStatus(`写串口异常(累计${serialState.err}) : ${err}`);
        }
      }

      serialState.sent += 1;
      if (ui.printFrames.checked && serialState.sent % serialState.printEvery === 0) {
        const crc = frame[FRAME_LEN - 2] | (frame[FRAME_LEN - 1] << 8);
        log(`[TX ${serialState.sent.toString().padStart(6, " ")}] bands=${audioState.latestBands} crc=0x${crc.toString(16).padStart(4, "0")}`);
      }

      const elapsed = (performance.now() - serialState.t0) / 1000;
      if (elapsed > 0) {
        ui.serialStats.textContent = `TX ${serialState.sent} / ${elapsed.toFixed(1)}s / err ${serialState.err}`;
      }

      const cost = performance.now() - start;
      const sleep = Math.max(0, period - cost);
      if (sleep > 0) {
        await new Promise((r) => setTimeout(r, Math.min(5, sleep)));
      }
    }
  };

  loop();
  setStatus(`串口发送开始: ${hz.toFixed(1)}Hz`);
}

async function stopSending() {
  serialState.sending = false;
  if (serialState.writer) {
    try {
      await serialState.writer.releaseLock();
    } catch (err) {
      log(`释放串口 writer 失败: ${err}`);
    }
  }
  serialState.writer = null;
}

async function refreshMics() {
  ui.micSelect.innerHTML = "";
  try {
    const devices = await navigator.mediaDevices.enumerateDevices();
    const mics = devices.filter((d) => d.kind === "audioinput");
    if (!mics.length) {
      ui.micSelect.innerHTML = "<option>未找到麦克风</option>";
      return;
    }
    mics.forEach((mic) => {
      const option = document.createElement("option");
      option.value = mic.deviceId;
      option.textContent = mic.label || `麦克风 ${mic.deviceId.slice(0, 6)}`;
      ui.micSelect.appendChild(option);
    });
  } catch (err) {
    ui.micSelect.innerHTML = "<option>枚举失败</option>";
    log(`枚举麦克风失败: ${err}`);
  }
}

function ensureAudioContext() {
  if (!audioState.ctx) {
    const wanted = parseInt(ui.srSelect.value, 10);
    audioState.ctx = new (window.AudioContext || window.webkitAudioContext)({ sampleRate: wanted });
  }
  return audioState.ctx;
}

async function setupMic() {
  const ctx = ensureAudioContext();
  const deviceId = ui.micSelect.value;
  const constraints = {
    audio: deviceId ? { deviceId: { exact: deviceId } } : true,
    video: false,
  };
  const stream = await navigator.mediaDevices.getUserMedia(constraints);
  audioState.stream = stream;
  const source = ctx.createMediaStreamSource(stream);
  const analyser = ctx.createAnalyser();
  analyser.fftSize = 2048;
  analyser.smoothingTimeConstant = 0.0;
  source.connect(analyser);
  audioState.sourceNode = source;
  audioState.analyser = analyser;
}

async function setupMediaElement() {
  const ctx = ensureAudioContext();
  const analyser = ctx.createAnalyser();
  analyser.fftSize = 2048;
  analyser.smoothingTimeConstant = 0.0;
  if (audioState.sourceNode) {
    audioState.sourceNode.disconnect();
  }
  if (!audioState.mediaSourceNode) {
    audioState.mediaSourceNode = ctx.createMediaElementSource(ui.media);
  }
  const source = audioState.mediaSourceNode;
  source.connect(analyser);
  analyser.connect(ctx.destination);
  audioState.sourceNode = source;
  audioState.analyser = analyser;
}

function stopAudio() {
  if (audioState.updateTimer) {
    clearInterval(audioState.updateTimer);
  }
  audioState.updateTimer = null;
  audioState.running = false;
  audioState.latestBands = new Array(BANDS).fill(0);
  if (audioState.stream) {
    audioState.stream.getTracks().forEach((t) => t.stop());
  }
  audioState.stream = null;
  if (audioState.sourceNode) {
    try { audioState.sourceNode.disconnect(); } catch (_) {}
  }
  audioState.sourceNode = null;
  audioState.analyser = null;
}

function computeBands(timeData, fftCache, bins, floorDb) {
  const mag2 = rfftMag2(timeData, fftCache.re, fftCache.im, fftCache.window, fftCache.bitrev);
  const bandDb = new Array(BANDS).fill(0.0);
  for (let i = 0; i < BANDS; i++) {
    const [lo, hi] = bins[i];
    let maxDb = -1e9;
    for (let k = lo; k < Math.min(hi, mag2.length); k++) {
      const db = 10.0 * Math.log10(mag2[k]) - floorDb;
      if (db > maxDb) maxDb = db;
    }
    const clipped = Math.max(0.0, Math.min(80.0, maxDb));
    bandDb[i] = clipped;
  }
  return bandDb;
}

function startProcessingLoop() {
  const analyser = audioState.analyser;
  if (!analyser) return;
  const nfft = analyser.fftSize;
  const fftCache = {
    re: new Float32Array(nfft),
    im: new Float32Array(nfft),
    ...createFFT(nfft),
  };
  const timeData = new Float32Array(nfft);
  const sr = audioState.ctx.sampleRate;
  const bins = makeLogBins(sr, nfft, BANDS, 30.0, 16000.0);
  const floorDb = parseFloat(ui.floorInput.value);
  audioState.processor = audioState.processor || new AdaptiveSpectrumProcessor(BANDS);
  audioState.running = true;
  audioState.lastProcessTs = performance.now();

  audioState.updateTimer = setInterval(() => {
    if (!audioState.running) return;
    analyser.getFloatTimeDomainData(timeData);
    const bandDb = computeBands(timeData, fftCache, bins, floorDb);
    const now = performance.now();
    const dt = (now - audioState.lastProcessTs) / 1000;
    audioState.lastProcessTs = now;
    audioState.latestBands = audioState.processor.process(bandDb, dt);
    setGenreText(audioState.processor.debugGenre());
  }, 30);
}

function updateMediaControls() {
  const isFile = ui.sourceSelect.value === "file";
  const ready = fileState.file && fileState.preAnalyzed;
  ui.playPauseBtn.disabled = !(isFile && ready);
  ui.stopMediaBtn.disabled = !(isFile && fileState.file);
}

async function loadMediaFile(file) {
  if (!file) return;
  const ext = file.name ? file.name.slice(file.name.lastIndexOf(".")).toLowerCase() : "";
  if (![...AUDIO_EXT, ...VIDEO_EXT].includes(ext)) {
    setStatus(`不支持的文件类型: ${ext}`);
    return;
  }

  ui.sourceSelect.value = "file";
  ui.micSelect.disabled = true;
  ui.refreshMicsBtn.disabled = true;

  if (fileState.url) URL.revokeObjectURL(fileState.url);
  fileState.file = file;
  fileState.url = URL.createObjectURL(file);
  fileState.preAnalyzed = false;
  fileState.genreFixed = 0.0;
  fileState.bandRefFixed = null;

  ui.media.src = fileState.url;
  ui.media.load();
  ui.media.pause();
  ui.playPauseBtn.textContent = "播放";
  ui.dropHint.style.display = "none";
  ui.progress.value = 0;
  ui.progress.max = 0;
  ui.progressTime.textContent = "00:00 / 00:00";

  setStatus(`已加载: ${file.name} (正在预分析)`);
  ui.genreText.textContent = "分析中...";
  await preAnalyzeFile(file);
  updateMediaControls();
}

async function preAnalyzeFile(file) {
  fileState.analysisToken += 1;
  const token = fileState.analysisToken;
  fileState.analyzing = true;

  const blockFrames = parseInt(ui.blockSelect.value, 10);
  const floorDb = parseFloat(ui.floorInput.value);
  const windowSize = 2048;

  try {
    const arrayBuf = await file.arrayBuffer();
    const ctx = new (window.AudioContext || window.webkitAudioContext)();
    const audioBuf = await ctx.decodeAudioData(arrayBuf.slice(0));
    await ctx.close();
    const sr = audioBuf.sampleRate;
    const bins = makeLogBins(sr, windowSize, BANDS, 30.0, 16000.0);
    const fftCache = {
      re: new Float32Array(windowSize),
      im: new Float32Array(windowSize),
      ...createFFT(windowSize),
    };
    const ring = new Float32Array(windowSize);
    const buf = new Float32Array(windowSize);
    let ringPos = 0;
    const hannWindow = fftCache.window;

    const analyzer = new AdaptiveSpectrumProcessor(BANDS);
    const scores = [];
    const bandSamples = Array.from({ length: BANDS }, () => []);
    let silentSkipped = 0;
    let framesTotal = 0;
    let lastMsg = performance.now();

    const channelCount = audioBuf.numberOfChannels;
    const chData = [];
    for (let c = 0; c < channelCount; c++) {
      chData.push(audioBuf.getChannelData(c));
    }

    for (let offset = 0; offset < audioBuf.length; offset += blockFrames) {
      if (fileState.analysisToken !== token) return;
      const block = new Float32Array(blockFrames);
      for (let i = 0; i < blockFrames; i++) {
        const idx = offset + i;
        if (idx >= audioBuf.length) break;
        let sum = 0;
        for (let c = 0; c < channelCount; c++) sum += chData[c][idx];
        block[i] = sum / channelCount;
      }

      const n = block.length;
      framesTotal += n;
      if (n >= windowSize) {
        ring.set(block.subarray(n - windowSize));
        ringPos = 0;
      } else {
        const end = ringPos + n;
        if (end < windowSize) {
          ring.set(block, ringPos);
          ringPos = end;
        } else {
          const first = windowSize - ringPos;
          ring.set(block.subarray(0, first), ringPos);
          ring.set(block.subarray(first), 0);
          ringPos = n - first;
        }
      }

      if (ringPos === 0) {
        buf.set(ring);
      } else {
        buf.set(ring.subarray(ringPos));
        buf.set(ring.subarray(0, ringPos), windowSize - ringPos);
      }
      let rms = 0.0;
      for (let i = 0; i < buf.length; i++) rms += buf[i] * buf[i];
      rms = Math.sqrt(rms / buf.length + 1e-12);
      if (rms < 0.004) {
        silentSkipped += 1;
        continue;
      }

      const mag2 = rfftMag2(buf, fftCache.re, fftCache.im, hannWindow, fftCache.bitrev);
      const bandDb = new Array(BANDS).fill(0.0);
      for (let b = 0; b < BANDS; b++) {
        const [lo, hi] = bins[b];
        let maxDb = -1e9;
        for (let k = lo; k < Math.min(hi, mag2.length); k++) {
          const db = 10.0 * Math.log10(mag2[k]) - floorDb;
          if (db > maxDb) maxDb = db;
        }
        bandDb[b] = Math.max(0.0, Math.min(80.0, maxDb));
      }

      const x01 = bandDb.map((v) => Math.max(0.0, Math.min(1.0, v / 45.0)));
      scores.push(analyzer._genreScore(x01));
      for (let b = 0; b < BANDS; b++) {
        bandSamples[b].push(x01[b]);
      }
      analyzer.updateTraining(bandDb, blockFrames / sr);

      const now = performance.now();
      if (now - lastMsg > 600) {
        const sec = framesTotal / sr;
        const preview = trimmedMean(scores, 0.1);
        setStatus(`预分析中... 已扫 ${sec.toFixed(1)}s  预估风格 ${preview.toFixed(2)} (有效${scores.length} 跳过静音${silentSkipped})`);
        lastMsg = now;
        await new Promise((r) => setTimeout(r, 0));
      }
    }

    let gFixed = 0.0;
    let ref = new Float32Array(BANDS).fill(0.25);
    if (scores.length < 40) {
      analyzer.freezeFromTraining();
      gFixed = analyzer.genreFixed;
      ref = analyzer.bandRefFixed;
    } else {
      gFixed = trimmedMean(scores, 0.1);
      for (let b = 0; b < BANDS; b++) {
        ref[b] = Math.max(0.04, Math.min(0.8, percentile(bandSamples[b], 60)));
      }
    }

    fileState.preAnalyzed = true;
    fileState.genreFixed = gFixed;
    fileState.bandRefFixed = ref;
    setStatus(`预分析完成: 风格锁定 ${gFixed.toFixed(2)} (有效${scores.length} 跳过静音${silentSkipped})`);
    ui.genreText.textContent = gFixed.toFixed(2);
  } catch (err) {
    fileState.preAnalyzed = true;
    fileState.genreFixed = 0.0;
    fileState.bandRefFixed = new Float32Array(BANDS).fill(0.25);
    setStatus(`预分析失败: ${err} (将使用在线自适应)`);
  } finally {
    fileState.analyzing = false;
  }
}

function setupProcessorForMode() {
  const proc = new AdaptiveSpectrumProcessor(BANDS);
  if (ui.sourceSelect.value === "file" && fileState.preAnalyzed && fileState.bandRefFixed) {
    proc.genreEma = fileState.genreFixed;
    proc.bandEma = new Float32Array(fileState.bandRefFixed);
    proc.freezeFromTraining();
  }
  audioState.processor = proc;
}

async function startAll() {
  if (audioState.running) return;
  setupProcessorForMode();
  try {
    if (ui.sourceSelect.value === "mic") {
      await setupMic();
      setStatus("麦克风频谱已启动");
    } else {
      if (!fileState.file) {
        setStatus("文件模式需要先加载文件");
        return;
      }
      if (!fileState.preAnalyzed) {
        setStatus("文件尚未预分析完成");
        return;
      }
      await setupMediaElement();
      setStatus("文件频谱已启动");
    }
    await audioState.ctx.resume();
    try {
      await sendModeSelect(6);
    } catch (err) {
      setStatus(`自动切换模式失败: ${err}`);
    }
    startProcessingLoop();
    await startSending();
    ui.startBtn.disabled = true;
    ui.stopBtn.disabled = false;
    ui.openBtn.disabled = true;
    ui.refreshPortsBtn.disabled = true;
    ui.connectBtn.disabled = true;
    ui.sourceSelect.disabled = true;
    ui.micSelect.disabled = ui.sourceSelect.value !== "mic";
    ui.refreshMicsBtn.disabled = ui.sourceSelect.value !== "mic";
  } catch (err) {
    await refreshPorts(false);
    setStatus(`启动失败: ${err}`);
  }
}

async function stopAll() {
  await stopSending();
  stopAudio();
  ui.startBtn.disabled = false;
  ui.stopBtn.disabled = true;
  ui.openBtn.disabled = false;
  ui.refreshPortsBtn.disabled = false;
  ui.connectBtn.disabled = false;
  ui.sourceSelect.disabled = false;
  ui.micSelect.disabled = ui.sourceSelect.value !== "mic";
  ui.refreshMicsBtn.disabled = ui.sourceSelect.value !== "mic";
  setStatus("已停止发送");
}

ui.openBtn.addEventListener("click", () => {
  log("点击打开文件");
  ui.filePicker.click();
});
ui.filePicker.addEventListener("change", (e) => {
  const file = e.target.files[0];
  if (file) loadMediaFile(file);
});

ui.media.addEventListener("loadedmetadata", () => {
  ui.progress.max = Math.floor(ui.media.duration * 1000);
  ui.progressTime.textContent = `${fmtMs(0)} / ${fmtMs(ui.media.duration * 1000)}`;
});

ui.media.addEventListener("timeupdate", () => {
  if (!ui.progress.matches(":active")) {
    ui.progress.value = Math.floor(ui.media.currentTime * 1000);
  }
  ui.progressTime.textContent = `${fmtMs(ui.media.currentTime * 1000)} / ${fmtMs(ui.media.duration * 1000)}`;
});

ui.progress.addEventListener("input", () => {
  ui.progressTime.textContent = `${fmtMs(ui.progress.value)} / ${fmtMs(ui.media.duration * 1000)}`;
});
ui.progress.addEventListener("change", () => {
  ui.media.currentTime = ui.progress.value / 1000;
});

ui.playPauseBtn.addEventListener("click", () => {
  if (ui.media.paused) {
    ui.media.play();
    ui.playPauseBtn.textContent = "暂停";
  } else {
    ui.media.pause();
    ui.playPauseBtn.textContent = "播放";
  }
});

ui.stopMediaBtn.addEventListener("click", () => {
  ui.media.pause();
  ui.media.currentTime = 0;
  ui.playPauseBtn.textContent = "播放";
});

ui.media.addEventListener("play", () => {
  ui.playPauseBtn.textContent = "暂停";
});
ui.media.addEventListener("pause", () => {
  ui.playPauseBtn.textContent = "播放";
});

ui.media.addEventListener("ended", () => {
  ui.playPauseBtn.textContent = "播放";
  ui.progress.value = 0;
  if (serialState.sending) {
    stopAll();
  }
});

ui.media.addEventListener("dragover", (e) => {
  e.preventDefault();
});

ui.media.addEventListener("drop", (e) => {
  e.preventDefault();
  const file = e.dataTransfer.files[0];
  if (file) loadMediaFile(file);
});

ui.refreshPortsBtn.addEventListener("click", () => {
  log("刷新按钮点击");
  refreshPorts(true);
});
ui.connectBtn.addEventListener("click", connectSerial);
ui.disconnectBtn.addEventListener("click", disconnectSerial);
ui.refreshMicsBtn.addEventListener("click", refreshMics);

ui.sourceSelect.addEventListener("change", () => {
  const isFile = ui.sourceSelect.value === "file";
  ui.micSelect.disabled = isFile;
  ui.refreshMicsBtn.disabled = isFile;
  updateMediaControls();
});

ui.modeSelect.addEventListener("change", () => {
  configState.mode = parseInt(ui.modeSelect.value, 10);
  updateParamUI();
  updateConfigVisibility();
});

ui.paramSlider.addEventListener("input", () => {
  configState.param = clamp(parseInt(ui.paramSlider.value, 10) || 0, 0, 255);
  ui.paramValue.value = String(configState.param);
  updateParamUI();
});

ui.paramValue.addEventListener("input", () => {
  const v = clamp(parseInt(ui.paramValue.value, 10) || 0, 0, 255);
  configState.param = v;
  ui.paramSlider.value = String(v);
  updateParamUI();
});

ui.floorInput.addEventListener("input", () => {
  const v = clamp(parseInt(ui.floorInput.value, 10) || 0, 0, 120);
  ui.floorInput.value = String(v);
  if (ui.floorValue) {
    ui.floorValue.textContent = `${v} dB`;
  }
});

ui.colorPicker.addEventListener("input", () => {
  const rgb = parseHexColor(ui.colorPicker.value);
  if (!rgb) return;
  const hsv = rgbToHsv(rgb.r, rgb.g, rgb.b);
  configState.hue = hsv.h;
  configState.sat = hsv.s;
  configState.val = hsv.v;
  ui.colorHex.value = ui.colorPicker.value;
  if (ui.colorSwatch) {
    ui.colorSwatch.style.backgroundColor = ui.colorPicker.value;
  }
});

ui.colorHex.addEventListener("change", () => {
  const rgb = parseHexColor(ui.colorHex.value);
  if (!rgb) {
    ui.colorHex.value = ui.colorPicker.value;
    return;
  }
  const hsv = rgbToHsv(rgb.r, rgb.g, rgb.b);
  configState.hue = hsv.h;
  configState.sat = hsv.s;
  configState.val = hsv.v;
  const hex = `#${toHex2(rgb.r)}${toHex2(rgb.g)}${toHex2(rgb.b)}`;
  ui.colorPicker.value = hex;
  if (ui.colorSwatch) {
    ui.colorSwatch.style.backgroundColor = hex;
  }
});

if (ui.colorSwatch && ui.colorPicker) {
  ui.colorSwatch.addEventListener("click", () => {
    ui.colorPicker.click();
  });
}

ui.serialToggle.addEventListener("click", () => {
  const hidden = ui.serialSettingsRow.dataset.hidden === "1";
  ui.serialSettingsRow.dataset.hidden = hidden ? "" : "1";
  ui.serialSettingsRow.style.display = hidden ? "" : "none";
});

ui.logToggle.addEventListener("click", () => {
  const hidden = ui.logPanel.dataset.hidden === "1";
  ui.logPanel.dataset.hidden = hidden ? "" : "1";
  ui.logPanel.style.display = hidden ? "" : "none";
  ui.logToggle.textContent = hidden ? "隐藏" : "显示";
});

ui.sendConfigBtn.addEventListener("click", async () => {
  const base = [
    configState.mode & 0xffff,
    clamp(configState.hue, 0, 359),
    clamp(configState.sat, 0, 255),
    clamp(configState.val, 0, 255),
  ];
  const values = (configState.mode === 6) ? base : base.concat([clamp(configState.param, 0, 255)]);
  try {
    const frame = buildWriteMultipleRegisters(0x0000, values);
    const logMsg = (values.length === 4)
      ? `cfg mode=${values[0]} hsv=${values[1]}/${values[2]}/${values[3]}`
      : `cfg mode=${values[0]} hsv=${values[1]}/${values[2]}/${values[3]} param=${values[4]}`;
    await sendOnce(frame, logMsg);
    setStatus("配置已发送");
  } catch (err) {
    setStatus(`发送配置失败: ${err}`);
  }
});

ui.readConfigBtn.addEventListener("click", readConfigFromDevice);

ui.startBtn.addEventListener("click", startAll);
ui.stopBtn.addEventListener("click", stopAll);

ui.progress.addEventListener("mousedown", () => {
  ui.progress.dataset.seeking = "1";
});

ui.progress.addEventListener("mouseup", () => {
  ui.progress.dataset.seeking = "";
});

window.addEventListener("beforeunload", () => {
  stopAudio();
  stopSending();
});

window.addEventListener("error", (e) => {
  log(`JS错误: ${e.message}`);
});

window.addEventListener("unhandledrejection", (e) => {
  log(`Promise错误: ${e.reason}`);
});

log(`页面: ${location.href}`);
log(`安全上下文: ${isSecureContext}`);
log(`Web Serial: ${!!navigator.serial}`);
ui.refreshPortsBtn.disabled = false;

refreshPorts(false);
refreshMics();
updateMediaControls();
setGenreText(0.0);
updateColorUIFromHSV();
updateParamUI();
updateConfigVisibility();
if (ui.floorValue && ui.floorInput) {
  ui.floorValue.textContent = `${ui.floorInput.value} dB`;
}
if (ui.logToggle) {
  ui.logPanel.dataset.hidden = "1";
  ui.logPanel.style.display = "none";
  ui.logToggle.textContent = "显示";
}
if (ui.serialSettingsRow) {
  ui.serialSettingsRow.dataset.hidden = "1";
  ui.serialSettingsRow.style.display = "none";
}



/* ===== 角色配色页（BanG Dream!） ===== */
(function () {
  function normalizeHex(hex) {
    if (!hex) return null;
    let s = String(hex).trim();
    if (!s.startsWith("#")) s = "#" + s;
    if (!/^#[0-9a-fA-F]{6}$/.test(s)) return null;
    return "#" + s.slice(1).toUpperCase();
  }

  function isGrayish(hex) {
    const rgb = parseHexColor(hex);
    if (!rgb) return false;
    const mx = Math.max(rgb.r, rgb.g, rgb.b);
    const mn = Math.min(rgb.r, rgb.g, rgb.b);
    return (mx - mn) <= 10; // 近似灰阶：不建议发给不支持灰/黑的灯
  }

  // 对外暴露：用于“角色配色”页点击后切换灯色
  window.setLampColorHex = async function (hex, opts = {}) {
    const send = (opts && typeof opts.send === "boolean") ? opts.send : true;
    const n = normalizeHex(hex);
    if (!n) return false;

    // 灰阶处理：如果你的灯不支持灰/黑，这里默认不发送
    if (isGrayish(n)) {
      logLine && logLine(`角色代表色 ${n} 接近灰阶，已仅更新 UI（未自动发送）`, "warn");
      // 仍更新 UI
    }

    const rgb = parseHexColor(n);
    if (!rgb) return false;
    const hsv = rgbToHsv(rgb.r, rgb.g, rgb.b);
    configState.hue = hsv.h;
    configState.sat = hsv.s;
    configState.val = hsv.v;

    if (ui.colorPicker) ui.colorPicker.value = n;
    if (ui.colorHex) ui.colorHex.value = n;
    if (ui.colorSwatch) ui.colorSwatch.style.backgroundColor = n;

    // 若你希望点击头像立即生效：发送当前配置
    if (send && ui.sendConfigBtn && !isGrayish(n)) {
      ui.sendConfigBtn.click();
    }
    return true;
  };

  function setActiveNav(active) {
    const mainBtn = document.getElementById("navMain");
    const charBtn = document.getElementById("navCharacters");
    if (mainBtn) mainBtn.classList.toggle("active", active === "main");
    if (charBtn) charBtn.classList.toggle("active", active === "characters");
  }

  function showPage(which) {
    const main = document.getElementById("page-main");
    const chars = document.getElementById("page-characters");
    if (main) main.classList.toggle("hidden", which !== "main");
    if (chars) chars.classList.toggle("hidden", which !== "characters");
    setActiveNav(which);
  }

  function initNav() {
    const mainBtn = document.getElementById("navMain");
    const charBtn = document.getElementById("navCharacters");
    if (mainBtn) mainBtn.addEventListener("click", () => {
      history.replaceState(null, "", "#main");
      showPage("main");
    });
    if (charBtn) charBtn.addEventListener("click", () => {
      history.replaceState(null, "", "#characters");
      showPage("characters");
    });

    const hash = (location.hash || "").toLowerCase();
    if (hash.includes("character")) showPage("characters");
    else showPage("main");
    window.addEventListener("hashchange", () => {
      const h = (location.hash || "").toLowerCase();
      if (h.includes("character")) showPage("characters");
      else showPage("main");
    });
  }

  function initCharacterPage() {
    const grid = document.getElementById("charGrid");
    const search = document.getElementById("charSearch");
    const filter = document.getElementById("bandFilter");
    const hint = document.getElementById("charHint");

    if (!grid || !search || !filter) return;
    const data = Array.isArray(window.BANGDREAM_CHARACTERS) ? window.BANGDREAM_CHARACTERS.slice() : [];

    // bands
    const bands = Array.from(new Set(data.map(d => d.band).filter(Boolean))).sort((a,b)=>a.localeCompare(b,"zh"));
    for (const b of bands) {
      const opt = document.createElement("option");
      opt.value = b;
      opt.textContent = b;
      filter.appendChild(opt);
    }

    function render() {
      const q = (search.value || "").trim().toLowerCase();
      const band = filter.value || "";
      let list = data;

      if (band) list = list.filter(x => x.band === band);
      if (q) {
        list = list.filter(x => (x.name || "").toLowerCase().includes(q) || (x.band || "").toLowerCase().includes(q));
      }

      grid.innerHTML = "";
      if (hint) hint.textContent = `共 ${list.length} 名角色`;

      for (const item of list) {
        const hex = normalizeHex(item.hex) || "#FF66AA";
        const name = item.name || "未知";
        const bandName = item.band || "";
        const gray = isGrayish(hex);

        const btn = document.createElement("button");
        btn.type = "button";
        btn.className = "char-card" + (gray ? " is-gray" : "");
        btn.setAttribute("data-hex", hex);
        btn.setAttribute("title", gray ? `${name}（${hex}，灰阶：默认不发送）` : `${name}（${hex}）`);

        const avatar = document.createElement("div");
        avatar.className = "char-avatar";
        avatar.style.background = `radial-gradient(circle at 30% 30%, ${hex}, rgba(0,0,0,0) 70%), ${hex}`;

        const initial = document.createElement("span");
        initial.className = "char-initial";
        initial.textContent = name.slice(0, 1);
        avatar.appendChild(initial);

        const meta = document.createElement("div");
        meta.className = "char-meta";

        const n = document.createElement("div");
        n.className = "char-name";
        n.textContent = name;

        const b = document.createElement("div");
        b.className = "char-band";
        b.textContent = bandName;

        meta.appendChild(n);
        meta.appendChild(b);

        const chip = document.createElement("span");
        chip.className = "char-chip";
        chip.style.backgroundColor = hex;

        btn.appendChild(avatar);
        btn.appendChild(meta);
        btn.appendChild(chip);

        btn.addEventListener("click", async () => {
          await window.setLampColorHex(hex, { send: true });
        });

        grid.appendChild(btn);
      }
    }

    search.addEventListener("input", render);
    filter.addEventListener("change", render);
    render();
  }

  // init after DOM ready (script is at end, but保险起见)
  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", () => {
      initNav();
      initCharacterPage();
    });
  } else {
    initNav();
    initCharacterPage();
  }
})();
