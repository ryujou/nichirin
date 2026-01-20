import argparse
import struct
import time
import serial
import random

ADDR = 0x01
FUNC = 0x20

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF

def build_frame(bands12):
    if len(bands12) != 12:
        raise ValueError("bands must be length 12")
    payload = bytes([ADDR, FUNC] + [int(x) & 0xFF for x in bands12])
    crc = crc16_modbus(payload)
    return payload + struct.pack("<H", crc)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="COM5 or /dev/ttyUSB0", default="COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", choices=["one", "ramp", "random", "sine"], default="one")
    ap.add_argument("--fps", type=float, default=50.0, help="send rate for continuous modes")
    ap.add_argument("--print", action="store_true", help="print hex of frames")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    print(f"opened {args.port} @ {args.baud}")

    if args.mode == "one":
        bands = [255] + [0]*11
        frame = build_frame(bands)
        if args.print:
            print("frame:", frame.hex(" "))
        ser.write(frame)
        ser.flush()
        print("sent one frame (band0=255)")
        return

    period = 1.0 / max(args.fps, 1e-6)
    t0 = time.time()
    k = 0

    while True:
        if args.mode == "ramp":
            # 一个频段来回扫，强度随 k 变化
            idx = (k // 4) % 12
            val = (k * 8) & 0xFF
            bands = [0]*12
            bands[idx] = val

        elif args.mode == "random":
            bands = [random.randint(0, 255) for _ in range(12)]

        elif args.mode == "sine":
            # 简易“伪频谱”：不同频段相位不同的正弦
            import math
            t = time.time() - t0
            bands = []
            for i in range(12):
                x = 0.5 + 0.5 * math.sin(2*math.pi*(0.8 + i*0.05)*t + i*0.6)
                bands.append(int(x * 255) & 0xFF)

        frame = build_frame(bands)
        if args.print and (k % int(args.fps) == 0):
            print("frame:", frame.hex(" "))

        ser.write(frame)
        ser.flush()

        k += 1
        time.sleep(period)

if __name__ == "__main__":
    main()
