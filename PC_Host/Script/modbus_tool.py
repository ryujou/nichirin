#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simple Modbus RTU CLI for Nichirin V3.

Examples:
  python modbus_tool.py --port COM3 read --start 0 --count 5
  python modbus_tool.py --port COM3 write --reg 0 --value 1
  python modbus_tool.py --port COM3 write-multi --start 0 --values 1 120 200 255 80
"""

from __future__ import annotations

import argparse
import time
from typing import List

import serial

from pc_common import (
    ADDR,
    REG_MODE,
    REG_HUE,
    REG_SAT,
    REG_VAL,
    REG_PARAM,
    build_read_holding,
    build_write_single,
    build_write_multi,
    parse_read_holding_response,
    verify_crc,
)


def _read_exact(ser: serial.Serial, size: int, timeout_s: float) -> bytes:
    deadline = time.monotonic() + max(0.01, timeout_s)
    buf = bytearray()
    while len(buf) < size and time.monotonic() < deadline:
        chunk = ser.read(size - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


def _read_response_read(ser: serial.Serial, timeout_s: float) -> bytes:
    header = _read_exact(ser, 3, timeout_s)
    if len(header) < 3:
        raise RuntimeError("timeout waiting for response header")
    byte_count = header[2]
    rest = _read_exact(ser, byte_count + 2, timeout_s)
    if len(rest) < byte_count + 2:
        raise RuntimeError("timeout waiting for response payload")
    return header + rest


def _read_response_fixed(ser: serial.Serial, size: int, timeout_s: float) -> bytes:
    data = _read_exact(ser, size, timeout_s)
    if len(data) < size:
        raise RuntimeError("timeout waiting for response")
    return data


def _open_serial(port: str, baud: int, timeout_s: float) -> serial.Serial:
    return serial.Serial(port, baud, timeout=0.01, write_timeout=timeout_s)


def cmd_read(args: argparse.Namespace) -> None:
    frame = build_read_holding(args.start, args.count, addr=args.addr)
    with _open_serial(args.port, args.baud, args.timeout) as ser:
        ser.reset_input_buffer()
        ser.write(frame)
        resp = _read_response_read(ser, args.timeout)
    values = parse_read_holding_response(resp)
    print("values:", values)


def cmd_read_config(args: argparse.Namespace) -> None:
    args.start = 0
    args.count = 5
    cmd_read(args)


def _check_write_echo(resp: bytes) -> None:
    if not verify_crc(resp):
        raise RuntimeError("CRC check failed")


def cmd_write(args: argparse.Namespace) -> None:
    frame = build_write_single(args.reg, args.value, addr=args.addr)
    with _open_serial(args.port, args.baud, args.timeout) as ser:
        ser.reset_input_buffer()
        ser.write(frame)
        resp = _read_response_fixed(ser, 8, args.timeout)
    _check_write_echo(resp)
    print("ok")


def cmd_write_multi(args: argparse.Namespace) -> None:
    values: List[int] = args.values
    if not values:
        raise RuntimeError("values is empty")
    frame = build_write_multi(args.start, values, addr=args.addr)
    with _open_serial(args.port, args.baud, args.timeout) as ser:
        ser.reset_input_buffer()
        ser.write(frame)
        resp = _read_response_fixed(ser, 8, args.timeout)
    _check_write_echo(resp)
    print("ok")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Nichirin V3 Modbus RTU CLI")
    parser.add_argument("--port", required=True, help="serial port, e.g. COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--addr", type=int, default=ADDR)
    parser.add_argument("--timeout", type=float, default=0.3)

    sub = parser.add_subparsers(dest="cmd", required=True)

    p_read = sub.add_parser("read", help="read holding registers")
    p_read.add_argument("--start", type=int, required=True)
    p_read.add_argument("--count", type=int, required=True)
    p_read.set_defaults(func=cmd_read)

    p_read_cfg = sub.add_parser("read-config", help="read MODE/HUE/SAT/VAL/PARAM")
    p_read_cfg.set_defaults(func=cmd_read_config)

    p_write = sub.add_parser("write", help="write single register")
    p_write.add_argument("--reg", type=int, required=True)
    p_write.add_argument("--value", type=int, required=True)
    p_write.set_defaults(func=cmd_write)

    p_wm = sub.add_parser("write-multi", help="write multiple registers")
    p_wm.add_argument("--start", type=int, required=True)
    p_wm.add_argument("--values", type=int, nargs="+", required=True)
    p_wm.set_defaults(func=cmd_write_multi)

    p_quick = sub.add_parser("quick", help="quick write MODE/HUE/SAT/VAL/PARAM")
    p_quick.add_argument("--mode", type=int, required=True)
    p_quick.add_argument("--hue", type=int, required=True)
    p_quick.add_argument("--sat", type=int, required=True)
    p_quick.add_argument("--val", type=int, required=True)
    p_quick.add_argument("--param", type=int, required=True)
    p_quick.set_defaults(func=cmd_write_multi, start=REG_MODE)

    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if args.cmd == "quick":
        args.values = [args.mode, args.hue, args.sat, args.val, args.param]
    args.func(args)


if __name__ == "__main__":
    main()
