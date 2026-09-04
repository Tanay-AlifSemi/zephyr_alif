#!/usr/bin/env python3
# Copyright (c) 2026 Alif Semiconductor
# SPDX-License-Identifier: Apache-2.0
"""PC TX/RX check for B1 blinky UART2 echo.

Firmware (after clock / LA tests) echoes every byte on UART2. This script
writes an incrementing pattern and reads the same bytes back.

    host TX -> device UART2 RX
    device UART2 TX -> host RX
    GND -- GND

Same USB-COM as the 115200 log. Close that terminal first. Match --baud
to CONFIG_ALIF_UART_ECHO_BAUD (firmware default is now 115200 so the
USB-COM path can PASS). Use --dtr off so opening the port does not
reset the board.

    pip install pyserial

    # USB-COM only works at 115200. Close the 115200 terminal first.
    python3 samples/basic/blinky/tools/uart_echo_test.py \
        --port /dev/ttyUSB2 --baud 115200 --dtr off \
        --iterations 500 --repeat 3
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required. Install it with: pip install pyserial")


def parse_args():
    p = argparse.ArgumentParser(
        description="UART2 echo TX/RX verify (PC side).")
    p.add_argument("--port", required=True,
                   help="USB-COM of UART2 (e.g. /dev/ttyUSB2).")
    p.add_argument("--baud", type=int, default=115200,
                   help="Must match firmware echo baud (default: 115200).")
    p.add_argument("--block-size", type=int, default=256,
                   help="Bytes per TX/RX block (default: 256).")
    p.add_argument("--iterations", type=int, default=2000,
                   help="Blocks to send, or 0 until Ctrl-C (default: 2000).")
    p.add_argument("--repeat", type=int, default=1,
                   help="Run the TX/RX session this many times (default: 1).")
    p.add_argument("--start", type=int, default=0,
                   help="First byte of the incrementing counter (0-255).")
    p.add_argument("--interval", type=float, default=0.0,
                   help="Pause between blocks in seconds (default: 0).")
    p.add_argument("--rx-timeout", type=float, default=1.0,
                   help="Seconds to wait for a full echo (default: 1.0).")
    p.add_argument("--progress-interval", type=int, default=200,
                   help="Progress line every N blocks (default: 200).")
    p.add_argument("--dtr", choices=["on", "off"], default="off",
                   help="DTR after open (default: off — avoids MCU reset).")
    p.add_argument("--rts", choices=["on", "off"], default="on")
    p.add_argument("--sync-timeout", type=float, default=90.0,
                   help="Seconds to wait for firmware ECHO banner (default: 90).")
    p.add_argument("--skip-sync", action="store_true",
                   help="Do not wait for ECHO; start TX/RX immediately.")
    return p.parse_args()


def make_block(size, start):
    cycle = bytes(range(256))
    block = bytearray(size)
    filled = 0
    c = start & 0xFF
    while filled < size:
        take = min(256 - c, size - filled)
        block[filled:filled + take] = cycle[c:c + take]
        filled += take
        c = (c + take) & 0xFF
    return bytes(block), c


def drain_until_quiet(ser, quiet_s=0.25, max_s=2.0):
    """Drop leftover ECHO banners / stale RX until the line is idle."""
    ser.timeout = 0.05
    deadline = time.monotonic() + max_s
    quiet_until = time.monotonic() + quiet_s
    while time.monotonic() < deadline and time.monotonic() < quiet_until:
        chunk = ser.read(256)
        if chunk:
            quiet_until = time.monotonic() + quiet_s
    ser.reset_input_buffer()


def wait_echo_live(ser, tries=8, timeout=1.0):
    """Do not score until firmware echoes a marker (avoids start-of-run timeouts)."""
    marker = b"\xA5\x5A\xC3\x3C"
    for attempt in range(1, tries + 1):
        drain_until_quiet(ser, quiet_s=0.05, max_s=0.4)
        ser.write(marker)
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            ser.timeout = min(0.1, max(0.01, deadline - time.monotonic()))
            chunk = ser.read(32)
            if not chunk:
                continue
            buf.extend(chunk)
            if marker in buf:
                print(f"Echo live (marker came back, try {attempt}).")
                drain_until_quiet(ser, quiet_s=0.05, max_s=0.3)
                return True
            if len(buf) > 128:
                buf = buf[-8:]
        print(f"  sync try {attempt}: marker not echoed yet")
    return False


def read_exact(ser, n, deadline):
    buf = bytearray()
    while len(buf) < n:
        remain = deadline - time.monotonic()
        if remain <= 0:
            break
        ser.timeout = remain
        chunk = ser.read(n - len(buf))
        if not chunk:
            break
        buf.extend(chunk)
    return bytes(buf)


def main():
    args = parse_args()
    if args.block_size <= 0:
        sys.exit("block-size must be positive.")
    if not 0 <= args.start <= 255:
        sys.exit("start must be in 0..255.")

    try:
        ser = serial.Serial()
        ser.port = args.port
        ser.baudrate = args.baud
        ser.timeout = args.rx_timeout
        ser.write_timeout = 5.0
        ser.dsrdtr = False
        ser.dtr = False
        ser.open()
    except serial.SerialException as exc:
        sys.exit(f"Could not open {args.port}: {exc}")

    for line, value in (("dtr", args.dtr == "on"), ("rts", args.rts == "on")):
        try:
            setattr(ser, line, value)
        except (OSError, ValueError) as exc:
            print(f"Note: could not set {line.upper()} ({exc}); continuing.")

    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    if args.repeat < 1:
        ser.close()
        sys.exit("repeat must be >= 1.")

    infinite = args.iterations <= 0
    if infinite and args.repeat > 1:
        ser.close()
        sys.exit("--repeat needs a finite --iterations (not 0).")

    print(f"Port         : {args.port}")
    print(f"Baud         : {args.baud}")
    print(f"Block size   : {args.block_size}")
    print(f"Iterations   : {'infinite' if infinite else args.iterations}")
    print(f"Repeat       : {args.repeat}")
    print("-" * 48)

    if not args.skip_sync:
        print(f"Waiting up to {args.sync_timeout:.0f}s for firmware ECHO ...")
        ser.timeout = 0.2
        deadline = time.monotonic() + args.sync_timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = ser.read(64)
            if chunk:
                buf.extend(chunk)
                if b"ECHO" in buf:
                    print("Got ECHO — waiting until firmware echoes a marker.")
                    break
                if len(buf) > 4096:
                    buf = buf[-64:]
        else:
            ser.close()
            sys.exit("No ECHO from firmware. Close the 115200 terminal, "
                     "flash blinky, match --baud, use --dtr off.")

    if not wait_echo_live(ser):
        ser.close()
        sys.exit("Firmware never echoed the sync marker. Flash blinky, "
                 "close the 115200 terminal, match --baud, use --dtr off.")

    session_pass = 0
    session_fail = 0
    rc = 0

    try:
        for run in range(1, args.repeat + 1):
            drain_until_quiet(ser)
            ser.timeout = args.rx_timeout
            if args.repeat > 1:
                print(f"\n===== session {run}/{args.repeat} =====")
            print("TX incrementing pattern, expect the same bytes on RX. "
                  "Ctrl-C to stop.")

            counter = args.start & 0xFF
            n = 0
            bytes_ok = 0
            block_fail = 0
            byte_err = 0
            timeouts = 0
            start = time.monotonic()

            while infinite or n < args.iterations:
                tx, counter = make_block(args.block_size, counter)
                ser.write(tx)
                deadline = time.monotonic() + args.rx_timeout
                rx = read_exact(ser, len(tx), deadline)
                n += 1

                if len(rx) != len(tx):
                    timeouts += 1
                    block_fail += 1
                    print(f"  TIMEOUT block {n}: got {len(rx)}/{len(tx)}",
                          flush=True)
                elif rx != tx:
                    errs = sum(1 for a, b in zip(tx, rx) if a != b)
                    byte_err += errs
                    block_fail += 1
                    print(f"  MISMATCH block {n}: {errs} byte errors "
                          f"(first tx={tx[0]:02x} rx={rx[0]:02x})",
                          flush=True)
                else:
                    bytes_ok += len(tx)

                if args.progress_interval and n % args.progress_interval == 0:
                    elapsed = time.monotonic() - start
                    rate = (bytes_ok / elapsed / 1000.0) if elapsed > 0 else 0.0
                    print(f"  blocks={n} ok_bytes={bytes_ok} fail={block_fail} "
                          f"byte_err={byte_err} timeout={timeouts} "
                          f"{rate:.1f} kB/s",
                          flush=True)

                if args.interval > 0:
                    time.sleep(args.interval)

            elapsed = time.monotonic() - start
            thr = (bytes_ok / elapsed) if elapsed > 0 else 0.0
            print("-" * 48)
            print(f"Blocks       : {n}  (fail {block_fail})")
            print(f"Bytes OK     : {bytes_ok}")
            print(f"Byte errors  : {byte_err}")
            print(f"Timeouts     : {timeouts}")
            print(f"Elapsed      : {elapsed:.3f} s")
            print(f"Throughput   : {thr / 1000.0:.1f} kB/s "
                  f"(~{thr * 10.0 / 1e6:.2f} Mbit/s wire, one way)")
            print("-" * 48)
            if n > 0 and block_fail == 0:
                print("RESULT: PASS  (TX and RX matched on the wire)")
                session_pass += 1
            else:
                print("RESULT: FAIL")
                session_fail += 1
                rc = 1
    except KeyboardInterrupt:
        print("\nStopped by user.")
        rc = 1
    except serial.SerialTimeoutException:
        print("\nWrite timed out.")
        rc = 1
    finally:
        ser.close()

    if args.repeat > 1:
        print(f"\nSessions PASS={session_pass} FAIL={session_fail} "
              f"of {args.repeat}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
