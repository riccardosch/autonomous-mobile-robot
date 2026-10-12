#!/usr/bin/env python3
"""Serial link speed test for the micro-ROS transport (test T7).

Needs the ESP32 flashed with this folder's firmware. For each baud rate the
script asks the ESP32 to switch, sends random bytes for a few seconds, and
checks that every byte comes back unchanged and in order.

    python3 serial_speed.py                         # 115200 ... 921600, /dev/ttyUSB0
    python3 serial_speed.py --bauds 921600 --seconds 30
"""

import argparse
import random
import time

import serial

BASE_BAUD = 115200
WINDOW_BYTES = 1024  # bytes sent but not yet echoed back, kept below the ESP32 buffers
CHUNK_BYTES = 256
BOOT_TIMEOUT_S = 5.0
BOOT_SETTLE_S = 0.5        # after "ready", before the first command
REPLY_TIMEOUT_S = 2.0
DRAIN_TIMEOUT_S = 0.5      # must stay below the ESP32's 1 s idle return
ESP32_IDLE_RETURN_S = 1.0  # the ESP32 goes back to BASE_BAUD after 1 s without data


def read_line(port, timeout_s):
    """One text line from the ESP32, or "" on timeout."""
    end = time.monotonic() + timeout_s
    buf = b""
    while time.monotonic() < end:
        buf += port.read(1)
        if buf.endswith(b"\n"):
            return buf.decode(errors="replace").strip()
    return ""


def wait_for(port, prefix, timeout_s):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        line = read_line(port, end - time.monotonic())
        if line.startswith(prefix):
            return line
    return ""


def run_baud(port, baud, seconds, rng):
    port.baudrate = BASE_BAUD
    port.reset_input_buffer()
    port.write(f"baud {baud}\n".encode())
    if not wait_for(port, f"ok {baud}", REPLY_TIMEOUT_S):
        print(f"{baud:>8}  no answer from the ESP32")
        return False
    time.sleep(0.05)
    port.baudrate = baud
    port.reset_input_buffer()

    sent = bytearray()
    received = bytearray()
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        if len(sent) - len(received) < WINDOW_BYTES:
            chunk = rng.randbytes(CHUNK_BYTES)
            port.write(chunk)
            sent += chunk
        received += port.read(port.in_waiting)
    end = time.monotonic() + DRAIN_TIMEOUT_S
    while len(received) < len(sent) and time.monotonic() < end:
        received += port.read(port.in_waiting or 1)
    elapsed = time.monotonic() - start

    compared = min(len(sent), len(received))
    wrong = sum(1 for a, b in zip(sent[:compared], received[:compared]) if a != b)
    first_wrong = next((i for i in range(compared) if sent[i] != received[i]), None)
    missing = len(sent) - len(received)
    ok = wrong == 0 and missing == 0
    print(f"{baud:>8}  {len(received):>9} bytes  {len(received) / elapsed / 1000:6.1f} kB/s each way  "
          f"wrong {wrong}  missing {missing}"
          + (f"  first error at byte {first_wrong}" if first_wrong is not None else "")
          + ("  PASS" if ok else "  FAIL"))

    # Back to BASE_BAUD before the ESP32 sends its report there.
    port.baudrate = BASE_BAUD
    report = wait_for(port, "echoed", ESP32_IDLE_RETURN_S + REPLY_TIMEOUT_S)
    if report:
        print(f"          ESP32: {report}")
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--bauds", type=int, nargs="+", default=[115200, 230400, 460800, 921600])
    parser.add_argument("--seconds", type=float, default=5.0, help="test length per baud rate")
    args = parser.parse_args()

    port = serial.Serial()
    port.port = args.port
    port.baudrate = BASE_BAUD
    port.timeout = 0.01
    port.dtr = False
    port.rts = False
    port.open()  # this resets the ESP32
    if not wait_for(port, "serial speed test ready", BOOT_TIMEOUT_S):
        print("no 'ready' from the ESP32: is the serial_speed firmware flashed?")
        return
    time.sleep(BOOT_SETTLE_S)
    print("    baud   echoed            throughput             result")
    rng = random.Random(1)
    results = [run_baud(port, b, args.seconds, rng) for b in args.bauds]
    port.close()
    print("all PASS" if all(results) else "some FAIL")


if __name__ == "__main__":
    main()
